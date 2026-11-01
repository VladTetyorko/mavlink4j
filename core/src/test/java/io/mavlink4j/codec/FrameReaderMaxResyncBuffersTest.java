/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Proves {@link MavlinkSettings#maxResyncBuffers()} genuinely bounds concurrent per-source
 * resync buffers via LRU eviction — not merely a value that reaches
 * a field nothing reads. {@link PerSourceResyncIsolationTest} proves isolation between two sources
 * that both fit under the default cap of 64; this test proves the cap itself, by configuring it low
 * enough for a second source's arrival to evict the first source's still-incomplete buffer.
 */
class FrameReaderMaxResyncBuffersTest {

    private static final LinkPeer TARGET = new LinkPeer("127.0.0.1", 14_550);
    private static final LinkPeer SOURCE_A = new LinkPeer("10.0.0.1", 14_550);
    private static final LinkPeer SOURCE_B = new LinkPeer("10.0.0.2", 14_550);

    @Test
    void aSecondSourceEvictsTheFirstSourcesIncompleteBufferWhenCapacityIsOne() {
        byte[] frameA = encodeOneHeartbeat(11);
        int splitA = frameA.length / 2;

        SendCapturingLink listenLink = new SendCapturingLink("udp-listen:0.0.0.0:14550", true, TARGET);
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().maxResyncBuffers(1).build();
        FrameReader reader = new FrameReader(listenLink, settings, Signing.off(), rejection -> { });
        List<Frame> frames = new ArrayList<>();

        // A's first half: allocates A's buffer (size 1, at the configured cap).
        reader.offer(chunk(Arrays.copyOfRange(frameA, 0, splitA), SOURCE_A), frames::add);
        assertEquals(0, frames.size());

        // A single byte from B: allocates B's buffer, pushing size to 2 -- over the cap of 1, so the
        // least-recently-touched buffer (A's) is evicted.
        reader.offer(chunk(new byte[] {0x00}, SOURCE_B), frames::add);
        assertEquals(0, frames.size());

        // A's second half arrives: A's buffer was evicted, so this creates a brand-new empty buffer
        // that never saw the frame's sync byte -- the frame is lost, not just delayed.
        reader.offer(chunk(Arrays.copyOfRange(frameA, splitA, frameA.length), SOURCE_A), frames::add);
        assertEquals(0, frames.size(), "A's frame must be lost: its resync buffer was evicted at capacity 1");
    }

    @Test
    void theSameSequenceCompletesTheFrameWhenCapacityCoversBothSources() {
        byte[] frameA = encodeOneHeartbeat(11);
        int splitA = frameA.length / 2;

        SendCapturingLink listenLink = new SendCapturingLink("udp-listen:0.0.0.0:14550", true, TARGET);
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().maxResyncBuffers(2).build();
        FrameReader reader = new FrameReader(listenLink, settings, Signing.off(), rejection -> { });
        List<Frame> frames = new ArrayList<>();

        reader.offer(chunk(Arrays.copyOfRange(frameA, 0, splitA), SOURCE_A), frames::add);
        reader.offer(chunk(new byte[] {0x00}, SOURCE_B), frames::add);
        reader.offer(chunk(Arrays.copyOfRange(frameA, splitA, frameA.length), SOURCE_A), frames::add);

        assertEquals(1, frames.size(), "capacity 2 must keep both A's and B's buffers alive at once");
        assertTrue(frames.get(0).is(Heartbeat.class));
        assertEquals(11L, frames.get(0).as(Heartbeat.class).customMode());
    }

    private static ByteChunk chunk(byte[] data, LinkPeer source) {
        return new ByteChunk(data, data.length, source);
    }

    private static byte[] encodeOneHeartbeat(long customMode) {
        SendCapturingLink encoderLink = new SendCapturingLink("encoder-link", true, TARGET);
        FrameWriter writer = new FrameWriter(new SysId(1), new CompId(1), Signing.off());
        writer.addLink(encoderLink);
        Heartbeat heartbeat = Heartbeat.builder()
                .type(MavType.MAV_TYPE_QUADROTOR)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(customMode)
                .systemStatus(MavState.MAV_STATE_STANDBY)
                .mavlinkVersion(3)
                .build();
        writer.sendTo(heartbeat, encoderLink.id(), encoderLink.defaultTarget(), new PeerId(new SysId(1), new CompId(1)));
        return encoderLink.sent().get(0);
    }
}
