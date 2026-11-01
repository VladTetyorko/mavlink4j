/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
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

/**
 * Interleaved traffic from two
 * different sources on one boundary-preserving link must not corrupt each other's resync state:
 * a single shared buffer would splice source A's tail onto source B's head and both frames
 * would fail to decode, or decode as garbage.
 */
class PerSourceResyncIsolationTest {

    private static final LinkPeer TARGET = new LinkPeer("127.0.0.1", 14_550);
    private static final LinkPeer SOURCE_A = new LinkPeer("10.0.0.1", 14_550);
    private static final LinkPeer SOURCE_B = new LinkPeer("10.0.0.2", 14_550);

    @Test
    void twoInterleavedSplitFramesFromDifferentSourcesBothDecodeCorrectly() {
        byte[] frameA = encodeOneHeartbeat(11);
        byte[] frameB = encodeOneHeartbeat(22);
        int splitA = frameA.length / 2;
        int splitB = frameB.length / 2;

        SendCapturingLink listenLink = new SendCapturingLink("udp-listen:0.0.0.0:14550", true, TARGET);
        FrameReader reader = new FrameReader(listenLink);
        List<Frame> frames = new ArrayList<>();

        // A's first half, then B's first half -- interleaved before either completes.
        reader.offer(chunk(Arrays.copyOfRange(frameA, 0, splitA), SOURCE_A), frames::add);
        assertEquals(0, frames.size());
        reader.offer(chunk(Arrays.copyOfRange(frameB, 0, splitB), SOURCE_B), frames::add);
        assertEquals(0, frames.size());

        // A's second half completes ONLY A's frame -- B's buffer must be untouched.
        reader.offer(chunk(Arrays.copyOfRange(frameA, splitA, frameA.length), SOURCE_A), frames::add);
        assertEquals(1, frames.size(), "A's frame should complete without needing B's bytes");
        assertEquals(11L, frames.get(0).as(Heartbeat.class).customMode());
        assertEquals(SOURCE_A, frames.get(0).source());

        // B's second half now completes B's frame, independently.
        reader.offer(chunk(Arrays.copyOfRange(frameB, splitB, frameB.length), SOURCE_B), frames::add);
        assertEquals(2, frames.size());
        assertEquals(22L, frames.get(1).as(Heartbeat.class).customMode());
        assertEquals(SOURCE_B, frames.get(1).source());
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
