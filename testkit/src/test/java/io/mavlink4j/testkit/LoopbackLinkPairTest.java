/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.FrameReader;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.MavlinkLink;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The link hinge proven in both directions over one in-memory pair: a boundary-preserving link and a
 * stream link must carry the same frames to {@link FrameReader}, however the bytes are sliced.
 * Replaces core's {@code TcpFramingLspGuardTest}-style hand-built chunks with a real send path.
 */
class LoopbackLinkPairTest {

    private static final Duration SHORT = Duration.ofMillis(200);
    private static final PeerId SENDER = new PeerId(new SysId(1), new CompId(1));

    @Test
    void datagramModeDeliversOneChunkPerSendInBothDirections() throws Exception {
        LoopbackLinkPair pair = new LoopbackLinkPair(true);
        assertTrue(pair.a().preservesMessageBoundaries());

        for (MavlinkLink[] ends : new MavlinkLink[][] {{pair.a(), pair.b()}, {pair.b(), pair.a()}}) {
            send(ends[0], 11);
            send(ends[0], 12);
            List<Frame> frames = readAll(ends[1], 2);
            assertEquals(2, frames.size());
            assertEquals(11L, frames.get(0).as(Heartbeat.class).customMode());
            assertEquals(12L, frames.get(1).as(Heartbeat.class).customMode());
            assertEquals(ends[0].defaultTarget().equals(ends[1].defaultTarget()), false);
        }
    }

    @Test
    void datagramChunksAreExactlyAsSentAndCarryTheSendersAddress() throws Exception {
        LoopbackLinkPair pair = new LoopbackLinkPair(true);
        byte[] raw = {1, 2, 3, 4};
        pair.a().send(raw, 0, raw.length, pair.a().defaultTarget());
        pair.a().send(raw, 0, raw.length, pair.a().defaultTarget());

        ByteChunk first = pair.b().poll(SHORT);
        assertEquals(4, first.length());
        assertEquals(pair.b().defaultTarget(), first.source(), "the source is the sender, who is b's default target");
        assertEquals(4, pair.b().poll(SHORT).length());
        assertNull(pair.b().poll(Duration.ofMillis(20)));
    }

    @Test
    void streamModeSplitsAFrameAcrossPollsAndStillDecodesOneFrame() throws Exception {
        LoopbackLinkPair pair = new LoopbackLinkPair(false, 3);
        assertFalse(pair.a().preservesMessageBoundaries());
        send(pair.a(), 5);

        ByteChunk firstSlice = pair.b().poll(SHORT);
        assertEquals(3, firstSlice.length(), "a stream poll returns at most streamChunkBytes");

        List<Frame> frames = new ArrayList<>();
        FrameReader reader = new FrameReader(pair.b());
        reader.offer(firstSlice, frames::add);
        assertEquals(0, frames.size(), "three bytes are not a frame");
        ByteChunk slice;
        while ((slice = pair.b().poll(Duration.ofMillis(20))) != null) {
            reader.offer(slice, frames::add);
        }
        assertEquals(1, frames.size());
        assertEquals(5L, frames.get(0).as(Heartbeat.class).customMode());
    }

    @Test
    void streamModeGluesBackToBackFramesIntoOneChunkAndBothDecode() throws Exception {
        LoopbackLinkPair pair = new LoopbackLinkPair(false);
        send(pair.b(), 1);
        send(pair.b(), 2);

        List<Frame> frames = readAll(pair.a(), 2);
        assertEquals(List.of(1L, 2L), frames.stream().map(f -> f.as(Heartbeat.class).customMode()).toList());
        assertNull(pair.a().poll(Duration.ofMillis(20)), "one poll drained both frames");
    }

    @Test
    void closeUnblocksAPendingPollWithNullAndDropsLaterSends() throws Exception {
        LoopbackLinkPair pair = new LoopbackLinkPair(true);
        Thread closer = new Thread(() -> {
            sleep(50);
            pair.b().close();
        });
        closer.start();
        assertNull(pair.b().poll(Duration.ofSeconds(5)));
        closer.join();

        byte[] raw = {1};
        pair.a().send(raw, 0, 1, pair.a().defaultTarget());
        assertNull(pair.b().poll(Duration.ofMillis(20)));
        assertNotNull(pair.a().id());
    }

    @Test
    void aNonPositiveStreamChunkIsRefused() throws Exception {
        assertThrows(IllegalArgumentException.class, () -> new LoopbackLinkPair(false, 0));
    }

    private static void send(MavlinkLink from, long customMode) throws Exception {
        FrameWriter writer = new FrameWriter(SENDER.system(), SENDER.component(), Signing.off());
        writer.addLink(from);
        writer.broadcast(Heartbeat.builder()
                .type(MavType.MAV_TYPE_QUADROTOR)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(customMode)
                .systemStatus(MavState.MAV_STATE_STANDBY)
                .mavlinkVersion(3)
                .build(), from.id());
    }

    private static List<Frame> readAll(MavlinkLink to, int expected) throws Exception {
        FrameReader reader = new FrameReader(to);
        List<Frame> frames = new ArrayList<>();
        ByteChunk chunk;
        while (frames.size() < expected && (chunk = to.poll(SHORT)) != null) {
            reader.offer(chunk, frames::add);
        }
        return frames;
    }

    private static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
