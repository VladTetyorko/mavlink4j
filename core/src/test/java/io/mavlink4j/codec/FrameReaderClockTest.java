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

import java.io.ByteArrayOutputStream;
import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * The codec layer owns no wall clock: {@link FrameReader} stamps {@link Frame#receivedAt()} from the
 * {@link MavlinkSettings#clock()} it was built with, once per chunk. A manual clock makes the stamp exact.
 */
class FrameReaderClockTest {

    private static final LinkPeer TARGET = new LinkPeer("127.0.0.1", 14_550);
    private static final LinkPeer SOURCE = new LinkPeer("10.0.0.1", 14_550);
    private static final Instant T0 = Instant.parse("2026-03-04T05:06:07Z");

    @Test
    void receivedAtIsTheInjectedClockReadingAtTheMomentTheChunkIsOffered() {
        ManualClock clock = new ManualClock(T0);
        FrameReader reader = readerOn(clock);
        byte[] heartbeat = encodeHeartbeat();
        List<Frame> frames = new ArrayList<>();

        reader.offer(new ByteChunk(heartbeat, heartbeat.length, SOURCE), frames::add);
        clock.advance(Duration.ofSeconds(90));
        reader.offer(new ByteChunk(heartbeat, heartbeat.length, SOURCE), frames::add);

        assertEquals(2, frames.size());
        assertEquals(T0, frames.get(0).receivedAt());
        assertEquals(T0.plusSeconds(90), frames.get(1).receivedAt());
    }

    @Test
    void everyFrameOfOneChunkShareOneStamp() {
        ManualClock clock = new ManualClock(T0);
        FrameReader reader = readerOn(clock);
        byte[] one = encodeHeartbeat();
        ByteArrayOutputStream two = new ByteArrayOutputStream();
        two.writeBytes(one);
        two.writeBytes(one);
        byte[] both = two.toByteArray();
        List<Frame> frames = new ArrayList<>();

        reader.offer(new ByteChunk(both, both.length, LinkPeer.NONE), frames::add);

        assertEquals(2, frames.size());
        assertEquals(T0, frames.get(0).receivedAt());
        assertEquals(T0, frames.get(1).receivedAt());
    }

    private static FrameReader readerOn(InstantSource clock) {
        SendCapturingLink link = new SendCapturingLink("udp-listen:0.0.0.0:14550", true, TARGET);
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().clock(clock).build();
        return new FrameReader(link, settings, Signing.off(), rejection -> { });
    }

    private static byte[] encodeHeartbeat() {
        SendCapturingLink link = new SendCapturingLink("encoder-link", true, TARGET);
        FrameWriter writer = new FrameWriter(new SysId(1), new CompId(1), Signing.off());
        writer.addLink(link);
        Heartbeat heartbeat = Heartbeat.builder()
                .type(MavType.MAV_TYPE_QUADROTOR)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(0)
                .systemStatus(MavState.MAV_STATE_STANDBY)
                .mavlinkVersion(3)
                .build();
        writer.sendTo(heartbeat, link.id(), link.defaultTarget(), new PeerId(new SysId(1), new CompId(1)));
        return link.sent().get(0);
    }

    private static final class ManualClock implements InstantSource {
        private Instant now;

        ManualClock(Instant start) {
            this.now = start;
        }

        void advance(Duration by) {
            now = now.plus(by);
        }

        @Override
        public Instant instant() {
            return now;
        }
    }
}
