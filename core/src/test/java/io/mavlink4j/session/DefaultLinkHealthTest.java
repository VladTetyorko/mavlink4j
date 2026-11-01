/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.FrameHeader;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.time.Clock;
import java.time.Duration;
import java.time.ZoneOffset;
import java.time.Instant;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Direct unit tests of {@link DefaultLinkHealth}'s drop-rate accounting. The MAVLink spec defines
 * no drop-rate formula; these tests pin <i>this class's own</i> documented rule:
 * {@code lost = sum(gap - 1)} over consecutive wrapped {@code seq} gaps, {@code dropRate =
 * lost / (received + lost)}.
 */
class DefaultLinkHealthTest {

    private static final MavlinkSettings.LinkHealth CONFIG = MavlinkSettings.LinkHealth.defaults();
    private final MutableClock clock = new MutableClock(Instant.now());
    private static final LinkId LINK = new LinkId("test-link");
    private static final PeerId PEER = new PeerId(new SysId(3), new CompId(1));

    @Test
    void noLossRunReportsExactlyZero() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);

        for (int seq = 0; seq < 10; seq++) {
            health.recordFrame(frame(seq));
        }

        LinkHealth.Health result = health.of(PEER);
        assertEquals(10, result.received());
        assertEquals(0, result.lost());
        assertEquals(0.0, result.dropRate());
        assertTrue(result.connected());
    }

    @Test
    void gapsAreCountedAsLostFrames() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);

        // seq stream: 10, 11, 14 (2 lost), 15, 20 (4 lost) -- 5 received, 6 lost, expected 11 total
        int[] seqs = {10, 11, 14, 15, 20};
        for (int seq : seqs) {
            health.recordFrame(frame(seq));
        }

        LinkHealth.Health result = health.of(PEER);
        assertEquals(5, result.received());
        assertEquals(6, result.lost());
        assertEquals(6.0 / 11.0, result.dropRate(), 1e-9);
    }

    @Test
    void dropRateAccountingHandlesTheEightBitSeqWrap() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);

        // 253, 254, 255, 0 (wraps, gap 1, no loss), 3 (gap 3, 2 lost)
        int[] seqs = {253, 254, 255, 0, 3};
        for (int seq : seqs) {
            health.recordFrame(frame(seq));
        }

        LinkHealth.Health result = health.of(PEER);
        assertEquals(5, result.received());
        assertEquals(2, result.lost());
        assertEquals(2.0 / 7.0, result.dropRate(), 1e-9);
    }

    @Test
    void aRepeatedSeqIsCountedAsReceivedButAddsNoLoss() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);

        health.recordFrame(frame(5));
        health.recordFrame(frame(5)); // duplicate -- gap 0

        LinkHealth.Health result = health.of(PEER);
        assertEquals(2, result.received());
        assertEquals(0, result.lost());
    }

    @Test
    void anUnheardPeerReportsDisconnectedWithZeroedCounters() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);

        LinkHealth.Health result = health.of(new PeerId(new SysId(200), new CompId(1)));

        assertFalse(result.connected());
        assertEquals(0, result.received());
        assertEquals(0, result.lost());
        assertEquals(0.0, result.dropRate());
    }

    @Test
    void movingToADifferentLinkStartsAFreshAccountingWindow() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);
        health.recordFrame(frame(LINK, 100));
        health.recordFrame(frame(LINK, 250)); // huge gap on the ORIGINAL link -- would be heavy loss

        LinkId otherLink = new LinkId("other-link");
        health.recordFrame(frame(otherLink, 0)); // fresh window: first sighting on this link

        LinkHealth.Health result = health.of(PEER);
        // the fresh window's first frame contributes 1 received, 0 lost, regardless of the old
        // link's running gap -- proving the two links' counters do not bleed into each other.
        assertEquals(1, result.received());
        assertEquals(0, result.lost());
    }

    @Test
    void aSenderRestartIsOneResyncAndAddsNoLoss() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);

        health.recordFrame(frame(249));
        health.recordFrame(frame(250));
        health.recordFrame(frame(100)); // wrapped gap 106 > resyncGap 64: restart, not 105 losses

        LinkHealth.Health result = health.of(PEER);
        assertEquals(3, result.received());
        assertEquals(0, result.lost());
        assertEquals(1, health.resyncs(PEER));
    }

    @Test
    void aSteadyStreamWithOneGapInTwentyReportsFiveLostPerHundredReceived() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);

        int seq = 0;
        for (int i = 0; i < 100; i++) {
            seq = (seq + 1) % 256;
            if (i % 20 == 19) {
                seq = (seq + 1) % 256; // one frame never arrived
            }
            health.recordFrame(frame(seq));
        }

        LinkHealth.Health result = health.of(PEER);
        assertEquals(100, result.received());
        assertEquals(5, result.lost());
        assertEquals(5.0 / 105.0, result.dropRate(), 1e-9);
    }

    @Test
    void lossOlderThanTheWindowNoLongerCounts() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);
        health.recordFrame(frame(10));
        health.recordFrame(frame(20)); // 9 lost, now

        clock.advance(Duration.ofSeconds(31)); // window is 30 s
        health.recordFrame(frame(21));

        LinkHealth.Health result = health.of(PEER);
        assertEquals(1, result.received());
        assertEquals(0, result.lost());
        assertEquals(0.0, result.dropRate());
    }

    @Test
    void lossInsideTheWindowStillCounts() {
        DefaultLinkHealth health = new DefaultLinkHealth(Duration.ofSeconds(5), CONFIG, clock);
        health.recordFrame(frame(10));
        health.recordFrame(frame(20));

        clock.advance(Duration.ofSeconds(20));

        assertEquals(9, health.of(PEER).lost());
    }

    private static final class MutableClock extends Clock {
        private Instant now;

        MutableClock(Instant start) {
            this.now = start;
        }

        void advance(Duration d) {
            now = now.plus(d);
        }

        @Override public java.time.ZoneId getZone() { return ZoneOffset.UTC; }
        @Override public Clock withZone(java.time.ZoneId zone) { return this; }
        @Override public Instant instant() { return now; }
    }

    private static Frame frame(int seq) {
        return frame(LINK, seq);
    }

    private static Frame frame(LinkId link, int seq) {
        FrameHeader header = new FrameHeader(2, seq, PEER.system(), PEER.component(), 0, 0, 0, false, SignatureStatus.UNSIGNED);
        Heartbeat heartbeat = Heartbeat.builder()
                .type(MavType.MAV_TYPE_GENERIC)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(0)
                .systemStatus(MavState.MAV_STATE_STANDBY)
                .mavlinkVersion(3)
                .build();
        return new Frame(header, heartbeat, link, LinkPeer.NONE, Instant.now());
    }
}
