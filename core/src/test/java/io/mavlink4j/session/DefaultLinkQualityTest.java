/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.CompId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.FrameHeader;
import io.mavlink4j.codec.SignatureRejection;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.common.RadioStatus;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pure in-memory tests of {@link DefaultLinkQuality} -- hand-built {@link Frame}s dispatched
 * directly through a real {@link DefaultDispatcher}, no sockets (including the signing counters).
 */
class DefaultLinkQualityTest {

    private static final LinkId LINK_A = new LinkId("test-link-a");
    private static final LinkId LINK_B = new LinkId("test-link-b");
    private static final SysId ORIGIN = new SysId(1);
    /** Generous window for tests that don't care about expiry timing. */
    private static final Duration DEFAULT_WINDOW = Duration.ofSeconds(5);

    @Test
    void recordsQualityFromRadioStatusOnTheTelemetryRadioComponent() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));

        dispatcher.dispatch(radioStatusFrame(LINK_A, 68, 200, 180, 40, 3, 0, 0));

        LinkQuality.Quality reading = quality.of(LINK_A).orElseThrow();
        assertEquals(LINK_A, reading.linkId());
        assertEquals(200, reading.rssi());
        assertEquals(180, reading.remoteRssi());
        assertEquals(40, reading.noise());
        assertEquals(3, reading.rxErrors());
        assertEquals(Boolean.FALSE, reading.fixed());
    }

    @Test
    void nonZeroFixedCountMapsToTrue() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));

        dispatcher.dispatch(radioStatusFrame(LINK_A, 110, 100, 100, 10, 0, 5, 0));

        assertEquals(Boolean.TRUE, quality.of(LINK_A).get().fixed());
    }

    @Test
    void ignoresRadioStatusFromAnUnrelatedComponent() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));

        // component 1 (autopilot) -- RADIO_STATUS never actually arrives from this component in
        // real MAVLink traffic, but the filter must reject it regardless of message type.
        dispatcher.dispatch(radioStatusFrame(LINK_A, 1, 200, 180, 40, 12, 0, 0));

        assertTrue(quality.of(LINK_A).isEmpty(), "a RADIO_STATUS from a non-radio component must not be recorded");
    }

    @Test
    void ignoresNonRadioStatusFrames() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));

        FrameHeader header = new FrameHeader(2, 0, ORIGIN, new CompId(68), 0, 0, 0, false, SignatureStatus.UNSIGNED);
        dispatcher.dispatch(new Frame(header, "not-a-radio-status", LINK_A, LinkPeer.NONE, Instant.now()));

        assertTrue(quality.of(LINK_A).isEmpty());
    }

    @Test
    void tracksEachLinkIndependently() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));

        dispatcher.dispatch(radioStatusFrame(LINK_A, 68, 200, 180, 40, 0, 0, 0));
        dispatcher.dispatch(radioStatusFrame(LINK_B, 111, 50, 40, 10, 0, 0, 0));

        assertEquals(200, quality.of(LINK_A).get().rssi());
        assertEquals(50, quality.of(LINK_B).get().rssi());
        assertTrue(quality.of(LINK_A).get().lastRadioStatusAt().isBefore(Instant.now().plusSeconds(1)));
    }

    @Test
    void ofIsEmptyForALinkNeverHeardFrom() {
        DefaultLinkQuality quality = new DefaultLinkQuality(new DefaultDispatcher(settings(DEFAULT_WINDOW)), settings(DEFAULT_WINDOW));
        assertTrue(quality.of(new LinkId("never-heard")).isEmpty());
    }

    @Test
    void signedRecentlyIsTrueRightAfterAValidFrameAndFalseOnceTheWindowExpires() {
        ManualClock clock = new ManualClock();
        MavlinkSettings settings = settings(Duration.ofMillis(150)).toBuilder().clock(clock).build();
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings);
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings);

        dispatcher.dispatch(signatureStatusFrame(LINK_A, ORIGIN, SignatureStatus.VALID));
        assertTrue(quality.signedRecently(LINK_A, ORIGIN), "a VALID frame just observed must read signed-recently");

        clock.advance(Duration.ofMillis(150));
        assertTrue(quality.signedRecently(LINK_A, ORIGIN), "exactly at the window edge is still recent");

        clock.advance(Duration.ofMillis(1)); // no new VALID frame arrives to refresh it
        assertFalse(quality.signedRecently(LINK_A, ORIGIN),
                "a link gone quiet for longer than the window must not keep reporting a frozen signed-recently");
    }

    @Test
    void signedRecentlyIsFalseForALinkThatHasNeverProducedAValidFrame() {
        DefaultLinkQuality quality = new DefaultLinkQuality(new DefaultDispatcher(settings(DEFAULT_WINDOW)), settings(DEFAULT_WINDOW));
        assertFalse(quality.signedRecently(new LinkId("never-signed"), ORIGIN));
    }

    @Test
    void unsignedFramesCountsOnlyFramesWithUnsignedStatus() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));

        dispatcher.dispatch(signatureStatusFrame(LINK_A, ORIGIN, SignatureStatus.UNSIGNED));
        dispatcher.dispatch(signatureStatusFrame(LINK_A, ORIGIN, SignatureStatus.UNSIGNED));
        dispatcher.dispatch(signatureStatusFrame(LINK_A, ORIGIN, SignatureStatus.VALID));

        assertEquals(2, quality.unsignedFrames(LINK_A, ORIGIN));
        assertEquals(0, quality.unsignedFrames(LINK_B, ORIGIN), "a link never dispatched to must read zero, not fail");
    }

    @Test
    void recordRejectionIncrementsTheCountAndRemembersTheLastRejection() {
        DefaultLinkQuality quality = new DefaultLinkQuality(new DefaultDispatcher(settings(DEFAULT_WINDOW)), settings(DEFAULT_WINDOW));
        SignatureRejection first = new SignatureRejection(LINK_A, ORIGIN, new CompId(1), 0,
                SignatureStatus.INVALID, SignatureRejection.Reason.BAD_SIGNATURE,
                0, LinkPeer.NONE, Instant.EPOCH);
        SignatureRejection second = new SignatureRejection(LINK_A, ORIGIN, new CompId(1), 0,
                SignatureStatus.INVALID, SignatureRejection.Reason.REPLAY,
                0, LinkPeer.NONE, Instant.EPOCH);

        quality.recordRejection(first);
        quality.recordRejection(second);

        assertEquals(2, quality.rejectedFrames(LINK_A, ORIGIN));
        assertEquals(second, quality.lastRejection(LINK_A, ORIGIN).orElseThrow());
        assertTrue(quality.lastRejection(LINK_B, ORIGIN).isEmpty(), "a link that never had a rejection must read null, not fail");
    }

    // --- per-sender scoping ------------------------------------------------------------------

    @Test
    void twoSendersOnOneLinkKeepSeparateSignedAndUnsignedCounters() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));
        SysId senderA = ORIGIN;
        SysId senderB = new SysId(2);

        dispatcher.dispatch(signatureStatusFrame(LINK_A, senderA, SignatureStatus.UNSIGNED));
        dispatcher.dispatch(signatureStatusFrame(LINK_A, senderA, SignatureStatus.UNSIGNED));
        dispatcher.dispatch(signatureStatusFrame(LINK_A, senderB, SignatureStatus.VALID));

        assertEquals(2, quality.unsignedFrames(LINK_A, senderA));
        assertEquals(0, quality.unsignedFrames(LINK_A, senderB),
                "sender B's own frame was VALID, not UNSIGNED -- it must not inherit sender A's count sharing this link");
        assertTrue(quality.signedRecently(LINK_A, senderB));
        assertFalse(quality.signedRecently(LINK_A, senderA), "sender A never produced a VALID frame on this link");
    }

    @Test
    void twoSendersOnOneLinkKeepSeparateRejectionCounters() {
        DefaultLinkQuality quality = new DefaultLinkQuality(new DefaultDispatcher(settings(DEFAULT_WINDOW)), settings(DEFAULT_WINDOW));
        SysId senderA = ORIGIN;
        SysId senderB = new SysId(2);
        SignatureRejection rejectionA = new SignatureRejection(LINK_A, senderA, new CompId(1), 0,
                SignatureStatus.INVALID, SignatureRejection.Reason.BAD_SIGNATURE,
                0, LinkPeer.NONE, Instant.EPOCH);

        quality.recordRejection(rejectionA);
        quality.recordRejection(rejectionA);

        assertEquals(2, quality.rejectedFrames(LINK_A, senderA));
        assertEquals(0, quality.rejectedFrames(LINK_A, senderB),
                "sender B shares the carrier but never itself had a rejection -- V2's exact symptom");
        assertEquals(rejectionA, quality.lastRejection(LINK_A, senderA).orElseThrow());
        assertTrue(quality.lastRejection(LINK_A, senderB).isEmpty());
    }

    @Test
    void radioQualityStaysPerLinkRegardlessOfWhichSenderIsCurrentlyTalking() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings(DEFAULT_WINDOW));
        DefaultLinkQuality quality = new DefaultLinkQuality(dispatcher, settings(DEFAULT_WINDOW));

        // Unlike the four signing reads above, of(LinkId) takes no SysId -- RADIO_STATUS is the
        // radio's own property, not a per-sender one (see LinkQuality's class javadoc).
        dispatcher.dispatch(radioStatusFrame(LINK_A, 68, 200, 180, 40, 0, 0, 0));

        assertEquals(200, quality.of(LINK_A).get().rssi());
    }

    private static Frame radioStatusFrame(LinkId link, int componentId, int rssi, int remrssi, int noise,
                                              int rxerrors, int fixed, int txbuf) {
        FrameHeader header = new FrameHeader(2, 0, ORIGIN, new CompId(componentId), 109, 0, 0, false, SignatureStatus.UNSIGNED);
        RadioStatus status = RadioStatus.builder()
                .rssi(rssi)
                .remrssi(remrssi)
                .txbuf(txbuf)
                .noise(noise)
                .remnoise(0)
                .rxerrors(rxerrors)
                .fixed(fixed)
                .build();
        return new Frame(header, status, link, LinkPeer.NONE, Instant.now());
    }

    /** A frame whose payload is irrelevant -- only {@code system}/{@code status} matter to the second subscription under test. */
    private static Frame signatureStatusFrame(LinkId link, SysId system, SignatureStatus status) {
        boolean signed = status != SignatureStatus.UNSIGNED;
        FrameHeader header = new FrameHeader(2, 0, system, new CompId(1), 0, 0, 0, signed, status);
        return new Frame(header, "payload", link, LinkPeer.NONE, Instant.now());
    }

    private static MavlinkSettings settings(Duration signedRecentlyWindow) {
        MavlinkSettings defaults = MavlinkSettings.defaults();
        MavlinkSettings.SigningSettings signing = new MavlinkSettings.SigningSettings(signedRecentlyWindow,
                defaults.signing().staleSignatureTolerance(), defaults.signing().timestampPersistInterval(),
                defaults.signing().maxStreams());
        return defaults.toBuilder().signing(signing).build();
    }

    private static final class ManualClock implements java.time.InstantSource {
        private Instant now = Instant.parse("2026-01-01T00:00:00Z");

        void advance(Duration by) {
            now = now.plus(by);
        }

        @Override
        public Instant instant() {
            return now;
        }
    }
}
