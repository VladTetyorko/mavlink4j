/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.codec.FrameHeader;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@link DefaultPeerDirectory#concurrentAddresses}: the evidence a sysid is shared by two vehicles
 * -- {@code peer()} keeps only the newest address, this keeps the others.
 */
class DefaultPeerDirectoryTest {

    private static final LinkId LINK = new LinkId("udp-listen:test");
    private static final SysId SYSID = new SysId(10);
    private static final PeerId PEER = new PeerId(SYSID, new CompId(1));
    private static final LinkPeer ROVER_A = new LinkPeer("10.0.0.1", 14550);
    private static final LinkPeer ROVER_B = new LinkPeer("10.0.0.2", 14550);
    private static final Instant NOW = Instant.parse("2026-01-01T00:00:00Z");
    private static final Duration WINDOW = Duration.ofSeconds(3);

    private static Frame frameFrom(LinkPeer source, Instant at) {
        FrameHeader header = new FrameHeader(2, 0, SYSID, new CompId(1), 0, 0, 0, false, SignatureStatus.UNSIGNED);
        return new Frame(header, "heartbeat-placeholder", LINK, source, at);
    }

    private static Frame frameFrom(LinkId link, LinkPeer source, Instant at) {
        FrameHeader header = new FrameHeader(2, 0, SYSID, new CompId(1), 0, 0, 0, false, SignatureStatus.UNSIGNED);
        return new Frame(header, "heartbeat-placeholder", link, source, at);
    }

    @Test
    void twoAddressesHeardTogetherOnOneLinkAreConcurrent() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults().toBuilder().clock(java.time.InstantSource.fixed(NOW)).build());
        Instant now = NOW;
        peers.recordFrame(frameFrom(ROVER_A, now.minusSeconds(1)));
        peers.recordFrame(frameFrom(ROVER_B, now));

        assertEquals(Set.of(ROVER_A, ROVER_B), peers.concurrentAddresses(PEER, WINDOW));
    }

    @Test
    void aRebootedVehicleOnANewPortIsNotAConflict() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults().toBuilder().clock(java.time.InstantSource.fixed(NOW)).build());
        Instant now = NOW;
        peers.recordFrame(frameFrom(ROVER_A, now.minusSeconds(4)));
        peers.recordFrame(frameFrom(ROVER_B, now));

        assertTrue(peers.concurrentAddresses(PEER, WINDOW).isEmpty(),
                "the old address went quiet 4 s before the new one spoke: a reboot, not two vehicles");
    }

    @Test
    void oneSysidOnTwoLinksIsNotAConflict() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults().toBuilder().clock(java.time.InstantSource.fixed(NOW)).build());
        Instant now = NOW;
        peers.recordFrame(frameFrom(LINK, ROVER_A, now));
        peers.recordFrame(frameFrom(new LinkId("serial:ttyUSB0"), ROVER_B, now));

        assertTrue(peers.concurrentAddresses(PEER, WINDOW).isEmpty(),
                "one vehicle heard through a UDP carrier and a serial radio has an address per link");
    }

    @Test
    void twoAddressesOnTheSecondLinkAreStillAConflict() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults().toBuilder().clock(java.time.InstantSource.fixed(NOW)).build());
        Instant now = NOW;
        LinkId serial = new LinkId("serial:ttyUSB0");
        peers.recordFrame(frameFrom(LINK, ROVER_A, now));
        peers.recordFrame(frameFrom(serial, ROVER_A, now));
        peers.recordFrame(frameFrom(serial, ROVER_B, now));

        assertEquals(Set.of(ROVER_A, ROVER_B), peers.concurrentAddresses(PEER, WINDOW));
    }

    @Test
    void anUnknownPeerHasNoAddresses() {
        assertTrue(new DefaultPeerDirectory(MavlinkSettings.defaults().toBuilder().clock(java.time.InstantSource.fixed(NOW)).build()).concurrentAddresses(PEER, WINDOW).isEmpty());
    }

    @Test
    void oneVehicleRepeatingItselfIsNotAConflict() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults().toBuilder().clock(java.time.InstantSource.fixed(NOW)).build());
        Instant now = NOW;
        peers.recordFrame(frameFrom(ROVER_A, now.minusSeconds(1)));
        peers.recordFrame(frameFrom(ROVER_A, now));

        assertTrue(peers.concurrentAddresses(PEER, WINDOW).isEmpty());
    }
}
