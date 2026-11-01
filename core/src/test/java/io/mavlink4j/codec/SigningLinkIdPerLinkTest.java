/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.Mavlink2Message;
import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.time.Duration;
import java.time.InstantSource;
import java.util.Arrays;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

/**
 * Proves each registered {@link io.mavlink4j.transport.MavlinkLink} gets its own wire-level MAVLink 2 signing {@code
 * link_id} ("one physical link, one signing identity, satisfies the
 * spec's monotonicity scope" -- mirrors pymavlink's own verified auto-increment-per-link
 * convention). This is deliberately a different axis from {@link
 * io.mavlink4j.transport.LinkId} (this codebase's own carrier identity) -- the two merely share
 * the word "link".
 */
class SigningLinkIdPerLinkTest {

    private static final SysId VEHICLE = new SysId(9);
    private static final CompId VEHICLE_COMPONENT = new CompId(1);
    private static final PeerId VEHICLE_PEER = new PeerId(VEHICLE, VEHICLE_COMPONENT);
    private static final byte[] KEY = key();

    @Test
    void twoRegisteredLinksGetDifferentSigningLinkIds() throws IOException {
        SigningKeyResolver keys = sysid -> sysid == VEHICLE.value() ? Optional.of(KEY) : Optional.empty();
        Signing signing = new Signing(keys, SigningPolicy.PERMISSIVE,
                SigningClocks.inMemory(InstantSource.system(), Duration.ofMinutes(1)));
        FrameWriter writer = new FrameWriter(VEHICLE, VEHICLE_COMPONENT, signing);

        SendCapturingLink linkA = new SendCapturingLink("signing-link-a", true, LinkPeer.NONE);
        SendCapturingLink linkB = new SendCapturingLink("signing-link-b", true, LinkPeer.NONE);
        writer.addLink(linkA);
        writer.addLink(linkB);

        writer.sendTo(heartbeat(), linkA.id(), new LinkPeer("127.0.0.1", 14_550), VEHICLE_PEER);
        writer.sendTo(heartbeat(), linkB.id(), new LinkPeer("127.0.0.1", 14_550), VEHICLE_PEER);

        int signingLinkIdA = signingLinkIdOf(onlySent(linkA));
        int signingLinkIdB = signingLinkIdOf(onlySent(linkB));

        assertNotEquals(signingLinkIdA, signingLinkIdB,
                "each registered link must carry its own signing link_id, not a shared one");
    }

    private static int signingLinkIdOf(byte[] wire) throws IOException {
        MavlinkConnection receiver = MavlinkConnection.create(
                new ByteArrayInputStream(wire), OutputStream.nullOutputStream());
        MavlinkMessage<?> message = receiver.next();
        return ((Mavlink2Message<?>) message).getSignatureLinkId();
    }

    private static byte[] onlySent(SendCapturingLink link) {
        assertEquals(1, link.sent().size());
        return link.sent().get(0);
    }

    private static Heartbeat heartbeat() {
        return Heartbeat.builder()
                .type(MavType.MAV_TYPE_QUADROTOR)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(0)
                .systemStatus(MavState.MAV_STATE_STANDBY)
                .mavlinkVersion(3)
                .build();
    }

    private static byte[] key() {
        byte[] key = new byte[32];
        Arrays.fill(key, (byte) 0x42);
        return key;
    }
}
