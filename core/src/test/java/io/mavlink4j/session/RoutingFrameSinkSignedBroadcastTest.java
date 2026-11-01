/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.CompId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameReader;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.codec.FrameHeader;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.codec.SigningClocks;
import io.mavlink4j.codec.SigningKeyResolver;
import io.mavlink4j.codec.SigningPolicy;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.MavlinkLink;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@link RoutingFrameSink#broadcast}'s own signing policy:
 * signed when the link carries exactly one distinct keyed system, plain otherwise -- see that
 * method's own javadoc for the ArduPilot GCS-failsafe rationale. Exercised through a fully in-memory
 * {@link MavlinkLink} double (no sockets): {@link FrameWriter} writes real signed/unsigned MAVLink 2
 * bytes and a plain {@link FrameReader} (with no signing configured -- {@link Signing#off()} would
 * also do, but a keyed one proves the wire {@code signed} bit regardless of verification outcome)
 * decodes them back to check {@link FrameHeader#signed()}.
 */
class RoutingFrameSinkSignedBroadcastTest {

    private static final LinkId LINK = new LinkId("broadcast-test-link");
    private static final LinkPeer PEER_ONE_ADDRESS = new LinkPeer("127.0.0.1", 14_550);
    private static final LinkPeer PEER_TWO_ADDRESS = new LinkPeer("127.0.0.1", 14_551);
    private static final SysId KEYED_ONE = new SysId(31);
    private static final SysId KEYED_TWO = new SysId(32);
    private static final byte[] KEY_ONE = key((byte) 1);
    private static final byte[] KEY_TWO = key((byte) 2);

    @Test
    void broadcastIsSignedWhenExactlyOneKeyedSystemSharesTheLink() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults());
        peers.recordFrame(heartbeatFrame(KEYED_ONE, PEER_ONE_ADDRESS));

        assertTrue(sentSigned(peers, keyed(KEYED_ONE, KEY_ONE)),
                "the one keyed vehicle sharing this link must receive a signed broadcast");
    }

    @Test
    void broadcastIsUnsignedWhenTwoDistinctKeyedSystemsShareTheLink() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults());
        peers.recordFrame(heartbeatFrame(KEYED_ONE, PEER_ONE_ADDRESS));
        peers.recordFrame(heartbeatFrame(KEYED_TWO, PEER_TWO_ADDRESS));

        SigningKeyResolver keys = sysid -> {
            if (sysid == KEYED_ONE.value()) {
                return Optional.of(KEY_ONE);
            }
            if (sysid == KEYED_TWO.value()) {
                return Optional.of(KEY_TWO);
            }
            return Optional.empty();
        };

        assertFalse(sentSigned(peers, keys),
                "a MAVLink 2 frame can only carry one signature -- two distinct keyed systems on one "
                        + "link must fall back to an unsigned broadcast rather than guess one of them");
    }

    @Test
    void broadcastIsUnsignedWhenNoPeerHasBeenHeardYet() {
        DefaultPeerDirectory peers = new DefaultPeerDirectory(MavlinkSettings.defaults());

        assertFalse(sentSigned(peers, SigningKeyResolver.NONE),
                "an empty peer set has no single addressee to sign for");
    }

    /** Wires a {@link RoutingFrameSink} over an in-memory link, broadcasts once, and decodes the result's wire {@code signed} bit. */
    private static boolean sentSigned(PeerDirectory peers, SigningKeyResolver keys) {
        RecordingSessionLink link = new RecordingSessionLink(LINK);
        Signing signing = new Signing(keys, SigningPolicy.PERMISSIVE,
                SigningClocks.inMemory(InstantSource.system(), Duration.ofMinutes(1)));
        FrameWriter writer = new FrameWriter(new SysId(255), new CompId(1), signing);
        writer.addLink(link);
        RoutingFrameSink sink = new RoutingFrameSink(peers, writer, keys, () -> java.util.Set.of(LINK));

        sink.broadcast(heartbeat(), LINK);

        List<Frame> decoded = new ArrayList<>();
        FrameReader reader = new FrameReader(link);
        reader.offer(new ByteChunk(link.onlySent(), link.onlySent().length, PEER_ONE_ADDRESS), decoded::add);
        assertEquals(1, decoded.size());
        return decoded.get(0).header().signed();
    }

    private static SigningKeyResolver keyed(SysId sysid, byte[] key) {
        return candidate -> candidate == sysid.value() ? Optional.of(key) : Optional.empty();
    }

    private static Frame heartbeatFrame(SysId sysid, LinkPeer address) {
        FrameHeader header = new FrameHeader(2, 0, sysid, new CompId(1), 0, 0, 0, false, SignatureStatus.UNSIGNED);
        return new Frame(header, "heartbeat-placeholder", LINK, address, Instant.now());
    }

    private static Heartbeat heartbeat() {
        return Heartbeat.builder()
                .type(MavType.MAV_TYPE_GCS)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_INVALID)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(0)
                .systemStatus(MavState.MAV_STATE_ACTIVE)
                .mavlinkVersion(3)
                .build();
    }

    private static byte[] key(byte fill) {
        byte[] key = new byte[32];
        Arrays.fill(key, fill);
        return key;
    }

    /** An in-memory {@link MavlinkLink} test double -- avoids a real socket's "no target yet" edge case entirely. */
    private static final class RecordingSessionLink implements MavlinkLink {
        private final LinkId id;
        private final List<byte[]> sent = new ArrayList<>();

        RecordingSessionLink(LinkId id) {
            this.id = id;
        }

        byte[] onlySent() {
            assertEquals(1, sent.size(), "expected exactly one physical send");
            return sent.get(0);
        }

        @Override
        public LinkId id() {
            return id;
        }

        @Override
        public boolean preservesMessageBoundaries() {
            return true;
        }

        @Override
        public ByteChunk poll(Duration timeout) {
            throw new UnsupportedOperationException("RecordingSessionLink is send-only in this test");
        }

        @Override
        public void send(byte[] frame, int off, int len, LinkPeer target) {
            sent.add(Arrays.copyOfRange(frame, off, off + len));
        }

        @Override
        public LinkPeer defaultTarget() {
            return PEER_ONE_ADDRESS;
        }

        @Override
        public void close() {
            // no real resource to release
        }
    }
}
