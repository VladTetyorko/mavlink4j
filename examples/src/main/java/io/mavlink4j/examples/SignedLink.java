/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import io.mavlink4j.CompId;
import io.mavlink4j.MavlinkNode;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.SignatureRejection;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.codec.SigningClocks;
import io.mavlink4j.codec.SigningPolicy;
import io.mavlink4j.testkit.FakeVehicle;
import io.mavlink4j.testkit.RecordingLink;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import java.io.PrintStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.time.Duration;
import java.time.InstantSource;
import java.util.HexFormat;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A receiving session with a signing-key resolver and a policy that requires a valid signature
 * ({@link #REQUIRED}); prints each {@link SignatureRejection} (first per system and reason).
 *
 * <p>Arguments: {@code [--fake] [--bind host:port] [--key 64-hex-chars] [--timeout seconds]}. Without
 * {@code --fake} it listens for {@code --timeout} seconds with the given 32-byte key and exits 0 (it only
 * reports). Under {@code --fake} it provokes both rejections and exits 0 only if both show: an unsigned
 * heartbeat from a fake vehicle ({@code POLICY}) and a signed heartbeat sent twice byte-for-byte
 * ({@code REPLAY} on the second, the first is accepted).
 */
public final class SignedLink {

    /** Accepts only frames whose signature verified; unsigned and unverifiable frames are rejected. */
    static final SigningPolicy REQUIRED = (sysid, status) -> status == SignatureStatus.VALID;

    private static final int KEY_BYTES = 32;
    private static final Duration SIGNING_PERSIST_INTERVAL = Duration.ofMinutes(1);
    private static final Duration REPLAY_GAP = Duration.ofMillis(50);
    private static final int ATTACKER_SYSID = 33;
    private static final Duration POLL = Duration.ofMillis(20);

    private SignedLink() {
    }

    public static void main(String[] args) {
        System.exit(run(args, System.out));
    }

    static int run(String[] args, PrintStream out) {
        return Runner.execute(out, () -> {
            Options options = Options.parse(args, "key");
            byte[] key = options.fake() ? new byte[KEY_BYTES] : parseKey(options.get("key", ""));
            if (options.fake()) {
                java.util.Arrays.fill(key, (byte) 0x2A);
            }
            Signing signing = new Signing(sysid -> Optional.of(key), REQUIRED,
                    SigningClocks.inMemory(InstantSource.system(), SIGNING_PERSIST_INTERVAL));
            try (Endpoint endpoint = Endpoint.open(options, signing)) {
                Map<String, SignatureRejection> seen = new ConcurrentHashMap<>();
                List<SignatureRejection> all = new CopyOnWriteArrayList<>();
                endpoint.session().onFrameRejected(rejection -> {
                    all.add(rejection);
                    String id = rejection.system().value() + "/" + rejection.reason();
                    if (seen.putIfAbsent(id, rejection) == null) {
                        out.println("rejected sysid=" + rejection.system().value()
                                + " status=" + rejection.status() + " reason=" + rejection.reason());
                    }
                });
                if (!options.fake()) {
                    Thread.sleep(options.timeout().toMillis());
                    out.println("listened " + options.timeout().toSeconds() + " s, " + all.size() + " frame(s) rejected");
                    return 0;
                }
                return provoke(endpoint, key, options.timeout(), out, all);
            }
        });
    }

    private static int provoke(Endpoint endpoint, byte[] key, Duration timeout, PrintStream out,
                               List<SignatureRejection> all) throws Exception {
        try (FakeVehicle unsigned = FakeVehicle.builder("127.0.0.1", endpoint.port()).sysid(32).build()) {
            boolean policy = await(all, SignatureRejection.Reason.POLICY, timeout);
            PeerId attacker = new PeerId(new SysId(ATTACKER_SYSID), new CompId(1));
            byte[] frame = signedHeartbeat(key, attacker);
            try (DatagramSocket socket = new DatagramSocket()) {
                send(socket, frame, endpoint.port());
                Thread.sleep(REPLAY_GAP.toMillis());
                send(socket, frame, endpoint.port());
            }
            boolean replay = await(all, SignatureRejection.Reason.REPLAY, timeout);
            boolean first = endpoint.session().peers().peer(attacker).isPresent();
            out.println("unsigned rejected: " + policy + ", signed original accepted: " + first
                    + ", replay rejected: " + replay);
            return policy && replay && first ? 0 : 1;
        }
    }

    private static byte[] signedHeartbeat(byte[] key, PeerId attacker) {
        Signing sender = new Signing(sysid -> Optional.of(key), SigningPolicy.PERMISSIVE,
                SigningClocks.inMemory(InstantSource.system(), SIGNING_PERSIST_INTERVAL));
        RecordingLink wire = new RecordingLink("capture", true, new LinkPeer("127.0.0.1", 1));
        FrameWriter writer = new FrameWriter(attacker.system(), attacker.component(), sender);
        writer.addLink(wire);
        MavlinkNode gcs = MavlinkNode.groundStation();
        Heartbeat heartbeat = Heartbeat.builder().type(MavType.MAV_TYPE_QUADROTOR)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA)
                .systemStatus(MavState.MAV_STATE_ACTIVE).mavlinkVersion(3).build();
        writer.sendTo(heartbeat, wire.id(), wire.defaultTarget(), new PeerId(gcs.system(), gcs.component()));
        return wire.sent().get(0);
    }

    private static void send(DatagramSocket socket, byte[] frame, int port) throws java.io.IOException {
        socket.send(new DatagramPacket(frame, frame.length, InetAddress.getLoopbackAddress(), port));
    }

    private static boolean await(List<SignatureRejection> all, SignatureRejection.Reason reason, Duration timeout)
            throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (System.nanoTime() < deadline) {
            if (all.stream().anyMatch(r -> r.reason() == reason)) {
                return true;
            }
            Thread.sleep(POLL.toMillis());
        }
        return false;
    }

    private static byte[] parseKey(String hex) {
        if (hex.length() != KEY_BYTES * 2) {
            throw new IllegalArgumentException("--key expects " + KEY_BYTES * 2 + " hex characters (a 32-byte key)");
        }
        return HexFormat.of().parseHex(hex);
    }
}
