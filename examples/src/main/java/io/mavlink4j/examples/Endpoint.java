/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import io.mavlink4j.MavlinkNode;
import io.mavlink4j.PeerId;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.session.MavlinkSession;
import io.mavlink4j.session.Peer;
import io.mavlink4j.transport.UdpListenLink;

import java.io.IOException;
import java.time.Duration;
import java.util.Optional;

/** A ground-station session on one {@link UdpListenLink}; under {@code --fake} it binds an ephemeral loopback port. */
final class Endpoint implements AutoCloseable {

    private static final Duration FAKE_REPLY_TIMEOUT = Duration.ofMillis(500);
    private static final Duration PEER_POLL = Duration.ofMillis(20);

    private final UdpListenLink link;
    private final MavlinkSession session;
    private final MavlinkSettings settings;

    private Endpoint(UdpListenLink link, MavlinkSession session, MavlinkSettings settings) {
        this.link = link;
        this.session = session;
        this.settings = settings;
    }

    static Endpoint open(Options options, Signing signing) throws IOException {
        MavlinkSettings settings = options.fake() ? fastSettings() : MavlinkSettings.defaults();
        String host = options.fake() ? "127.0.0.1" : options.bindHost();
        int port = options.fake() ? 0 : options.bindPort();
        UdpListenLink link = new UdpListenLink(host, port);
        MavlinkSession session = new MavlinkSession(MavlinkNode.groundStation(), settings, signing);
        session.addLink(link);
        return new Endpoint(link, session, settings);
    }

    /** Short timeouts so an unanswered request under {@code --fake} ends the run quickly. */
    private static MavlinkSettings fastSettings() {
        return MavlinkSettings.defaults().toBuilder()
                .commandTimeout(FAKE_REPLY_TIMEOUT).commandRetries(1)
                .parameter(new MavlinkSettings.Parameter(FAKE_REPLY_TIMEOUT, 1, 8)).build();
    }

    MavlinkSession session() {
        return session;
    }

    MavlinkSettings settings() {
        return settings;
    }

    /** @return the locally bound UDP port (the real one when {@code --bind} asked for port 0) */
    int port() {
        String id = link.id().value();
        return Integer.parseInt(id.substring(id.lastIndexOf(':') + 1));
    }

    /**
     * Waits for a heartbeat-announced peer.
     *
     * @param sysid the system id to wait for, or a negative value for the first peer heard
     * @param timeout how long to wait
     * @return the peer's id
     * @throws IllegalStateException if none is heard in time
     */
    PeerId awaitPeer(int sysid, Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (System.nanoTime() < deadline) {
            Optional<Peer> found = session.peers().peers().stream()
                    .filter(p -> sysid < 0 || p.id().system().value() == sysid).findFirst();
            if (found.isPresent()) {
                return found.get().id();
            }
            Thread.sleep(PEER_POLL.toMillis());
        }
        throw new IllegalStateException("no vehicle heard within " + timeout.toSeconds() + " s"
                + (sysid < 0 ? "" : " (sysid " + sysid + ")") + " on port " + port());
    }

    @Override
    public void close() {
        session.close();
        link.close();
    }
}
