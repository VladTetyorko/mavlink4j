/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.service;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameSink;
import io.mavlink4j.session.Peer;
import io.mavlink4j.session.PeerDirectory;
import io.mavlink4j.session.TxScheduler;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.minimal.Heartbeat;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@link HeartbeatService} is driven tick by tick: a hand-cranked {@link TxScheduler} and a manual
 * {@link InstantSource} replace sleeps, so every assertion is about a named tick, never a wall-clock window.
 */
class HeartbeatServiceTest {

    private static final Duration PERIOD = Duration.ofMillis(20);
    private static final Duration PEER_TIMEOUT = Duration.ofMillis(80);

    private final ManualClock clock = new ManualClock();
    private final ManualScheduler scheduler = new ManualScheduler();

    @Test
    void registersItsTickAtTheConfiguredPeriodAndStopCancelsIt() {
        HeartbeatService service = service(new CountingFrameSink(), new FakePeerDirectory());

        service.start();
        service.start(); // idempotent
        assertEquals(PERIOD, scheduler.period.get());
        assertEquals(1, scheduler.registrations.get());

        service.stop();
        assertTrue(scheduler.cancelled.get());
    }

    @Test
    void eachTickBroadcastsOncePerLinkWithAHeardPeer() {
        CountingFrameSink sink = new CountingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(peerAt(id(1), "link-a", clock.instant()));
        peers.refresh(id(2), "link-a", clock.instant());
        peers.refresh(id(3), "link-b", clock.instant());
        service(sink, peers).start();

        scheduler.tick();
        scheduler.tick();

        assertEquals(4, sink.broadcastCount.get(), "two links x two ticks");
    }

    @Test
    void aPeerFlipsToDisconnectedOncePeerTimeoutElapsesOnTheInjectedClock() {
        PeerId id = id(5);
        FakePeerDirectory peers = FakePeerDirectory.knowing(peerAt(id, "link-a", clock.instant()));
        HeartbeatService service = service(new CountingFrameSink(), peers);
        service.start();

        scheduler.tick();
        assertTrue(service.isConnected(id));

        clock.advance(PEER_TIMEOUT);
        scheduler.tick();
        assertTrue(service.isConnected(id), "exactly at the timeout is still connected");

        clock.advance(Duration.ofMillis(1));
        scheduler.tick();
        assertFalse(service.isConnected(id));
    }

    @Test
    void aPeerReconnectingIsReflectedOnceItIsHeardFromAgain() {
        PeerId id = id(6);
        FakePeerDirectory peers = FakePeerDirectory.knowing(peerAt(id, "link-a", clock.instant()));
        HeartbeatService service = service(new CountingFrameSink(), peers);
        service.start();
        clock.advance(PEER_TIMEOUT.plusMillis(1));
        scheduler.tick();
        assertFalse(service.isConnected(id));

        peers.refresh(id, "link-a", clock.instant());
        scheduler.tick();

        assertTrue(service.isConnected(id));
    }

    @Test
    void isConnectedIsFalseForAnUnknownPeer() {
        assertFalse(service(new CountingFrameSink(), new FakePeerDirectory()).isConnected(id(99)));
    }

    private HeartbeatService service(FrameSink sink, PeerDirectory peers) {
        return new HeartbeatService(sink, scheduler, peers,
                MavlinkSettings.defaults().toBuilder()
                        .heartbeatPeriod(PERIOD).peerTimeout(PEER_TIMEOUT).clock(clock).build(),
                HeartbeatContent.groundStation());
    }

    private static PeerId id(int sysid) {
        return new PeerId(new SysId(sysid), new CompId(1));
    }

    private static Peer peerAt(PeerId id, String linkValue, Instant lastHeard) {
        return new Peer(id, new LinkId(linkValue), LinkPeer.NONE, lastHeard, lastHeard, Optional.empty());
    }

    private static final class ManualClock implements InstantSource {
        private volatile Instant now = Instant.parse("2026-01-01T00:00:00Z");

        void advance(Duration by) {
            now = now.plus(by);
        }

        @Override
        public Instant instant() {
            return now;
        }
    }

    /** Captures the one repeated task so the test fires it by hand. */
    private static final class ManualScheduler implements TxScheduler {
        final AtomicReference<Duration> period = new AtomicReference<>();
        final AtomicInteger registrations = new AtomicInteger();
        final java.util.concurrent.atomic.AtomicBoolean cancelled = new java.util.concurrent.atomic.AtomicBoolean();
        private volatile Runnable task;

        void tick() {
            task.run();
        }

        @Override
        public Handle repeat(String name, Duration period, Runnable task) {
            this.period.set(period);
            this.task = task;
            registrations.incrementAndGet();
            return () -> cancelled.set(true);
        }

        @Override
        public void submit(String name, Runnable task) {
            task.run();
        }
    }

    private static final class CountingFrameSink implements FrameSink {
        final AtomicInteger broadcastCount = new AtomicInteger();

        @Override
        public void send(Object message, PeerId target) {
            throw new UnsupportedOperationException("HeartbeatService only ever broadcasts");
        }

        @Override
        public void broadcast(Object message, LinkId link) {
            if (message instanceof Heartbeat) {
                broadcastCount.incrementAndGet();
            }
        }

        @Override
        public Set<LinkId> links() {
            return Set.of();
        }
    }

    /** A {@link PeerDirectory} the test can mutate freely, unlike the real (append-only) one. */
    private static final class FakePeerDirectory implements PeerDirectory {
        private final Map<PeerId, Peer> peers = new ConcurrentHashMap<>();

        static FakePeerDirectory knowing(Peer peer) {
            FakePeerDirectory directory = new FakePeerDirectory();
            directory.peers.put(peer.id(), peer);
            return directory;
        }

        void refresh(PeerId id, String linkValue, Instant heardAt) {
            peers.put(id, peerAt(id, linkValue, heardAt));
        }

        @Override
        public java.util.Collection<Peer> peers() {
            return List.copyOf(peers.values());
        }

        @Override
        public Optional<Peer> peer(PeerId id) {
            return Optional.ofNullable(peers.get(id));
        }

        @Override
        public List<Peer> peersOnLink(LinkId link) {
            return peers.values().stream().filter(p -> p.link().equals(link)).toList();
        }

        @Override
        public Set<LinkPeer> concurrentAddresses(PeerId id, Duration window) {
            return Set.of();
        }
    }
}
