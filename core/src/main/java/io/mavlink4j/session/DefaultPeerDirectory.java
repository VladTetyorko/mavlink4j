/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.PeerId;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.minimal.Heartbeat;

import java.lang.System.Logger.Level;
import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The one {@link PeerDirectory} implementation. Thread-safe: {@link MavlinkSession} calls
 * {@link #recordFrame} from whichever link's own reader thread decoded a frame, so several threads
 * (one per registered link) may call it concurrently — backed by a {@link ConcurrentHashMap} keyed
 * by {@link PeerId}, updated atomically per peer via {@link Map#compute}.
 *
 * <p>{@link #recordFrame} is package-private on purpose: it is not part of the {@link PeerDirectory}
 * seam (read-only facts for L4 services), only {@link MavlinkSession} — the sole mutator — calls it.
 */
final class DefaultPeerDirectory implements PeerDirectory {

    private static final System.Logger LOG = System.getLogger(DefaultPeerDirectory.class.getName());

    /**
     * Per-peer-and-link cap on remembered addresses, oldest evicted first. A NAT that rebinds the source port
     * on every datagram would otherwise grow this map without bound; far more than the handful of
     * vehicles that can honestly share one sysid.
     */
    private static final int MAX_ADDRESSES_PER_PEER = 8;

    private final InstantSource clock;
    /**
     * How recently the previous address must have been heard for a move to count as a <b>flap</b>
     * rather than an ordinary re-registration (a lease renewal or reconnect is normal; transmitting
     * from two addresses at once is not).
     */
    private final Duration flapWindow;
    /** Per-peer warning interval. A genuine roam must not be able to flood the log. */
    private final Duration warnInterval;
    private final Map<PeerId, Peer> peers = new ConcurrentHashMap<>();
    /** Address &rarr; last heard, per peer and link: what {@link #concurrentAddresses} and the flap warning read. */
    private final Map<PeerId, Map<LinkId, Map<LinkPeer, Instant>>> addresses = new ConcurrentHashMap<>();
    private final Map<PeerId, Instant> lastFlapWarning = new ConcurrentHashMap<>();

    DefaultPeerDirectory(MavlinkSettings settings) {
        Objects.requireNonNull(settings, "settings");
        this.clock = settings.clock();
        this.flapWindow = settings.peer().flapWindow();
        this.warnInterval = settings.peer().warnInterval();
    }

    @Override
    public Collection<Peer> peers() {
        return List.copyOf(peers.values());
    }

    @Override
    public Optional<Peer> peer(PeerId id) {
        Objects.requireNonNull(id, "id");
        return Optional.ofNullable(peers.get(id));
    }

    @Override
    public List<Peer> peersOnLink(LinkId link) {
        Objects.requireNonNull(link, "link");
        List<Peer> onLink = new ArrayList<>();
        for (Peer peer : peers.values()) {
            if (peer.link().equals(link)) {
                onLink.add(peer);
            }
        }
        return List.copyOf(onLink);
    }

    @Override
    public Set<LinkPeer> concurrentAddresses(PeerId id, Duration window) {
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(window, "window");
        Map<LinkId, Map<LinkPeer, Instant>> byLink = addresses.get(id);
        if (byLink == null) {
            return Set.of();
        }
        Instant cutoff = clock.instant().minus(window);
        Set<LinkPeer> concurrent = new LinkedHashSet<>();
        for (Map<LinkPeer, Instant> heard : byLink.values()) {
            synchronized (heard) {
                Set<LinkPeer> onThisLink = new LinkedHashSet<>();
                heard.forEach((address, at) -> {
                    if (!at.isBefore(cutoff)) {
                        onThisLink.add(address);
                    }
                });
                if (onThisLink.size() >= 2) {
                    concurrent.addAll(onThisLink);
                }
            }
        }
        return Set.copyOf(concurrent);
    }

    /**
     * Records {@code frame} as protocol evidence of its origin peer: refreshes {@code lastHeard}/
     * {@code link}/{@code address}, preserves {@code firstHeard} and the last known
     * {@link HeartbeatInfo} across updates, and replaces the heartbeat info when {@code frame}
     * itself is a {@code HEARTBEAT}. Called by {@link MavlinkSession} for every decoded frame,
     * before {@link Correlator} and {@link Dispatcher} see it.
     */
    void recordFrame(Frame frame) {
        PeerId id = new PeerId(frame.header().system(), frame.header().component());
        peers.compute(id, (key, existing) -> {
            Instant firstHeard = existing == null ? frame.receivedAt() : existing.firstHeard();
            Optional<HeartbeatInfo> heartbeat = frame.is(Heartbeat.class)
                    ? Optional.of(heartbeatInfoOf(frame.as(Heartbeat.class)))
                    : (existing == null ? Optional.empty() : existing.heartbeat());
            return new Peer(id, frame.link(), frame.source(), firstHeard, frame.receivedAt(), heartbeat);
        });
        noteAddress(id, frame);
        // Logged outside compute(): the remapping function must stay side-effect free.
        warnOnAddressFlap(id, frame);
    }

    private void noteAddress(PeerId id, Frame frame) {
        Map<LinkPeer, Instant> heard = addresses.computeIfAbsent(id, k -> new ConcurrentHashMap<>())
                .computeIfAbsent(frame.link(), k -> new LinkedHashMap<>());
        synchronized (heard) {
            heard.remove(frame.source());   // remove-then-put keeps insertion order == recency order
            heard.put(frame.source(), frame.receivedAt());
            while (heard.size() > MAX_ADDRESSES_PER_PEER) {
                heard.remove(heard.keySet().iterator().next());
            }
        }
    }

    /**
     * Makes a peer address that keeps moving visible, because everything downstream silently
     * follows it.
     *
     * <p>Every transmit to this peer goes to whatever address was recorded last, so two systems
     * sharing one sysid do not collide loudly — they take turns. A 33&nbsp;Hz RC override stream
     * split between them reaches each in bursts with gaps long enough to trip the vehicle's own
     * command-loss failsafe, and the only symptom is a vehicle that keeps disarming for no visible
     * reason. Overwriting the address is still the right behaviour (a lease renewal or a NAT
     * rebinding genuinely moves a peer, as {@link Peer} itself notes) — doing it in silence is not.
     */
    private void warnOnAddressFlap(PeerId id, Frame frame) {
        Set<LinkPeer> concurrent = concurrentAddresses(id, flapWindow);
        if (concurrent.size() < 2) {
            return;   // it had gone quiet; coming back elsewhere is a reconnect, not a conflict
        }
        Instant last = lastFlapWarning.get(id);
        if (last != null && Duration.between(last, frame.receivedAt()).compareTo(warnInterval) < 0) {
            return;
        }
        lastFlapWarning.put(id, frame.receivedAt());
        LOG.log(Level.WARNING,
                "MAVLink sysid {0}/comp {1} is transmitting from two addresses at once: {2}."
                        + " Everything sent to this peer follows the most recent one, so a command"
                        + " stream is being split between them. Two vehicles sharing one sysid is the"
                        + " usual cause; a NAT or DHCP rebinding is the benign one.",
                id.system().value(), id.component().value(), concurrent);
    }

    private static HeartbeatInfo heartbeatInfoOf(Heartbeat heartbeat) {
        return new HeartbeatInfo(
                heartbeat.autopilot().value(),
                heartbeat.type().value(),
                heartbeat.baseMode().value(),
                heartbeat.customMode(),
                heartbeat.systemStatus().value());
    }
}
