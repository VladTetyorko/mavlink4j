/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.PeerId;
import io.mavlink4j.PeerUnreachableException;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameSink;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.SigningKeyResolver;
import io.mavlink4j.transport.LinkId;

import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;

/**
 * The {@link FrameSink} implementation: resolves a {@link PeerId}'s link <i>and</i> address through
 * {@link PeerDirectory}, then delegates to {@link FrameWriter#sendTo}, which sends via that link's
 * own persistent {@code MavlinkConnection} (correct per-link sequence numbering).
 *
 * <h2>"You cannot send to a peer you have not heard from"</h2>
 * This is not a local safety choice — it is the protocol's own forwarding rule: a router only
 * forwards a targeted message to a link a message from that target previously arrived on.
 * {@link #send} throws {@link PeerUnreachableException} when {@code target} is not (yet) in
 * {@link PeerDirectory} for exactly that reason.
 */
final class RoutingFrameSink implements FrameSink {

    private final PeerDirectory peers;
    private final FrameWriter writer;
    private final SigningKeyResolver keys;
    private final Supplier<Set<LinkId>> links;

    RoutingFrameSink(PeerDirectory peers, FrameWriter writer, SigningKeyResolver keys, Supplier<Set<LinkId>> links) {
        this.peers = Objects.requireNonNull(peers, "peers");
        this.writer = Objects.requireNonNull(writer, "writer");
        this.keys = Objects.requireNonNull(keys, "keys");
        this.links = Objects.requireNonNull(links, "links");
    }

    @Override
    public Set<LinkId> links() {
        return links.get();
    }

    @Override
    public void send(Object message, PeerId target) {
        Objects.requireNonNull(message, "message");
        Objects.requireNonNull(target, "target");
        Peer peer = peers.peer(target).orElseThrow(() -> new PeerUnreachableException(target));
        // sendTo, not broadcast: on a shared listen link `defaultTarget()` is "whoever spoke most
        // recently", which for a targeted command on a fleet gateway can be the wrong aircraft.
        // Signed when `keys` resolves a key for this peer's own system, unsigned otherwise.
        writer.sendTo(message, peer.link(), peer.address(), target);
    }

    /**
     * Signs the broadcast when {@code link} currently carries exactly one distinct keyed system,
     * sends it plain otherwise. A MAVLink 2 frame carries exactly one signature against exactly one
     * key, so a true broadcast (several different systems sharing one listen link — one
     * UDP listen link before the host narrows it to one vehicle)
     * cannot be signed correctly for more than one recipient at once: picking one system's key
     * would make the frame fail signature validation on every other system present, and ArduPilot
     * enters its own GCS-failsafe the moment a vehicle that expects signing stops seeing valid
     * signed traffic — so guessing wrong is worse than sending unsigned. The unambiguous case (one
     * keyed system, or the lobby has already narrowed to a single paired vehicle) is exactly where
     * skipping the signature would otherwise trip that same failsafe for no reason, so it is signed
     * whenever the peer set makes that safe.
     */
    @Override
    public void broadcast(Object message, LinkId link) {
        Objects.requireNonNull(message, "message");
        Objects.requireNonNull(link, "link");
        List<Peer> onLink = peers.peersOnLink(link);
        Set<SysId> keyedSystems = new HashSet<>();
        Peer keyedPeer = null;
        for (Peer peer : onLink) {
            SysId system = peer.id().system();
            if (keys.keyFor(system.value()).isPresent() && keyedSystems.add(system)) {
                keyedPeer = peer;
            }
        }
        if (keyedSystems.size() == 1) {
            writer.sendTo(message, link, keyedPeer.address(), keyedPeer.id());
            return;
        }
        writer.broadcast(message, link);
    }
}
