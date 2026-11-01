/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.PeerId;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import java.util.Collection;
import java.time.Duration;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * Who is out there — read-only protocol facts, updated by {@link MavlinkSession} as frames arrive.
 * Deliberately carries no ownership or access logic: which sysid a host's device owns is host
 * policy, not a protocol fact, and belongs to the calling application. This interface only
 * ever answers "what has been observed," never "who is allowed to use it."
 */
public interface PeerDirectory {

    /** Every peer currently known, in no particular order. */
    Collection<Peer> peers();

    /** The peer identified by {@code id}; empty if nothing has been heard from it yet. */
    Optional<Peer> peer(PeerId id);

    /** Peers most recently heard on {@code link}. */
    List<Peer> peersOnLink(LinkId link);

    /**
     * The source addresses {@code id} is <b>concurrently</b> transmitting from: on any one link, the
     * addresses each heard within the last {@code window}, provided there are two or more of them
     * (otherwise nothing is reported for that link). That is the signature of two vehicles sharing
     * one sysid; {@link #peer} keeps only the newest address, so this is the one place the other is
     * still visible.
     *
     * <p>Counted per link on purpose: one vehicle heard on two links (a UDP carrier and a serial
     * radio, a failover pair) has one address per link and is not a conflict. And {@code window} is
     * a <i>concurrency</i> window, not a silence window: a vehicle that reboots and comes back from
     * a new source port stops being heard at the old one, so the two never overlap for long.
     * Empty when no link has two such addresses.
     */
    Set<LinkPeer> concurrentAddresses(PeerId id, Duration window);
}
