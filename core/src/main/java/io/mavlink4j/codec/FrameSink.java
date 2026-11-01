/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.PeerId;
import io.mavlink4j.transport.LinkId;

import java.util.Set;

/**
 * Frames out — the one public TX door. Sequence numbering, our own sysid/compid stamping, signing
 * and framing are all handled by the implementation; callers hand over a message and where it
 * should go. A {@code message} is a dialect message class (for example {@code CommandLong}): the
 * dialect model has no common supertype, hence {@code Object}.
 *
 * <p><b>Thread-safety:</b> Implementations must be thread-safe: services on several threads send through one sink.
 */
public interface FrameSink {

    /**
     * Sends {@code message} addressed to {@code target}, on the link it was last heard on and to the
     * address it was last heard at.
     *
     * @param message a dialect message object — see {@link FrameWriter} for the validation applied
     * @param target  who this is for
     * @throws io.mavlink4j.PeerUnreachableException if nothing has been heard from {@code
     *                                                     target} yet, so no link is known to reach it
     */
    void send(Object message, PeerId target);

    /**
     * Sends {@code message} out {@code link} unconditionally, using that link's own
     * {@link io.mavlink4j.transport.MavlinkLink#defaultTarget()} — matches the protocol's
     * own broadcast dispatch path (spec: {@code target_system == 0} is an unconditional forward,
     * no learned-route check).
     */
    void broadcast(Object message, LinkId link);

    /** Every link a broadcast can go out on right now. */
    Set<LinkId> links();

    /**
     * Broadcasts {@code message} on every link in {@link #links()} — how a heartbeat reaches a
     * dial-out link nobody has spoken on yet. Goes through {@link #broadcast}, so any gate a wrapping
     * sink applies per link applies here too. Best effort per link: a link that cannot be addressed
     * yet (a listen link that has heard nothing, so has no learned target) is logged and skipped, it
     * never stops the others.
     */
    default void broadcastAll(Object message) {
        for (LinkId link : links()) {
            try {
                broadcast(message, link);
            } catch (RuntimeException e) {
                System.getLogger(FrameSink.class.getName())
                        .log(System.Logger.Level.WARNING, "Broadcast on link " + link + " failed; skipping it", e);
            }
        }
    }
}
