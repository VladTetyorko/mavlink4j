/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j;

import java.util.Objects;

/**
 * A targeted send named a peer nothing has been heard from yet, so no link is known to reach it.
 *
 * <p>This is a runtime condition, not a caller bug: MAVLink's own forwarding rule only routes a
 * targeted message to a link a message from that target previously arrived on, so the same call
 * succeeds a moment later once the peer's first frame has been seen. Callers that can wait catch
 * this specifically; the services surface it as a failed future.
 *
 * <p><b>Thread-safety:</b> Unchecked; carries no mutable state.
 */
public final class PeerUnreachableException extends RuntimeException {

    private static final long serialVersionUID = 1L;

    private final transient PeerId peer;

    public PeerUnreachableException(PeerId peer) {
        super("Cannot send to peer " + Objects.requireNonNull(peer, "peer")
                + " -- it has not been heard from on any link yet. MAVLink's own forwarding rule: a message"
                + " is only ever routed to a link a message from that target previously arrived on.");
        this.peer = peer;
    }

    /** The peer that has not been heard from. */
    public PeerId peer() {
        return peer;
    }
}
