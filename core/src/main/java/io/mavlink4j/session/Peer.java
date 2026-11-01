/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.PeerId;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import java.time.Instant;
import java.util.Objects;
import java.util.Optional;

/**
 * One MAVLink peer this session has observed traffic from — protocol facts only, latest-wins.
 *
 * <p>{@code id} is the peer's identity ({@code sysid}/{@code compid}), never {@code address}: a
 * link can multiplex several components, and NAT can move the observed address between packets
 * from the same system. {@code link}/{@code address} record <b>where</b> to reach this peer (needed
 * for reply routing, see {@link RoutingFrameSink}), not who it is.
 *
 * <p>{@code heartbeat} is empty until the first {@code HEARTBEAT} from this peer has
 * arrived — a peer can be known (something with its sysid/compid has been heard) before anything
 * is known about its firmware or mode.
 *
 * <p><b>Thread-safety:</b> Immutable snapshot; thread-safe.
 */
public record Peer(PeerId id, LinkId link, LinkPeer address, Instant firstHeard, Instant lastHeard,
                    Optional<HeartbeatInfo> heartbeat) {

    public Peer {
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(link, "link");
        Objects.requireNonNull(address, "address");
        Objects.requireNonNull(firstHeard, "firstHeard");
        Objects.requireNonNull(lastHeard, "lastHeard");
        Objects.requireNonNull(heartbeat, "heartbeat");
    }
}
