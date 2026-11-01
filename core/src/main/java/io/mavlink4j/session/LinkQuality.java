/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.SysId;
import io.mavlink4j.codec.SignatureRejection;
import io.mavlink4j.transport.LinkId;

import java.time.Instant;
import java.util.Optional;

/**
 * Per-link radio-quality telemetry, keyed by {@link LinkId} rather than {@link
 * io.mavlink4j.PeerId} — a deliberate, separate port from {@link LinkHealth}:
 * {@code LinkHealth.Health} already collapses to
 * "whichever link this peer was most recently heard on," and folding a per-link RSSI field into it
 * would either go stale the moment a peer is heard on a second link, or force a
 * host's status provider to reason about a dimension it doesn't need. This port answers a
 * different question — "how good is this <i>link</i> right now" — that a future election layer
 * (L3) needs and {@code LinkHealth} was never shaped to answer.
 *
 * <p><b>{@link #of} stays link-scoped; the four signing reads are link-and-sender-scoped.</b> RADIO_STATUS is a fact
 * about the radio itself — legitimately one number per {@link LinkId} regardless of which vehicle
 * is currently talking over it. A signature, by contrast, belongs to whoever signed it: several
 * vehicles sharing one UDP lobby carrier (one {@link LinkId}) must never blend each other's
 * signed/unsigned/rejected counts, so every signing read also takes the sender's {@link SysId}.
 *
 * <p><b>Thread-safety:</b> Implementations are thread-safe: written from receive threads, read from any thread.
 */
public interface LinkQuality {

    /** @return the last known quality for {@code id}; empty if no RADIO_STATUS has ever been heard on it */
    Optional<Quality> of(LinkId id);

    /**
     * @return {@code true} if a {@link io.mavlink4j.codec.SignatureStatus#VALID} frame from
     * {@code system} has arrived on {@code id} within the configured window; {@code false} once
     * that sender has gone quiet longer than that on this link, or has never produced one at all
     */
    boolean signedRecently(LinkId id, SysId system);

    /** @return how many frames from {@code system} on {@code id} have carried {@link io.mavlink4j.codec.SignatureStatus#UNSIGNED}; {@code 0} for a sender never dispatched to on this link */
    int unsignedFrames(LinkId id, SysId system);

    /** @return how many times a frame from {@code system} on {@code id} has been reported via {@code MavlinkSession#onFrameRejected}; {@code 0} for a sender with no rejection yet on this link */
    int rejectedFrames(LinkId id, SysId system);

    /** @return the most recent {@link SignatureRejection} reported for {@code system} on {@code id}; empty if none has been */
    Optional<SignatureRejection> lastRejection(LinkId id, SysId system);

    /**
     * One RADIO_STATUS (#109) snapshot. Every numeric field is nullable-boxed rather than a
     * primitive default because "never heard" and "heard a genuine zero" must stay distinguishable
     * — a primitive {@code int} default of {@code 0} would silently claim "zero RSSI," a real and
     * very different reading from "no radio has ever reported in."
     *
     * @param fixed a corrected-packet count in the real MAVLink spec (dronefleet's {@code
     *              RadioStatus.fixed()} is an {@code int}, not a boolean) — this port follows the
     *              published contract's {@code Boolean} shape (true iff nonzero), a deliberate
     *              simplification: L1 only needs "is FEC currently correcting anything," not the count
     */
    record Quality(LinkId linkId, Instant lastRadioStatusAt, Integer rssi, Integer remoteRssi,
                    Integer noise, Integer rxErrors, Boolean fixed) {
    }
}
