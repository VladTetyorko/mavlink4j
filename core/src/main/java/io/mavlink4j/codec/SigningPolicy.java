/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

/**
 * Decides whether a decoded frame is dispatched, given its origin sysid and {@link
 * SignatureStatus}. Lives in {@code mavlink-core}, not the adapter layer, specifically so a future
 * per-vehicle "signing required" mode is a different {@link SigningPolicy} supplied by the host,
 * not a change to the library.
 *
 * <p><b>Implementer contract:</b> {@link #accept} runs on the receive thread for every decoded
 * frame; it must be fast, stateless or thread-safe, and never throw. By convention a policy never
 * accepts {@link SignatureStatus#INVALID}.
 *
 * <p>{@link #PERMISSIVE} is the default implementation: it accepts everything except a
 * genuinely bad signature. {@link SignatureStatus#INVALID} is dropped under every policy this
 * module will ever define, {@code PERMISSIVE} included -- a bad signature is never "prefer to
 * accept anyway" (see the <a href="https://mavlink.io/en/guide/message_signing.html">signing spec</a>).
 */
public interface SigningPolicy {

    /** @return {@code true} if a frame with this origin sysid and status should be dispatched */
    boolean accept(int sysid, SignatureStatus status);

    /** Accepts {@link SignatureStatus#UNSIGNED}/{@link SignatureStatus#UNVERIFIABLE}/{@link SignatureStatus#VALID}; drops {@link SignatureStatus#INVALID}. */
    SigningPolicy PERMISSIVE = (sysid, status) -> status != SignatureStatus.INVALID;
}
