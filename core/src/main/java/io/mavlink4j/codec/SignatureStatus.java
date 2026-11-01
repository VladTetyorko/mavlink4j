/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

/**
 * The cryptographic-validity dimension of a decoded MAVLink 2 frame, orthogonal to {@link
 * FrameHeader#signed()} (the raw wire bit). {@code signed} answers "did the sender claim to sign
 * this"; {@code signatureStatus} answers "did that claim check out" — a frame can have {@code
 * signed == true} and still be {@link #UNVERIFIABLE} (no key known yet) or {@link #INVALID} (a
 * genuine forgery, replay, or clock-skew rejection).
 *
 * <p><b>Thread-safety:</b> Enum; thread-safe.
 */
public enum SignatureStatus {

    /** The wire's own signed bit was not set. Never a rejection reason on its own (see {@link SigningPolicy}). */
    UNSIGNED,

    /** Signed, and {@code Mavlink2Message#validateSignature(byte[])} accepted it against a known key. */
    VALID,

    /**
     * Signed, but rejected -- a failed HMAC, a replayed or stale timestamp. See {@link
     * SignatureRejection.Reason} for which. Dropped under every {@link SigningPolicy}, including
     * {@link SigningPolicy#PERMISSIVE} -- a bad signature is never "prefer to accept anyway."
     */
    INVALID,

    /** Signed, but no key is known for the origin system yet -- cannot be checked either way. */
    UNVERIFIABLE
}
