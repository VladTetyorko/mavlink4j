/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.SysId;

import java.util.Objects;

/**
 * The wire-level facts of one MAVLink frame, decoded but not interpreted. {@code version} is 1 or
 * 2 (the magic-byte discriminator, translated to the number everyone actually means); the MAVLink
 * spec itself defines no formula for {@code sequence} beyond "wraps at 255, used to detect loss" —
 * drop-rate accounting on top of it is an L3 concern.
 *
 * <p>{@code incompatFlags}/{@code compatFlags} are always {@code 0} for a version-1 frame (v1 has
 * no such fields). Per spec, an unrecognized bit in {@code incompatFlags} must cause the whole
 * frame to be discarded — the underlying library already enforces this during decode, so a
 * {@code FrameHeader} that exists at all has already passed that check.
 *
 * <p>{@code signed} is the raw wire bit ("did the sender claim to sign this"); {@code
 * signatureStatus} is the cryptographic-validity dimension {@link SignatureVerifier} computed
 * ("did that claim check out") — see {@link SignatureStatus}'s own javadoc for why they are kept
 * separate rather than collapsed into one boolean.
 *
 * <p><b>Thread-safety:</b> Immutable and thread-safe.
 */
public record FrameHeader(int version, int sequence, SysId system, CompId component, int messageId,
                         int incompatFlags, int compatFlags, boolean signed, SignatureStatus signatureStatus) {

    public FrameHeader {
        if (version != 1 && version != 2) {
            throw new IllegalArgumentException("MAVLink version must be 1 or 2, got " + version);
        }
        if (sequence < 0 || sequence > 255) {
            throw new IllegalArgumentException("sequence must be 0..255, got " + sequence);
        }
        Objects.requireNonNull(system, "system");
        Objects.requireNonNull(component, "component");
        if (messageId < 0) {
            throw new IllegalArgumentException("messageId must be >= 0, got " + messageId);
        }
        Objects.requireNonNull(signatureStatus, "signatureStatus");
        if (!signed && signatureStatus != SignatureStatus.UNSIGNED) {
            throw new IllegalArgumentException(
                    "signatureStatus must be UNSIGNED when signed is false, got " + signatureStatus);
        }
    }
}
