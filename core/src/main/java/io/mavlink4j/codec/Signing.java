/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import java.time.Duration;
import java.time.InstantSource;
import java.util.Objects;

/**
 * The one collaborator bundle {@link FrameReader}/{@link FrameWriter}/{@code
 * io.mavlink4j.session.MavlinkSession} thread signing through: a key resolver, a policy and a
 * clock registry. The numeric tuning (windows, tolerance, a bound) lives apart, in {@code
 * MavlinkSettings.SigningSettings}.
 */
public record Signing(SigningKeyResolver keys, SigningPolicy policy, SigningClocks clocks) {

    public Signing {
        Objects.requireNonNull(keys, "keys");
        Objects.requireNonNull(policy, "policy");
        Objects.requireNonNull(clocks, "clocks");
    }

    /** No key ever resolves, every frame is accepted regardless of status, clocks are in-memory only. */
    public static Signing off() {
        return new Signing(SigningKeyResolver.NONE, SigningPolicy.PERMISSIVE,
                SigningClocks.inMemory(InstantSource.system(), Duration.ofMinutes(1)));
    }
}
