/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.Objects;

/**
 * An {@link InstantSource} that only moves when the test says so, for
 * {@code MavlinkSettings.builder().clock(...)}. Thread-safe: the library reads the clock from its
 * reader threads while the test advances it.
 */
public final class ManualInstantSource implements InstantSource {

    /** The instant a source created without an argument starts at. */
    public static final Instant DEFAULT_START = Instant.parse("2026-01-01T00:00:00Z");

    private volatile Instant now;

    /** Starts at {@link #DEFAULT_START}. */
    public ManualInstantSource() {
        this(DEFAULT_START);
    }

    /**
     * Starts at {@code start}.
     *
     * @param start the first instant this source reports
     */
    public ManualInstantSource(Instant start) {
        this.now = Objects.requireNonNull(start, "start");
    }

    /**
     * Moves the clock forward (or backward, for a negative duration).
     *
     * @param by how far to move
     */
    public synchronized void advance(Duration by) {
        now = now.plus(Objects.requireNonNull(by, "by"));
    }

    /**
     * Jumps the clock to an absolute instant.
     *
     * @param instant the new current instant
     */
    public void set(Instant instant) {
        this.now = Objects.requireNonNull(instant, "instant");
    }

    @Override
    public Instant instant() {
        return now;
    }
}
