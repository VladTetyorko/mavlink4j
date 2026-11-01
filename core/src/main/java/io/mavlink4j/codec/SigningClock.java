/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.Objects;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;

/**
 * The MAVLink 2 signing timestamp for one remote system (VERIFIED,
 * <a href="https://mavlink.io/en/guide/message_signing.html">mavlink.io message-signing spec</a>):
 * a 48-bit counter in 10&nbsp;microsecond ticks since {@link #SIGNING_EPOCH}, not Unix epoch -- a
 * naive millisecond- or microsecond-since-1970 implementation produces a value that looks
 * plausible and fails every verification silently (QA item 3).
 *
 * <h2>The spec's own persistence rule, implemented exactly</h2>
 * <i>"[The timestamp] should be monotonically increasing... incremented... at least once every
 * time a message is sent... stored in persistent storage, ideally at least once a minute; the
 * timestamp used on startup should be the maximum of the timestamp implied by the system clock and
 * the stored timestamp."</i> This class is that rule and nothing else: {@code loadPersisted}/{@code
 * persist} are plain functional collaborators (the library has no storage of its own; the host
 * supplies one), so a backward-jumped local clock (a VM
 * snapshot restore, an NTP step) can never regress the sequence below what was last durably saved
 * (QA item 1) -- {@link #next()}'s first call seeds from {@code max(system, stored)}, never system
 * alone.
 *
 * <p>One instance is scoped to one remote system id (see {@link SigningClocks}), shared between the
 * outbound-signing use ({@link FrameWriter} calling {@link #next()} for every frame addressed to
 * that vehicle) and the inbound-verification use ({@link SignatureVerifier} calling {@link
 * #observe(long)} after accepting a signed frame from it) -- the spec's own "keep systems
 * synchronised" rule, applied per remote system rather than process-wide.
 *
 * <h2>Two meanings, two fields</h2>
 * {@link #current()} is this side's own outbound sequence -- monotonic across a restart because
 * it is seeded {@code max(system, stored)}, wall clock included. {@link #floor()} is the inbound
 * freshness reference {@link SignatureVerifier} checks a vehicle's frames against: the highest
 * timestamp this side has accepted from, or handed to, this vehicle, seeded from {@code
 * loadPersisted} alone -- never the wall clock. A vehicle with no real-time clock (pymavlink, or an
 * ArduPilot before its GPS/GCS sets one) can only advance its own clock from frames this side
 * sends it, so measuring it against a wall-clock-seeded value after a restart longer than the stale
 * tolerance would reject it forever; measuring it against the floor lets a resumed vehicle keep
 * sending at the pace it already proved it could reach.
 *
 * <p>Thread-safe via a single monitor; called at most once per outbound/inbound frame, never a
 * contention hot path.
 */
public final class SigningClock {

    /**
     * MAVLink 2 signing epoch (VERIFIED, mavlink.io): timestamps are 10&micro;s ticks since this
     * instant, not since the Unix epoch.
     */
    static final Instant SIGNING_EPOCH = Instant.parse("2015-01-01T00:00:00Z");

    private static final long TICKS_PER_SECOND = 100_000L; // 10us resolution

    private final InstantSource now;
    private final LongSupplier loadPersisted;
    private final LongConsumer persist;
    private final Duration persistInterval;

    private boolean initialized;
    private long current;
    private long floor;
    private Instant lastPersistAt;

    public SigningClock(InstantSource now, LongSupplier loadPersisted, LongConsumer persist, Duration persistInterval) {
        this.now = Objects.requireNonNull(now, "now");
        this.loadPersisted = Objects.requireNonNull(loadPersisted, "loadPersisted");
        this.persist = Objects.requireNonNull(persist, "persist");
        this.persistInterval = Objects.requireNonNull(persistInterval, "persistInterval");
        if (persistInterval.isZero() || persistInterval.isNegative()) {
            throw new IllegalArgumentException("persistInterval must be positive, got " + persistInterval);
        }
    }

    /** The next value to stamp an outbound signed frame with: strictly increasing, +1 per call. */
    public synchronized long next() {
        ensureInitialized();
        current = current + 1;
        maybePersist();
        return current;
    }

    /** The last value handed out by {@link #next()} or observed via {@link #observe(long)}. Never increments. */
    public synchronized long current() {
        ensureInitialized();
        return current;
    }

    /**
     * The highest signature timestamp this side has accepted from, or handed to, this remote
     * system -- see the class-level "Two meanings, two fields" note. Seeded from {@code
     * loadPersisted} only (so a never-heard vehicle floors at {@code 0}, not this process's boot
     * time); raised only by {@link #observe(long)}, never by {@link #next()}.
     */
    public synchronized long floor() {
        ensureInitialized();
        return floor;
    }

    /**
     * Folds a remote timestamp into this clock: both {@link #current()} and {@link #floor()} become
     * {@code max(existing, remote)}. Never moves either backwards -- the spec's own "keep systems
     * synchronised" rule, applied after accepting a signed frame from this remote system, or after
     * handing one a timestamp via {@code SETUP_SIGNING} (see {@link SignatureVerifier} and {@code
     * MavlinkSigningPushService} respectively).
     */
    public synchronized void observe(long remote) {
        ensureInitialized();
        if (remote > current) {
            current = remote;
        }
        if (remote > floor) {
            floor = remote;
        }
        maybePersist();
    }

    private void ensureInitialized() {
        if (!initialized) {
            long persisted = loadPersisted.getAsLong();
            current = Math.max(timestampOf(now.instant()), persisted);
            floor = persisted;
            lastPersistAt = now.instant();
            initialized = true;
        }
    }

    /**
     * Rate-limits {@link #persist} to at most once per {@link #persistInterval} -- an internal
     * detail, not a caller contract. Persists {@link #floor}, never {@link #current}: a persisted
     * wall-clock value would be re-loaded as this vehicle's floor after the next restart and
     * deadlock a vehicle that never received it, whereas the outbound sequence stays monotonic
     * through the wall-clock seed alone, which is always at least as high as anything the vehicle
     * has echoed back.
     */
    private void maybePersist() {
        Instant nowInstant = now.instant();
        if (Duration.between(lastPersistAt, nowInstant).compareTo(persistInterval) >= 0) {
            lastPersistAt = nowInstant;
            persist.accept(floor);
        }
    }

    /** Pure conversion: {@code instant} expressed as 10&micro;s ticks since {@link #SIGNING_EPOCH}. */
    static long timestampOf(Instant instant) {
        Objects.requireNonNull(instant, "instant");
        Duration since = Duration.between(SIGNING_EPOCH, instant);
        return since.getSeconds() * TICKS_PER_SECOND + since.getNano() / 10_000L;
    }

    /** Pure conversion, the inverse of {@link #timestampOf(Instant)}. */
    static Instant instantOf(long timestamp) {
        long seconds = timestamp / TICKS_PER_SECOND;
        long nanos = (timestamp % TICKS_PER_SECOND) * 10_000L;
        return SIGNING_EPOCH.plusSeconds(seconds).plusNanos(nanos);
    }

    /** {@code duration} expressed as a 10&micro;s-tick count -- used to compare a {@link Duration} tolerance against a tick value. */
    static long ticksOf(Duration duration) {
        Objects.requireNonNull(duration, "duration");
        return duration.getSeconds() * TICKS_PER_SECOND + duration.getNano() / 10_000L;
    }
}
