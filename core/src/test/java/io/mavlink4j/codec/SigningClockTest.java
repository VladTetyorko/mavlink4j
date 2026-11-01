/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Unit tests of {@link SigningClock} against the MAVLink 2 signing-timestamp spec (VERIFIED,
 * <a href="https://mavlink.io/en/guide/message_signing.html">mavlink.io message-signing spec</a>),
 * in isolation from any real clock or persistence store: every collaborator ({@link InstantSource},
 * {@code loadPersisted}, {@code persist}) is a controllable test double (covers clock skew across
 * a fresh station restart, and the 10&micro;s unit conversion).
 */
class SigningClockTest {

    @Test
    void timestampOfConvertsAKnownInstantToTheSpecCorrectTickCount() {
        // Exactly 1 second after the signing epoch = 100_000 ticks at 10us resolution.
        Instant oneSecondAfterEpoch = SigningClock.SIGNING_EPOCH.plusSeconds(1);
        assertEquals(100_000L, SigningClock.timestampOf(oneSecondAfterEpoch));
    }

    @Test
    void instantOfIsTheExactInverseOfTimestampOf() {
        Instant original = SigningClock.SIGNING_EPOCH.plusSeconds(12_345).plusNanos(670_000);
        long ticks = SigningClock.timestampOf(original);
        assertEquals(original, SigningClock.instantOf(ticks));
    }

    @Test
    void ticksOfConvertsADurationTheSameWayTimestampOfConvertsAnInstant() {
        assertEquals(500_000L, SigningClock.ticksOf(Duration.ofSeconds(5)));
    }

    @Test
    void nextIsStrictlyIncreasingByOnePerCall() {
        SigningClock clock = new SigningClock(InstantSource.fixed(SigningClock.SIGNING_EPOCH.plusSeconds(100)),
                () -> 0L, discarded -> { }, Duration.ofMinutes(1));

        long first = clock.next();
        long second = clock.next();
        long third = clock.next();

        assertEquals(first + 1, second);
        assertEquals(second + 1, third);
    }

    @Test
    void currentReflectsTheBaselineWithoutIncrementingIt() {
        SigningClock clock = new SigningClock(InstantSource.fixed(SigningClock.SIGNING_EPOCH.plusSeconds(100)),
                () -> 0L, discarded -> { }, Duration.ofMinutes(1));

        long baseline = clock.current();
        assertEquals(baseline, clock.current(), "current() must not move the clock forward on its own");
    }

    @Test
    void startsFromTheSystemClockWhenItIsAheadOfWhatWasPersisted() {
        Instant now = SigningClock.SIGNING_EPOCH.plusSeconds(1_000);
        SigningClock clock = new SigningClock(InstantSource.fixed(now), () -> 1L, discarded -> { }, Duration.ofMinutes(1));

        assertEquals(SigningClock.timestampOf(now), clock.current());
    }

    @Test
    void startsFromThePersistedValueWhenItIsAheadOfTheSystemClock() {
        // A station clock that jumped backwards (VM snapshot restore, an NTP step) must never
        // regress the sequence below what was last durably saved -- spec: "the timestamp used on
        // startup should be the maximum of the timestamp implied by the system clock and the stored
        // timestamp."
        Instant now = SigningClock.SIGNING_EPOCH.plusSeconds(1);
        long persisted = SigningClock.timestampOf(SigningClock.SIGNING_EPOCH.plusSeconds(999_999));
        SigningClock clock = new SigningClock(InstantSource.fixed(now), () -> persisted, discarded -> { }, Duration.ofMinutes(1));

        assertEquals(persisted, clock.current());
    }

    @Test
    void observeNeverMovesTheClockBackwards() {
        SigningClock clock = new SigningClock(InstantSource.fixed(SigningClock.SIGNING_EPOCH.plusSeconds(100)),
                () -> 0L, discarded -> { }, Duration.ofMinutes(1));
        long baseline = clock.current();

        clock.observe(baseline - 1_000); // an older remote timestamp
        assertEquals(baseline, clock.current(), "observe() must never move the clock backwards");

        clock.observe(baseline + 1_000); // a newer remote timestamp -- the spec's "keep systems synchronised" rule
        assertEquals(baseline + 1_000, clock.current());
    }

    @Test
    void floorStaysAtThePersistedValueAcrossARestartWhileCurrentFollowsTheWallClock() {
        // Simulates a station restart: a station that
        // comes back up long after it last persisted must keep signing outbound frames forward from
        // the wall clock, but must not judge an already-paired vehicle's freshness against that same
        // wall-clock value -- the vehicle has no way to have kept up with it.
        List<Long> persisted = new ArrayList<>();
        Instant floorInstant = SigningClock.SIGNING_EPOCH.plusSeconds(10_000);
        long floorValue = SigningClock.timestampOf(floorInstant);
        Instant wallInstant = SigningClock.instantOf(floorValue + SigningClock.ticksOf(Duration.ofMinutes(10)));
        MutableInstantSource now = new MutableInstantSource(wallInstant);
        SigningClock clock = new SigningClock(now, () -> floorValue, persisted::add, Duration.ofMinutes(1));

        assertEquals(floorValue, clock.floor(), "floor seeds from the persisted value only");
        long wallValue = clock.current();
        assertEquals(SigningClock.timestampOf(wallInstant), wallValue, "current seeds from the wall clock");

        clock.observe(floorValue + 5);
        assertEquals(floorValue + 5, clock.floor(), "observe() raises the floor to the accepted-or-handed value");
        assertEquals(wallValue, clock.current(), "observe() below current must not move current");

        long currentBeforeNext = clock.current();
        clock.next();
        clock.next();
        assertEquals(floorValue + 5, clock.floor(), "next() must never raise the floor");
        assertEquals(currentBeforeNext + 2, clock.current());

        now.advance(Duration.ofMinutes(2)); // past persistInterval
        clock.next();
        assertEquals(floorValue + 5, persisted.get(persisted.size() - 1),
                "the value handed to persist() is the floor, never current");
    }

    @Test
    void freshPairingFloorsAtZeroWhileCurrentStillSeedsFromTheWallClock() {
        // Fresh-pairing case: nothing has ever been persisted for this sysid.
        Instant wallInstant = SigningClock.SIGNING_EPOCH.plusSeconds(500_000);
        SigningClock clock = new SigningClock(InstantSource.fixed(wallInstant), () -> 0L, discarded -> { }, Duration.ofMinutes(1));

        assertEquals(0L, clock.floor());
        assertEquals(SigningClock.timestampOf(wallInstant), clock.current());
    }

    @Test
    void observeAboveBothFloorAndCurrentRaisesBoth() {
        SigningClock clock = new SigningClock(InstantSource.fixed(SigningClock.SIGNING_EPOCH.plusSeconds(100)),
                () -> 0L, discarded -> { }, Duration.ofMinutes(1));
        long wall = clock.current();

        clock.observe(wall + 7);

        assertEquals(wall + 7, clock.floor());
        assertEquals(wall + 7, clock.current());
    }

    @Test
    void persistIsRateLimitedToAtMostOncePerConfiguredInterval() {
        List<Long> persisted = new ArrayList<>();
        MutableInstantSource now = new MutableInstantSource(SigningClock.SIGNING_EPOCH.plusSeconds(100));
        SigningClock clock = new SigningClock(now, () -> 0L, persisted::add, Duration.ofSeconds(60));

        clock.next();
        int afterFirstCall = persisted.size();
        clock.next();
        clock.next();
        assertEquals(afterFirstCall, persisted.size(),
                "no further persist() call must happen before persistInterval has elapsed");

        now.advance(Duration.ofSeconds(61));
        clock.next();
        assertEquals(afterFirstCall + 1, persisted.size(),
                "a persist() call must happen once persistInterval has elapsed");
    }

    /** A mutable {@link InstantSource} test double -- {@link InstantSource#fixed} cannot advance. */
    private static final class MutableInstantSource implements InstantSource {
        private Instant instant;

        MutableInstantSource(Instant instant) {
            this.instant = instant;
        }

        void advance(Duration by) {
            instant = instant.plus(by);
        }

        @Override
        public Instant instant() {
            return instant;
        }
    }
}
