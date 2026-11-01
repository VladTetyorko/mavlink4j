/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.session.TxScheduler;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TimeDoublesTest {

    @Test
    void manualClockStandsStillUntilAdvancedOrSet() {
        ManualInstantSource clock = new ManualInstantSource();
        assertEquals(ManualInstantSource.DEFAULT_START, clock.instant());
        clock.advance(Duration.ofSeconds(5));
        assertEquals(ManualInstantSource.DEFAULT_START.plusSeconds(5), clock.instant());
        clock.set(Instant.EPOCH);
        assertEquals(Instant.EPOCH, clock.instant());
    }

    @Test
    void repeatingTaskFiresOnlyOnTickAndStopsWhenItsHandleCloses() {
        DirectTxScheduler scheduler = new DirectTxScheduler();
        AtomicInteger runs = new AtomicInteger();
        TxScheduler.Handle handle = scheduler.repeat("beat", Duration.ofSeconds(1), runs::incrementAndGet);

        assertEquals(0, runs.get(), "nothing runs until the test ticks");
        scheduler.tick(3);
        assertEquals(3, runs.get());
        assertEquals(Duration.ofSeconds(1), scheduler.registrations().get(0).period());

        handle.close();
        scheduler.tick();
        assertEquals(3, runs.get());
        assertTrue(scheduler.registrations().get(0).cancelled());
    }

    @Test
    void submitRunsInlineAndAThrowingRepeatDoesNotStopTheOthers() {
        DirectTxScheduler scheduler = new DirectTxScheduler();
        AtomicInteger ran = new AtomicInteger();
        scheduler.submit("once", ran::incrementAndGet);
        assertEquals(1, ran.get());

        scheduler.repeat("bad", Duration.ofMillis(1), () -> {
            throw new IllegalStateException("boom");
        });
        scheduler.repeat("good", Duration.ofMillis(1), ran::incrementAndGet);
        scheduler.tick();
        assertEquals(2, ran.get());
    }
}
