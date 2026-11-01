/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.config.MavlinkSettings;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * {@link DefaultTxScheduler} tests use fast, injected periods (tens of ms) and assert a fired
 * <i>count</i> over a bounded polling window, never a wall-clock Hz -- exactly the acceptance
 * gate's own instruction, since asserting real-time cadence is inherently flaky under load.
 */
class DefaultTxSchedulerTest {

    @Test
    void firesRoughlyAtTheRequestedPeriod() throws Exception {
        TxScheduler scheduler = TxScheduler.pooled(MavlinkSettings.defaults());
        try {
            AtomicInteger count = new AtomicInteger();
            TxScheduler.Handle handle = scheduler.repeat("test-task", Duration.ofMillis(20), count::incrementAndGet);

            awaitAtLeast(count, 5, Duration.ofSeconds(2));

            handle.close();
        } finally {
            scheduler.close();
        }
    }

    @Test
    void handleCloseIsIdempotentAndStopsFurtherFiring() throws Exception {
        TxScheduler scheduler = TxScheduler.pooled(MavlinkSettings.defaults());
        try {
            AtomicInteger count = new AtomicInteger();
            TxScheduler.Handle handle = scheduler.repeat("test-task", Duration.ofMillis(15), count::incrementAndGet);

            awaitAtLeast(count, 2, Duration.ofSeconds(2));
            handle.close();
            handle.close(); // must not throw

            int countAfterClose = count.get();
            Thread.sleep(100); // give any still-in-flight execution a chance to land, then confirm no more
            assertTrue(count.get() <= countAfterClose + 1,
                    "no further executions should fire after close(), got " + count.get() + " vs " + countAfterClose);
        } finally {
            scheduler.close();
        }
    }

    @Test
    void aThrowingTaskDoesNotStopTheSchedule() throws Exception {
        TxScheduler scheduler = TxScheduler.pooled(MavlinkSettings.defaults());
        try {
            AtomicInteger attempts = new AtomicInteger();
            TxScheduler.Handle handle = scheduler.repeat("throwing-task", Duration.ofMillis(15), () -> {
                attempts.incrementAndGet();
                throw new RuntimeException("boom -- a deliberately broken periodic task");
            });

            awaitAtLeast(attempts, 5, Duration.ofSeconds(2));

            handle.close();
        } finally {
            scheduler.close();
        }
    }

    @Test
    void twoTasksShareTheSchedulerIndependently() throws Exception {
        TxScheduler scheduler = TxScheduler.pooled(MavlinkSettings.defaults());
        try {
            AtomicInteger a = new AtomicInteger();
            AtomicInteger b = new AtomicInteger();
            TxScheduler.Handle handleA = scheduler.repeat("task-a", Duration.ofMillis(15), a::incrementAndGet);
            TxScheduler.Handle handleB = scheduler.repeat("task-b", Duration.ofMillis(15), b::incrementAndGet);

            awaitAtLeast(a, 4, Duration.ofSeconds(2));
            awaitAtLeast(b, 4, Duration.ofSeconds(2));

            handleA.close();
            handleB.close();
        } finally {
            scheduler.close();
        }
    }

    @Test
    void threadsComeFromTheInjectedFactoryAndAreNamedByTheLibrary() throws Exception {
        java.util.concurrent.CopyOnWriteArrayList<Thread> made = new java.util.concurrent.CopyOnWriteArrayList<>();
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder()
                .threadFactory(task -> {
                    Thread thread = new Thread(task);
                    made.add(thread);
                    return thread;
                }).build();
        TxScheduler scheduler = TxScheduler.pooled(settings);
        try {
            java.util.concurrent.CountDownLatch ran = new java.util.concurrent.CountDownLatch(1);
            scheduler.submit("one-shot", ran::countDown);
            assertTrue(ran.await(2, java.util.concurrent.TimeUnit.SECONDS));
            assertTrue(!made.isEmpty(), "the pool must create its threads through the injected factory");
            assertTrue(made.get(0).isDaemon());
            assertTrue(made.get(0).getName().startsWith("mavlink-tx"), made.get(0).getName());
        } finally {
            scheduler.close();
        }
    }

    @Test
    void pooledSchedulerHonoursTxPoolSize() {
        java.util.concurrent.atomic.AtomicInteger created = new java.util.concurrent.atomic.AtomicInteger();
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().txPoolSize(3)
                .threadFactory(task -> {
                    created.incrementAndGet();
                    return new Thread(task);
                }).build();
        TxScheduler scheduler = TxScheduler.pooled(settings);
        try {
            scheduler.repeat("a", Duration.ofSeconds(60), () -> { });
            scheduler.repeat("b", Duration.ofSeconds(60), () -> { });
            scheduler.repeat("c", Duration.ofSeconds(60), () -> { });
            assertEquals(3, created.get(), "core pool threads are spawned one per scheduled task up to txPoolSize");
        } finally {
            scheduler.close();
        }
    }

    private static void awaitAtLeast(AtomicInteger counter, int target, Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (System.nanoTime() < deadline) {
            if (counter.get() >= target) {
                return;
            }
            Thread.sleep(5);
        }
        assertTrue(counter.get() >= target,
                "expected at least " + target + " firings within " + timeout + ", got " + counter.get());
    }
}
