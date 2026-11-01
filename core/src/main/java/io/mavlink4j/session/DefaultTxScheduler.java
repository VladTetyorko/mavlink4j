/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.config.MavlinkSettings;

import java.lang.System.Logger.Level;
import java.time.Duration;
import java.util.Objects;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * The one {@link TxScheduler} implementation: one shared {@link ScheduledThreadPoolExecutor} for
 * every registered {@link #repeat}/{@link #submit} task, named threads from the settings' {@link
 * MavlinkSettings#threadFactory()}, and a pool of {@link MavlinkSettings#txPoolSize()} threads —
 * enough that one slow/blocked TX task cannot delay every other periodic task sharing this scheduler,
 * while still being unambiguously "one shared pool," not one thread per feature.
 *
 * <p>Reached through {@link MavlinkSession#scheduler()} or {@link TxScheduler#pooled}; the class
 * itself is an implementation detail. Threads are created lazily by the pool, so a scheduler that
 * never receives a task costs no thread.
 */
final class DefaultTxScheduler implements TxScheduler {

    private static final System.Logger LOG = System.getLogger(DefaultTxScheduler.class.getName());

    private final Duration closeJoinTimeout;
    private final ScheduledThreadPoolExecutor executor;
    private final AtomicBoolean closed = new AtomicBoolean(false);
    private final AtomicInteger threadCounter = new AtomicInteger();
    private final ThreadFactory threadFactory;

    DefaultTxScheduler(MavlinkSettings settings) {
        Objects.requireNonNull(settings, "settings");
        this.closeJoinTimeout = settings.closeJoinTimeout();
        this.threadFactory = settings.threadFactory();
        this.executor = new ScheduledThreadPoolExecutor(settings.txPoolSize(), this::newThread);
    }

    private Thread newThread(Runnable r) {
        return LibraryThreads.create(threadFactory, "mavlink-tx-" + threadCounter.incrementAndGet(), r);
    }

    @Override
    public Handle repeat(String name, Duration period, Runnable task) {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(period, "period");
        Objects.requireNonNull(task, "task");
        long periodMillis = Math.max(1, period.toMillis());
        ScheduledFuture<?> future = executor.scheduleAtFixedRate(
                () -> runSafely(name, task), 0, periodMillis, TimeUnit.MILLISECONDS);
        return new HandleImpl(future);
    }

    @Override
    public void submit(String name, Runnable task) {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(task, "task");
        try {
            executor.execute(() -> runSafely(name, task));
        } catch (RejectedExecutionException e) {
            // Raced close() -- see TxScheduler#submit's own "dropped, not thrown" contract.
            LOG.log(Level.DEBUG, "TxScheduler one-shot '" + name + "' dropped; the scheduler is shut down");
        }
    }

    private void runSafely(String name, Runnable task) {
        try {
            task.run();
        } catch (RuntimeException e) {
            LOG.log(Level.WARNING, "TxScheduler task '" + name + "' threw; the schedule continues", e);
        }
    }

    /** Idempotent. Shuts down the shared executor, bounded by {@code closeJoinTimeout}. */
    @Override
    public void close() {
        if (!closed.compareAndSet(false, true)) {
            return;
        }
        executor.shutdownNow();
        try {
            if (!executor.awaitTermination(closeJoinTimeout.toMillis(), TimeUnit.MILLISECONDS)) {
                LOG.log(Level.WARNING, "TxScheduler did not terminate within " + closeJoinTimeout);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private static final class HandleImpl implements Handle {
        private final ScheduledFuture<?> future;
        private final AtomicBoolean closed = new AtomicBoolean(false);

        HandleImpl(ScheduledFuture<?> future) {
            this.future = future;
        }

        @Override
        public void close() {
            if (closed.compareAndSet(false, true)) {
                future.cancel(false);
            }
        }
    }
}
