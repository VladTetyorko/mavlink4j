/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.config.MavlinkSettings;

import java.time.Duration;

/**
 * Runs named periodic TX tasks (heartbeat emission, RC override relay, ...) on one shared thread
 * pool rather than one hand-rolled thread loop per feature, so every periodic sender shares one
 * thread lifecycle.
 */
public interface TxScheduler extends AutoCloseable {

    /**
     * A scheduler backed by its own small thread pool ({@link MavlinkSettings#txPoolSize()} threads from
     * {@link MavlinkSettings#threadFactory()}), for a caller that wants one outside a
     * {@link MavlinkSession} — a session already owns one, see {@link MavlinkSession#scheduler()}. The caller
     * owns it and must {@link #close()} it.
     */
    static TxScheduler pooled(MavlinkSettings settings) {
        return new DefaultTxScheduler(settings);
    }

    /**
     * Runs {@code task} every {@code period}, starting immediately, until the returned
     * {@link Handle} is closed. A {@code task} that throws is logged and the schedule continues —
     * one bad periodic task must never silently stop firing without a trace, and must never take
     * down another task sharing the same scheduler.
     *
     * @param name used only for diagnostics (thread naming, log messages) — not an identity key
     */
    Handle repeat(String name, Duration period, Runnable task);

    /**
     * Runs {@code task} once, as soon as a pool thread is free — the latency-sensitive counterpart
     * to {@link #repeat} for a TX path that must not wait for the next tick of a periodic task
     * (a stick that moves 1 ms after a tick should not pay a full period for a clock that had no
     * reason to be where it was). Same catch-and-log
     * contract as {@link #repeat} — a task that throws is logged and never reaches the pool — and
     * the same "already shut down" tolerance: a submission racing scheduler shutdown is dropped,
     * not thrown, since callers submit from threads that must not fail on teardown.
     *
     * <p>No {@link Handle} is returned: a one-shot that has already been handed to the pool cannot
     * usefully be cancelled, and every caller re-checks its own state when the task actually runs.
     *
     * @param name used only for diagnostics (log messages) — not an identity key
     */
    void submit(String name, Runnable task);

    /**
     * Releases the threads this scheduler owns; idempotent. A scheduler that borrows its threads has
     * nothing to release, hence the no-op default.
     */
    @Override
    default void close() {
    }

    /** A live {@link #repeat} registration. */
    interface Handle {

        /** Stops this task from firing again. Idempotent. */
        void close();
    }
}
