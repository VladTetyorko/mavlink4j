/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.session.TxScheduler;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * A {@link TxScheduler} with no threads and no timers: {@link #submit} runs inline and a
 * {@link #repeat} registration only fires when the test calls {@link #tick()}. Lets a streaming service
 * (heartbeat, manual control) be driven tick by tick, deterministically.
 */
public final class DirectTxScheduler implements TxScheduler {

    /** One {@link #repeat} registration, observable by the test. */
    public static final class Registration {
        private final String name;
        private final Duration period;
        private final Runnable task;
        private volatile boolean cancelled;

        private Registration(String name, Duration period, Runnable task) {
            this.name = name;
            this.period = period;
            this.task = task;
        }

        /** @return the diagnostic name given to {@code repeat} */
        public String name() {
            return name;
        }

        /** @return the period the service asked for (never waited on) */
        public Duration period() {
            return period;
        }

        /** @return {@code true} once the returned {@link Handle} was closed */
        public boolean cancelled() {
            return cancelled;
        }
    }

    private final List<Registration> registrations = new CopyOnWriteArrayList<>();

    @Override
    public Handle repeat(String name, Duration period, Runnable task) {
        Registration registration = new Registration(
                Objects.requireNonNull(name, "name"),
                Objects.requireNonNull(period, "period"),
                Objects.requireNonNull(task, "task"));
        registrations.add(registration);
        return () -> registration.cancelled = true;
    }

    @Override
    public void submit(String name, Runnable task) {
        Objects.requireNonNull(task, "task").run();
    }

    /** Runs every live repeating task once, in registration order. A throwing task is logged-and-skipped as the real scheduler does. */
    public void tick() {
        for (Registration registration : registrations) {
            if (!registration.cancelled) {
                runQuietly(registration.task);
            }
        }
    }

    /**
     * Runs {@link #tick()} {@code times} times.
     *
     * @param times how many rounds to run
     */
    public void tick(int times) {
        for (int i = 0; i < times; i++) {
            tick();
        }
    }

    /** @return a snapshot of every registration made so far, cancelled ones included */
    public List<Registration> registrations() {
        return new ArrayList<>(registrations);
    }

    private static void runQuietly(Runnable task) {
        try {
            task.run();
        } catch (RuntimeException e) {
            System.getLogger(DirectTxScheduler.class.getName())
                    .log(System.Logger.Level.WARNING, "repeating task threw; continuing", e);
        }
    }
}
