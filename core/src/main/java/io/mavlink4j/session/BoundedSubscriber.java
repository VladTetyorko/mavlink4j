/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.codec.Frame;
import io.mavlink4j.config.MavlinkSettings;

import java.lang.System.Logger.Level;
import java.time.Duration;
import java.util.ArrayDeque;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;

/**
 * A {@link Dispatcher} handler wrapper for a downstream consumer that cannot process frames as
 * fast as they arrive (a slow HTTP/SSE push, a message-bus producer); built by {@link
 * Dispatcher#subscribeAsync}, never by a caller. {@link #accept}
 * is the handler actually registered with {@link Dispatcher#subscribe}: it only ever enqueues,
 * never calls the slow downstream consumer itself, so it never blocks the RX thread. A dedicated
 * drain thread calls the downstream consumer at whatever pace it can sustain.
 *
 * <h2>Backpressure policy: bounded, drop-oldest</h2>
 * When the queue is at {@code capacity}, {@link #accept} evicts the oldest queued frame to make
 * room for the newest one rather than blocking or growing unbounded — the newest telemetry is what
 * a stalled consumer most needs once it catches up (project rule 9: newest data over stale data).
 * {@link #droppedCount()} reports how many frames were evicted this way, so a caller can alarm on
 * sustained backpressure instead of silently losing data forever.
 *
 * <h2>Threading</h2>
 * {@link #accept} may be called concurrently from any RX thread that dispatches a matching frame
 * (a session may have several links); the internal queue is guarded by one monitor. Exactly one
 * drain thread, named after this subscriber, delivers to {@code downstream}. A downstream that
 * throws is logged and skipped — the drain thread keeps running.
 */
final class BoundedSubscriber implements Consumer<Frame> {

    private static final System.Logger LOG = System.getLogger(BoundedSubscriber.class.getName());

    private final String name;
    private final int capacity;
    private final Duration closeJoinTimeout;
    private final Consumer<Frame> downstream;
    private final ArrayDeque<Frame> queue = new ArrayDeque<>();
    private final Object lock = new Object();
    private final AtomicLong dropped = new AtomicLong();
    private final AtomicBoolean closed = new AtomicBoolean(false);
    private final Thread worker;

    BoundedSubscriber(String name, MavlinkSettings settings, Consumer<Frame> downstream) {
        this.name = Objects.requireNonNull(name, "name");
        Objects.requireNonNull(settings, "settings");
        this.capacity = settings.dispatchQueueCapacity();
        this.closeJoinTimeout = settings.closeJoinTimeout();
        this.downstream = Objects.requireNonNull(downstream, "downstream");
        this.worker = LibraryThreads.create(settings.threadFactory(), "mavlink-subscriber-" + name, this::drainLoop);
        this.worker.start();
    }

    /** Never blocks: enqueues, dropping the oldest queued frame first if already at capacity. */
    @Override
    public void accept(Frame frame) {
        Objects.requireNonNull(frame, "frame");
        synchronized (lock) {
            if (closed.get()) {
                return;
            }
            if (queue.size() >= capacity) {
                queue.pollFirst();
                dropped.incrementAndGet();
            }
            queue.addLast(frame);
            lock.notify();
        }
    }

    /** Frames evicted so far because {@code downstream} could not keep up. */
    long droppedCount() {
        return dropped.get();
    }

    /** Idempotent. Stops accepting new frames and joins the drain thread, bounded by {@code closeJoinTimeout}. */
    void close() {
        if (!closed.compareAndSet(false, true)) {
            return;
        }
        synchronized (lock) {
            lock.notifyAll();
        }
        try {
            worker.join(closeJoinTimeout.toMillis());
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void drainLoop() {
        while (true) {
            Frame frame;
            synchronized (lock) {
                while (queue.isEmpty() && !closed.get()) {
                    try {
                        lock.wait();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }
                frame = queue.pollFirst();
                if (frame == null) {
                    return; // closed and fully drained
                }
            }
            try {
                downstream.accept(frame);
            } catch (RuntimeException e) {
                LOG.log(Level.WARNING, "BoundedSubscriber '" + name + "' downstream consumer threw; draining continues", e);
            }
        }
    }
}
