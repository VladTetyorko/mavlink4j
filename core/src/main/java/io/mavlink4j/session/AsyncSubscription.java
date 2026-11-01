/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

/**
 * A live {@link Dispatcher#subscribeAsync} registration: closing it stops delivery <i>and</i> shuts
 * the subscriber's own drain thread down.
 */
public interface AsyncSubscription extends Subscription {

    /**
     * How many frames were evicted because the handler could not keep up (the queue holds the newest
     * {@code dispatchQueueCapacity} frames and drops the oldest). Sustained growth means the handler
     * is too slow for the stream.
     */
    long droppedCount();
}
