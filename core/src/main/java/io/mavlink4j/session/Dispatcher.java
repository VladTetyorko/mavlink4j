/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.codec.Frame;

import java.util.function.Consumer;

/**
 * Fan-out of decoded frames to subscribers, filtered by {@link MessageFilter}.
 *
 * <h2>The one rule that matters</h2>
 * With {@link #subscribe}, {@code handler} runs synchronously on the RX thread that decoded the
 * frame, and <b>must not block</b>. This dispatcher does not queue and does not retry — a handler
 * that needs to buffer (a slow HTTP/SSE push, a message-bus producer) uses {@link #subscribeAsync}
 * instead of blocking here. A handler that throws is logged and isolated: it never prevents another
 * subscriber from receiving the same frame, and never kills the read loop.
 */
public interface Dispatcher {

    /** Registers {@code handler} for every frame {@code filter} matches; runs on the RX thread. */
    Subscription subscribe(MessageFilter filter, Consumer<Frame> handler);

    /**
     * Registers {@code handler} for every frame {@code filter} matches, but runs it on a thread of
     * its own: the RX thread only enqueues. The queue is bounded by {@code dispatchQueueCapacity} and
     * drops the <b>oldest</b> frame when full — a stalled consumer needs the newest data once it
     * catches up, and a slow consumer must never stall reception. A handler that throws is logged and
     * skipped. Close the returned subscription to stop the thread.
     */
    AsyncSubscription subscribeAsync(MessageFilter filter, Consumer<Frame> handler);
}
