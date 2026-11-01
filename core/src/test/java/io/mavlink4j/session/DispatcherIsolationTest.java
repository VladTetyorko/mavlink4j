/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.CompId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.FrameHeader;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Direct unit tests of {@link DefaultDispatcher} and {@link BoundedSubscriber} -- pure in-memory
 * components, hand-built {@link Frame}s, no sockets.
 */
class DispatcherIsolationTest {

    private static final LinkId LINK = new LinkId("test-link");
    private static final SysId ORIGIN = new SysId(1);

    @Test
    void aThrowingSubscriberDoesNotPreventAnotherFromReceivingTheSameFrame() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(MavlinkSettings.defaults());
        AtomicInteger goodSubscriberCount = new AtomicInteger();

        dispatcher.subscribe(MessageFilter.any(), frame -> {
            throw new RuntimeException("boom -- a deliberately broken subscriber");
        });
        dispatcher.subscribe(MessageFilter.any(), frame -> goodSubscriberCount.incrementAndGet());

        dispatcher.dispatch(heartbeatFrame());
        dispatcher.dispatch(heartbeatFrame());

        assertEquals(2, goodSubscriberCount.get(), "the throwing subscriber must not stop the other one");
    }

    @Test
    void unsubscribingOneSubscriptionDoesNotAffectAnotherWithAnEqualFilterAndHandler() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(MavlinkSettings.defaults());
        AtomicInteger count = new AtomicInteger();
        // deliberately identical filter/handler for both subscriptions -- reference identity, not
        // structural equality, must decide which one close() removes (see DefaultDispatcher javadoc).
        MessageFilter filter = MessageFilter.any();
        Consumer<Frame> handler = frame -> count.incrementAndGet();

        Subscription first = dispatcher.subscribe(filter, handler);
        dispatcher.subscribe(filter, handler);

        first.close();
        dispatcher.dispatch(heartbeatFrame());

        assertEquals(1, count.get(), "closing one subscription must not remove the other");
    }

    @Test
    void subscriptionCloseIsIdempotent() {
        DefaultDispatcher dispatcher = new DefaultDispatcher(MavlinkSettings.defaults());
        Subscription subscription = dispatcher.subscribe(MessageFilter.any(), frame -> { });
        subscription.close();
        subscription.close(); // must not throw
    }

    @Test
    void aSlowSubscriberWrappedInBoundedSubscriberDropsOldestAndReportsItsCount() throws Exception {
        CountDownLatch releaseSlowConsumer = new CountDownLatch(1);
        List<Frame> delivered = new CopyOnWriteArrayList<>();
        MavlinkSettings small = MavlinkSettings.defaults().toBuilder().dispatchQueueCapacity(2).build();
        DefaultDispatcher dispatcher = new DefaultDispatcher(small);
        AsyncSubscription slow = dispatcher.subscribeAsync(MessageFilter.any(), frame -> {
            try {
                releaseSlowConsumer.await(5, TimeUnit.SECONDS); // simulates a stalled downstream
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            delivered.add(frame);
        });

        AtomicInteger fastSubscriberCount = new AtomicInteger();
        dispatcher.subscribe(MessageFilter.any(), frame -> fastSubscriberCount.incrementAndGet());

        long start = System.nanoTime();
        // capacity 2: the drain thread picks up frame 0 immediately and blocks on it inside
        // downstream.accept -- frames 1..4 queue up, evicting oldest each time it overflows.
        for (int i = 0; i < 5; i++) {
            dispatcher.dispatch(heartbeatFrame());
        }
        long elapsedMillis = Duration.ofNanos(System.nanoTime() - start).toMillis();

        assertEquals(5, fastSubscriberCount.get(), "the fast subscriber must see every frame");
        assertTrue(elapsedMillis < 1000,
                "dispatch must never block on the slow subscriber's downstream consumer, took " + elapsedMillis + "ms");
        assertTrue(slow.droppedCount() > 0, "capacity 2 with 5 dispatched and one held by the drain "
                + "thread must have dropped at least one");

        releaseSlowConsumer.countDown();
        slow.close();
        dispatcher.close();
    }

    @Test
    void boundedSubscriberDeliversWhateverSurvivedTheDropsInOrder() throws Exception {
        List<Frame> delivered = new CopyOnWriteArrayList<>();
        CountDownLatch allDelivered = new CountDownLatch(3);
        DefaultDispatcher dispatcher = new DefaultDispatcher(MavlinkSettings.defaults());
        AsyncSubscription subscriber = dispatcher.subscribeAsync(MessageFilter.any(), frame -> {
            delivered.add(frame);
            allDelivered.countDown();
        });

        dispatcher.dispatch(heartbeatFrame(1));
        dispatcher.dispatch(heartbeatFrame(2));
        dispatcher.dispatch(heartbeatFrame(3));

        assertTrue(allDelivered.await(2, TimeUnit.SECONDS));
        assertEquals(List.of(1, 2, 3), delivered.stream().map(f -> f.header().sequence()).toList());
        assertEquals(0, subscriber.droppedCount());
        subscriber.close();
        dispatcher.close();
    }

    @Test
    void closingTheDispatcherStopsEveryAsyncSubscriberThread() throws Exception {
        java.util.concurrent.CopyOnWriteArrayList<Thread> made = new java.util.concurrent.CopyOnWriteArrayList<>();
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder()
                .threadFactory(task -> {
                    Thread thread = new Thread(task);
                    made.add(thread);
                    return thread;
                }).build();
        DefaultDispatcher dispatcher = new DefaultDispatcher(settings);
        dispatcher.subscribeAsync(MessageFilter.any(), frame -> { });
        dispatcher.subscribeAsync(MessageFilter.any(), frame -> { });

        dispatcher.close();

        assertEquals(2, made.size(), "each async subscriber owns one thread from the injected factory");
        for (Thread thread : made) {
            thread.join(2_000);
            assertTrue(!thread.isAlive(), thread.getName() + " must exit once the dispatcher closes");
        }
    }

    private static Frame heartbeatFrame() {
        return heartbeatFrame(0);
    }

    private static Frame heartbeatFrame(int seq) {
        FrameHeader header = new FrameHeader(2, seq, ORIGIN, new CompId(1), 0, 0, 0, false, SignatureStatus.UNSIGNED);
        Heartbeat heartbeat = Heartbeat.builder()
                .type(MavType.MAV_TYPE_GENERIC)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(0)
                .systemStatus(MavState.MAV_STATE_STANDBY)
                .mavlinkVersion(3)
                .build();
        return new Frame(header, heartbeat, LINK, LinkPeer.NONE, Instant.now());
    }
}
