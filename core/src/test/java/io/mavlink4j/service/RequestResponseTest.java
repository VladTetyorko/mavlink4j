/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.service;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameSink;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.FrameHeader;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.session.Correlator;
import io.mavlink4j.session.MatchKey;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.BiConsumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Direct unit tests of {@link RequestResponse} -- the DRY Family-A machine {@link CommandService}
 * (and the future Mission/FTP services) is built on. No sockets, no real {@code MavlinkSession}: a small
 * in-memory {@link FakeCorrelator} (deliberately not {@code DefaultCorrelator}, whose {@code offer}
 * only ever recognizes {@code CommandAck} payloads -- this test wants to drive matching directly with
 * plain marker payloads) paired with a hand-written {@link FrameSink}, so retry/extension timing is
 * exercised with short, deterministic durations rather than a real network round trip.
 */
class RequestResponseTest {

    private static final LinkId LINK = new LinkId("test-link");
    private static final SysId VEHICLE = new SysId(9);
    private static final PeerId TARGET = new PeerId(VEHICLE, new CompId(1));

    @Test
    void silenceRetriesUpToMaxRetriesThenFailsWithTheLastTimeout() throws Exception {
        FakeCorrelator correlator = new FakeCorrelator();
        List<Integer> sentAttempts = new CopyOnWriteArrayList<>();
        FrameSink sink = new SendSink((payload, target) -> sentAttempts.add((Integer) payload));
        RequestResponse requestResponse = new RequestResponse(correlator, sink);
        MatchKey key = key(1L);

        CompletableFuture<Frame> future = requestResponse.exchange(TARGET, key, Duration.ofMillis(40), 2,
                attempt -> attempt, reply -> null);

        ExecutionException e = assertThrows(ExecutionException.class, () -> future.get(3, TimeUnit.SECONDS));
        assertTrue(e.getCause() instanceof TimeoutException, "expected a TimeoutException, got " + e.getCause());
        assertEquals(List.of(0, 1, 2), sentAttempts, "expected one initial send plus two retries, attempts 0,1,2");
    }

    @Test
    void aTerminalReplyCompletesImmediatelyWithoutAnyRetry() throws Exception {
        FakeCorrelator correlator = new FakeCorrelator();
        List<Integer> sentAttempts = new CopyOnWriteArrayList<>();
        MatchKey key = key(1L);
        FrameSink sink = new SendSink((payload, target) -> {
            sentAttempts.add((Integer) payload);
            correlator.offer(key, frameOf(key, "terminal"));
        });
        RequestResponse requestResponse = new RequestResponse(correlator, sink);

        CompletableFuture<Frame> future = requestResponse.exchange(TARGET, key, Duration.ofSeconds(2), 2,
                attempt -> attempt, reply -> null);

        Frame result = future.get(2, TimeUnit.SECONDS);
        assertEquals("terminal", result.payload());
        assertEquals(List.of(0), sentAttempts, "a terminal reply must not trigger any retry");
    }

    @Test
    void aClassifierExtensionKeepsWaitingWithoutResendingUntilATerminalReplyArrives() throws Exception {
        FakeCorrelator correlator = new FakeCorrelator();
        List<Integer> sentAttempts = new CopyOnWriteArrayList<>();
        MatchKey key = key(2L);
        FrameSink sink = new SendSink((payload, target) -> {
            sentAttempts.add((Integer) payload);
            // The vehicle answers IN_PROGRESS at once (the waiter is registered before send), so the
            // 30 ms send timeout can never fire first, however slow the machine running the test is.
            correlator.offer(key, frameOf(key, "in-progress"));
        });
        RequestResponse requestResponse = new RequestResponse(correlator, sink);

        CompletableFuture<Frame> future = requestResponse.exchange(TARGET, key, Duration.ofMillis(30), 2,
                attempt -> attempt, reply -> "in-progress".equals(reply.payload()) ? Duration.ofSeconds(5) : null);

        // Well past the 30 ms send timeout: only the extension can still be holding the exchange open.
        Thread.sleep(150);
        assertFalse(future.isDone(), "the extension must keep the exchange waiting past the send timeout");
        correlator.offer(key, frameOf(key, "terminal"));

        Frame result = future.get(3, TimeUnit.SECONDS);
        assertEquals("terminal", result.payload());
        assertEquals(List.of(0), sentAttempts, "an extension must never trigger a resend");
    }

    @Test
    void extensionWindowTimingOutIsATerminalFailureNotARetry() throws Exception {
        FakeCorrelator correlator = new FakeCorrelator();
        List<Integer> sentAttempts = new CopyOnWriteArrayList<>();
        MatchKey key = key(3L);
        FrameSink sink = new SendSink((payload, target) -> {
            sentAttempts.add((Integer) payload);
            correlator.offer(key, frameOf(key, "in-progress"));
            // deliberately never offers a terminal reply -- the extension window itself must time out
        });
        RequestResponse requestResponse = new RequestResponse(correlator, sink);

        CompletableFuture<Frame> future = requestResponse.exchange(TARGET, key, Duration.ofMillis(30), 2,
                attempt -> attempt, reply -> "in-progress".equals(reply.payload()) ? Duration.ofMillis(40) : null);

        ExecutionException e = assertThrows(ExecutionException.class, () -> future.get(3, TimeUnit.SECONDS));
        assertTrue(e.getCause() instanceof TimeoutException, "expected a TimeoutException, got " + e.getCause());
        assertEquals(List.of(0), sentAttempts, "a timed-out extension must not resend -- it is a terminal failure");
    }

    @Test
    void aSendFailureFailsTheExchangeAndReleasesTheWaiter() throws Exception {
        FakeCorrelator correlator = new FakeCorrelator();
        MatchKey key = key(4L);
        RuntimeException boom = new RuntimeException("send boom");
        FrameSink sink = new SendSink((payload, target) -> {
            throw boom;
        });
        RequestResponse requestResponse = new RequestResponse(correlator, sink);

        CompletableFuture<Frame> future =
                requestResponse.exchange(TARGET, key, Duration.ofMillis(50), 0, attempt -> attempt, reply -> null);

        ExecutionException e = assertThrows(ExecutionException.class, () -> future.get(2, TimeUnit.SECONDS));
        assertTrue(e.getCause() == boom, "expected the exchange to fail with the send()-thrown exception itself");

        // The waiter must have been released (cancel()), not left dangling -- a fresh await for the
        // same key must succeed rather than throw IllegalStateException.
        correlator.await(key, Duration.ofMillis(50)).cancel(false);
    }

    private static MatchKey key(long discriminator) {
        return new MatchKey(VEHICLE, 77, discriminator);
    }

    private static Frame frameOf(MatchKey key, Object payload) {
        FrameHeader header = new FrameHeader(2, 0, key.system(), new CompId(1), key.messageId(), 0, 0, false, SignatureStatus.UNSIGNED);
        return new Frame(header, payload, LINK, LinkPeer.NONE, Instant.now());
    }

    /** {@link FrameSink} has two abstract methods -- not a lambda target; this wraps only {@code send}. */
    private static final class SendSink implements FrameSink {
        private final BiConsumer<Object, PeerId> onSend;

        SendSink(BiConsumer<Object, PeerId> onSend) {
            this.onSend = onSend;
        }

        @Override
        public java.util.Set<LinkId> links() {
            return java.util.Set.of();
        }

        @Override
        public void send(Object payload, PeerId target) {
            onSend.accept(payload, target);
        }

        @Override
        public void broadcast(Object payload, LinkId link) {
            throw new UnsupportedOperationException("not used by these tests");
        }
    }

    /**
     * A {@link Correlator} this test drives directly via {@link #offer} -- unlike
     * {@code DefaultCorrelator}, whose extraction only ever recognizes a {@code CommandAck} payload,
     * this fake matches purely on {@link MatchKey} so the test can use plain marker payloads.
     */
    private static final class FakeCorrelator implements Correlator {
        private final Map<MatchKey, CompletableFuture<Frame>> waiters = new ConcurrentHashMap<>();

        @Override
        public CompletableFuture<Frame> await(MatchKey key, Duration timeout) {
            Objects.requireNonNull(key, "key");
            Objects.requireNonNull(timeout, "timeout");
            CompletableFuture<Frame> future = new CompletableFuture<>();
            if (waiters.putIfAbsent(key, future) != null) {
                throw new IllegalStateException("A live await is already registered for " + key);
            }
            future.orTimeout(timeout.toMillis(), TimeUnit.MILLISECONDS);
            future.whenComplete((frame, error) -> waiters.remove(key, future));
            return future;
        }

        @Override
        public void cancel(MatchKey key) {
            CompletableFuture<Frame> removed = waiters.remove(key);
            if (removed != null) {
                removed.cancel(false);
            }
        }

        void offer(MatchKey key, Frame frame) {
            CompletableFuture<Frame> future = waiters.remove(key);
            if (future != null) {
                future.complete(frame);
            }
        }
    }
}
