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

import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavResult;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Direct unit tests of {@link DefaultCorrelator} -- no sockets, no {@link MavlinkSession}: this
 * class is pure in-memory state, so hand-built {@link Frame}s (matching this module's own
 * {@code FrameWriterFrameReaderRoundTripTest} precedent of building payloads directly) exercise it
 * fully and fast.
 *
 * <p>{@code MavCmd}'s wire value ({@code MAV_CMD_DO_SET_MODE} = 176) is always read back off a
 * built {@link CommandAck} via {@code ack.command().value()} rather than hardcoded or taken from
 * {@code Enum#ordinal()} -- the Java enum's declaration order is not the protocol's wire value.
 */
class DefaultCorrelatorTest {

    private static final LinkId LINK = new LinkId("test-link");
    private static final SysId VEHICLE = new SysId(7);

    @Test
    void matchesACommandAckBySysidAndCommandIdOnly() throws Exception {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        CommandAck ack = ackFor(MavCmd.MAV_CMD_DO_SET_MODE, 99, 99); // wrong target*/*, must not matter
        MatchKey key = new MatchKey(VEHICLE, 77, ack.command().value());

        CompletableFuture<Frame> future = correlator.await(key, Duration.ofSeconds(2));
        Frame ackFrame = frameOf(VEHICLE, 77, ack);
        correlator.offer(ackFrame);

        Frame delivered = future.get(1, TimeUnit.SECONDS);
        assertSame(ackFrame, delivered);
    }

    @Test
    void aCommandAckFromADifferentSystemDoesNotMatch() {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        CommandAck ack = ackFor(MavCmd.MAV_CMD_DO_SET_MODE, 7, 1);
        MatchKey key = new MatchKey(VEHICLE, 77, ack.command().value());
        CompletableFuture<Frame> future = correlator.await(key, Duration.ofMillis(200));

        correlator.offer(frameOf(new SysId(9), 77, ack)); // different origin sysid

        assertFalse(future.isDone());
        correlator.cancel(key);
    }

    @Test
    void aNonAckFrameNeverMatchesAnything() {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        MatchKey key = new MatchKey(VEHICLE, 77, 176L);
        CompletableFuture<Frame> future = correlator.await(key, Duration.ofMillis(200));

        CommandLong command = CommandLong.builder()
                .targetSystem(1).targetComponent(1)
                .command(MavCmd.MAV_CMD_DO_SET_MODE).confirmation(0).build();
        correlator.offer(frameOf(VEHICLE, 76, command));

        assertFalse(future.isDone());
        correlator.cancel(key);
    }

    @Test
    void awaitTimesOutExceptionallyWhenNothingArrives() {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        MatchKey key = new MatchKey(VEHICLE, 77, 1L);

        CompletableFuture<Frame> future = correlator.await(key, Duration.ofMillis(50));

        ExecutionException e = assertThrows(ExecutionException.class, () -> future.get(2, TimeUnit.SECONDS));
        assertTrue(e.getCause() instanceof TimeoutException);
    }

    @Test
    void cancelIsIdempotent() {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        MatchKey key = new MatchKey(VEHICLE, 77, 1L);
        correlator.await(key, Duration.ofSeconds(5));

        correlator.cancel(key);
        correlator.cancel(key); // must not throw
    }

    @Test
    void cancelOnAKeyNeverRegisteredIsANoOp() {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        correlator.cancel(new MatchKey(VEHICLE, 77, 42L)); // must not throw
    }

    @Test
    void aRepeatAwaitOnALiveKeyFailsLoudlyInsteadOfClobberingTheFirstWaiter() throws Exception {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        CommandAck ack = ackFor(MavCmd.MAV_CMD_DO_SET_MODE, 1, 1);
        MatchKey key = new MatchKey(VEHICLE, 77, ack.command().value());
        CompletableFuture<Frame> first = correlator.await(key, Duration.ofSeconds(5));

        assertThrows(IllegalStateException.class, () -> correlator.await(key, Duration.ofSeconds(5)));

        // the first waiter must still be the one registered -- prove it still receives the match,
        // i.e. the rejected second call did not silently steal or clear the registration.
        Frame ackFrame = frameOf(VEHICLE, 77, ack);
        correlator.offer(ackFrame);
        assertSame(ackFrame, first.get(1, TimeUnit.SECONDS));
    }

    @Test
    void awaitAfterANaturalCompletionForTheSameKeyIsAllowedAgain() throws Exception {
        DefaultCorrelator correlator = new DefaultCorrelator(MavlinkSettings.defaults());
        CommandAck ack = ackFor(MavCmd.MAV_CMD_DO_SET_MODE, 1, 1);
        MatchKey key = new MatchKey(VEHICLE, 77, ack.command().value());

        CompletableFuture<Frame> first = correlator.await(key, Duration.ofSeconds(2));
        correlator.offer(frameOf(VEHICLE, 77, ack));
        first.get(1, TimeUnit.SECONDS);

        // the key was released on natural completion -- a fresh await for it must succeed, not throw.
        CompletableFuture<Frame> second = correlator.await(key, Duration.ofSeconds(2));
        correlator.offer(frameOf(VEHICLE, 77, ack));
        Frame delivered = second.get(1, TimeUnit.SECONDS);
        assertEquals(77, delivered.header().messageId());
    }

    private static CommandAck ackFor(MavCmd command, int targetSystem, int targetComponent) {
        return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_ACCEPTED)
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .build();
    }

    @Test
    void aMatchCompletesOnTheCallbackExecutorNotOnTheOfferingThread() throws Exception {
        java.util.ArrayDeque<Runnable> queued = new java.util.ArrayDeque<>();
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().callbackExecutor(queued::add).build();
        DefaultCorrelator correlator = new DefaultCorrelator(settings);
        CommandAck ack = ackFor(MavCmd.MAV_CMD_DO_SET_MODE, 1, 1);
        MatchKey key = new MatchKey(VEHICLE, 77, ack.command().value());
        CompletableFuture<Frame> future = correlator.await(key, Duration.ofSeconds(30));

        correlator.offer(frameOf(VEHICLE, 77, ack));

        assertFalse(future.isDone(), "completion is handed to the host's executor, not run inline");
        assertTrue(queued.size() >= 1);
        queued.pollLast().run(); // the match, queued after the timeout task registered by await
        assertTrue(future.isDone());
        assertFalse(future.isCompletedExceptionally());
    }

    private static Frame frameOf(SysId origin, int messageId, Object payload) {
        FrameHeader header = new FrameHeader(2, 0, origin, new CompId(1), messageId, 0, 0, false, SignatureStatus.UNSIGNED);
        return new Frame(header, payload, LINK, LinkPeer.NONE, Instant.now());
    }
}
