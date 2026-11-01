/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.service;

import io.mavlink4j.PeerId;
import io.mavlink4j.codec.FrameSink;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.session.CorrelationKeys;
import io.mavlink4j.session.Correlator;
import io.mavlink4j.session.MatchKey;

import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.CommandInt;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavResult;
import io.dronefleet.mavlink.util.EnumValue;

import java.time.Duration;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * MAVLink's command microservice (<a href="https://mavlink.io/en/services/command.html">command protocol</a>): {@code COMMAND_LONG} and {@code COMMAND_INT}, awaited
 * as {@code COMMAND_ACK} matched on {@code (origin sysid, command id)} only. Built on
 * {@link RequestResponse} — the retry/extension mechanics live there; this class only supplies
 * Command's own policy: how to build the two payload shapes, how a {@code COMMAND_ACK} maps to a
 * verdict, and how a {@code MAV_RESULT} maps to an outcome.
 *
 * <h2>{@code confirmation} vs. {@code IN_PROGRESS}</h2>
 * A silent retry (no reply of any kind within {@link #timeout}) resends with {@code confirmation}
 * incremented — the vehicle's own way of telling a retransmit from a fresh command. A
 * reply of {@code MAV_RESULT_IN_PROGRESS} is not silence: it is data, so it never triggers a resend —
 * it only extends how long we keep waiting for a truly terminal result, capped at
 * {@link MavlinkSettings#commandMaxInProgressExtensions()} extensions so a vehicle that reports progress forever cannot
 * wedge a caller's future indefinitely. If that cap is reached, the exchange still completes — with
 * {@link CommandResult.Status#IN_PROGRESS} rather than silently hanging or reporting a bare timeout.
 *
 * <h2>{@code COMMAND_INT} has no {@code confirmation} field</h2>
 * The spec gives {@code COMMAND_INT} no retry-counter field at all — {@link #send}'s payload
 * builder for a {@link Command.Positioned} ignores the attempt number it is handed (accepted, not a bug).
 */
public final class CommandService {

    /** {@code COMMAND_ACK}'s wire message id — re-exported from {@link CorrelationKeys}, which owns the value. */
    public static final int COMMAND_ACK_MESSAGE_ID = CorrelationKeys.COMMAND_ACK_MESSAGE_ID;

    /**
     * Bounds how many times a single exchange re-extends its deadline in response to
     * {@code MAV_RESULT_IN_PROGRESS} before giving up and reporting {@link CommandResult.Status#IN_PROGRESS}
     * as terminal. Not a spec number — the spec places no bound on how many progress updates a
     * long-running command may send — so it comes from {@link MavlinkSettings#commandMaxInProgressExtensions()}.
     */
    private final int maxInProgressExtensions;

    private final RequestResponse requestResponse;
    private final Duration timeout;
    private final int retries;

    public CommandService(FrameSink sink, Correlator correlator, MavlinkSettings settings) {
        Objects.requireNonNull(settings, "settings");
        this.requestResponse = new RequestResponse(correlator, sink);
        this.timeout = settings.commandTimeout();
        this.retries = settings.commandRetries();
        this.maxInProgressExtensions = settings.commandMaxInProgressExtensions();
    }

    /**
     * Sends {@code command} to {@code target} — a {@code COMMAND_LONG} or a {@code COMMAND_INT}
     * depending on how it was built — and awaits its {@code COMMAND_ACK}. Never throws for a
     * protocol-level outcome (denial, timeout, ...) — those are values in the returned
     * {@link CommandResult}; the returned future fails only for a send-level fault (e.g. the target
     * has never been heard from — see {@code RoutingFrameSink}).
     *
     * <p>{@code COMMAND_INT} carries no {@code confirmation} field (the spec's own field list for
     * this message has none), so a retry of a {@link Command.Positioned} resends byte-identical
     * params — there is nothing else for the vehicle to key a retry-vs-fresh distinction on.
     */
    public CompletableFuture<CommandResult> send(PeerId target, Command command) {
        Objects.requireNonNull(target, "target");
        Objects.requireNonNull(command, "command");
        // Command is sealed (Simple | Positioned); an instanceof chain keeps this at the Java 17 floor.
        RequestResponse.AttemptPayload payload;
        if (command instanceof Command.Simple c) {
            payload = attempt -> CommandLong.builder()
                    .targetSystem(target.system().value())
                    .targetComponent(target.component().value())
                    .command(c.command())
                    .confirmation(attempt)
                    .param1(c.param1()).param2(c.param2()).param3(c.param3()).param4(c.param4())
                    .param5(c.param5()).param6(c.param6()).param7(c.param7())
                    .build();
        } else if (command instanceof Command.Positioned c) {
            payload = attempt -> CommandInt.builder()
                    .targetSystem(target.system().value())
                    .targetComponent(target.component().value())
                    .frame(c.frame())
                    .command(c.command())
                    .current(0)
                    .autocontinue(0)
                    .param1(c.param1()).param2(c.param2()).param3(c.param3()).param4(c.param4())
                    .x(c.x()).y(c.y()).z(c.z())
                    .build();
        } else {
            throw new IllegalArgumentException("unknown command kind " + command.getClass());
        }
        return exchange(target, command.command(), payload);
    }

    private CompletableFuture<CommandResult> exchange(PeerId target, MavCmd command, RequestResponse.AttemptPayload payload) {
        int commandId = EnumValue.of(command).value();
        MatchKey key = CorrelationKeys.forCommandAck(target.system(), commandId);
        AtomicBoolean sawInProgress = new AtomicBoolean(false);
        AtomicInteger extensionsUsed = new AtomicInteger();
        RequestResponse.ReplyClassifier classifier = reply -> classify(reply, sawInProgress, extensionsUsed);
        return requestResponse.exchange(target, key, timeout, retries, payload, classifier)
                .thenApply(CommandService::terminalOutcome)
                .exceptionallyCompose(error -> recoverFromTimeout(error, sawInProgress));
    }

    /** {@code null} = terminal; a positive {@link Duration} = keep waiting, no resend. */
    private Duration classify(Frame reply, AtomicBoolean sawInProgress, AtomicInteger extensionsUsed) {
        CommandAck ack = reply.as(CommandAck.class);
        MavResult result = ack.result().entry();
        if (result != MavResult.MAV_RESULT_IN_PROGRESS) {
            return null;
        }
        sawInProgress.set(true);
        if (extensionsUsed.incrementAndGet() > maxInProgressExtensions) {
            return null; // give up extending -- report this same (still IN_PROGRESS) reply as terminal
        }
        return timeout;
    }

    private static CommandResult terminalOutcome(Frame reply) {
        CommandAck ack = reply.as(CommandAck.class);
        MavResult result = ack.result().entry();
        return new CommandResult(statusFor(result), result, ack.resultParam2(), ack);
    }

    private static CommandResult.Status statusFor(MavResult result) {
        if (result == null) {
            // The library returns entry() == null when the wire value matches no declared MavResult
            // constant -- an unrecognized/future result code. Treat conservatively as a failure
            // rather than guessing at a more specific bucket.
            return CommandResult.Status.FAILED;
        }
        return switch (result) {
            case MAV_RESULT_ACCEPTED -> CommandResult.Status.ACCEPTED;
            case MAV_RESULT_TEMPORARILY_REJECTED -> CommandResult.Status.TEMPORARILY_REJECTED;
            case MAV_RESULT_DENIED -> CommandResult.Status.DENIED;
            case MAV_RESULT_UNSUPPORTED -> CommandResult.Status.UNSUPPORTED;
            case MAV_RESULT_FAILED -> CommandResult.Status.FAILED;
            case MAV_RESULT_IN_PROGRESS -> CommandResult.Status.IN_PROGRESS;
            case MAV_RESULT_CANCELLED -> CommandResult.Status.CANCELLED;
        };
    }

    private static CompletableFuture<CommandResult> recoverFromTimeout(Throwable error, AtomicBoolean sawInProgress) {
        if (unwrap(error) instanceof TimeoutException) {
            CommandResult outcome = sawInProgress.get()
                    ? new CommandResult(CommandResult.Status.IN_PROGRESS, MavResult.MAV_RESULT_IN_PROGRESS, -1, null)
                    : new CommandResult(CommandResult.Status.NO_ACK, null, -1, null);
            return CompletableFuture.completedFuture(outcome);
        }
        return CompletableFuture.failedFuture(error);
    }

    private static Throwable unwrap(Throwable t) {
        return (t instanceof java.util.concurrent.CompletionException && t.getCause() != null) ? t.getCause() : t;
    }
}
