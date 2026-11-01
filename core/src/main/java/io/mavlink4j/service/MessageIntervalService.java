/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.service;

import io.mavlink4j.PeerId;

import io.dronefleet.mavlink.common.MavCmd;

import java.time.Duration;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

/**
 * {@code MAV_CMD_SET_MESSAGE_INTERVAL} / {@code MAV_CMD_REQUEST_MESSAGE} — both plain
 * {@code COMMAND_LONG} exchanges, so this class is a thin, named convenience over
 * {@link CommandService} rather than a service with any state or ack logic of its own: the ack
 * cycle is implemented exactly once.
 *
 * <h2>The one easy-to-get-wrong number</h2>
 * {@code MAV_CMD_SET_MESSAGE_INTERVAL}'s {@code param2} is the interval in <b>microseconds</b>, not
 * milliseconds or Hz — an ArduPilot/PX4-wide convention this class exists partly to stop every
 * caller from re-deriving by hand. {@link #setMessageInterval} takes a {@link Duration} and converts.
 */
public final class MessageIntervalService {

    /** {@code param2} value meaning "stop streaming this message" — spec convention, not a magic literal callers should redeclare. */
    public static final float DISABLE_STREAM = -1f;

    /** {@code param2} value meaning "resume this message's default/recommended rate." */
    public static final float DEFAULT_RATE = 0f;

    private final CommandService commandService;

    public MessageIntervalService(CommandService commandService) {
        this.commandService = Objects.requireNonNull(commandService, "commandService");
    }

    /**
     * Requests {@code messageId} be streamed at {@code interval}. {@code param2} is sent in
     * microseconds — {@code Duration.ofSeconds(1)} (1 Hz) becomes {@code 1_000_000}.
     *
     * @throws IllegalArgumentException if {@code interval} is negative — use {@link #disableMessage}
     *                                   to stop a stream, rather than relying on a negative Duration
     *                                   to mean the same thing
     */
    public CompletableFuture<CommandResult> setMessageInterval(PeerId target, int messageId, Duration interval) {
        Objects.requireNonNull(interval, "interval");
        if (interval.isNegative()) {
            throw new IllegalArgumentException("interval must not be negative -- call disableMessage() instead");
        }
        return sendSetInterval(target, messageId, microseconds(interval));
    }

    /** Stops {@code messageId} from streaming ({@code param2 = -1}, the spec's own "disable" sentinel). */
    public CompletableFuture<CommandResult> disableMessage(PeerId target, int messageId) {
        return sendSetInterval(target, messageId, DISABLE_STREAM);
    }

    /** One-shot request for {@code messageId} — {@code MAV_CMD_REQUEST_MESSAGE}, {@code param1 = messageId}. */
    public CompletableFuture<CommandResult> requestMessage(PeerId target, int messageId) {
        Objects.requireNonNull(target, "target");
        return commandService.send(target, Command.of(MavCmd.MAV_CMD_REQUEST_MESSAGE).param1(messageId));
    }

    private CompletableFuture<CommandResult> sendSetInterval(PeerId target, int messageId, float param2Micros) {
        Objects.requireNonNull(target, "target");
        return commandService.send(target,
                Command.of(MavCmd.MAV_CMD_SET_MESSAGE_INTERVAL).param1(messageId).param2(param2Micros));
    }

    private static float microseconds(Duration interval) {
        return interval.toNanos() / (float) TimeUnit.MICROSECONDS.toNanos(1);
    }
}
