/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.service;

import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.MavResult;

/**
 * A {@code COMMAND_LONG}/{@code COMMAND_INT} exchange's terminal result. {@code mavResult} and
 * {@code ack} are {@code null} for {@link Status#NO_ACK} (nothing was ever heard); {@code ack} is
 * also {@code null} for the rare {@link Status#IN_PROGRESS}-by-extension-exhaustion case (the
 * last reply frame is deliberately not retained past classification — a caller wanting the raw
 * frame history should observe replies via a {@code Dispatcher} subscription instead).
 *
 * @param resultCode {@code CommandAck.resultParam2()} — an additional, command-specific result
 *                   parameter (e.g. which param of a rejected command was invalid); {@code -1}
 *                   when no ack was ever seen
 *
 * <p><b>Thread-safety:</b> Immutable and thread-safe.
 */
public record CommandResult(Status status, MavResult mavResult, int resultCode, CommandAck ack) {

    /** How the exchange ended; {@link #NO_ACK} is silence, the rest map {@code MAV_RESULT}. */
    public enum Status {
        ACCEPTED, TEMPORARILY_REJECTED, DENIED, UNSUPPORTED, FAILED, CANCELLED, IN_PROGRESS, NO_ACK
    }
}
