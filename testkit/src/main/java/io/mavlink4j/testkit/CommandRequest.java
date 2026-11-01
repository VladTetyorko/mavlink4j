/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.dronefleet.mavlink.common.MavCmd;

import java.util.Objects;

/**
 * A command a {@link FakeVehicle} received.
 *
 * @param command the {@code MAV_CMD}
 * @param message the raw {@code CommandLong} or {@code CommandInt} payload, for asserting on parameters
 */
public record CommandRequest(MavCmd command, Object message) {

    /** Validates that both components are present. */
    public CommandRequest {
        Objects.requireNonNull(command, "command");
        Objects.requireNonNull(message, "message");
    }
}
