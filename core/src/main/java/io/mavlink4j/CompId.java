/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j;

/**
 * A MAVLink component id — one software/hardware component within a system (autopilot, camera,
 * gimbal, a ground controller's own commanding component, ...). Valid range is 1..255 per
 * the wire format, same rationale as {@link SysId}.
 */
public record CompId(int value) {

    public CompId {
        if (value < 1 || value > 255) {
            throw new IllegalArgumentException("CompId must be 1..255, got " + value);
        }
    }
}
