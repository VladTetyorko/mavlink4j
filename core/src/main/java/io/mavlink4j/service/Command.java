/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.service;

import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;

import java.util.Objects;

/**
 * One MAVLink command to send through {@link CommandService#send}: which {@link MavCmd}, and its
 * parameters, named instead of positional. Two wire shapes, picked by how the command is built:
 * <ul>
 *   <li>{@link #of(MavCmd)} — a {@code COMMAND_LONG}: seven float parameters, all {@code 0} until set.</li>
 *   <li>{@link #positioned} — a {@code COMMAND_INT}: the spec-preferred shape for anything with a
 *       coordinate (integer lat/lon scaled by 1e7, an explicit coordinate frame).</li>
 * </ul>
 * Values are immutable; each {@code paramN} returns a modified copy.
 */
public sealed interface Command {

    /** The command being sent. */
    MavCmd command();

    /** A {@code COMMAND_LONG} command with every parameter {@code 0}. */
    static Simple of(MavCmd command) {
        return new Simple(command, 0, 0, 0, 0, 0, 0, 0);
    }

    /**
     * A {@code COMMAND_INT} command at the given position, parameters 1..4 {@code 0}. {@code x}/{@code y}
     * are the wire integers (for global frames, degrees scaled by 1e7); {@code z} is in {@code frame}'s unit.
     *
     * @param frame the wire coordinate-frame enum ({@code MAV_FRAME}), not this library's frame envelope
     */
    static Positioned positioned(MavCmd command, MavFrame frame, int x, int y, float z) {
        return new Positioned(command, frame, 0, 0, 0, 0, x, y, z);
    }

    /** {@code COMMAND_LONG}: seven float parameters. */
    record Simple(MavCmd command, float param1, float param2, float param3, float param4,
                  float param5, float param6, float param7) implements Command {

        public Simple {
            Objects.requireNonNull(command, "command");
        }

        public Simple param1(float value) {
            return new Simple(command, value, param2, param3, param4, param5, param6, param7);
        }

        public Simple param2(float value) {
            return new Simple(command, param1, value, param3, param4, param5, param6, param7);
        }

        public Simple param3(float value) {
            return new Simple(command, param1, param2, value, param4, param5, param6, param7);
        }

        public Simple param4(float value) {
            return new Simple(command, param1, param2, param3, value, param5, param6, param7);
        }

        public Simple param5(float value) {
            return new Simple(command, param1, param2, param3, param4, value, param6, param7);
        }

        public Simple param6(float value) {
            return new Simple(command, param1, param2, param3, param4, param5, value, param7);
        }

        public Simple param7(float value) {
            return new Simple(command, param1, param2, param3, param4, param5, param6, value);
        }
    }

    /** {@code COMMAND_INT}: four float parameters, a coordinate frame and an integer position. */
    record Positioned(MavCmd command, MavFrame frame, float param1, float param2, float param3, float param4,
                      int x, int y, float z) implements Command {

        public Positioned {
            Objects.requireNonNull(command, "command");
            Objects.requireNonNull(frame, "frame");
        }

        public Positioned param1(float value) {
            return new Positioned(command, frame, value, param2, param3, param4, x, y, z);
        }

        public Positioned param2(float value) {
            return new Positioned(command, frame, param1, value, param3, param4, x, y, z);
        }

        public Positioned param3(float value) {
            return new Positioned(command, frame, param1, param2, value, param4, x, y, z);
        }

        public Positioned param4(float value) {
            return new Positioned(command, frame, param1, param2, param3, value, x, y, z);
        }
    }
}
