/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.dronefleet.mavlink.common.MavResult;

/**
 * What a {@link FakeVehicle} does when a scripted command arrives: send zero or more
 * {@code COMMAND_ACK}s. Use the factories for the common shapes, or implement it for anything else.
 */
@FunctionalInterface
public interface CommandReply {

    /**
     * Answers {@code request} through {@code vehicle}.
     *
     * @param vehicle the vehicle to reply from
     * @param request the command that arrived
     */
    void answer(FakeVehicle vehicle, CommandRequest request);

    /**
     * Acknowledges with {@code result}.
     *
     * @param result the result to send
     * @return a reply sending one {@code COMMAND_ACK}
     */
    static CommandReply ack(MavResult result) {
        return (vehicle, request) -> vehicle.replyAck(request.command(), result);
    }

    /**
     * Sends {@code IN_PROGRESS} with {@code progress}, then the final {@code result}.
     *
     * @param progress percent complete, 0..100
     * @param result the final result
     * @return a reply sending two acks
     */
    static CommandReply inProgressThen(int progress, MavResult result) {
        return (vehicle, request) -> {
            vehicle.replyAckInProgress(request.command(), progress);
            vehicle.replyAck(request.command(), result);
        };
    }

    /** @return a reply that stays silent, for exercising the caller's timeout and retry path */
    static CommandReply silent() {
        return (vehicle, request) -> { };
    }
}
