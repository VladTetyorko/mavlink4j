/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import io.mavlink4j.PeerId;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.service.Command;
import io.mavlink4j.service.CommandResult;
import io.mavlink4j.service.CommandService;
import io.mavlink4j.testkit.CommandReply;
import io.mavlink4j.testkit.FakeVehicle;

import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavResult;

import java.io.PrintStream;
import java.util.concurrent.TimeUnit;

/**
 * Sends {@code COMMAND_LONG} arm and prints the {@link CommandResult}, including the
 * {@code IN_PROGRESS} extension and {@code NO_ACK}.
 *
 * <p>Arguments: {@code [--fake] [--bind host:port] [--sysid N] [--timeout seconds] [--confirm-arm]}.
 * Without {@code --fake} the example really arms the vehicle, so it refuses to run without
 * {@code --confirm-arm}; it targets {@code --sysid} (default: the first vehicle heard) and exits 0
 * only if the vehicle accepts. Under {@code --fake} the vehicle accepts arm, answers takeoff with
 * {@code IN_PROGRESS} then {@code ACCEPTED}, and never answers set-mode (so it ends {@code NO_ACK}).
 */
public final class ArmWithAck {

    private static final float ARM = 1f;

    private ArmWithAck() {
    }

    public static void main(String[] args) {
        System.exit(run(args, System.out));
    }

    static int run(String[] args, PrintStream out) {
        return Runner.execute(out, () -> {
            Options options = Options.parse(args, "sysid", "confirm-arm");
            if (!options.fake() && !options.flag("confirm-arm")) {
                out.println("error: this arms a real vehicle; pass --confirm-arm to proceed");
                return 2;
            }
            try (Endpoint endpoint = Endpoint.open(options, Signing.off());
                 FakeVehicle vehicle = options.fake() ? scripted(endpoint.port()) : null) {
                PeerId target = endpoint.awaitPeer(Integer.parseInt(options.get("sysid", "-1")), options.timeout());
                CommandService commands = new CommandService(
                        endpoint.session().sink(), endpoint.session().correlator(), endpoint.settings());
                long wait = options.timeout().toSeconds();

                CommandResult arm = commands.send(target, Command.of(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM).param1(ARM))
                        .get(wait, TimeUnit.SECONDS);
                print(out, "ARM_DISARM", arm);
                if (!options.fake()) {
                    return arm.status() == CommandResult.Status.ACCEPTED ? 0 : 1;
                }
                CommandResult takeoff = commands.send(target, Command.of(MavCmd.MAV_CMD_NAV_TAKEOFF))
                        .get(wait, TimeUnit.SECONDS);
                print(out, "NAV_TAKEOFF (IN_PROGRESS first, then final ack)", takeoff);
                CommandResult mode = commands.send(target, Command.of(MavCmd.MAV_CMD_DO_SET_MODE).param1(1f))
                        .get(wait, TimeUnit.SECONDS);
                print(out, "DO_SET_MODE (vehicle stays silent)", mode);
                boolean expected = arm.status() == CommandResult.Status.ACCEPTED
                        && takeoff.status() == CommandResult.Status.ACCEPTED
                        && mode.status() == CommandResult.Status.NO_ACK;
                return expected ? 0 : 1;
            }
        });
    }

    private static FakeVehicle scripted(int port) throws java.io.IOException {
        return FakeVehicle.builder("127.0.0.1", port).sysid(21)
                .onCommand(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, CommandReply.ack(MavResult.MAV_RESULT_ACCEPTED))
                .onCommand(MavCmd.MAV_CMD_NAV_TAKEOFF, CommandReply.inProgressThen(40, MavResult.MAV_RESULT_ACCEPTED))
                .build();
    }

    private static void print(PrintStream out, String what, CommandResult result) {
        out.println(what + " -> " + result.status() + " (mavResult=" + result.mavResult() + ")");
    }
}
