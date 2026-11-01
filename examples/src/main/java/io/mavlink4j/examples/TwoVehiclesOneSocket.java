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
import io.mavlink4j.session.Peer;
import io.mavlink4j.testkit.CommandReply;
import io.mavlink4j.testkit.FakeVehicle;

import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavResult;

import java.io.PrintStream;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Two vehicles share one {@code UdpListenLink}; a targeted command to each is addressed to that vehicle's own
 * UDP source address and reaches only it.
 *
 * <p>Arguments: {@code [--fake] [--bind host:port] [--sysids A,B] [--timeout seconds]}. Without
 * {@code --fake} it sends a disarm to each of the two systems (default {@code 1,2}) and prints the address
 * each was heard from and its result; exit 0 if both answer. Under {@code --fake} it also proves the other
 * vehicle did not receive the command.
 */
public final class TwoVehiclesOneSocket {

    private static final Duration NOT_DELIVERED_WAIT = Duration.ofMillis(300);
    private static final Duration DELIVERED_WAIT = Duration.ofSeconds(5);
    private static final int[] FAKE_SYSIDS = {41, 42};

    private TwoVehiclesOneSocket() {
    }

    public static void main(String[] args) {
        System.exit(run(args, System.out));
    }

    static int run(String[] args, PrintStream out) {
        return Runner.execute(out, () -> {
            Options options = Options.parse(args, "sysids");
            int[] sysids = options.fake() ? FAKE_SYSIDS : parse(options.get("sysids", "1,2"));
            List<FakeVehicle> vehicles = new ArrayList<>();
            try (Endpoint endpoint = Endpoint.open(options, Signing.off())) {
                try {
                    if (options.fake()) {
                        for (int sysid : sysids) {
                            vehicles.add(FakeVehicle.builder("127.0.0.1", endpoint.port()).sysid(sysid)
                                    .onCommand(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM,
                                            CommandReply.ack(MavResult.MAV_RESULT_ACCEPTED)).build());
                        }
                    }
                    return exercise(options, endpoint, sysids, vehicles, out);
                } finally {
                    vehicles.forEach(FakeVehicle::close);
                }
            }
        });
    }

    private static int exercise(Options options, Endpoint endpoint, int[] sysids, List<FakeVehicle> vehicles,
                                PrintStream out) throws Exception {
        List<PeerId> peers = new ArrayList<>();
        for (int sysid : sysids) {
            peers.add(endpoint.awaitPeer(sysid, options.timeout()));
        }
        for (PeerId id : peers) {
            Peer peer = endpoint.session().peers().peer(id).orElseThrow();
            out.println("vehicle sysid=" + id.system().value() + " heard from "
                    + peer.address().host() + ":" + peer.address().port());
        }
        CommandService commands = new CommandService(
                endpoint.session().sink(), endpoint.session().correlator(), endpoint.settings());
        boolean ok = true;
        for (int i = 0; i < peers.size(); i++) {
            vehicles.forEach(FakeVehicle::clearReceived);
            CommandResult result = commands
                    .send(peers.get(i), Command.of(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM).param1(0f))
                    .get(options.timeout().toSeconds(), TimeUnit.SECONDS);
            out.println("disarm -> sysid=" + peers.get(i).system().value() + ": " + result.status());
            ok &= result.status() == CommandResult.Status.ACCEPTED;
            if (options.fake()) {
                ok &= deliveredOnlyTo(vehicles, i, out);
            }
        }
        return ok ? 0 : 1;
    }

    private static boolean deliveredOnlyTo(List<FakeVehicle> vehicles, int target, PrintStream out)
            throws InterruptedException {
        boolean reached = received(vehicles.get(target), DELIVERED_WAIT);
        boolean leaked = false;
        for (int i = 0; i < vehicles.size(); i++) {
            if (i != target) {
                leaked |= received(vehicles.get(i), NOT_DELIVERED_WAIT);
            }
        }
        out.println("  reached sysid=" + vehicles.get(target).id().system().value() + ": " + reached
                + ", leaked to the other: " + leaked);
        return reached && !leaked;
    }

    private static boolean received(FakeVehicle vehicle, Duration wait) throws InterruptedException {
        try {
            vehicle.awaitFrame(CommandLong.class, wait);
            return true;
        } catch (AssertionError none) {
            return false;
        }
    }

    private static int[] parse(String csv) {
        String[] parts = csv.split(",");
        if (parts.length != 2) {
            throw new IllegalArgumentException("--sysids expects two ids, e.g. 1,2");
        }
        return new int[] {Integer.parseInt(parts[0].trim()), Integer.parseInt(parts[1].trim())};
    }
}
