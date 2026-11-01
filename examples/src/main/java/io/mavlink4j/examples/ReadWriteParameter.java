/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import io.mavlink4j.PeerId;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.service.ParameterOutcome;
import io.mavlink4j.service.ParameterService;
import io.mavlink4j.testkit.FakeVehicle;

import io.dronefleet.mavlink.common.MavParamType;
import io.dronefleet.mavlink.common.ParamSet;

import java.io.PrintStream;
import java.time.Duration;
import java.util.concurrent.TimeUnit;

/**
 * Reads {@code MAV_SYSID}, then writes {@code RC1_MIN} and reports {@code MISMATCH} when the vehicle
 * stored a different value (a clamp).
 *
 * <p>Arguments: {@code [--fake] [--bind host:port] [--sysid N] [--timeout seconds] [--read NAME]
 * [--write NAME=VALUE]} (defaults {@code MAV_SYSID} and {@code RC1_MIN=900}; the write is a
 * {@code REAL32}). Without {@code --fake} it writes to the real vehicle's parameter; exit 0 when the read is
 * {@code OK} and the write is {@code OK} or {@code MISMATCH}, 1 on {@code NO_REPLY}. Under {@code --fake}
 * the vehicle clamps {@code RC1_MIN} to 1100 and the run is only green if that {@code MISMATCH} shows.
 */
public final class ReadWriteParameter {

    private static final float FAKE_SYSID_VALUE = 21f;
    private static final float FAKE_CLAMP = 1100f;
    private static final Duration FAKE_SET_WAIT = Duration.ofSeconds(5);

    private ReadWriteParameter() {
    }

    public static void main(String[] args) {
        System.exit(run(args, System.out));
    }

    static int run(String[] args, PrintStream out) {
        return Runner.execute(out, () -> {
            Options options = Options.parse(args, "sysid", "read", "write");
            String readName = options.get("read", "MAV_SYSID");
            String[] write = options.get("write", "RC1_MIN=900").split("=", 2);
            if (write.length != 2) {
                throw new IllegalArgumentException("--write expects NAME=VALUE");
            }
            float requested = Float.parseFloat(write[1]);
            try (Endpoint endpoint = Endpoint.open(options, Signing.off());
                 FakeVehicle vehicle = options.fake() ? FakeVehicle.builder("127.0.0.1", endpoint.port()).sysid(21)
                         .parameter("MAV_SYSID", FAKE_SYSID_VALUE, MavParamType.MAV_PARAM_TYPE_INT32).build() : null) {
                PeerId target = endpoint.awaitPeer(Integer.parseInt(options.get("sysid", "-1")), options.timeout());
                ParameterService params = new ParameterService(
                        endpoint.session().sink(), endpoint.session().correlator(), endpoint.settings());
                long wait = options.timeout().toSeconds();

                ParameterOutcome read = params.read(target, readName).get(wait, TimeUnit.SECONDS);
                out.println("read " + readName + " -> " + read.status()
                        + (read.value() != null ? " value=" + read.value().value() : " " + read.detail()));

                Thread clamp = options.fake() ? clampOnSet(vehicle, write[0]) : null;
                ParameterOutcome written = params.write(target, write[0], requested, MavParamType.MAV_PARAM_TYPE_REAL32)
                        .get(wait, TimeUnit.SECONDS);
                if (clamp != null) {
                    clamp.join(FAKE_SET_WAIT.toMillis());
                }
                out.println("write " + write[0] + "=" + requested + " -> " + written.status()
                        + (written.status() == ParameterOutcome.Status.NO_REPLY ? "" : " (" + describe(written) + ")"));

                boolean readOk = read.ok();
                boolean writeAnswered = written.status() != ParameterOutcome.Status.NO_REPLY;
                boolean expectedClamp = !options.fake() || written.status() == ParameterOutcome.Status.MISMATCH;
                return readOk && writeAnswered && expectedClamp ? 0 : 1;
            }
        });
    }

    /**
     * The fake does not know {@code name}, so it stays silent on the {@code PARAM_SET}; this thread answers it
     * with a clamped {@code PARAM_VALUE}, as an autopilot with a range limit would.
     */
    private static Thread clampOnSet(FakeVehicle vehicle, String name) {
        Thread thread = new Thread(() -> {
            try {
                vehicle.awaitFrame(ParamSet.class, FAKE_SET_WAIT);
                vehicle.replyParamValue(name, FAKE_CLAMP, MavParamType.MAV_PARAM_TYPE_REAL32, 0, 1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }, "example-clamp");
        thread.setDaemon(true);
        thread.start();
        return thread;
    }

    private static String describe(ParameterOutcome outcome) {
        return outcome.ok() ? "vehicle stored " + outcome.value().value() : outcome.detail();
    }
}
