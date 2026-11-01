/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import io.mavlink4j.VehicleKind;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.session.MessageFilter;
import io.mavlink4j.session.Subscription;
import io.mavlink4j.testkit.FakeVehicle;

import io.dronefleet.mavlink.minimal.Heartbeat;

import java.io.PrintStream;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Binds UDP (default 14550) and prints {@code (sysid, compid, VehicleKind, mode)} per heartbeat.
 *
 * <p>Arguments: {@code [--fake] [--bind host:port] [--count N] [--timeout seconds]}. {@code --count}
 * (default 2) is how many heartbeats to print before exiting 0; exit 1 if fewer arrive within
 * {@code --timeout} (default 10 s).
 */
public final class ListenHeartbeats {

    private static final String DEFAULT_COUNT = "2";

    private ListenHeartbeats() {
    }

    public static void main(String[] args) {
        System.exit(run(args, System.out));
    }

    static int run(String[] args, PrintStream out) {
        return Runner.execute(out, () -> {
            Options options = Options.parse(args, "count");
            int count = Integer.parseInt(options.get("count", DEFAULT_COUNT));
            try (Endpoint endpoint = Endpoint.open(options, Signing.off())) {
                CountDownLatch done = new CountDownLatch(count);
                AtomicInteger printed = new AtomicInteger();
                Subscription subscription = endpoint.session().dispatcher().subscribe(
                        MessageFilter.type(Heartbeat.class), frame -> {
                            if (printed.incrementAndGet() > count) {
                                return;
                            }
                            Heartbeat heartbeat = frame.as(Heartbeat.class);
                            out.println("heartbeat sysid=" + frame.header().system().value()
                                    + " compid=" + frame.header().component().value()
                                    + " kind=" + VehicleKind.of(heartbeat.type().value())
                                    + " mode=custom:" + heartbeat.customMode()
                                    + "/base:0x" + Integer.toHexString(heartbeat.baseMode().value()));
                            done.countDown();
                        });
                try (FakeVehicle vehicle = options.fake()
                        ? FakeVehicle.builder("127.0.0.1", endpoint.port()).sysid(21).build() : null) {
                    boolean all = done.await(options.timeout().toMillis(), TimeUnit.MILLISECONDS);
                    subscription.close();
                    out.println(all ? "received " + count + " heartbeat(s)"
                            : "error: only " + (count - done.getCount()) + " of " + count + " heartbeat(s) heard");
                    return all ? 0 : 1;
                }
            }
        });
    }
}
