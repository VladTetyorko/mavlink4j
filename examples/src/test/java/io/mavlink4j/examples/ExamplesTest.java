/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.examples;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.TimeUnit;
import java.util.function.BiFunction;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Runs every example's {@code main} body with {@code --fake}: exit code 0 and the key output line. */
@Timeout(value = 60, unit = TimeUnit.SECONDS)
class ExamplesTest {

    private static String run(BiFunction<String[], PrintStream, Integer> example, String... args) {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        int code = example.apply(args, new PrintStream(buffer, true, StandardCharsets.UTF_8));
        String output = buffer.toString(StandardCharsets.UTF_8);
        assertEquals(0, code, output);
        return output;
    }

    @Test
    void listenHeartbeatsPrintsTwoHeartbeats() {
        String out = run(ListenHeartbeats::run, "--fake");
        assertEquals(2, out.lines().filter(l -> l.startsWith("heartbeat sysid=21 ")).count(), out);
        assertTrue(out.contains("received 2 heartbeat(s)"), out);
    }

    @Test
    void armWithAckShowsAcceptedExtensionAndNoAck() {
        String out = run(ArmWithAck::run, "--fake");
        assertTrue(out.contains("ARM_DISARM -> ACCEPTED"), out);
        assertTrue(out.contains("NAV_TAKEOFF (IN_PROGRESS first, then final ack) -> ACCEPTED"), out);
        assertTrue(out.contains("DO_SET_MODE (vehicle stays silent) -> NO_ACK"), out);
    }

    @Test
    void armWithAckRefusesARealVehicleWithoutConfirmation() {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        assertEquals(2, ArmWithAck.run(new String[0], new PrintStream(buffer, true, StandardCharsets.UTF_8)));
    }

    @Test
    void readWriteParameterShowsMismatchOnAClamp() {
        String out = run(ReadWriteParameter::run, "--fake");
        assertTrue(out.contains("read MAV_SYSID -> OK value=21.0"), out);
        assertTrue(out.contains("write RC1_MIN=900.0 -> MISMATCH"), out);
    }

    @Test
    void twoVehiclesOneSocketDeliversEachCommandOnlyToItsVehicle() {
        String out = run(TwoVehiclesOneSocket::run, "--fake");
        assertTrue(out.contains("disarm -> sysid=41: ACCEPTED"), out);
        assertTrue(out.contains("disarm -> sysid=42: ACCEPTED"), out);
        assertEquals(2, out.lines().filter(l -> l.endsWith("leaked to the other: false")).count(), out);
    }

    @Test
    void signedLinkRejectsAnUnsignedFrameAndAReplay() {
        String out = run(SignedLink::run, "--fake");
        assertTrue(out.contains("rejected sysid=32 status=UNSIGNED reason=POLICY"), out);
        assertTrue(out.contains("rejected sysid=33 status=INVALID reason=REPLAY"), out);
        assertTrue(out.contains("signed original accepted: true"), out);
    }
}
