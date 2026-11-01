/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.sitl;

import io.dronefleet.mavlink.common.MavParamType;
import io.dronefleet.mavlink.common.MavProtocolCapability;
import io.mavlink4j.PeerId;
import io.mavlink4j.service.CapabilityReport;
import io.mavlink4j.service.ParameterOutcome;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Five checks of the library against real ArduPilot firmware. Each starts its own
 * vehicle so a failure cannot leak state into the next. They skip when no Docker daemon answers.
 */
class SitlSmokeTest {

    private static final int SYSID = 1;
    /** {@code MAV_AUTOPILOT_ARDUPILOTMEGA} on the wire. */
    private static final int MAV_AUTOPILOT_ARDUPILOTMEGA = 3;

    @BeforeAll
    static void needsDocker() {
        assumeTrue(SitlContainer.dockerAvailable(), "no Docker daemon");
    }

    @Test
    @Timeout(value = 5, unit = TimeUnit.MINUTES)
    void heartbeatIsHeard() throws Exception {
        try (SitlRig rig = SitlRig.open().withVehicle(SYSID)) {
            PeerId vehicle = rig.awaitVehicle(SYSID);

            var heartbeat = rig.session.peers().peer(vehicle).orElseThrow().heartbeat().orElseThrow();

            assertEquals(MAV_AUTOPILOT_ARDUPILOTMEGA, heartbeat.autopilot());
        }
    }

    @Test
    @Timeout(value = 5, unit = TimeUnit.MINUTES)
    void requestMessageYieldsAutopilotVersion() throws Exception {
        try (SitlRig rig = SitlRig.open().withVehicle(SYSID)) {
            PeerId vehicle = rig.awaitVehicle(SYSID);

            CapabilityReport report = rig.capabilities.requestAutopilotVersion(vehicle)
                    .get(SitlRig.CALL.toSeconds(), TimeUnit.SECONDS);

            assertEquals(CapabilityReport.Status.OK, report.status());
            assertTrue(report.firmwareVersion().startsWith("4."), report.firmwareVersion());
            assertTrue(report.supports(MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MAVLINK2));
        }
    }

    @Test
    @Timeout(value = 5, unit = TimeUnit.MINUTES)
    void parameterReadByNameReturnsTheConfiguredSysid() throws Exception {
        try (SitlRig rig = SitlRig.open().withVehicle(SYSID)) {
            PeerId vehicle = rig.awaitVehicle(SYSID);

            // ArduPilot 4.7 renamed SYSID_THISMAV to MAV_SYSID; the old spelling gets no answer.
            ParameterOutcome outcome = rig.parameters.read(vehicle, "MAV_SYSID")
                    .get(SitlRig.CALL.toSeconds(), TimeUnit.SECONDS);

            assertEquals(ParameterOutcome.Status.OK, outcome.status());
            assertEquals(SYSID, Math.round(outcome.value().value()));
        }
    }

    @Test
    @Timeout(value = 5, unit = TimeUnit.MINUTES)
    void parameterWriteOfAnOutOfRangeValueIsReportedAsMismatch() throws Exception {
        try (SitlRig rig = SitlRig.open().withVehicle(SYSID)) {
            PeerId vehicle = rig.awaitVehicle(SYSID);
            float tooBigForInt8 = 1000f;

            // LOG_BACKEND_TYPE is an INT8 on the vehicle, so whether it clamps or wraps, it cannot hold 1000.
            ParameterOutcome outcome = rig.parameters
                    .write(vehicle, "LOG_BACKEND_TYPE", tooBigForInt8, MavParamType.MAV_PARAM_TYPE_INT8)
                    .get(SitlRig.CALL.toSeconds(), TimeUnit.SECONDS);

            assertEquals(ParameterOutcome.Status.MISMATCH, outcome.status());
            assertNotEquals(tooBigForInt8, outcome.value().value());
        }
    }

    @Test
    @Timeout(value = 8, unit = TimeUnit.MINUTES)
    void twoVehiclesOnOneListenPortAreRoutedBySysid() throws Exception {
        try (SitlRig rig = SitlRig.open().withVehicle(1).withVehicle(2)) {
            PeerId first = rig.awaitVehicle(1);
            PeerId second = rig.awaitVehicle(2);

            ParameterOutcome one = rig.parameters.read(first, "MAV_SYSID").get(SitlRig.CALL.toSeconds(), TimeUnit.SECONDS);
            ParameterOutcome two = rig.parameters.read(second, "MAV_SYSID").get(SitlRig.CALL.toSeconds(), TimeUnit.SECONDS);

            assertEquals(1, Math.round(one.value().value()));
            assertEquals(2, Math.round(two.value().value()));
            assertNotEquals(rig.session.peers().peer(first).orElseThrow().address(),
                    rig.session.peers().peer(second).orElseThrow().address());
        }
    }
}
