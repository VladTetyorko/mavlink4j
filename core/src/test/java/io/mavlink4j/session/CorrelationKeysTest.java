/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.CompId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.FrameHeader;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.common.AutopilotVersion;
import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavParamType;
import io.dronefleet.mavlink.common.MavResult;
import io.dronefleet.mavlink.common.MissionAck;
import io.dronefleet.mavlink.common.MissionCount;
import io.dronefleet.mavlink.common.MissionRequest;
import io.dronefleet.mavlink.common.MissionRequestInt;
import io.dronefleet.mavlink.common.ParamValue;
import io.dronefleet.mavlink.minimal.Heartbeat;

import org.junit.jupiter.api.Test;

import java.time.Instant;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

/**
 * The D6 key table, tested directly rather than only through a live correlator — the whole risk this
 * table carries is that the request side and the reply side compute a different key for the same
 * exchange, which shows up as a silent hang, never as an assertion failure somewhere else. Every
 * test here therefore pairs a named factory against {@link CorrelationKeys#keyFor} on a frame.
 */
class CorrelationKeysTest {

    private static final SysId VEHICLE = new SysId(42);
    private static final SysId OTHER_VEHICLE = new SysId(43);

    @Test
    void commandAckKeyIsUnchangedFromTheSingleBranchItReplaced() {
        Frame frame = frameFrom(VEHICLE, CommandAck.builder()
                .command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
                .result(MavResult.MAV_RESULT_ACCEPTED)
                .build());

        assertEquals(CorrelationKeys.forCommandAck(VEHICLE, 400), CorrelationKeys.keyFor(frame));
        assertEquals(77, CorrelationKeys.keyFor(frame).messageId());
    }

    @Test
    void paramValueRoutesToTheSameKeyTheRequesterAwaitsUnder() {
        Frame frame = frameFrom(VEHICLE, paramValue("SR2_EXTRA2", 5f));

        assertEquals(CorrelationKeys.forParamValue(VEHICLE, "SR2_EXTRA2"), CorrelationKeys.keyFor(frame));
    }

    @Test
    void paramValueKeyIsPerVehicleAndPerName() {
        Frame frame = frameFrom(VEHICLE, paramValue("SR2_EXTRA2", 5f));

        assertNotEquals(CorrelationKeys.forParamValue(OTHER_VEHICLE, "SR2_EXTRA2"), CorrelationKeys.keyFor(frame));
        assertNotEquals(CorrelationKeys.forParamValue(VEHICLE, "SR2_EXTRA1"), CorrelationKeys.keyFor(frame));
    }

    /** The wire pads {@code param_id} to 16 chars with NULs; a name must survive the round trip. */
    @Test
    void aNulPaddedNameOffTheWireMatchesTheUnpaddedNameTheRequesterUsed() {
        Frame frame = frameFrom(VEHICLE, paramValue("BATT_CAPACITY\0\0\0", 5000f));

        assertEquals(CorrelationKeys.forParamValue(VEHICLE, "BATT_CAPACITY"), CorrelationKeys.keyFor(frame));
        assertEquals("BATT_CAPACITY", CorrelationKeys.normalizeParamId("BATT_CAPACITY\0\0\0"));
    }

    @Test
    void aNameLongerThanSixteenCharactersIsTruncatedTheWayTheWireWouldTruncateIt() {
        assertEquals("ABCDEFGHIJKLMNOP", CorrelationKeys.normalizeParamId("ABCDEFGHIJKLMNOPQRS"));
    }

    @Test
    void autopilotVersionNeedsNoDiscriminatorBecauseThereIsOnlyOnePerVehicle() {
        Frame frame = frameFrom(VEHICLE, AutopilotVersion.builder().flightSwVersion(0x04050700L).build());

        assertEquals(CorrelationKeys.forAutopilotVersion(VEHICLE), CorrelationKeys.keyFor(frame));
        assertEquals(CorrelationKeys.NO_DISCRIMINATOR, CorrelationKeys.keyFor(frame).discriminator());
    }

    /** The mission transfer protocol is not a core concern: its replies must never occupy a waiter. */
    @Test
    void missionTransferMessagesYieldNoKey() {
        assertNull(CorrelationKeys.keyFor(frameFrom(VEHICLE, MissionRequest.builder().seq(7).build())));
        assertNull(CorrelationKeys.keyFor(frameFrom(VEHICLE, MissionRequestInt.builder().seq(7).build())));
        assertNull(CorrelationKeys.keyFor(frameFrom(VEHICLE, MissionCount.builder().count(12).build())));
        assertNull(CorrelationKeys.keyFor(frameFrom(VEHICLE, MissionAck.builder().build())));
    }

    /** Telemetry is not a reply: a stream must reach subscribers via `Dispatcher` and never occupy a waiter. */
    @Test
    void anUncorrelatedMessageTypeYieldsNoKeyAtAll() {
        assertNull(CorrelationKeys.keyFor(frameFrom(VEHICLE, Heartbeat.builder().build())));
    }

    private static ParamValue paramValue(String name, float value) {
        return ParamValue.builder()
                .paramId(name)
                .paramValue(value)
                .paramType(MavParamType.MAV_PARAM_TYPE_INT8)
                .paramIndex(1)
                .paramCount(2)
                .build();
    }

    private static Frame frameFrom(SysId system, Object payload) {
        return new Frame(new FrameHeader(2, 0, system, new CompId(1), 0, 0, 0, false, SignatureStatus.UNSIGNED), payload,
                new LinkId("test"), new LinkPeer("127.0.0.1", 14550), Instant.EPOCH);
    }
}
