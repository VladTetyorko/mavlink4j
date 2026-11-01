/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * The one {@code MAV_TYPE} taxonomy. Pins the exact
 * numeric membership of every one of the four outcomes so a future edit that moves a number
 * between buckets (or reintroduces a second, locally-hand-maintained table elsewhere) shows up as
 * a failing assertion here rather than a silent drift.
 */
class VehicleKindTest {

    @Test
    void copterFamilyCoversEveryRotorcraftIncludingTheOnesNoPriorTableKnew() {
        for (int mavType : new int[] {2, 3, 4, 13, 14, 15, 29, 35, 43}) {
            assertEquals(VehicleKind.COPTER, VehicleKind.of(mavType), "mavType " + mavType);
        }
    }

    @Test
    void planeFamilyCoversFixedWingAndEveryVtolVariant() {
        assertEquals(VehicleKind.PLANE, VehicleKind.of(1));
        for (int mavType = 19; mavType <= 25; mavType++) {
            assertEquals(VehicleKind.PLANE, VehicleKind.of(mavType), "VTOL mavType " + mavType);
        }
    }

    @Test
    void roverFamilyNamesAGroundRoverAndASurfaceBoatIdentically() {
        assertEquals(VehicleKind.ROVER, VehicleKind.of(10));
        assertEquals(VehicleKind.ROVER, VehicleKind.of(11));
    }

    @Test
    void submarineIsItsOwnSlotNotFoldedIntoAnythingElse() {
        assertEquals(VehicleKind.SUBMARINE, VehicleKind.of(12));
    }

    @Test
    void unsupportedVehiclesAreRecognizedNotUnknown() {
        int[] unsupported = {7, 8, 9, 16, 17, 28};
        for (int mavType : unsupported) {
            assertEquals(VehicleKind.OTHER_VEHICLE, VehicleKind.of(mavType), "mavType " + mavType);
        }
    }

    @Test
    void nonVehicleInstrumentsAreClassifiedDistinctlyFromAnUnrecognizedVehicle() {
        int[] notVehicles = {5, 6, 18, 26, 27, 30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 41, 42, 44, 45};
        for (int mavType : notVehicles) {
            assertEquals(VehicleKind.NOT_A_VEHICLE, VehicleKind.of(mavType), "mavType " + mavType);
        }
    }

    @Test
    void aGenuinelyUnrecognizedNumberIsUnknown() {
        assertEquals(VehicleKind.UNKNOWN, VehicleKind.of(250));
        // MAV_TYPE_GENERIC (0) never told us anything specific about the airframe either.
        assertEquals(VehicleKind.UNKNOWN, VehicleKind.of(0));
    }

    @Test
    void unknownAndNotAVehicleAreDistinctOutcomes() {
        // The whole point of NOT_A_VEHICLE existing: a gimbal must never look like "unidentified".
        assertEquals(VehicleKind.NOT_A_VEHICLE, VehicleKind.of(26));
        assertEquals(VehicleKind.UNKNOWN, VehicleKind.of(9999));
        org.junit.jupiter.api.Assertions.assertNotEquals(VehicleKind.of(26), VehicleKind.of(9999));
    }
}
