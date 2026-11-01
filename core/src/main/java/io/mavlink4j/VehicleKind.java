/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j;

import java.util.Map;

/**
 * The vehicle family a MAVLink {@code HEARTBEAT.type} ({@code MAV_TYPE}) belongs to. Grouping the
 * wire enum into families is protocol knowledge; display names and "supported" judgements are the
 * caller's.
 *
 * <p>Four outcomes beyond the families, so a caller can refuse for the right reason:
 * <ul>
 *   <li>{@link #COPTER}, {@link #PLANE}, {@link #ROVER}, {@link #SUBMARINE} — a vehicle family
 *       (VTOL variants are {@code PLANE}; a surface boat is {@code ROVER}).</li>
 *   <li>{@link #OTHER_VEHICLE} — a defined airframe outside those families (airship, balloon,
 *       rocket, flapping-wing, kite, parafoil).</li>
 *   <li>{@link #NOT_A_VEHICLE} — an instrument or ground system heartbeating on the same link
 *       (gimbal, GCS, antenna tracker, camera, battery, ...).</li>
 *   <li>{@link #UNKNOWN} — a number this table has never seen; the only "no information" outcome.</li>
 * </ul>
 *
 * <p>Membership verified against MAVLink's {@code minimal.xml}/{@code common.xml} {@code MAV_TYPE} enum.
 */
public enum VehicleKind {

    /** Multirotor or helicopter — tri/quad/hexa/octo/deca/dodeca-rotor, coaxial, generic multirotor. */
    COPTER,

    /** Fixed-wing, including every VTOL variant. */
    PLANE,

    /** Ground rover or surface boat. */
    ROVER,

    /** Submarine ({@code MAV_TYPE} 12). */
    SUBMARINE,

    /** A defined airframe outside the families above. */
    OTHER_VEHICLE,

    /** Not a vehicle: an instrument or ground system on the same link. */
    NOT_A_VEHICLE,

    /** A {@code MAV_TYPE} number this table has never seen. */
    UNKNOWN;

    private static final Map<Integer, VehicleKind> MAV_TYPES = Map.ofEntries(
            Map.entry(2, COPTER),
            Map.entry(3, COPTER),
            Map.entry(4, COPTER),
            Map.entry(13, COPTER),
            Map.entry(14, COPTER),
            Map.entry(15, COPTER),
            Map.entry(29, COPTER),
            Map.entry(35, COPTER),
            Map.entry(43, COPTER),
            Map.entry(1, PLANE),
            Map.entry(19, PLANE),
            Map.entry(20, PLANE),
            Map.entry(21, PLANE),
            Map.entry(22, PLANE),
            Map.entry(23, PLANE),
            Map.entry(24, PLANE),
            Map.entry(25, PLANE),
            Map.entry(10, ROVER),
            Map.entry(11, ROVER),
            Map.entry(12, SUBMARINE),
            Map.entry(7, OTHER_VEHICLE),
            Map.entry(8, OTHER_VEHICLE),
            Map.entry(9, OTHER_VEHICLE),
            Map.entry(16, OTHER_VEHICLE),
            Map.entry(17, OTHER_VEHICLE),
            Map.entry(28, OTHER_VEHICLE),
            Map.entry(5, NOT_A_VEHICLE),
            Map.entry(6, NOT_A_VEHICLE),
            Map.entry(18, NOT_A_VEHICLE),
            Map.entry(26, NOT_A_VEHICLE),
            Map.entry(27, NOT_A_VEHICLE),
            Map.entry(30, NOT_A_VEHICLE),
            Map.entry(31, NOT_A_VEHICLE),
            Map.entry(32, NOT_A_VEHICLE),
            Map.entry(33, NOT_A_VEHICLE),
            Map.entry(34, NOT_A_VEHICLE),
            Map.entry(36, NOT_A_VEHICLE),
            Map.entry(37, NOT_A_VEHICLE),
            Map.entry(38, NOT_A_VEHICLE),
            Map.entry(39, NOT_A_VEHICLE),
            Map.entry(40, NOT_A_VEHICLE),
            Map.entry(41, NOT_A_VEHICLE),
            Map.entry(42, NOT_A_VEHICLE),
            Map.entry(44, NOT_A_VEHICLE),
            Map.entry(45, NOT_A_VEHICLE));

    /**
     * @param mavType raw {@code HEARTBEAT.type} wire value
     * @return the family for this number, or {@link #UNKNOWN} if it is not in the table — never a guess
     */
    public static VehicleKind of(int mavType) {
        return MAV_TYPES.getOrDefault(mavType, UNKNOWN);
    }
}
