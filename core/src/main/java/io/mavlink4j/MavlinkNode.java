/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j;

import java.util.Objects;

/**
 * This node's own MAVLink identity — who a {@code MavlinkSession} presents itself as.
 *
 * <p><b>Thread-safety:</b> Immutable and thread-safe.
 */
public record MavlinkNode(SysId system, CompId component) {

    public MavlinkNode {
        Objects.requireNonNull(system, "system");
        Objects.requireNonNull(component, "component");
    }

    /** The conventional GCS identity: sysid 255, compid 190 (QGC's own default). */
    public static MavlinkNode groundStation() {
        return new MavlinkNode(new SysId(255), new CompId(190));
    }
}
