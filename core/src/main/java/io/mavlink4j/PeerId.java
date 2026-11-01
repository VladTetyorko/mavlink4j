/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j;

import java.util.Objects;

/**
 * The identity of a MAVLink peer: {@code (system, component)}. This is the only identity that is
 * ever valid to route or match on — never a transport address. One link can multiplex several
 * components (a companion computer relaying for itself and its autopilot), and NAT can move the
 * observed address between packets from the very same system.
 *
 * <p><b>Thread-safety:</b> Immutable and thread-safe.
 */
public record PeerId(SysId system, CompId component) {

    public PeerId {
        Objects.requireNonNull(system, "system");
        Objects.requireNonNull(component, "component");
    }
}
