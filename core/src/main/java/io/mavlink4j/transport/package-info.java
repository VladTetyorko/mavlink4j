/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * L1 — bytes in, bytes out over a {@code MavlinkLink} (UDP listen/target, TCP client, serial). Knows nothing about frames. May depend on L0 only.
 *
 * <p>Layer rule: a package never reaches two levels down; see {@code docs/architecture.md}.
 */
package io.mavlink4j.transport;
