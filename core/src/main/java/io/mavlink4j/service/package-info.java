/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * L4 — one class per MAVLink microservice (command, parameter, heartbeat, manual control, message interval, capability). Depends on L0 and L3's ports, never on a concrete session.
 *
 * <p>Layer rule: a package never reaches two levels down; see {@code docs/architecture.md}.
 */
package io.mavlink4j.service;
