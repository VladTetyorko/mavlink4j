/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * L3 — the {@code MavlinkSession} composition root: reader threads, peer directory, dispatcher, correlator, TX scheduler. May depend on L0–L2.
 *
 * <p>Layer rule: a package never reaches two levels down; see {@code docs/architecture.md}.
 */
package io.mavlink4j.session;
