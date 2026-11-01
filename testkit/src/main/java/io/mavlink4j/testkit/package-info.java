/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * Test doubles for code that talks MAVLink through mavlink4j, with no SITL, socket or hardware needed:
 * a scripted {@link io.mavlink4j.testkit.FakeVehicle}, in-memory links
 * ({@link io.mavlink4j.testkit.RecordingLink}, {@link io.mavlink4j.testkit.LoopbackLinkPair}) and
 * deterministic time ({@link io.mavlink4j.testkit.ManualInstantSource},
 * {@link io.mavlink4j.testkit.DirectTxScheduler}).
 */
package io.mavlink4j.testkit;
