/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.SysId;

import java.util.Objects;

/**
 * The key a {@link Correlator} waiter is registered and matched under.
 *
 * @param system        the origin system of the expected reply — never the requester's own sysid,
 *                       and never {@code targetSystem}/{@code targetComponent} wire extension
 *                       fields, which are not reliably populated
 * @param messageId      the expected reply's wire message id
 * @param discriminator meaning is per-service: the command id for {@code COMMAND_ACK}, the item
 *                      seq for mission transfers, the session/seq for FTP (both future services)
 *
 * <p><b>Thread-safety:</b> Immutable and thread-safe.
 */
public record MatchKey(SysId system, int messageId, long discriminator) {

    public MatchKey {
        Objects.requireNonNull(system, "system");
    }
}
