/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import java.time.Instant;
import java.util.Objects;

/**
 * One fully decoded MAVLink frame: header plus payload, stamped with where it came from.
 *
 * <p>{@code payload} is typed {@code Object} — a known wart, accepted deliberately (the
 * underlying library's ~500 generated message classes flow through this layer as-is rather than
 * being re-modelled as this module's own DTOs). {@link #is} / {@link #as} exist so callers rarely
 * need a raw {@code instanceof}/cast pair at every use site.
 *
 * <p><b>Thread-safety:</b> Immutable and thread-safe; the {@code payload} is a generated message object that callers must treat as read-only.
 */
public record Frame(FrameHeader header, Object payload, LinkId link, LinkPeer source, Instant receivedAt) {

    public Frame {
        Objects.requireNonNull(header, "header");
        Objects.requireNonNull(payload, "payload");
        Objects.requireNonNull(link, "link");
        Objects.requireNonNull(source, "source");
        Objects.requireNonNull(receivedAt, "receivedAt");
    }

    /** {@code true} if {@code payload} is an instance of {@code type}. */
    public boolean is(Class<?> type) {
        Objects.requireNonNull(type, "type");
        return type.isInstance(payload);
    }

    /**
     * @throws ClassCastException if {@code payload} is not an instance of {@code type} — check
     *                             with {@link #is} first when the type is not already known
     */
    public <T> T as(Class<T> type) {
        Objects.requireNonNull(type, "type");
        return type.cast(payload);
    }
}
