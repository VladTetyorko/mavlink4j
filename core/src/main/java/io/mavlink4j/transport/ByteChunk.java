/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.transport;

import java.util.Objects;

/**
 * A slice of bytes read off a {@link MavlinkLink} in one {@link MavlinkLink#poll} call. Carries no
 * timestamp: a transport is pure I/O, and the codec stamps the frames it decodes with the session's own
 * clock.
 *
 * <p>{@code data} may be longer than {@code length} — an implementation is free to reuse a
 * backing buffer across polls rather than allocate exactly-sized arrays; callers must only ever
 * consider {@code data[0, length)} valid.
 * Ownership: the array belongs to the chunk and is not reused by the link after {@code poll} returns it, but a
 * consumer that keeps bytes past the decode should copy them — the record stays zero-copy on purpose. {@code source} is the sender for a datagram-shaped link,
 * or the link's one connected peer for a stream-shaped link.
 */
public record ByteChunk(byte[] data, int length, LinkPeer source) {

    public ByteChunk {
        Objects.requireNonNull(data, "data");
        Objects.requireNonNull(source, "source");
        if (length < 0 || length > data.length) {
            throw new IllegalArgumentException(
                    "length " + length + " out of bounds for data of size " + data.length);
        }
    }
}
