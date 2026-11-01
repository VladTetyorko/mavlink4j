/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import java.time.Duration;
import java.time.InstantSource;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

/**
 * One {@link SigningClock} per remote system id -- a signing timestamp is a per-vehicle fact (two
 * vehicles must never share a monotonic counter), never a process-wide one. {@link #inMemory} is
 * the built-in implementation: no persistence across a restart, since the library has no
 * storage to persist through. A host that needs replay protection to survive restarts supplies its
 * own registry whose clocks carry a durable {@code loadPersisted}/{@code persist} pair.
 */
public interface SigningClocks {

    /** Lazily creates (and thereafter reuses) the one clock for {@code sysid}. */
    SigningClock forSystem(int sysid);

    /**
     * A registry backed by an in-memory map: every clock starts from {@code loadPersisted = () -> 0}
     * (nothing was ever persisted) and its {@code persist} callback is a no-op. Fine for {@link
     * Signing#off()} and for a process that never restarts mid-flight; a real deployment's clocks
     * need a host-supplied registry with durable persistence.
     */
    static SigningClocks inMemory(InstantSource now, Duration persistInterval) {
        Objects.requireNonNull(now, "now");
        Objects.requireNonNull(persistInterval, "persistInterval");
        Map<Integer, SigningClock> clocks = new ConcurrentHashMap<>();
        return sysid -> clocks.computeIfAbsent(sysid,
                ignored -> new SigningClock(now, () -> 0L, discarded -> { }, persistInterval));
    }
}
