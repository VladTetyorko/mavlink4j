/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.dronefleet.mavlink.Mavlink2Message;
import io.dronefleet.mavlink.MavlinkMessage;

import java.time.Duration;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Implements MAVLink 2's own "Accepting signed packets" algorithm exactly (VERIFIED,
 * <a href="https://mavlink.io/en/guide/message_signing.html">mavlink.io message-signing spec</a>):
 * {@link Mavlink2Message#validateSignature(byte[])} checks only the HMAC -- replay and staleness
 * are this class's own responsibility, the library does not check timestamps at all.
 *
 * <p>Owns one small per-stream table, keyed by {@code (sysid, compid, signingLinkId)} -- the
 * spec's own monotonicity scope, distinct from this codebase's {@link
 * io.mavlink4j.transport.LinkId} (a carrier identity; the two merely share the word "link").
 * Bounded LRU at a configured {@code maxStreams} so a scanning/spoofing source sending many distinct
 * {@code (sysid, compid, signingLinkId)} tuples cannot grow it without limit -- the same discipline
 * {@link ResyncBuffer}'s own {@code maxResyncBuffers} bound uses.
 *
 * <p>Package-private: {@link FrameReader} is the only caller, exactly as {@link ResyncBuffer} is.
 */
final class SignatureVerifier {

    private final Signing signing;
    private final long staleToleranceTicks;
    private final Map<StreamKey, Long> lastTimestamps;

    SignatureVerifier(Signing signing, Duration staleSignatureTolerance, int maxStreams) {
        this.signing = Objects.requireNonNull(signing, "signing");
        Objects.requireNonNull(staleSignatureTolerance, "staleSignatureTolerance");
        this.staleToleranceTicks = SigningClock.ticksOf(staleSignatureTolerance);
        if (maxStreams < 1) {
            throw new IllegalArgumentException("maxStreams must be >= 1, got " + maxStreams);
        }
        this.lastTimestamps = new LinkedHashMap<>(16, 0.75f, true) {
            @Override
            protected boolean removeEldestEntry(Map.Entry<StreamKey, Long> eldest) {
                return size() > maxStreams;
            }
        };
    }

    /** {@code sysid}/{@code compid} are the message's own raw origin ids -- not yet validated into {@link io.mavlink4j.SysId}/{@link io.mavlink4j.CompId}. */
    Result verify(MavlinkMessage<?> message, int sysid, int compid) {
        if (!(message instanceof Mavlink2Message<?> v2) || !v2.isSigned()) {
            return new Result(SignatureStatus.UNSIGNED, null);
        }
        Optional<byte[]> key = signing.keys().keyFor(sysid);
        if (key.isEmpty()) {
            return new Result(SignatureStatus.UNVERIFIABLE, null);
        }
        if (!v2.validateSignature(key.get())) {
            return new Result(SignatureStatus.INVALID, SignatureRejection.Reason.BAD_SIGNATURE);
        }
        long timestamp = v2.getSignatureTimestamp();
        StreamKey stream = new StreamKey(sysid, compid, v2.getSignatureLinkId());
        Long last;
        synchronized (lastTimestamps) {
            last = lastTimestamps.get(stream);
        }
        if (last != null && timestamp <= last) {
            return new Result(SignatureStatus.INVALID, SignatureRejection.Reason.REPLAY);
        }
        SigningClock clock = signing.clocks().forSystem(sysid);
        // Freshness is measured against what this vehicle has already proven it sent, not against
        // the local wall clock -- a vehicle without a real-time clock can only advance its own
        // clock from frames the local side sends it (SigningClock#floor()).
        if (timestamp < clock.floor() - staleToleranceTicks) {
            return new Result(SignatureStatus.INVALID, SignatureRejection.Reason.STALE_TIMESTAMP);
        }
        synchronized (lastTimestamps) {
            lastTimestamps.put(stream, timestamp);
        }
        clock.observe(timestamp);
        return new Result(SignatureStatus.VALID, null);
    }

    /** {@code reason} is {@code null} except for {@link SignatureStatus#INVALID}. */
    record Result(SignatureStatus status, SignatureRejection.Reason reason) {
    }

    /** The spec's own per-stream monotonicity scope: one system's one component, on one signing link id. */
    private record StreamKey(int sysid, int compid, int signingLinkId) {
    }
}
