/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.SysId;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;

import java.time.Instant;
import java.util.Objects;

/**
 * A frame {@link FrameReader} decoded but did not dispatch, because {@link SignatureVerifier}
 * itself refused it or because {@link SigningPolicy#accept(int, SignatureStatus)} did, given a
 * verifier result it chose not to accept. Reported via {@code
 * io.mavlink4j.session.MavlinkSession#onFrameRejected} -- never thrown, never logged above
 * {@code DEBUG} (a storm of bad frames must not flood the log at WARNING).
 *
 * <p>{@code sequence}, {@code source} and {@code receivedAt} are the link-layer facts of the datagram: the
 * session counts the sequence for link health even though the frame was refused (a policy drop is not
 * radio loss), and a consumer can tell which address an unsigned sender used.
 *
 * <p><b>Thread-safety:</b> Immutable and thread-safe; delivered on the receive thread, so a listener must not block.
 */
public record SignatureRejection(LinkId link, SysId system, CompId component, int messageId,
                                  SignatureStatus status, Reason reason, int sequence, LinkPeer source,
                                  Instant receivedAt) {

    public SignatureRejection {
        Objects.requireNonNull(link, "link");
        Objects.requireNonNull(system, "system");
        Objects.requireNonNull(component, "component");
        if (messageId < 0) {
            throw new IllegalArgumentException("messageId must be >= 0, got " + messageId);
        }
        Objects.requireNonNull(status, "status");
        Objects.requireNonNull(reason, "reason");
        Objects.requireNonNull(source, "source");
        Objects.requireNonNull(receivedAt, "receivedAt");
    }

    /** Why the frame was refused. {@link #POLICY} covers a status the verifier itself did not consider bad (e.g. UNSIGNED under REQUIRED) but the policy still refused. */
    public enum Reason {
        /** {@code Mavlink2Message#validateSignature(byte[])} returned {@code false}. */
        BAD_SIGNATURE,
        /** The signature timestamp did not increase for this {@code (sysid, compid, signingLinkId)} stream. */
        REPLAY,
        /** The signature timestamp was older than the configured tolerance behind the local clock. */
        STALE_TIMESTAMP,
        /** The verifier's own status was acceptable on its face; {@link SigningPolicy} refused it anyway. */
        POLICY
    }
}
