/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import java.util.Optional;

/**
 * The sysid&rarr;key lookup the library cannot own: which 32-byte secret belongs to which
 * vehicle is the host application's knowledge (a key store, a device registry), so the host supplies
 * the implementation and the library only consumes the interface. See the
 * <a href="https://mavlink.io/en/guide/message_signing.html">message signing spec</a>.
 *
 * <p><b>Implementer contract:</b> {@link #keyFor} is called on the receive and send paths for every
 * frame, so it must be fast, non-blocking and thread-safe, and it must never throw; return
 * {@link Optional#empty()} when no key is known (the frame is then treated as unverifiable, not
 * invalid). A returned array is read, never retained or mutated by the library.
 */
public interface SigningKeyResolver {

    /** @return the 32-byte MAVLink 2 signing key for {@code sysid}, or empty if none is known yet */
    Optional<byte[]> keyFor(int sysid);

    /** No-op constant, not a null parameter: "signing is off" is this, never {@code null}. */
    SigningKeyResolver NONE = sysid -> Optional.empty();
}
