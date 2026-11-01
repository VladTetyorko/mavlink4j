/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

/**
 * A live {@link Dispatcher#subscribe} registration.
 *
 * <p><b>Thread-safety:</b> {@code close} may be called from any thread.
 */
public interface Subscription {

    /** Stops delivering frames to this subscription's handler. Idempotent. */
    void close();
}
