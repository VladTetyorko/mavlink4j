/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.transport;

import java.io.IOException;

/**
 * The byte-level seam {@link SerialLink} polls/writes through. Public so a host can plug in any
 * serial backend and a test can fake a port without loading a native library, exactly like
 * {@link MavlinkLink} itself lets a test fake a whole link.
 *
 * <p><b>Implementer contract:</b> {@link #read} is called from one reader thread and must never
 * block past its timeout; {@link #write} may be called from several threads and must be safe for
 * that (or be serialized by the caller's {@link SerialLink}); {@link #close} is idempotent and
 * unblocks a pending read.
 *
 * <p>Not a bare {@link java.io.InputStream}/{@link java.io.OutputStream} pair: jSerialComm has no
 * {@code Socket.setSoTimeout}-equivalent on the stream it hands back — the read timeout is a
 * property of the underlying {@code SerialPort} object, set via {@code setComPortTimeouts}, not of
 * any one read call. {@link #read} threads the timeout through explicitly instead, so {@link
 * SerialLink#poll} can honor a fresh {@link java.time.Duration} on every call the same way {@code
 * TcpClientLink#poll} does with {@code Socket#setSoTimeout}.
 */
public interface SerialChannel extends AutoCloseable {

    /** The stable port name this channel talks to, e.g. {@code "/dev/ttyUSB0"} or {@code "COM3"}. */
    String portDescriptor();

    /**
     * Reads up to {@code buffer.length} bytes, waiting at most {@code timeoutMillis} for at least
     * one to arrive.
     *
     * @return the number of bytes read, {@code 0} on timeout with nothing available (jSerialComm's
     *         semi-blocking read contract — never blocks past {@code timeoutMillis}), or a
     *         negative value if the underlying port has reached end-of-stream (unplugged mid-read)
     */
    int read(byte[] buffer, int timeoutMillis) throws IOException;

    /** Writes {@code len} bytes starting at {@code off} and flushes. Not required to be thread-safe. */
    void write(byte[] data, int off, int len) throws IOException;

    /** Idempotent; never throws. */
    @Override
    void close();
}
