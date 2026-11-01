/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.serial;

import io.mavlink4j.transport.SerialLink;

import java.io.IOException;

/**
 * Production factory for {@link SerialLink}: opens a real serial port through jSerialComm, so the
 * core library itself stays free of any native dependency.
 */
public final class SerialLinks {

    private SerialLinks() {
    }

    /**
     * Opens {@code port} (e.g. {@code "/dev/ttyUSB0"} or {@code "COM3"}) at {@code baud}, 8N1, and
     * wraps it in a {@link SerialLink}. Dials a local device; nothing is bound or listened on.
     *
     * @throws IOException if the port is busy, missing or not permitted
     */
    public static SerialLink open(String port, int baud) throws IOException {
        return new SerialLink(JSerialCommChannel.open(port, baud));
    }
}
