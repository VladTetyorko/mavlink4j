/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.transport;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class LinkIntakeTest {

    @Test
    void startsAtZero() {
        LinkIntake intake = new LinkIntake(0, 0);
        assertEquals(0, intake.datagramsReceived());
        assertEquals(0, intake.bytesReceived());
    }

    @Test
    void carriesRealCounts() {
        LinkIntake intake = new LinkIntake(412, 51_236);
        assertEquals(412, intake.datagramsReceived());
        assertEquals(51_236, intake.bytesReceived());
    }

    @Test
    void rejectsNegativeDatagramsReceived() {
        assertThrows(IllegalArgumentException.class, () -> new LinkIntake(-1, 0));
    }

    @Test
    void rejectsNegativeBytesReceived() {
        assertThrows(IllegalArgumentException.class, () -> new LinkIntake(0, -1));
    }
}
