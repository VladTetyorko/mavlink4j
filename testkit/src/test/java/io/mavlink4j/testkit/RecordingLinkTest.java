/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkPeer;

import org.junit.jupiter.api.Test;

import java.time.Duration;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

class RecordingLinkTest {

    private static final LinkPeer PEER = new LinkPeer("127.0.0.1", 5760);

    @Test
    void recordsExactlyTheSlicesSent() {
        RecordingLink link = new RecordingLink("rec", true, PEER);
        link.send(new byte[] {9, 1, 2, 3, 9}, 1, 3, PEER);
        link.send(new byte[] {7}, 0, 1, PEER);

        assertEquals(2, link.sent().size());
        assertArrayEquals(new byte[] {1, 2, 3}, link.sent().get(0));
        assertArrayEquals(new byte[] {7}, link.sent().get(1));
        assertEquals(PEER, link.defaultTarget());
    }

    @Test
    void injectedBytesComeBackFromPollAndAnEmptyPollTimesOutToNull() {
        RecordingLink link = new RecordingLink("rec", false, PEER);
        assertNull(link.poll(Duration.ofMillis(10)));

        link.inject(new byte[] {4, 5}, PEER);
        ByteChunk chunk = link.poll(Duration.ofMillis(100));
        assertEquals(2, chunk.length());
        assertEquals(PEER, chunk.source());

        link.close();
        link.inject(new byte[] {1}, PEER);
        assertNull(link.poll(Duration.ofMillis(10)), "a closed link polls null");
    }
}
