/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.PeerId;
import io.mavlink4j.transport.LinkId;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** {@link FrameSink#broadcastAll} is best-effort per link: one dead link never starves the others. */
class FrameSinkContractTest {

    private static final LinkId BAD = new LinkId("bad");
    private static final LinkId GOOD_A = new LinkId("good-a");
    private static final LinkId GOOD_B = new LinkId("good-b");

    @Test
    void broadcastAllReachesEveryLinkEvenWhenOneThrows() {
        List<LinkId> reached = new ArrayList<>();
        FrameSink sink = new FrameSink() {
            @Override
            public void send(Object message, PeerId target) {
                throw new UnsupportedOperationException();
            }

            @Override
            public void broadcast(Object message, LinkId link) {
                if (link.equals(BAD)) {
                    throw new IllegalStateException("listen link with no learned target");
                }
                reached.add(link);
            }

            @Override
            public Set<LinkId> links() {
                return new LinkedHashSet<>(List.of(GOOD_A, BAD, GOOD_B));
            }
        };

        sink.broadcastAll("hello");

        assertEquals(List.of(GOOD_A, GOOD_B), reached);
    }
}
