/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.PeerUnreachableException;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.codec.SigningKeyResolver;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.LinkId;

import org.junit.jupiter.api.Test;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class RoutingFrameSinkTest {

    private static final LinkId LINK = new LinkId("routing-test");

    @Test
    void sendingToAPeerNeverHeardThrowsPeerUnreachableNamingThePeer() {
        RoutingFrameSink sink = new RoutingFrameSink(new DefaultPeerDirectory(MavlinkSettings.defaults()),
                new FrameWriter(new SysId(255), new CompId(1), Signing.off()), SigningKeyResolver.NONE, () -> Set.of(LINK));
        PeerId unheard = new PeerId(new SysId(9), new CompId(1));

        PeerUnreachableException e = assertThrows(PeerUnreachableException.class, () -> sink.send("anything", unheard));

        assertEquals(unheard, e.peer());
    }

    @Test
    void linksReflectsTheSessionsCurrentLinkSet() {
        java.util.concurrent.atomic.AtomicReference<Set<LinkId>> current = new java.util.concurrent.atomic.AtomicReference<>(Set.of());
        RoutingFrameSink sink = new RoutingFrameSink(new DefaultPeerDirectory(MavlinkSettings.defaults()),
                new FrameWriter(new SysId(255), new CompId(1), Signing.off()), SigningKeyResolver.NONE, current::get);

        assertEquals(Set.of(), sink.links());
        current.set(Set.of(LINK));
        assertEquals(Set.of(LINK), sink.links());
    }
}
