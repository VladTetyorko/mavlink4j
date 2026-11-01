/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.config;

import org.junit.jupiter.api.Test;

import java.time.Duration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

/** Every default here must stay byte-identical to API.md's configuration table -- a mismatch is a real regression. */
class MavlinkSettingsTest {

    @Test
    void defaultsMatchTheFrozenTable() {
        MavlinkSettings settings = MavlinkSettings.defaults();

        assertEquals(Duration.ofSeconds(1), settings.heartbeatPeriod());
        assertEquals(Duration.ofSeconds(5), settings.peerTimeout());
        assertEquals(Duration.ofSeconds(2), settings.commandTimeout());
        assertEquals(2, settings.commandRetries());
        assertEquals(33, settings.rc().overrideHz());
        assertEquals(10, settings.rc().minOverrideHz());
        assertEquals(50, settings.rc().maxOverrideHz());
        assertEquals(3, settings.rc().releaseFrames());
        assertEquals(Duration.ofSeconds(5), settings.closeJoinTimeout());
        assertEquals(64, settings.maxResyncBuffers());
        assertEquals(256, settings.dispatchQueueCapacity());
        assertEquals(20, settings.commandMaxInProgressExtensions());
        assertEquals(Duration.ofMillis(200), settings.pollTimeout());
        assertEquals(2, settings.txPoolSize());
        assertEquals(8, settings.parameter().maxForeignReplies());
    }

    @Test
    void builderInjectsClockThreadFactoryAndCallbackExecutor() {
        java.time.InstantSource clock = java.time.InstantSource.fixed(java.time.Instant.EPOCH);
        java.util.concurrent.Executor direct = Runnable::run;
        java.util.concurrent.ThreadFactory factory = Thread::new;
        MavlinkSettings settings = MavlinkSettings.builder()
                .clock(clock).threadFactory(factory).callbackExecutor(direct).build();
        org.junit.jupiter.api.Assertions.assertSame(clock, settings.clock());
        org.junit.jupiter.api.Assertions.assertSame(factory, settings.threadFactory());
        org.junit.jupiter.api.Assertions.assertSame(direct, settings.callbackExecutor());
    }

    @Test
    void toBuilderRoundTripsEveryField() {
        MavlinkSettings base = MavlinkSettings.defaults();
        org.junit.jupiter.api.Assertions.assertEquals(base, base.toBuilder().build());
    }

    @Test
    void rejectsNonPositivePollTimeoutAndPoolSize() {
        assertThrows(IllegalArgumentException.class, () -> MavlinkSettings.builder().pollTimeout(Duration.ZERO).build());
        assertThrows(IllegalArgumentException.class, () -> MavlinkSettings.builder().txPoolSize(0).build());
    }

    @Test
    void withMaxResyncBuffersReturnsACopyLeavingEverythingElseUnchanged() {
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().maxResyncBuffers(128).build();
        assertEquals(128, settings.maxResyncBuffers());
        assertEquals(MavlinkSettings.defaults().heartbeatPeriod(), settings.heartbeatPeriod());
    }

    @Test
    void withCommandRetriesReturnsACopy() {
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().commandRetries(5).build();
        assertEquals(5, settings.commandRetries());
    }

    @Test
    void rejectsNonPositiveHeartbeatPeriod() {
        assertThrows(IllegalArgumentException.class, () -> MavlinkSettings.defaults().toBuilder().heartbeatPeriod(Duration.ZERO).build());
    }

    @Test
    void rejectsNegativeCommandRetries() {
        assertThrows(IllegalArgumentException.class, () -> MavlinkSettings.defaults().toBuilder().commandRetries(-1).build());
    }

    @Test
    void rejectsMaxResyncBuffersBelowOne() {
        assertThrows(IllegalArgumentException.class, () -> MavlinkSettings.defaults().toBuilder().maxResyncBuffers(0).build());
    }

    @Test
    void rcClampsOverrideHzIntoItsOwnBounds() {
        MavlinkSettings.Rc rc = new MavlinkSettings.Rc(999, 10, 50, 3);
        assertEquals(50, rc.clampedOverrideHz());
    }

    @Test
    void rcRejectsMaxBelowMin() {
        assertThrows(IllegalArgumentException.class, () -> new MavlinkSettings.Rc(33, 50, 10, 3));
    }
}
