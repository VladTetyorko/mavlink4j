/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.config;

import java.time.Duration;
import java.time.InstantSource;
import java.util.Objects;
import java.util.concurrent.Executor;
import java.util.concurrent.ThreadFactory;

/**
 * Every value the MAVLink spec itself declines to pin (heartbeat rate, peer timeout, command
 * retry policy, ...), the module's own internal bounds (resync buffer cap, dispatch queue
 * capacity, poll interval, scheduler pool size) and the three runtime collaborators the library
 * must never hard-wire: the clock, the thread factory and the executor completions are delivered
 * on. One framework-free record — no {@code System.getenv}, no {@code System.getProperty}, no
 * Spring anywhere in this module.
 *
 * <p>Build one with {@link #builder()} (every field starts at its default) or derive one from an
 * existing instance with {@link #toBuilder()}; {@link #defaults()} is {@code builder().build()}.
 *
 * <h2>Runtime collaborators</h2>
 * <ul>
 *   <li>{@link #clock()} — the only source of "now" in the library. Transports stamp nothing; the
 *       codec stamps {@code Frame.receivedAt} from this clock, so a test advances time by hand.</li>
 *   <li>{@link #threadFactory()} — every thread the library starts (a reader per link, the
 *       TX scheduler pool, one drain thread per async subscriber) comes from it. The library names
 *       the thread it receives and marks it daemon; the factory chooses the kind (a Java 21 host
 *       may pass {@code Thread.ofVirtual().factory()}).</li>
 *   <li>{@link #callbackExecutor()} — where a future handed out by the library is completed, hence
 *       where its dependent continuations run. The default runs them directly on the completing
 *       thread (the reader thread, or the timer that fired a timeout); never {@code null}.</li>
 * </ul>
 */
public record MavlinkSettings(
        Duration heartbeatPeriod,
        Duration peerTimeout,
        Duration commandTimeout,
        int commandRetries,
        int commandMaxInProgressExtensions,
        Rc rc,
        Duration closeJoinTimeout,
        int maxResyncBuffers,
        int dispatchQueueCapacity,
        Duration pollTimeout,
        int txPoolSize,
        Parameter parameter,
        SigningSettings signing,
        LinkHealth linkHealth,
        PeerSettings peer,
        InstantSource clock,
        ThreadFactory threadFactory,
        Executor callbackExecutor) {

    /** The default thread source: plain OS threads; the library names them and marks them daemon. */
    static final ThreadFactory PLATFORM_THREADS = Thread::new;

    /** The default completion executor: run the continuation on the completing thread. */
    static final Executor DIRECT = Runnable::run;

    public MavlinkSettings {
        requirePositive(heartbeatPeriod, "heartbeatPeriod");
        requirePositive(peerTimeout, "peerTimeout");
        requirePositive(commandTimeout, "commandTimeout");
        if (commandRetries < 0) {
            throw new IllegalArgumentException("commandRetries must be >= 0, got " + commandRetries);
        }
        if (commandMaxInProgressExtensions < 0) {
            throw new IllegalArgumentException(
                    "commandMaxInProgressExtensions must be >= 0, got " + commandMaxInProgressExtensions);
        }
        Objects.requireNonNull(rc, "rc");
        requirePositive(closeJoinTimeout, "closeJoinTimeout");
        if (maxResyncBuffers < 1) {
            throw new IllegalArgumentException("maxResyncBuffers must be >= 1, got " + maxResyncBuffers);
        }
        if (dispatchQueueCapacity < 1) {
            throw new IllegalArgumentException("dispatchQueueCapacity must be >= 1, got " + dispatchQueueCapacity);
        }
        requirePositive(pollTimeout, "pollTimeout");
        if (txPoolSize < 1) {
            throw new IllegalArgumentException("txPoolSize must be >= 1, got " + txPoolSize);
        }
        Objects.requireNonNull(parameter, "parameter");
        Objects.requireNonNull(signing, "signing");
        Objects.requireNonNull(linkHealth, "linkHealth");
        Objects.requireNonNull(peer, "peer");
        Objects.requireNonNull(clock, "clock");
        Objects.requireNonNull(threadFactory, "threadFactory");
        Objects.requireNonNull(callbackExecutor, "callbackExecutor");
    }

    /** {@code builder().build()} — every field at its default. */
    public static MavlinkSettings defaults() {
        return builder().build();
    }

    /** A builder whose every field starts at its default. */
    public static Builder builder() {
        return new Builder(
                Duration.ofSeconds(1), Duration.ofSeconds(5), Duration.ofSeconds(2), 2, 20,
                Rc.defaults(), Duration.ofSeconds(5), 64, 256, Duration.ofMillis(200), 2, Parameter.defaults(),
                SigningSettings.defaults(), LinkHealth.defaults(), PeerSettings.defaults(),
                InstantSource.system(), PLATFORM_THREADS, DIRECT);
    }

    /** A builder pre-loaded with every field of this instance. */
    public Builder toBuilder() {
        return new Builder(heartbeatPeriod, peerTimeout, commandTimeout, commandRetries,
                commandMaxInProgressExtensions, rc, closeJoinTimeout, maxResyncBuffers, dispatchQueueCapacity,
                pollTimeout, txPoolSize, parameter, signing, linkHealth, peer, clock, threadFactory,
                callbackExecutor);
    }

    /** Mutable, single-use staging area for a {@link MavlinkSettings}; validation happens in {@link #build()}. */
    public static final class Builder {
        private Duration heartbeatPeriod;
        private Duration peerTimeout;
        private Duration commandTimeout;
        private int commandRetries;
        private int commandMaxInProgressExtensions;
        private Rc rc;
        private Duration closeJoinTimeout;
        private int maxResyncBuffers;
        private int dispatchQueueCapacity;
        private Duration pollTimeout;
        private int txPoolSize;
        private Parameter parameter;
        private SigningSettings signing;
        private LinkHealth linkHealth;
        private PeerSettings peer;
        private InstantSource clock;
        private ThreadFactory threadFactory;
        private Executor callbackExecutor;

        private Builder(Duration heartbeatPeriod, Duration peerTimeout, Duration commandTimeout, int commandRetries,
                        int commandMaxInProgressExtensions, Rc rc,
                        Duration closeJoinTimeout, int maxResyncBuffers, int dispatchQueueCapacity,
                        Duration pollTimeout, int txPoolSize, Parameter parameter, SigningSettings signing,
                        LinkHealth linkHealth, PeerSettings peer, InstantSource clock, ThreadFactory threadFactory,
                        Executor callbackExecutor) {
            this.heartbeatPeriod = heartbeatPeriod;
            this.peerTimeout = peerTimeout;
            this.commandTimeout = commandTimeout;
            this.commandRetries = commandRetries;
            this.commandMaxInProgressExtensions = commandMaxInProgressExtensions;
            this.rc = rc;
            this.closeJoinTimeout = closeJoinTimeout;
            this.maxResyncBuffers = maxResyncBuffers;
            this.dispatchQueueCapacity = dispatchQueueCapacity;
            this.pollTimeout = pollTimeout;
            this.txPoolSize = txPoolSize;
            this.parameter = parameter;
            this.signing = signing;
            this.linkHealth = linkHealth;
            this.peer = peer;
            this.clock = clock;
            this.threadFactory = threadFactory;
            this.callbackExecutor = callbackExecutor;
        }

        public Builder heartbeatPeriod(Duration heartbeatPeriod) {
            this.heartbeatPeriod = heartbeatPeriod;
            return this;
        }

        public Builder peerTimeout(Duration peerTimeout) {
            this.peerTimeout = peerTimeout;
            return this;
        }

        public Builder commandTimeout(Duration commandTimeout) {
            this.commandTimeout = commandTimeout;
            return this;
        }

        public Builder commandRetries(int commandRetries) {
            this.commandRetries = commandRetries;
            return this;
        }

        public Builder commandMaxInProgressExtensions(int commandMaxInProgressExtensions) {
            this.commandMaxInProgressExtensions = commandMaxInProgressExtensions;
            return this;
        }

        public Builder rc(Rc rc) {
            this.rc = rc;
            return this;
        }

        public Builder closeJoinTimeout(Duration closeJoinTimeout) {
            this.closeJoinTimeout = closeJoinTimeout;
            return this;
        }

        public Builder maxResyncBuffers(int maxResyncBuffers) {
            this.maxResyncBuffers = maxResyncBuffers;
            return this;
        }

        public Builder dispatchQueueCapacity(int dispatchQueueCapacity) {
            this.dispatchQueueCapacity = dispatchQueueCapacity;
            return this;
        }

        public Builder pollTimeout(Duration pollTimeout) {
            this.pollTimeout = pollTimeout;
            return this;
        }

        public Builder txPoolSize(int txPoolSize) {
            this.txPoolSize = txPoolSize;
            return this;
        }

        public Builder parameter(Parameter parameter) {
            this.parameter = parameter;
            return this;
        }

        public Builder signing(SigningSettings signing) {
            this.signing = signing;
            return this;
        }

        public Builder linkHealth(LinkHealth linkHealth) {
            this.linkHealth = linkHealth;
            return this;
        }

        public Builder peer(PeerSettings peer) {
            this.peer = peer;
            return this;
        }

        public Builder clock(InstantSource clock) {
            this.clock = clock;
            return this;
        }

        public Builder threadFactory(ThreadFactory threadFactory) {
            this.threadFactory = threadFactory;
            return this;
        }

        public Builder callbackExecutor(Executor callbackExecutor) {
            this.callbackExecutor = callbackExecutor;
            return this;
        }

        public MavlinkSettings build() {
            return new MavlinkSettings(heartbeatPeriod, peerTimeout, commandTimeout, commandRetries,
                    commandMaxInProgressExtensions, rc, closeJoinTimeout, maxResyncBuffers,
                    dispatchQueueCapacity, pollTimeout, txPoolSize, parameter, signing, linkHealth, peer, clock,
                    threadFactory, callbackExecutor);
        }
    }

    /**
     * Address-flap detection in the peer directory: a peer heard from two addresses on one link
     * within {@code flapWindow} is flagged, and the warning for one peer is repeated at most once per
     * {@code warnInterval} so a genuine roam cannot flood the log.
     */
    public record PeerSettings(Duration flapWindow, Duration warnInterval) {

        public PeerSettings {
            requirePositive(flapWindow, "flapWindow");
            requirePositive(warnInterval, "warnInterval");
        }

        public static PeerSettings defaults() {
            return new PeerSettings(Duration.ofSeconds(3), Duration.ofSeconds(30));
        }
    }

    /**
     * {@code DefaultLinkHealth}'s drop-rate accounting: loss is measured over the trailing
     * {@code window} only (a restart or a second sender must not haunt the rate forever), and a
     * wrapped {@code seq} gap larger than {@code resyncGap} is a sender restart / second source, not
     * that many lost frames -- it is counted as one resync and adds no loss.
     */
    public record LinkHealth(Duration window, int resyncGap) {

        public LinkHealth {
            requirePositive(window, "window");
            if (resyncGap < 1 || resyncGap > 254) {
                throw new IllegalArgumentException("resyncGap must be in [1,254], got " + resyncGap);
            }
        }

        public static LinkHealth defaults() {
            return new LinkHealth(Duration.ofSeconds(30), 64);
        }
    }

    /**
     * Parameter read/write timing. The MAVLink parameter-protocol page is one of the few that
     * states its own numbers: a targeted {@code PARAM_REQUEST_READ} should be retried on a ~1 s
     * timeout, a small number of times — unlike a full parameter <i>download</i>, which this module
     * deliberately does not implement. {@code maxForeignReplies} bounds how many replies about some
     * <i>other</i> parameter a read tolerates before giving up on the one it asked for.
     */
    public record Parameter(Duration timeout, int retries, int maxForeignReplies) {

        public Parameter {
            requirePositive(timeout, "timeout");
            if (retries < 0) {
                throw new IllegalArgumentException("retries must be >= 0, got " + retries);
            }
            if (maxForeignReplies < 0) {
                throw new IllegalArgumentException("maxForeignReplies must be >= 0, got " + maxForeignReplies);
            }
        }

        public static Parameter defaults() {
            return new Parameter(Duration.ofSeconds(1), 3, 8);
        }
    }

    private static void requirePositive(Duration value, String name) {
        Objects.requireNonNull(value, name);
        if (value.isZero() || value.isNegative()) {
            throw new IllegalArgumentException(name + " must be positive, got " + value);
        }
    }

    /** The fixed-rate {@code RC_CHANNELS_OVERRIDE} relay's cadence. */
    public record Rc(int overrideHz, int minOverrideHz, int maxOverrideHz, int releaseFrames) {

        public Rc {
            if (minOverrideHz < 1) {
                throw new IllegalArgumentException("minOverrideHz must be >= 1, got " + minOverrideHz);
            }
            if (maxOverrideHz < minOverrideHz) {
                throw new IllegalArgumentException(
                        "maxOverrideHz must be >= minOverrideHz, got " + maxOverrideHz + " < " + minOverrideHz);
            }
            if (overrideHz < 1) {
                throw new IllegalArgumentException("overrideHz must be >= 1, got " + overrideHz);
            }
            if (releaseFrames < 1) {
                throw new IllegalArgumentException("releaseFrames must be >= 1, got " + releaseFrames);
            }
        }

        public static Rc defaults() {
            return new Rc(33, 10, 50, 3);
        }

        /** {@link #overrideHz} clamped into {@code [minOverrideHz, maxOverrideHz]}. */
        public int clampedOverrideHz() {
            return Math.max(minOverrideHz, Math.min(maxOverrideHz, overrideHz));
        }
    }

    /**
     * Pure numeric signing tuning -- windows, a tolerance, a bound. The behavioural bundle (a key
     * resolver, a policy, a clock registry) is {@code io.mavlink4j.codec.Signing}; the
     * {@code Settings} suffix keeps the two from sharing a simple name.
     */
    public record SigningSettings(Duration signedRecentlyWindow, Duration staleSignatureTolerance,
                                  Duration timestampPersistInterval, int maxStreams) {

        public SigningSettings {
            requirePositive(signedRecentlyWindow, "signedRecentlyWindow");
            requirePositive(staleSignatureTolerance, "staleSignatureTolerance");
            requirePositive(timestampPersistInterval, "timestampPersistInterval");
            if (maxStreams < 1) {
                throw new IllegalArgumentException("maxStreams must be >= 1, got " + maxStreams);
            }
        }

        /**
         * A frame counts as "signed recently" for 5s (the link-quality decay window); a signature
         * timestamp up to 1 minute behind the local clock is still accepted (clock skew across a
         * fresh link); the local signing clock persists at most once a minute (mavlink.io's own
         * "ideally at least once a minute" recommendation); at most 256 concurrent
         * (sysid, compid, signingLinkId) streams are tracked for replay detection.
         */
        public static SigningSettings defaults() {
            return new SigningSettings(Duration.ofSeconds(5), Duration.ofMinutes(1), Duration.ofMinutes(1), 256);
        }
    }
}
