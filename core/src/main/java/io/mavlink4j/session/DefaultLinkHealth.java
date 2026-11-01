/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.PeerId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.transport.LinkId;

import io.mavlink4j.config.MavlinkSettings;

import java.time.InstantSource;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayDeque;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The one {@link LinkHealth} implementation.
 *
 * <h2>Drop-rate accounting — ours, not standard</h2>
 * The MAVLink spec pins no formula for {@code seq}-based loss; this class's rule: for
 * two consecutive frames from the same {@code (PeerId, LinkId)} with sequence numbers {@code prev}
 * and {@code next}, the wrapped gap is {@code (next - prev) mod 256}. A gap of {@code 1} is the
 * expected back-to-back case (0 additional loss); a gap of {@code g > 1} means {@code g - 1} frames
 * are presumed lost between them. A gap of {@code 0} (a repeated sequence number — either a genuine
 * duplicate or, vanishingly unlikely, an exact 256-frame-later coincidence) is counted as received
 * but contributes no loss, since the two cases are indistinguishable from {@code seq} alone.
 * {@code dropRate = lost / (received + lost)} — the fraction of the <i>expected</i> stream that
 * never arrived, {@code 0.0} before anything has been received.
 *
 * <h2>Sliding window and resync</h2>
 * Counters cover only the trailing {@link MavlinkSettings.LinkHealth#window()} (kept as
 * {@value #BUCKETS} time buckets, so a loss ages out instead of haunting the rate forever). A
 * wrapped gap larger than {@link MavlinkSettings.LinkHealth#resyncGap()} is a sender restart or a
 * second source on one sysid, not that many lost frames: it counts as one resync ({@link
 * #resyncs}) and adds no loss. A restart whose fresh {@code seq} lands within {@code resyncGap}
 * ahead of the old one is indistinguishable from real loss and is counted as such -- but only for
 * one window.
 *
 * <h2>Accounting is per {@code (PeerId, LinkId)}</h2>
 * If a peer is next heard on a <b>different</b> link than the one its running counters belong to,
 * this class starts a fresh window rather than diffing sequence numbers across two physically
 * distinct transmitters' counters (plan: "expected-vs-received seq accounting per (PeerId, LinkId),
 * handling the 8-bit wrap"). Since {@link Peer} keeps only the single most-recently-heard link per
 * peer (latest wins, matching {@link PeerDirectory}'s own protocol-facts-only, no-history
 * philosophy), {@link #of} answers for whichever link this peer was most recently heard on.
 *
 * <h2>Threading</h2>
 * Backed by a {@link ConcurrentHashMap} of per-peer {@code Window}s; {@link #recordFrame} is called
 * from whichever link's reader thread decoded the frame, {@link #of} from any caller thread. Each
 * window's methods are {@code synchronized}, so {@link #of} never observes a torn update.
 */
final class DefaultLinkHealth implements LinkHealth {

    static final int BUCKETS = 30;

    private final Duration peerTimeout;
    private final MavlinkSettings.LinkHealth config;
    private final InstantSource clock;
    private final long bucketMillis;
    private final Map<PeerId, Window> windows = new ConcurrentHashMap<>();

    DefaultLinkHealth(Duration peerTimeout, MavlinkSettings.LinkHealth config, InstantSource clock) {
        this.peerTimeout = Objects.requireNonNull(peerTimeout, "peerTimeout");
        this.config = Objects.requireNonNull(config, "config");
        this.clock = Objects.requireNonNull(clock, "clock");
        this.bucketMillis = Math.max(1, config.window().toMillis() / BUCKETS);
    }

    @Override
    public Health of(PeerId id) {
        Objects.requireNonNull(id, "id");
        Window window = windows.get(id);
        if (window == null) {
            return new Health(id, false, null, 0, 0, 0.0);
        }
        return window.snapshot(id);
    }

    /** Lifetime count of sender restarts / second sources seen for {@code id} (not windowed). */
    long resyncs(PeerId id) {
        Window window = windows.get(id);
        return window == null ? 0 : window.resyncs();
    }

    /**
     * Called by {@link MavlinkSession} for every decoded frame, before {@link Correlator}/
     * {@link Dispatcher} see it.
     */
    void recordFrame(Frame frame) {
        recordSample(new PeerId(frame.header().system(), frame.header().component()),
                frame.link(), frame.header().sequence(), frame.receivedAt());
    }

    /**
     * The same accounting for a datagram the signing policy refused: the sequence number is a
     * link-layer property, so a policy drop must not read as radio loss.
     */
    void recordSample(PeerId id, LinkId link, int sequence, Instant receivedAt) {
        windows.computeIfAbsent(id, key -> new Window()).record(link, sequence, receivedAt, clock.millis());
    }

    private final class Window {
        private final ArrayDeque<long[]> buckets = new ArrayDeque<>(); // {startMillis, received, lost}
        private LinkId link;
        private int lastSeq;
        private Instant lastHeard;
        private long resyncs;

        synchronized void record(LinkId frameLink, int seq, Instant receivedAt, long nowMillis) {
            long received = 1;
            long lost = 0;
            if (link == null || !link.equals(frameLink)) {
                buckets.clear();
                link = frameLink;
            } else {
                int gap = Math.floorMod(seq - lastSeq, 256);
                if (gap > config.resyncGap()) {
                    resyncs++;
                } else if (gap > 1) {
                    lost = gap - 1;
                }
            }
            lastSeq = seq;
            lastHeard = receivedAt;
            expire(nowMillis);
            long start = nowMillis - Math.floorMod(nowMillis, bucketMillis);
            long[] last = buckets.peekLast();
            if (last == null || last[0] != start) {
                last = new long[] {start, 0, 0};
                buckets.addLast(last);
            }
            last[1] += received;
            last[2] += lost;
        }

        synchronized Health snapshot(PeerId id) {
            expire(clock.millis());
            long received = 0;
            long lost = 0;
            for (long[] b : buckets) {
                received += b[1];
                lost += b[2];
            }
            boolean connected = Duration.between(lastHeard, clock.instant()).compareTo(peerTimeout) <= 0;
            double dropRate = (received + lost) == 0 ? 0.0 : (double) lost / (received + lost);
            return new Health(id, connected, lastHeard, received, lost, dropRate);
        }

        synchronized long resyncs() {
            return resyncs;
        }

        private void expire(long nowMillis) {
            long cutoff = nowMillis - config.window().toMillis();
            while (!buckets.isEmpty() && buckets.peekFirst()[0] + bucketMillis <= cutoff) {
                buckets.removeFirst();
            }
        }
    }
}
