/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.MavlinkLink;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * An in-memory {@link MavlinkLink} that records every {@link #send} instead of touching a socket, and
 * hands back whatever the test {@linkplain #inject injects} from {@link #poll}. Drives
 * {@code FrameWriter}/{@code FrameReader} (or a whole session) with no I/O.
 */
public final class RecordingLink implements MavlinkLink {

    private final LinkId id;
    private final boolean preservesMessageBoundaries;
    private final LinkPeer defaultTarget;
    private final List<byte[]> sent = new ArrayList<>();
    private final BlockingQueue<ByteChunk> inbound = new LinkedBlockingQueue<>();
    private volatile boolean closed;

    /**
     * Creates a recording link.
     *
     * @param id the link id value
     * @param preservesMessageBoundaries what {@link #preservesMessageBoundaries()} reports
     * @param defaultTarget what {@link #defaultTarget()} reports
     */
    public RecordingLink(String id, boolean preservesMessageBoundaries, LinkPeer defaultTarget) {
        this.id = new LinkId(Objects.requireNonNull(id, "id"));
        this.preservesMessageBoundaries = preservesMessageBoundaries;
        this.defaultTarget = Objects.requireNonNull(defaultTarget, "defaultTarget");
    }

    /** @return a snapshot copy of every frame sent so far, oldest first */
    public synchronized List<byte[]> sent() {
        return new ArrayList<>(sent);
    }

    /**
     * Queues bytes for the next {@link #poll}.
     *
     * @param data the bytes to deliver
     * @param source the sender the chunk claims to come from
     */
    public void inject(byte[] data, LinkPeer source) {
        inbound.add(new ByteChunk(data.clone(), data.length, source));
    }

    @Override
    public LinkId id() {
        return id;
    }

    @Override
    public boolean preservesMessageBoundaries() {
        return preservesMessageBoundaries;
    }

    @Override
    public ByteChunk poll(Duration timeout) {
        if (closed) {
            return null;
        }
        try {
            return inbound.poll(timeout.toNanos(), TimeUnit.NANOSECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return null;
        }
    }

    @Override
    public synchronized void send(byte[] frame, int off, int len, LinkPeer target) {
        sent.add(Arrays.copyOfRange(frame, off, off + len));
    }

    @Override
    public LinkPeer defaultTarget() {
        return defaultTarget;
    }

    @Override
    public void close() {
        closed = true;
    }
}
