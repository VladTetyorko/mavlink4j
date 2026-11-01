/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.MavlinkLink;

import java.io.ByteArrayOutputStream;
import java.time.Duration;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Two {@link MavlinkLink}s wired back to back in memory: what one {@code send}s, the other
 * {@code poll}s. The {@code preservesMessageBoundaries} switch is what makes the link hinge testable
 * both ways with no socket:
 * <ul>
 *   <li>{@code true} (datagram shaped) &mdash; one send is exactly one polled chunk;</li>
 *   <li>{@code false} (stream shaped) &mdash; sent bytes are concatenated, and a poll returns up to
 *       {@code streamChunkBytes} of them, so a frame can arrive split across polls or glued to the next.</li>
 * </ul>
 */
public final class LoopbackLinkPair {

    private static final int UNBOUNDED_CHUNK = Integer.MAX_VALUE;

    private final Endpoint a;
    private final Endpoint b;

    /**
     * Creates a pair whose stream mode hands back everything buffered in one chunk.
     *
     * @param preservesMessageBoundaries {@code true} for datagram semantics, {@code false} for a byte stream
     */
    public LoopbackLinkPair(boolean preservesMessageBoundaries) {
        this(preservesMessageBoundaries, UNBOUNDED_CHUNK);
    }

    /**
     * Creates a pair.
     *
     * @param preservesMessageBoundaries {@code true} for datagram semantics, {@code false} for a byte stream
     * @param streamChunkBytes in stream mode, the most bytes one poll returns (use a small value to force
     *                         frames to split across polls); ignored in datagram mode
     * @throws IllegalArgumentException if {@code streamChunkBytes} is not positive
     */
    public LoopbackLinkPair(boolean preservesMessageBoundaries, int streamChunkBytes) {
        if (streamChunkBytes <= 0) {
            throw new IllegalArgumentException("streamChunkBytes must be positive, got " + streamChunkBytes);
        }
        LinkPeer peerA = new LinkPeer("loopback-a", 1);
        LinkPeer peerB = new LinkPeer("loopback-b", 2);
        this.a = new Endpoint(new LinkId("loopback-a"), peerA, preservesMessageBoundaries, streamChunkBytes);
        this.b = new Endpoint(new LinkId("loopback-b"), peerB, preservesMessageBoundaries, streamChunkBytes);
        a.remote = b;
        b.remote = a;
    }

    /** @return the first endpoint; its {@code defaultTarget()} is the second */
    public MavlinkLink a() {
        return a;
    }

    /** @return the second endpoint; its {@code defaultTarget()} is the first */
    public MavlinkLink b() {
        return b;
    }

    private static final class Endpoint implements MavlinkLink {
        private final LinkId id;
        private final LinkPeer self;
        private final boolean boundaries;
        private final int chunkBytes;
        private final ReentrantLock lock = new ReentrantLock();
        private final Condition arrived = lock.newCondition();
        private final Deque<byte[]> pending = new ArrayDeque<>();
        private Endpoint remote;
        private boolean closed;

        Endpoint(LinkId id, LinkPeer self, boolean boundaries, int chunkBytes) {
            this.id = id;
            this.self = self;
            this.boundaries = boundaries;
            this.chunkBytes = chunkBytes;
        }

        @Override
        public LinkId id() {
            return id;
        }

        @Override
        public boolean preservesMessageBoundaries() {
            return boundaries;
        }

        @Override
        public ByteChunk poll(Duration timeout) {
            long nanos = timeout.toNanos();
            lock.lock();
            try {
                while (pending.isEmpty() && !closed && nanos > 0) {
                    nanos = arrived.awaitNanos(nanos);
                }
                if (closed || pending.isEmpty()) {
                    return null;
                }
                byte[] data = boundaries ? pending.poll() : drainStream();
                return new ByteChunk(data, data.length, remote.self);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            } finally {
                lock.unlock();
            }
        }

        private byte[] drainStream() {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            while (!pending.isEmpty() && out.size() < chunkBytes) {
                byte[] head = pending.poll();
                int take = Math.min(head.length, chunkBytes - out.size());
                out.write(head, 0, take);
                if (take < head.length) {
                    byte[] rest = new byte[head.length - take];
                    System.arraycopy(head, take, rest, 0, rest.length);
                    pending.addFirst(rest);
                }
            }
            return out.toByteArray();
        }

        @Override
        public void send(byte[] frame, int off, int len, LinkPeer target) {
            byte[] copy = new byte[len];
            System.arraycopy(frame, off, copy, 0, len);
            remote.deliver(copy);
        }

        private void deliver(byte[] bytes) {
            lock.lock();
            try {
                if (!closed) {
                    pending.add(bytes);
                    arrived.signalAll();
                }
            } finally {
                lock.unlock();
            }
        }

        @Override
        public LinkPeer defaultTarget() {
            return remote.self;
        }

        @Override
        public void close() {
            lock.lock();
            try {
                closed = true;
                arrived.signalAll();
            } finally {
                lock.unlock();
            }
        }
    }
}
