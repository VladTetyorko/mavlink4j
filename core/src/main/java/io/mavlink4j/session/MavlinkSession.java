/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.MavlinkNode;

import io.mavlink4j.codec.FrameReader;
import io.mavlink4j.codec.FrameSink;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.SignatureRejection;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.PeerId;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.MavlinkLink;

import java.io.IOException;
import java.lang.System.Logger.Level;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/**
 * The L3 composition root: one reader thread per registered {@link MavlinkLink}, driving that
 * link's own {@link FrameReader} and, for every frame it decodes, updating {@link PeerDirectory}
 * and {@link LinkHealth}, then offering the frame to {@link Correlator} and finally to
 * {@link Dispatcher} — in that fixed order, never any other. A waiter registered with
 * {@link Correlator#await} must never miss a reply because a slow {@link Dispatcher} handler ran
 * first, which is exactly why {@code Correlator} is offered the frame before {@code Dispatcher} is.
 *
 * <h2>Ownership</h2>
 * A {@code MavlinkSession} does not own the {@link MavlinkLink}s registered with it — callers open
 * and close them; {@link #removeLink} and {@link #close} stop <i>processing</i> a link (its reader
 * thread exits) without closing the link itself, mirroring {@link FrameWriter#removeLink}'s own
 * "caller owns that" convention. A registered link's send side <i>is</i> owned here: {@link #close}
 * also unregisters every link from the internal {@link FrameWriter}.
 *
 * <h2>Threading</h2>
 * {@link #addLink}/{@link #removeLink}/{@link #close} are guarded by one private monitor so they
 * are safe against each other and against the per-link reader threads they start/stop ("one lock
 * protects the registration table"). Each reader thread (named {@code mavlink-rx-<linkId>}) is the
 * sole caller of {@link #onFrame} for its own link; {@link PeerDirectory}, {@link LinkHealth},
 * {@link Correlator} and {@link Dispatcher} are all independently safe for concurrent callers from
 * several such threads (a session may register more than one link). {@link #close} is idempotent
 * and joins every reader thread bounded by {@link MavlinkSettings#closeJoinTimeout()}.
 *
 * <p>A reader thread never blocks indefinitely: {@link MavlinkLink#poll} is called with {@link
 * MavlinkSettings#pollTimeout()} — purely how often the loop re-checks whether it should stop —
 * so a link that never delivers anything still lets {@link #removeLink}/{@link #close} return
 * promptly instead of wedging on a join. Every thread the session starts comes from {@link
 * MavlinkSettings#threadFactory()}, and every timestamp it takes from {@link
 * MavlinkSettings#clock()}.
 *
 * <h2>What the session owns</h2>
 * Besides the registered links' send side, the session owns one {@link TxScheduler} ({@link
 * #scheduler()}) and one {@link LinkQuality} ({@link #linkQuality()}), both closed with it. Services
 * take their scheduler from here instead of building a pool of their own.
 */
public final class MavlinkSession implements AutoCloseable {

    private static final System.Logger LOG = System.getLogger(MavlinkSession.class.getName());

    private final MavlinkSettings settings;
    private final Signing signing;
    private final DefaultPeerDirectory peerDirectory;
    private final DefaultLinkHealth linkHealth;
    private final DefaultDispatcher dispatcher;
    private final DefaultCorrelator correlator;
    private final DefaultLinkQuality linkQuality;
    private final DefaultTxScheduler scheduler;
    private final FrameWriter frameWriter;
    private final RoutingFrameSink sink;

    private final Object lock = new Object();
    private final Map<LinkId, LinkRuntime> links = new HashMap<>(); // guarded by lock
    /** When the last chunk of bytes arrived per link, before any decoding — the "did anything reach us" clock. */
    private final Map<LinkId, Instant> lastReceivedAt = new ConcurrentHashMap<>();
    private final AtomicBoolean closed = new AtomicBoolean(false);

    /**
     * {@code volatile}, not guarded by {@link #lock}: read from whichever reader thread just hit a
     * genuine I/O failure, written (rarely, typically once) from whatever thread wires this session
     * up. A plain no-op default (never {@code null} — CLAUDE.md's "no parameter means off by being
     * null" rule) means a session nobody has called {@link #onLinkFailure} on behaves exactly as
     * before this field existed.
     */
    private volatile BiConsumer<LinkId, IOException> linkFailureListener = (id, cause) -> { };

    /**
     * {@code volatile}, not guarded by {@link #lock}: same rationale as {@link #linkFailureListener}
     * -- read from whichever reader thread just decoded a frame {@code SigningPolicy} refused,
     * written (rarely, typically once) from whatever thread wires this session up. A plain no-op
     * default means a session nobody has called {@link #onFrameRejected} on behaves exactly as
     * before this field existed.
     */
    private volatile Consumer<SignatureRejection> frameRejectedListener = rejection -> { };

    public MavlinkSession(MavlinkNode self, MavlinkSettings settings, Signing signing) {
        Objects.requireNonNull(self, "self");
        this.settings = Objects.requireNonNull(settings, "settings");
        this.signing = Objects.requireNonNull(signing, "signing");
        this.peerDirectory = new DefaultPeerDirectory(settings);
        this.linkHealth = new DefaultLinkHealth(settings.peerTimeout(), settings.linkHealth(), settings.clock());
        this.dispatcher = new DefaultDispatcher(settings);
        this.correlator = new DefaultCorrelator(settings);
        this.linkQuality = new DefaultLinkQuality(dispatcher, settings);
        this.scheduler = new DefaultTxScheduler(settings);
        this.frameWriter = new FrameWriter(self.system(), self.component(), signing);
        this.sink = new RoutingFrameSink(peerDirectory, frameWriter, signing.keys(), this::links);
    }

    /** Registers {@code link} and starts its dedicated reader thread. The session does not close it. */
    public void addLink(MavlinkLink link) {
        Objects.requireNonNull(link, "link");
        synchronized (lock) {
            if (closed.get()) {
                throw new IllegalStateException("MavlinkSession is closed");
            }
            if (links.containsKey(link.id())) {
                throw new IllegalArgumentException("Link " + link.id() + " is already registered");
            }
            frameWriter.addLink(link);
            LinkRuntime runtime = new LinkRuntime(link);
            links.put(link.id(), runtime);
            runtime.start();
        }
    }

    /** Stops processing {@code id}'s link (bounded join) and unregisters it from the sender. Idempotent. */
    public void removeLink(LinkId id) {
        Objects.requireNonNull(id, "id");
        LinkRuntime runtime;
        synchronized (lock) {
            runtime = links.remove(id);
        }
        if (runtime != null) {
            frameWriter.removeLink(id);
            runtime.stop();
            lastReceivedAt.remove(id);
        }
    }

    public PeerDirectory peers() {
        return peerDirectory;
    }

    public Dispatcher dispatcher() {
        return dispatcher;
    }

    public Correlator correlator() {
        return correlator;
    }

    public FrameSink sink() {
        return sink;
    }

    public LinkHealth health() {
        return linkHealth;
    }

    /** Per-link radio quality and per-sender signing counters, fed by this session's own frames. */
    public LinkQuality linkQuality() {
        return linkQuality;
    }

    /** The one TX scheduler this session owns; closed with the session. Do not close it yourself. */
    public TxScheduler scheduler() {
        return scheduler;
    }

    /** Ids of the links currently registered — a read-only snapshot. */
    public Set<LinkId> links() {
        synchronized (lock) {
            return Set.copyOf(links.keySet());
        }
    }

    /**
     * When bytes last arrived on {@code id}, counted before any decoding — so garbage on the wire
     * still moves it. Empty if nothing has ever arrived (or the link is not registered).
     */
    public Optional<Instant> lastReceivedAt(LinkId id) {
        Objects.requireNonNull(id, "id");
        return Optional.ofNullable(lastReceivedAt.get(id));
    }

    /**
     * Sends {@code message} to one explicit {@code address} on {@code link}, bypassing the {@link
     * PeerDirectory}. For the one caller that must reach an address the directory has not learned
     * (the operator-confirmed re-push of a signing key to where an unsigned vehicle is now heard);
     * signed with the key {@code target}'s system resolves to, plain when it has none.
     *
     * @throws IllegalArgumentException if {@code link} is not registered
     */
    public void sendTo(Object message, LinkId link, LinkPeer address, PeerId target) {
        frameWriter.sendTo(message, link, address, target);
    }

    /**
     * Registers the listener a reader thread invokes when {@link MavlinkLink#poll} throws — a
     * genuine I/O failure, not a timeout or an intentional close (see that method's own contract),
     * and not this session winding down (see below) — instead of {@code runLoop}'s previous behavior
     * of logging a WARNING and silently returning. At
     * most one listener is meaningful per session; a second call replaces the first rather than
     * chaining, matching every other single-collaborator setter in this codebase.
     *
     * <p><b>Contract the listener must honor</b>
     * <b>Called synchronously from the dying reader thread, inside the same {@code catch} block that
     * is about to let that thread exit.</b> The listener must not block — this thread is already
     * unwinding, and a hang here means it never actually exits — and must not throw — any
     * {@link RuntimeException} it raises is caught and logged by this class rather than propagating,
     * so a buggy listener cannot itself become a second silent-death path. Do the real work
     * (deciding what "this link is dead" means for a caller — e.g. closing affected publishers
     * exceptionally, D5) on whatever thread the listener hands off to; do not do it inline unless it
     * is already non-blocking.
     *
     * <p><b>Shutdown must not masquerade as failure</b>
     * A link intentionally closed while this session (or the caller) is shutting down must not be
     * reported as a failure — an orderly {@link #close()}/{@link #removeLink} is not "the radio
     * died". {@code runLoop} checks the same {@link LinkRuntime#running} flag {@link
     * LinkRuntime#stop()} already sets before ever notifying, rather than inventing a second
     * "are we shutting down" signal: {@code running} already answers exactly that question, and it
     * is set (by {@code stop()}) <i>before</i> the link is closed, so by the time an intentional
     * close makes {@link MavlinkLink#poll} throw — if a particular {@link MavlinkLink} ever did that
     * for a close, rather than honoring its own null-on-close contract — {@code running} already
     * reads {@code false} and this path stays silent, matching a conforming link's own already-silent
     * behavior on close.
     */
    public void onLinkFailure(BiConsumer<LinkId, IOException> listener) {
        this.linkFailureListener = Objects.requireNonNull(listener, "listener");
    }

    /**
     * Registers the listener a reader thread invokes for every frame {@code SigningPolicy} refused
     * -- a frame that was fully decoded but never reached {@link #onFrame} (see {@link
     * SignatureRejection}'s own javadoc for the two ways that happens). Same single-listener, same
     * calling-thread and no-throw/no-block contract as {@link #onLinkFailure} — a second call
     * replaces the first, the listener is invoked synchronously from the reader thread that decoded
     * the frame, and any {@link RuntimeException} it raises is caught and logged here rather than
     * propagating, so a buggy listener cannot stall or kill that link's reader thread.
     */
    public void onFrameRejected(Consumer<SignatureRejection> listener) {
        this.frameRejectedListener = Objects.requireNonNull(listener, "listener");
    }

    /**
     * Idempotent. Stops every reader thread (bounded join), unregisters every link from the sender,
     * then stops the async subscribers and the scheduler.
     */
    @Override
    public void close() {
        if (!closed.compareAndSet(false, true)) {
            return;
        }
        List<LinkRuntime> snapshot;
        synchronized (lock) {
            snapshot = new ArrayList<>(links.values());
            links.clear();
        }
        for (LinkRuntime runtime : snapshot) {
            frameWriter.removeLink(runtime.link.id());
            runtime.stop();
        }
        dispatcher.close();
        scheduler.close();
    }

    /** The fixed per-frame order this class exists to guarantee — see the class javadoc. */
    private void onFrame(Frame frame) {
        peerDirectory.recordFrame(frame);
        linkHealth.recordFrame(frame);
        correlator.offer(frame);
        dispatcher.dispatch(frame);
    }

    /** See {@link #onFrameRejected}'s javadoc for the contract this call site honors. */
    private void frameRejected(SignatureRejection rejection) {
        linkQuality.recordRejection(rejection);
        // Seq is a link-layer fact: counting only accepted frames made every policy drop a seq gap,
        // i.e. phantom radio loss (M6 D5). Recorded before the listener so a throwing one cannot skip it.
        linkHealth.recordSample(new PeerId(rejection.system(), rejection.component()), rejection.link(),
                rejection.sequence(), rejection.receivedAt());
        try {
            frameRejectedListener.accept(rejection);
        } catch (RuntimeException e) {
            LOG.log(Level.WARNING, "Frame-rejected listener threw for link " + rejection.link(), e);
        }
    }

    private final class LinkRuntime {
        private final MavlinkLink link;
        private final FrameReader reader;
        private final Thread thread;
        private final AtomicBoolean running = new AtomicBoolean(true);

        LinkRuntime(MavlinkLink link) {
            this.link = link;
            this.reader = new FrameReader(link, settings, signing, MavlinkSession.this::frameRejected);
            this.thread = LibraryThreads.create(settings.threadFactory(), "mavlink-rx-" + link.id().value(), this::runLoop);
        }

        void start() {
            thread.start();
        }

        void stop() {
            running.set(false);
            // A Dispatcher handler may itself call removeLink/close on its own link's frame,
            // synchronously, from this very reader thread (e.g. reacting to a link-error message).
            // Joining self would not deadlock forever (join(millis) always returns after the bound)
            // but would needlessly stall this thread for up to closeJoinTimeout -- skip it, matching
            // MavlinkSocketHub#shutdown's own "thread != Thread.currentThread()" guard.
            if (thread == Thread.currentThread()) {
                return;
            }
            try {
                thread.join(settings.closeJoinTimeout().toMillis());
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        private void runLoop() {
            while (running.get()) {
                try {
                    ByteChunk chunk = link.poll(settings.pollTimeout());
                    if (chunk == null) {
                        continue; // timeout, or the caller closed the link -- either way, re-check running
                    }
                    lastReceivedAt.put(link.id(), settings.clock().instant());
                    reader.offer(chunk, MavlinkSession.this::onFrame);
                } catch (IOException e) {
                    LOG.log(Level.WARNING, "Read error on link " + link.id() + "; its reader thread is stopping", e);
                    // running.get() distinguishes a genuine failure from stop() having already fired
                    // (see onLinkFailure's javadoc, "shutdown must not masquerade as failure") --
                    // reusing this flag rather than adding a second "are we shutting down" signal.
                    if (running.get()) {
                        notifyLinkFailure(e);
                    }
                    return;
                } catch (RuntimeException e) {
                    // Deliberately does NOT notify the link-failure listener:
                    // this branch means one frame's processing blew up (a decoder bug, a malformed
                    // buffer FrameReader itself did not resync past) while the link and the socket
                    // underneath it are still perfectly healthy -- the loop does not even return, it
                    // keeps polling the same link. Firing a listener whose whole contract is "this
                    // link is dead, go close things" here would tear down live publishers over a
                    // transient, unrelated bug that has nothing to do with the radio.
                    LOG.log(Level.WARNING, "Unexpected error processing a frame on link " + link.id(), e);
                }
            }
        }

        /** See {@link MavlinkSession#onLinkFailure}'s javadoc for the contract this call site honors. */
        private void notifyLinkFailure(IOException cause) {
            try {
                linkFailureListener.accept(link.id(), cause);
            } catch (RuntimeException e) {
                LOG.log(Level.WARNING, "Link-failure listener threw for link " + link.id(), e);
            }
        }
    }
}
