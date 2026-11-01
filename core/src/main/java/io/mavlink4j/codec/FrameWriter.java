/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.MavlinkLink;

import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * The <b>only</b> class in this module permitted to construct an
 * {@code io.dronefleet.mavlink.MavlinkConnection} — every other class reaches the wire through
 * {@link MavlinkLink}/{@link FrameReader}/this class, never the library directly. This is the
 * one-place library-swap seam: replacing the underlying MAVLink library only has to change this
 * one file (and {@link FrameReader}'s decode path).
 *
 * <h2>Sequence numbering</h2>
 * "Owned per {@code (link, our sysid, our compid)}, wrapping at 255" is satisfied by construction
 * rather than by hand: this class keeps exactly one {@code MavlinkConnection} per registered link,
 * built once in {@link #addLink} and reused for every subsequent send on that link. A
 * {@code MavlinkConnection}'s own sequence counter already increments correctly and wraps (the
 * wire byte is naturally truncated to 8 bits during serialization) — since {@code ourSystem}/
 * {@code ourComponent} are fixed for this {@code FrameWriter}'s whole life, "per link" is the only
 * axis that actually varies, and one connection per link is exactly that.
 *
 * <h2>Routing lives in the session</h2>
 * This class is the routed half of the codec: it sends to a named link ({@link #broadcast}) or to a
 * named link <i>and</i> address ({@link #sendTo}). It cannot answer "which link is this {@link PeerId}
 * reachable on"; that needs the session's peer directory, which is why {@link FrameSink} — whose
 * implementation resolves the peer and delegates here — is the public TX door.
 */
public final class FrameWriter {

    private final SysId ourSystem;
    private final CompId ourComponent;
    private final Signing signing;
    private final Map<LinkId, LinkConnection> links = new ConcurrentHashMap<>();
    /**
     * The wire-level MAVLink 2 signing {@code link_id} (a 1-byte field, 0..255) — assigned once per
     * registered {@link MavlinkLink}, distinct from {@link LinkId} (this codebase's own carrier
     * identity). Per spec this id only has to be unique among the links <i>this system</i> signs
     * on, so a simple wrapping counter is sufficient; it is never compared across systems.
     */
    private final AtomicInteger nextSigningLinkId = new AtomicInteger();

    public FrameWriter(SysId ourSystem, CompId ourComponent, Signing signing) {
        this.ourSystem = Objects.requireNonNull(ourSystem, "ourSystem");
        this.ourComponent = Objects.requireNonNull(ourComponent, "ourComponent");
        this.signing = Objects.requireNonNull(signing, "signing");
    }

    /** Registers {@code link} for sending, building its dedicated {@code MavlinkConnection} once. */
    public void addLink(MavlinkLink link) {
        Objects.requireNonNull(link, "link");
        int signingLinkId = nextSigningLinkId.getAndIncrement() & 0xFF;
        links.put(link.id(), new LinkConnection(link, signingLinkId));
    }

    /** Unregisters a link. Idempotent. Does not close the link itself — the caller owns that. */
    public void removeLink(LinkId id) {
        Objects.requireNonNull(id, "id");
        links.remove(id);
    }

    /**
     * Sends {@code message} (a dialect message class) out {@code link} unconditionally, using that
     * link's own {@link MavlinkLink#defaultTarget()} — matches the protocol's own broadcast dispatch
     * path (spec: {@code target_system == 0} is an unconditional forward, no learned-route check).
     *
     * <p>Always unsigned: a broadcast by definition has no single intended recipient system to
     * resolve a signing key for. The session's {@link FrameSink} upgrades this to a signed {@link
     * #sendTo} when exactly one keyed system is actually listening on {@code link}.
     */
    public void broadcast(Object message, LinkId link) {
        Objects.requireNonNull(message, "message");
        Objects.requireNonNull(link, "link");
        connectionFor(link).send(message, null, null);
    }

    /**
     * Sends {@code message} out {@code link} to one explicit {@code address}, bypassing
     * {@link MavlinkLink#defaultTarget()} entirely, signed for {@code target} when {@link
     * SigningKeyResolver#keyFor(int)} resolves a key for it (unsigned otherwise).
     *
     * <p>This is the only correct primitive for a <b>targeted</b> message on a shared listen link.
     * {@code defaultTarget()} is "whoever transmitted most recently", so on a fleet gateway — several
     * vehicles pushing to one bound port, a normal production shape — a targeted send
     * that trusted it would deliver to whichever aircraft happened to speak last rather than to the
     * one addressed. For an {@code ARM} or a mode change that is not a routing inaccuracy, it is the
     * wrong aircraft. The session's {@link FrameSink} resolves the intended peer's own last-known
     * address from its peer directory and passes it here.
     */
    public void sendTo(Object message, LinkId link, LinkPeer address, PeerId target) {
        Objects.requireNonNull(message, "message");
        Objects.requireNonNull(link, "link");
        Objects.requireNonNull(address, "address");
        Objects.requireNonNull(target, "target");
        connectionFor(link).send(message, address, target.system());
    }

    private LinkConnection connectionFor(LinkId link) {
        LinkConnection connection = links.get(link);
        if (connection == null) {
            throw new IllegalArgumentException("No link registered with id " + link);
        }
        return connection;
    }

    /**
     * One link's persistent {@link MavlinkConnection} — persistent specifically so the library's own
     * sequence counter keeps incrementing across sends; a fresh connection per send would reset
     * {@code seq} to 0 every time and make packet-loss accounting on the far side meaningless.
     *
     * <p>{@link #send} is {@code synchronized} because {@code MavlinkConnection#send2} performs a
     * non-atomic {@code write(...)} then {@code flush()} against one shared buffer, and several
     * services legitimately share one link concurrently — a heartbeat at 1 Hz, an RC override stream
     * at up to 50 Hz and a command send can all be in flight at once. Without the monitor their
     * bytes would interleave inside a single datagram. The monitor also makes the per-send target
     * override below safe: the address is set, used and cleared entirely within one critical section.
     */
    private final class LinkConnection {

        private final MavlinkLink link;
        private final LinkOutputStream out;
        private final MavlinkConnection connection;
        private final int signingLinkId;

        LinkConnection(MavlinkLink link, int signingLinkId) {
            this.link = link;
            this.out = new LinkOutputStream(link);
            this.connection = MavlinkConnection.create(InputStream.nullInputStream(), out);
            this.signingLinkId = signingLinkId;
        }

        /**
         * @param signFor the intended recipient system, used only to resolve a signing key; {@code
         *                null} means "no single recipient" (a broadcast) and is always sent unsigned,
         *                matching {@code address}'s own existing null-means-default-target convention
         *                on this same internal method
         */
        synchronized void send(Object payload, LinkPeer address, SysId signFor) {
            if (payload.getClass().getAnnotation(MavlinkMessageInfo.class) == null) {
                throw new IllegalArgumentException(
                        "payload " + payload.getClass().getName()
                                + " has no @MavlinkMessageInfo -- not a MAVLink message payload");
            }
            out.target(address);
            try {
                Optional<byte[]> key = signFor != null
                        ? signing.keys().keyFor(signFor.value())
                        : Optional.empty();
                if (key.isPresent()) {
                    long timestamp = signing.clocks().forSystem(signFor.value()).next();
                    connection.send2(ourSystem.value(), ourComponent.value(), payload,
                            signingLinkId, timestamp, key.get());
                } else {
                    connection.send2(ourSystem.value(), ourComponent.value(), payload);
                }
            } catch (IOException e) {
                throw new UncheckedIOException("Failed to send a MAVLink frame on link " + link.id(), e);
            } finally {
                out.target(null);
            }
        }
    }

    /**
     * Bridges {@code MavlinkConnection}'s blocking-write {@link OutputStream} expectation to
     * {@link MavlinkLink#send}. Buffers across {@code write} calls and sends exactly one
     * {@link MavlinkLink#send} per {@code flush()} — {@code MavlinkConnection#send1/send2} always
     * call {@code write(...)} then {@code flush()} exactly once per outgoing message, producing
     * one physical send per MAVLink message, matching how a real telemetry radio pushes MAVLink.
     * Destination resolution at flush time: an explicit {@link #target(LinkPeer)} set for this one
     * send wins; otherwise {@link MavlinkLink#defaultTarget()}, so a broadcast on a listen link uses
     * that link's freshest learned peer rather than a stale one captured at construction.
     */
    private static final class LinkOutputStream extends OutputStream {

        private final MavlinkLink link;
        private final ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        private LinkPeer target;

        LinkOutputStream(MavlinkLink link) {
            this.link = link;
        }

        /** Set (or cleared, with {@code null}) by the caller's monitor in {@link LinkConnection#send}. */
        void target(LinkPeer target) {
            this.target = target;
        }

        @Override
        public void write(int b) {
            buffer.write(b);
        }

        @Override
        public void write(byte[] b, int off, int len) {
            buffer.write(b, off, len);
        }

        @Override
        public void flush() throws IOException {
            if (buffer.size() == 0) {
                return;
            }
            byte[] data = buffer.toByteArray();
            buffer.reset();
            link.send(data, 0, data.length, target != null ? target : link.defaultTarget());
        }
    }
}
