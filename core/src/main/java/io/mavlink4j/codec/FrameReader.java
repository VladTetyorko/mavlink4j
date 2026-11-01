/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.SysId;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.MavlinkLink;

import io.dronefleet.mavlink.Mavlink2Message;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo;

import java.lang.System.Logger.Level;
import java.time.Instant;
import java.time.InstantSource;
import java.util.LinkedHashMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;

/**
 * Bytes in, frames out. Delegates all real parsing (CRC, resync-past-garbage, dialect-specific
 * field layout) to {@code io.dronefleet.mavlink} — this class's only job is bridging the
 * incremental {@link ByteChunk}s a {@link MavlinkLink} produces into that library's frame decoder,
 * one independent {@link ResyncBuffer} per source.
 *
 * <h2>One resync buffer per source</h2>
 * For a boundary-preserving link ({@link MavlinkLink#preservesMessageBoundaries()} {@code true} —
 * UDP), each distinct {@link LinkPeer} gets its own buffer. For a stream link ({@code false} — TCP)
 * every chunk shares exactly one buffer regardless of what {@link ByteChunk#source()} reports,
 * since a stream link has exactly one logical byte sequence by construction. This is not an
 * optimisation: concatenating datagrams from different vehicles into one byte stream splices one
 * sender's partial frame onto another's and corrupts both, which is exactly what rules out a single
 * shared buffer on a datagram link. Buffers are bounded and evicted
 * least-recently-used at {@link MavlinkSettings#maxResyncBuffers()} so a spoofing or scanning
 * source cannot grow memory without bound.
 *
 * <p>This is where received time is decided: a {@link ByteChunk} carries none, so every {@link Frame}
 * (and every {@link SignatureRejection}) is stamped from {@link MavlinkSettings#clock()}.
 *
 * <p>Not thread-safe: intended to be driven by exactly one reader thread per link (matching L3's
 * {@code MavlinkSession}, which owns one reader thread per link), the same "one thread owns the
 * read side" convention every link implementation in {@code io.mavlink4j.transport} follows.
 */
public final class FrameReader {

    private static final System.Logger LOG = System.getLogger(FrameReader.class.getName());

    /** Sentinel key for the single shared buffer a stream link's chunks all fall into. */
    private static final LinkPeer STREAM_KEY = new LinkPeer("<stream>", 1);

    private final LinkId link;
    private final boolean preservesMessageBoundaries;
    private final int maxResyncBuffers;
    private final InstantSource clock;
    /**
     * Learned dialect per MAVLink system id, shared by every {@link ResyncBuffer} on this link — a
     * vehicle's dialect belongs to the vehicle, not to the address its datagrams arrive from. See
     * {@code ResyncBuffer#dialectForPendingFrame} for the full reasoning.
     */
    private final Map<Integer, io.dronefleet.mavlink.MavlinkDialect> dialectsBySystem = new ConcurrentHashMap<>();
    private final Map<LinkPeer, ResyncBuffer> buffers;
    private final Signing signing;
    private final Consumer<SignatureRejection> rejected;
    private final SignatureVerifier verifier;

    /**
     * @param signing  key/policy/clock bundle; {@link Signing#off()} makes every frame {@link
     *                 SignatureStatus#UNSIGNED} and dispatches unconditionally
     * @param rejected notified once per frame {@link SigningPolicy} refuses -- never thrown, see
     *                 {@link SignatureRejection}'s own javadoc for why
     */
    public FrameReader(MavlinkLink link, MavlinkSettings settings, Signing signing, Consumer<SignatureRejection> rejected) {
        Objects.requireNonNull(link, "link");
        Objects.requireNonNull(settings, "settings");
        Objects.requireNonNull(signing, "signing");
        Objects.requireNonNull(rejected, "rejected");
        this.link = link.id();
        this.preservesMessageBoundaries = link.preservesMessageBoundaries();
        this.maxResyncBuffers = settings.maxResyncBuffers();
        this.clock = settings.clock();
        this.signing = signing;
        this.rejected = rejected;
        this.verifier = new SignatureVerifier(signing, settings.signing().staleSignatureTolerance(), settings.signing().maxStreams());
        this.buffers = new LinkedHashMap<>(16, 0.75f, true) {
            @Override
            protected boolean removeEldestEntry(Map.Entry<LinkPeer, ResyncBuffer> eldest) {
                return size() > FrameReader.this.maxResyncBuffers;
            }
        };
    }

    /** Convenience for a caller with no signing to configure: {@link Signing#off()}, rejections silently discarded. */
    public FrameReader(MavlinkLink link) {
        this(link, MavlinkSettings.defaults(), Signing.off(), rejection -> { });
    }

    /**
     * Feeds {@code chunk}'s bytes into that source's resync buffer and hands every frame that
     * becomes fully decodable as a result to {@code out}, in order. May decode zero, one, or
     * several frames from a single chunk; never blocks.
     */
    public void offer(ByteChunk chunk, Consumer<Frame> out) {
        Objects.requireNonNull(chunk, "chunk");
        Objects.requireNonNull(out, "out");
        LinkPeer key = preservesMessageBoundaries ? chunk.source() : STREAM_KEY;
        ResyncBuffer buffer = buffers.computeIfAbsent(key, k -> new ResyncBuffer(dialectsBySystem));
        buffer.append(chunk.data(), chunk.length());
        // One stamp per chunk, taken before decoding: every frame the chunk completes was received
        // when its bytes arrived, not when this thread got round to decoding the last of them.
        Instant receivedAt = clock.instant();
        buffer.drain(message -> emit(message, chunk.source(), receivedAt, out));
    }

    private void emit(MavlinkMessage<?> message, LinkPeer source, Instant receivedAt, Consumer<Frame> out) {
        try {
            Frame frame = toFrame(message, source, receivedAt);
            if (frame != null) {
                out.accept(frame);
            }
        } catch (IllegalArgumentException e) {
            // A malformed-but-CRC-valid origin (e.g. sysid/compid 0) -- skip this one frame rather
            // than let a single bad frame stall every other source's decoding.
            LOG.log(Level.WARNING, "Discarding a decoded MAVLink frame with an invalid origin identity", e);
        }
    }

    /**
     * Decodes one frame's header and verifies its signature, in that order. Returns {@code null} --
     * never throws for this reason -- when {@link SigningPolicy#accept(int, SignatureStatus)}
     * refuses the frame, after reporting it to {@code rejected}; a storm of bad frames must not
     * flood the log above {@code DEBUG} (see {@link SignatureRejection}'s own javadoc) or stall
     * decoding of every other source.
     */
    private Frame toFrame(MavlinkMessage<?> message, LinkPeer source, Instant receivedAt) {
        boolean isV2 = message instanceof Mavlink2Message<?>;
        int version = isV2 ? 2 : 1;
        byte[] raw = message.getRawBytes();
        int incompatFlags = isV2 ? (raw[2] & 0xFF) : 0;
        int compatFlags = isV2 ? (raw[3] & 0xFF) : 0;
        boolean signed = isV2 && ((Mavlink2Message<?>) message).isSigned();
        int originSystemId = message.getOriginSystemId();
        int originComponentId = message.getOriginComponentId();
        SysId system = new SysId(originSystemId);
        CompId component = new CompId(originComponentId);
        int messageId = messageIdOf(message.getPayload());
        SignatureVerifier.Result result = verifier.verify(message, originSystemId, originComponentId);
        if (!signing.policy().accept(originSystemId, result.status())) {
            SignatureRejection.Reason reason = result.reason() != null
                    ? result.reason()
                    : SignatureRejection.Reason.POLICY;
            LOG.log(Level.DEBUG, () -> "Rejecting MAVLink frame sysid=" + originSystemId
                    + " compid=" + originComponentId + " messageId=" + messageId
                    + " status=" + result.status() + " reason=" + reason);
            rejected.accept(new SignatureRejection(link, system, component, messageId, result.status(), reason,
                    message.getSequence(), source, receivedAt));
            return null;
        }
        FrameHeader header = new FrameHeader(
                version,
                message.getSequence(),
                system,
                component,
                messageId,
                incompatFlags,
                compatFlags,
                signed,
                result.status());
        return new Frame(header, message.getPayload(), link, source, receivedAt);
    }

    private static int messageIdOf(Object payload) {
        MavlinkMessageInfo info = payload.getClass().getAnnotation(MavlinkMessageInfo.class);
        if (info == null) {
            // Should be impossible: the library only ever resolves a payload class via a dialect's
            // own registry, and every registered class carries this annotation.
            throw new IllegalStateException(
                    "Decoded MAVLink payload " + payload.getClass() + " has no @MavlinkMessageInfo");
        }
        return info.id();
    }
}
