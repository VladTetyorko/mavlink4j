/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.SysId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.SignatureRejection;
import io.mavlink4j.codec.SignatureStatus;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.LinkId;

import io.dronefleet.mavlink.common.RadioStatus;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Subscribes to inbound {@code RADIO_STATUS} (#109) frames whose source component is {@code
 * MAV_COMP_ID_TELEMETRY_RADIO} (68) or {@code MAV_COMP_ID_RADIO}/{@code RADIO2}/{@code RADIO3}
 * (110-112) — the well-known component ids a telemetry radio (ground or air side) reports its own
 * link quality under, distinct from the autopilot's own component id — and keeps the latest
 * reading per {@link LinkId} (the link the frame arrived <b>on</b>, not the peer it is about:
 * RADIO_STATUS is a radio-to-radio fact, and this module has no other way to attribute it to a
 * specific carrier once several links exist).
 *
 * <p>Constructed with a {@link Dispatcher} and subscribes once, in the constructor — no change to
 * {@link MavlinkSession}, {@code onFrame}, or any existing call site ({@link MavlinkSession#dispatcher()}/{@link Dispatcher#subscribe}
 * are public API).
 * The handler runs synchronously on the RX thread per {@link Dispatcher}'s own contract — a plain
 * map write, never blocking.
 *
 * <h2>Signing counters</h2>
 * A second, independent {@link Dispatcher} subscription (matching every frame, not just
 * {@code RADIO_STATUS}) tracks the signing dimension per <b>(link, sender)</b> — not per link alone
 * (several vehicles sharing one carrier would otherwise blend each other's counts, since a signature is a fact about who sent it, not
 * about the wire it arrived on): {@link #signedRecently} decays a per-{@link SenderKey} "last
 * {@link SignatureStatus#VALID} frame" timestamp against {@code signedRecentlyWindow} at read time
 * (no background timer — the same "compute the decay at read time" shape {@code
 * MavlinkMessageInventory}'s own decay already uses), {@link #unsignedFrames} counts every {@link
 * SignatureStatus#UNSIGNED} frame seen from that sender on that link. {@link #recordRejection} is a
 * plain public method rather than a third {@code Dispatcher} subscription, since a rejected frame is
 * reported by {@code MavlinkSession#onFrameRejected} — a callback, not a dispatched frame — see
 * {@link io.mavlink4j.codec.SignatureRejection}'s own javadoc. {@link #of}/{@code byLink}
 * stays keyed by {@link LinkId} alone — RADIO_STATUS is the radio's own property, not a per-sender one.
 */
final class DefaultLinkQuality implements LinkQuality {

    /**
     * {@code MAV_COMP_ID_TELEMETRY_RADIO} (68) and {@code MAV_COMP_ID_PERIPHERAL}'s radio slots
     * (110-112): the component ids a telemetry radio reports {@code RADIO_STATUS} under — protocol
     * facts, not tunables.
     */
    private static final Set<Integer> RADIO_COMPONENT_IDS = Set.of(68, 110, 111, 112);

    private final Map<LinkId, Quality> byLink = new ConcurrentHashMap<>();
    private final Map<SenderKey, Instant> lastValidAt = new ConcurrentHashMap<>();
    private final Map<SenderKey, AtomicInteger> unsignedCounts = new ConcurrentHashMap<>();
    private final Map<SenderKey, AtomicInteger> rejectedCounts = new ConcurrentHashMap<>();
    private final Map<SenderKey, SignatureRejection> lastRejections = new ConcurrentHashMap<>();
    private final Duration signedRecentlyWindow;
    private final InstantSource clock;

    DefaultLinkQuality(Dispatcher dispatcher, MavlinkSettings settings) {
        Objects.requireNonNull(dispatcher, "dispatcher");
        Objects.requireNonNull(settings, "settings");
        this.signedRecentlyWindow = settings.signing().signedRecentlyWindow();
        this.clock = settings.clock();
        dispatcher.subscribe(
                MessageFilter.and(MessageFilter.type(RadioStatus.class), DefaultLinkQuality::fromRadioComponent),
                this::onRadioStatus);
        dispatcher.subscribe(MessageFilter.any(), this::onFrame);
    }

    @Override
    public Optional<Quality> of(LinkId id) {
        Objects.requireNonNull(id, "id");
        return Optional.ofNullable(byLink.get(id));
    }

    @Override
    public boolean signedRecently(LinkId id, SysId system) {
        Instant last = lastValidAt.get(new SenderKey(id, system));
        if (last == null) {
            return false;
        }
        return Duration.between(last, clock.instant()).compareTo(signedRecentlyWindow) <= 0;
    }

    @Override
    public int unsignedFrames(LinkId id, SysId system) {
        AtomicInteger count = unsignedCounts.get(new SenderKey(id, system));
        return count == null ? 0 : count.get();
    }

    @Override
    public int rejectedFrames(LinkId id, SysId system) {
        AtomicInteger count = rejectedCounts.get(new SenderKey(id, system));
        return count == null ? 0 : count.get();
    }

    @Override
    public Optional<SignatureRejection> lastRejection(LinkId id, SysId system) {
        return Optional.ofNullable(lastRejections.get(new SenderKey(id, system)));
    }

    /** Called by {@link MavlinkSession} for every rejected frame, before the host's own listener sees it. */
    void recordRejection(SignatureRejection rejection) {
        Objects.requireNonNull(rejection, "rejection");
        SenderKey key = new SenderKey(rejection.link(), rejection.system());
        rejectedCounts.computeIfAbsent(key, ignored -> new AtomicInteger()).incrementAndGet();
        lastRejections.put(key, rejection);
    }

    private void onRadioStatus(Frame frame) {
        RadioStatus status = frame.as(RadioStatus.class);
        Quality quality = new Quality(frame.link(), frame.receivedAt(), status.rssi(), status.remrssi(),
                status.noise(), status.rxerrors(), status.fixed() != 0);
        byLink.put(frame.link(), quality);
    }

    private void onFrame(Frame frame) {
        SenderKey key = new SenderKey(frame.link(), frame.header().system());
        switch (frame.header().signatureStatus()) {
            case VALID -> lastValidAt.put(key, clock.instant());
            case UNSIGNED -> unsignedCounts.computeIfAbsent(key, ignored -> new AtomicInteger()).incrementAndGet();
            default -> { }
        }
    }

    private static boolean fromRadioComponent(Frame frame) {
        return RADIO_COMPONENT_IDS.contains(frame.header().component().value());
    }

    /**
     * A signature belongs to a sender, not to a socket — every signing counter is keyed by the {@link LinkId} it arrived on
     * <em>and</em> the {@link SysId} that sent it, so two vehicles sharing one carrier never blend
     * each other's signed/unsigned/rejected counts.
     */
    private record SenderKey(LinkId link, SysId system) {
        private SenderKey {
            Objects.requireNonNull(link, "link");
            Objects.requireNonNull(system, "system");
        }
    }
}
