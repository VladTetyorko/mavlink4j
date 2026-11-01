/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.codec;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkPeer;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end {@link FrameWriter} sign &rarr; {@link FrameReader}/{@link SignatureVerifier} verify
 * round trip, covering every {@link SignatureStatus} and {@link SignatureRejection.Reason} the
 * inbound acceptance algorithm can produce.
 *
 * <h2>Why both ends share one system id</h2>
 * A {@link FrameWriter} always signs using {@code keyFor(target.system())} (the addressee) and a
 * {@link FrameReader} always verifies using {@code keyFor(originSystemId)} (the sender) -- correct
 * in production because one {@code MavlinkSession}'s {@link FrameWriter} only ever addresses
 * vehicles and its {@link FrameReader}s only ever receive from vehicles, so both resolve the same
 * "vehicle sysid" key space (see {@code MavlinkSession}'s constructor). To reuse one {@link
 * SigningKeyResolver} for both halves of a single-process round-trip test, {@link #VEHICLE} is used
 * as <i>both</i> the writer's own identity and the send target: the resulting wire frame's origin
 * (the writer's {@code ourSystem}) is exactly the sysid the reader's resolver is keyed on.
 */
class SignedRoundTripTest {

    private static final SysId VEHICLE = new SysId(42);
    private static final CompId VEHICLE_COMPONENT = new CompId(1);
    private static final PeerId VEHICLE_PEER = new PeerId(VEHICLE, VEHICLE_COMPONENT);
    private static final LinkPeer SOURCE = new LinkPeer("127.0.0.1", 55_555);
    private static final byte[] KEY = key((byte) 0x11);
    private static final byte[] WRONG_KEY = key((byte) 0x22);

    @Test
    void aFrameSignedWithTheKeyTheReaderAlsoKnowsIsAcceptedAsValid() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        SigningKeyResolver keys = keyed(KEY);
        writerSignedWith(link, keys, InstantSource.system()).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = readerVerifiedWith(link, keys, InstantSource.system(), Duration.ofMinutes(1), rejected::add);
        decode(reader, onlySent(link), accepted::add);

        assertEquals(1, accepted.size());
        assertTrue(accepted.get(0).header().signed());
        assertEquals(SignatureStatus.VALID, accepted.get(0).header().signatureStatus());
        assertTrue(rejected.isEmpty());
    }

    @Test
    void aSignedFrameWithNoKeyKnownYetIsDispatchedAsUnverifiable() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        writerSignedWith(link, keyed(KEY), InstantSource.system()).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = readerVerifiedWith(link, SigningKeyResolver.NONE, InstantSource.system(),
                Duration.ofMinutes(1), rejected::add);
        decode(reader, onlySent(link), accepted::add);

        assertEquals(1, accepted.size(), "an unverifiable frame is still dispatched under the permissive policy");
        assertEquals(SignatureStatus.UNVERIFIABLE, accepted.get(0).header().signatureStatus());
        assertTrue(rejected.isEmpty());
    }

    @Test
    void anUnsignedFrameIsDispatchedAsUnsigned() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        writerSignedWith(link, SigningKeyResolver.NONE, InstantSource.system()).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);

        List<Frame> accepted = new ArrayList<>();
        FrameReader reader = readerVerifiedWith(link, keyed(KEY), InstantSource.system(), Duration.ofMinutes(1), r -> { });
        decode(reader, onlySent(link), accepted::add);

        assertEquals(1, accepted.size());
        assertFalse(accepted.get(0).header().signed());
        assertEquals(SignatureStatus.UNSIGNED, accepted.get(0).header().signatureStatus());
    }

    @Test
    void aFrameSignedWithAKeyTheReaderDoesNotShareFailsAsBadSignature() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        writerSignedWith(link, keyed(KEY), InstantSource.system()).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = readerVerifiedWith(link, keyed(WRONG_KEY), InstantSource.system(),
                Duration.ofMinutes(1), rejected::add);
        decode(reader, onlySent(link), accepted::add);

        assertTrue(accepted.isEmpty(), "a bad signature must never be dispatched, even under a permissive policy");
        assertEquals(1, rejected.size());
        assertEquals(SignatureStatus.INVALID, rejected.get(0).status());
        assertEquals(SignatureRejection.Reason.BAD_SIGNATURE, rejected.get(0).reason());
    }

    @Test
    void aReplayedSignedFrameIsRejectedOnItsSecondDelivery() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        SigningKeyResolver keys = keyed(KEY);
        writerSignedWith(link, keys, InstantSource.system()).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);
        byte[] captured = onlySent(link);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = readerVerifiedWith(link, keys, InstantSource.system(), Duration.ofMinutes(1), rejected::add);
        decode(reader, captured, accepted::add); // first delivery
        decode(reader, captured, accepted::add); // an attacker replaying the exact same captured bytes

        assertEquals(1, accepted.size(), "only the first delivery of a given signature timestamp may be accepted");
        assertEquals(1, rejected.size());
        assertEquals(SignatureRejection.Reason.REPLAY, rejected.get(0).reason());
    }

    @Test
    void aSignatureTimestampOlderThanTheStaleToleranceIsRejected() {
        // The reader must already have an established floor for "stale" to mean anything
        // -- a reader that has never heard from this
        // vehicle floors at 0, and any real timestamp is then "fresh" by construction (that is
        // exactly the fresh-pairing case, covered separately below), so this test seeds the floor
        // at roughly "now" the way a station that already knows this vehicle would.
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        SigningKeyResolver keys = keyed(KEY);
        InstantSource anHourAgo = InstantSource.fixed(Instant.now().minus(Duration.ofHours(1)));
        writerSignedWith(link, keys, anHourAgo).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        long establishedFloor = SigningClock.timestampOf(Instant.now());
        FrameReader reader = readerVerifiedWithFloor(link, keys, establishedFloor, Instant.now(), Duration.ofSeconds(1), rejected::add);
        decode(reader, onlySent(link), accepted::add);

        assertTrue(accepted.isEmpty());
        assertEquals(1, rejected.size());
        assertEquals(SignatureRejection.Reason.STALE_TIMESTAMP, rejected.get(0).reason());
    }

    /**
     * The four cases below exercise the
     * {@code floor()}/{@code current()} split directly: a resumed vehicle's freshness is checked
     * against the persisted floor, never against a wall-clock-seeded value that a station restart
     * can leave far ahead of anything a vehicle without a real-time clock could have kept up with.
     */
    @Test
    void aResumedVehicleAheadOfTheStationsFloorIsAcceptedEvenLongAfterARestart() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        SigningKeyResolver keys = keyed(KEY);
        Instant floorInstant = Instant.now().minus(Duration.ofHours(1));
        long floor = SigningClock.timestampOf(floorInstant);
        Instant wallInstant = SigningClock.instantOf(floor + SigningClock.ticksOf(Duration.ofMinutes(10)));

        writerSignedWithTimestamp(link, keys, floor + 5).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = readerVerifiedWithFloor(link, keys, floor, wallInstant, Duration.ofMinutes(1), rejected::add);
        decode(reader, onlySent(link), accepted::add);

        assertEquals(1, accepted.size(), "was STALE before the floor/current split -- current() alone was ~10 minutes ahead");
        assertEquals(SignatureStatus.VALID, accepted.get(0).header().signatureStatus());
        assertTrue(rejected.isEmpty());
    }

    @Test
    void aTimestampBehindTheStationsFloorByMoreThanToleranceIsStillStale() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        SigningKeyResolver keys = keyed(KEY);
        Instant floorInstant = Instant.now().minus(Duration.ofHours(1));
        long floor = SigningClock.timestampOf(floorInstant);
        Instant wallInstant = SigningClock.instantOf(floor + SigningClock.ticksOf(Duration.ofMinutes(10)));
        long farBehindTheFloor = floor - SigningClock.ticksOf(Duration.ofMinutes(2));

        writerSignedWithTimestamp(link, keys, farBehindTheFloor).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = readerVerifiedWithFloor(link, keys, floor, wallInstant, Duration.ofMinutes(1), rejected::add);
        decode(reader, onlySent(link), accepted::add);

        assertTrue(accepted.isEmpty());
        assertEquals(1, rejected.size());
        assertEquals(SignatureRejection.Reason.STALE_TIMESTAMP, rejected.get(0).reason());
    }

    @Test
    void aResumedVehiclesFrameStillFailsReplayOnASecondDeliveryOfTheSameStream() {
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        SigningKeyResolver keys = keyed(KEY);
        Instant floorInstant = Instant.now().minus(Duration.ofHours(1));
        long floor = SigningClock.timestampOf(floorInstant);
        Instant wallInstant = SigningClock.instantOf(floor + SigningClock.ticksOf(Duration.ofMinutes(10)));

        writerSignedWithTimestamp(link, keys, floor + 5).sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER);
        byte[] captured = onlySent(link);

        List<Frame> accepted = new ArrayList<>();
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = readerVerifiedWithFloor(link, keys, floor, wallInstant, Duration.ofMinutes(1), rejected::add);
        decode(reader, captured, accepted::add); // first delivery
        decode(reader, captured, accepted::add); // an attacker (or a dumb resend) replaying the same bytes

        assertEquals(1, accepted.size());
        assertEquals(1, rejected.size());
        assertEquals(SignatureRejection.Reason.REPLAY, rejected.get(0).reason());
    }

    @Test
    void aFreshlyPairedVehicleHandedTheStationsCurrentValueIsAcceptedFromItsFirstSignedFrame() {
        // Mirrors MavlinkSigningPushService: a fresh pairing's floor is 0 until the push hands the
        // vehicle the station's live current() value and immediately observe()s it, raising the
        // floor in step -- pymavlink's first signed frame then carries exactly that handed-out
        // value, incrementing by 1 per frame after.
        SendCapturingLink link = new SendCapturingLink("round-trip-link", true, LinkPeer.NONE);
        SigningKeyResolver keys = keyed(KEY);
        Instant wallInstant = Instant.now();
        SigningClock stationClock = new SigningClock(InstantSource.fixed(wallInstant), () -> 0L, discarded -> { }, Duration.ofMinutes(1));
        long wall = stationClock.current();
        assertEquals(0L, stationClock.floor(), "a fresh pairing has nothing persisted yet");
        stationClock.observe(wall); // the push handing the vehicle `wall` and raising the floor to it

        FrameWriter writer = writerSignedWithTimestamp(link, keys, wall);
        writer.sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER); // first signed frame: the handed-out value itself
        writer.sendTo(heartbeat(), link.id(), link.defaultTarget(), VEHICLE_PEER); // second: incremented by one, same as every next() call

        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().signing(
                new MavlinkSettings.SigningSettings(Duration.ofSeconds(5), Duration.ofMinutes(1), Duration.ofMinutes(1), 256)).build();
        Signing signing = new Signing(keys, SigningPolicy.PERMISSIVE, sysid -> stationClock);
        List<SignatureRejection> rejected = new ArrayList<>();
        FrameReader reader = new FrameReader(link, settings, signing, rejected::add);

        List<Frame> accepted = new ArrayList<>();
        for (byte[] frame : link.sent()) {
            decode(reader, frame, accepted::add);
        }

        assertEquals(2, accepted.size());
        assertTrue(rejected.isEmpty());
        accepted.forEach(frame -> assertEquals(SignatureStatus.VALID, frame.header().signatureStatus()));
    }

    /**
     * A {@link FrameWriter} whose vehicle-side clock is seeded so its very first {@code next()}
     * call (i.e. its first signed send) yields exactly {@code firstTimestamp}, incrementing by one
     * on every subsequent send -- lets a test dictate the exact signature timestamp a "vehicle"
     * without a real-time clock would have produced, the same way pymavlink's own {@code
     * timestamp += 1} per packet behaves.
     */
    private static FrameWriter writerSignedWithTimestamp(SendCapturingLink link, SigningKeyResolver keys, long firstTimestamp) {
        Map<Integer, SigningClock> clocks = new ConcurrentHashMap<>();
        Signing signing = new Signing(keys, SigningPolicy.PERMISSIVE, sysid -> clocks.computeIfAbsent(sysid,
                ignored -> new SigningClock(InstantSource.fixed(SigningClock.SIGNING_EPOCH), () -> firstTimestamp - 1,
                        discarded -> { }, Duration.ofMinutes(1))));
        FrameWriter writer = new FrameWriter(VEHICLE, VEHICLE_COMPONENT, signing);
        writer.addLink(link);
        return writer;
    }

    /**
     * A {@link FrameReader} whose station-side clock floors at exactly {@code floorValue} (as if
     * that were the persisted value loaded at boot) while its outbound/wall-clock reading comes
     * from {@code wallInstant} -- lets a test recreate a station restart where the two have
     * diverged, without touching {@link SignatureVerifier}'s production code.
     */
    private static FrameReader readerVerifiedWithFloor(SendCapturingLink link, SigningKeyResolver keys, long floorValue,
                                                         Instant wallInstant, Duration staleSignatureTolerance,
                                                         Consumer<SignatureRejection> rejected) {
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().signing(
                new MavlinkSettings.SigningSettings(Duration.ofSeconds(5), staleSignatureTolerance, Duration.ofMinutes(1), 256)).build();
        Signing signing = new Signing(keys, SigningPolicy.PERMISSIVE, sysid ->
                new SigningClock(InstantSource.fixed(wallInstant), () -> floorValue, discarded -> { }, Duration.ofMinutes(1)));
        return new FrameReader(link, settings, signing, rejected);
    }

    private static FrameWriter writerSignedWith(SendCapturingLink link, SigningKeyResolver keys, InstantSource clockSource) {
        FrameWriter writer = new FrameWriter(VEHICLE, VEHICLE_COMPONENT, signing(keys, clockSource));
        writer.addLink(link);
        return writer;
    }

    private static FrameReader readerVerifiedWith(SendCapturingLink link, SigningKeyResolver keys, InstantSource clockSource,
                                                    Duration staleSignatureTolerance, Consumer<SignatureRejection> rejected) {
        MavlinkSettings settings = MavlinkSettings.defaults().toBuilder().signing(
                new MavlinkSettings.SigningSettings(Duration.ofSeconds(5), staleSignatureTolerance, Duration.ofMinutes(1), 256)).build();
        return new FrameReader(link, settings, signing(keys, clockSource), rejected);
    }

    private static Signing signing(SigningKeyResolver keys, InstantSource clockSource) {
        return new Signing(keys, SigningPolicy.PERMISSIVE, SigningClocks.inMemory(clockSource, Duration.ofMinutes(1)));
    }

    /** A resolver that answers {@code key} for {@link #VEHICLE} and nothing for every other sysid. */
    private static SigningKeyResolver keyed(byte[] key) {
        return sysid -> sysid == VEHICLE.value() ? Optional.of(key) : Optional.empty();
    }

    private static void decode(FrameReader reader, byte[] bytes, Consumer<Frame> out) {
        ByteChunk chunk = new ByteChunk(bytes, bytes.length, SOURCE);
        reader.offer(chunk, out);
    }

    private static byte[] onlySent(SendCapturingLink link) {
        assertEquals(1, link.sent().size());
        return link.sent().get(0);
    }

    private static Heartbeat heartbeat() {
        return Heartbeat.builder()
                .type(MavType.MAV_TYPE_QUADROTOR)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_GENERIC)
                .baseMode(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                .customMode(0)
                .systemStatus(MavState.MAV_STATE_STANDBY)
                .mavlinkVersion(3)
                .build();
    }

    private static byte[] key(byte fill) {
        byte[] key = new byte[32];
        Arrays.fill(key, fill);
        return key;
    }
}
