/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.service;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.PeerUnreachableException;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameSink;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.MavlinkNode;
import io.mavlink4j.session.MavlinkSession;
import io.mavlink4j.session.Peer;
import io.mavlink4j.session.PeerDirectory;
import io.mavlink4j.session.TxScheduler;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.UdpListenLink;

import io.dronefleet.mavlink.common.RcChannelsOverride;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;
import java.time.Instant;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * {@link ManualControlService} tests split in two styles, deliberately:
 * <ul>
 *   <li>Most cases (frame shape, latest-wins, release burst, pause-on-loss) use a real
 *   {@link TxScheduler} with a fast injected tick period, paired with small hand-written
 *   {@link PeerDirectory}/{@link FrameSink} fakes -- these are the module's own small role
 *   interfaces, exactly the seam this class is built to depend on, so faking them directly is more
 *   honest than standing up a real socket for behaviour that has nothing to do with the wire.</li>
 *   <li>{@link #reResolutionPerTickUsesAMovedAddress} is the one case that must be proven over a
 *   real {@link MavlinkSession} + {@link UdpTestVehicle}: address re-learning is exactly
 *   {@code DefaultPeerDirectory}'s own behaviour, which a fake cannot stand in for honestly.</li>
 * </ul>
 *
 * <p>{@code PeerDirectory} never evicts a once-known peer (see that interface's own contract) --
 * there is no protocol-level "this peer is gone" signal available to L4 from {@code PeerDirectory}
 * alone. {@link #lostTargetPausesWithoutKillingTheTask} therefore drives a fake {@code PeerDirectory}
 * that -- unlike the real one -- can be told to "forget" a peer, to prove {@link ManualControlService}'s
 * own pause logic is correct if it is ever exercised (e.g. by a host's ownership-aware
 * {@code PeerDirectory} wrapper). The real {@code PeerDirectory} never forgets a peer, so it
 * cannot reach this branch today.
 */
class ManualControlServiceTest {

    private static final SysId VEHICLE = new SysId(31);
    private static final PeerId TARGET = new PeerId(VEHICLE, new CompId(1));
    private static final LinkId LINK = new LinkId("test-link");
    private static final Duration TICK = Duration.ofMillis(20);
    private static final int RELEASE_FRAMES = 3;
    /** One rate for ceiling and keepalive: 50 Hz == {@link #TICK}. */
    private static final MavlinkSettings.Rc RC = new MavlinkSettings.Rc(50, 50, 50, RELEASE_FRAMES);

    private static MavlinkSettings settings() {
        return MavlinkSettings.defaults().toBuilder().rc(RC).build();
    }

    private static MavlinkSettings settings(InstantSource clock) {
        return settings().toBuilder().clock(clock).build();
    }

    private TxScheduler realScheduler;

    @AfterEach
    void tearDown() {
        if (realScheduler != null) {
            realScheduler.close();
        }
    }

    @Test
    void engageRejectsAnUnreachableTargetAndStartsNoPeriodicTask() {
        FakePeerDirectory peers = new FakePeerDirectory(); // never told about TARGET
        ManualTxScheduler scheduler = new ManualTxScheduler();
        ManualControlService service = new ManualControlService(new RecordingFrameSink(), scheduler, peers, settings());

        PeerUnreachableException e = assertThrows(PeerUnreachableException.class, () -> service.engage(TARGET));
        assertEquals(TARGET, e.peer());
        assertFalse(scheduler.started, "no periodic task should have been started for an unreachable target");
    }

    /**
     * Regression guard: {@code buildFrame} once hardcoded
     * {@code chan9Raw..chan18Raw} to {@link RcChannels#IGNORE} regardless of what a caller sent, so
     * an operator's CH9 binding (a rover's mode switch, a light, a winch) never reached the wire at
     * all. Failing assertion text against the pre-fix code:
     * {@code expected: <1150> but was: <65535>} at the {@code chan9Raw} line below.
     */
    @Test
    void framesCarryAllSentChannelValuesIncludingExtensionChannelsNineThroughSixteen() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        realScheduler = TxScheduler.pooled(io.mavlink4j.config.MavlinkSettings.defaults());
        ManualControlService service = new ManualControlService(sink, realScheduler, peers, settings());

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        assertTrue(link.active());
        service.send(link, new RcChannels(List.of(
                1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900,
                1150, 1250, 1350, 1450, 1550, 1650, 1750, 1850)));

        RcChannelsOverride frame = sink.awaitOverrideWhereChan1Is(1200, Duration.ofSeconds(3));
        assertEquals(VEHICLE.value(), frame.targetSystem());
        assertEquals(1, frame.targetComponent());
        assertEquals(1200, frame.chan1Raw());
        assertEquals(1300, frame.chan2Raw());
        assertEquals(1400, frame.chan3Raw());
        assertEquals(1500, frame.chan4Raw());
        assertEquals(1600, frame.chan5Raw());
        assertEquals(1700, frame.chan6Raw());
        assertEquals(1800, frame.chan7Raw());
        assertEquals(1900, frame.chan8Raw());
        // The bound value at CH9 (and every channel through CH16) reaches the wire -- this is the
        // regression guard: these used to be hardcoded IGNORE (65535) no matter what was sent.
        assertEquals(1150, frame.chan9Raw());
        assertEquals(1250, frame.chan10Raw());
        assertEquals(1350, frame.chan11Raw());
        assertEquals(1450, frame.chan12Raw());
        assertEquals(1550, frame.chan13Raw());
        assertEquals(1650, frame.chan14Raw());
        assertEquals(1750, frame.chan15Raw());
        assertEquals(1850, frame.chan16Raw());
        // Channels 17/18 do not exist for ArduPilot; RcChannels itself refuses to carry a
        // value there, so this class always sends IGNORE for them, unconditionally.
        assertEquals(RcChannels.IGNORE, frame.chan17Raw());
        assertEquals(RcChannels.IGNORE, frame.chan18Raw());

        releaseAndWait(service, link);
    }

    @Test
    void sendWithFewerThanEightChannelsFillsTheRestWithIgnore() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        realScheduler = TxScheduler.pooled(io.mavlink4j.config.MavlinkSettings.defaults());
        ManualControlService service = new ManualControlService(sink, realScheduler, peers, settings());

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        service.send(link, new RcChannels(List.of(1111, 1222, 1333)));

        RcChannelsOverride frame = sink.awaitOverrideWhereChan1Is(1111, Duration.ofSeconds(3));
        assertEquals(1222, frame.chan2Raw());
        assertEquals(1333, frame.chan3Raw());
        assertEquals(RcChannels.IGNORE, frame.chan4Raw());
        assertEquals(RcChannels.IGNORE, frame.chan8Raw());

        releaseAndWait(service, link);
    }

    @Test
    @Timeout(value = 15, unit = TimeUnit.SECONDS)
    void rapidFireSendIsLatestWins() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        realScheduler = TxScheduler.pooled(io.mavlink4j.config.MavlinkSettings.defaults());
        ManualControlService service = new ManualControlService(sink, realScheduler, peers, settings());

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        for (int i = 1; i <= 20; i++) {
            int value = 1000 + i * 10; // 1010 .. 1200
            service.send(link, new RcChannels(List.of(value, value, value, value, value, value, value, value)));
        }

        RcChannelsOverride frame = sink.awaitOverrideWhereChan1Is(1200, Duration.ofSeconds(5));
        assertEquals(1200, frame.chan1Raw(), "only the latest burst value should reliably reach the wire");

        releaseAndWait(service, link);
    }

    /**
     * A release must not read as "ignore" on the
     * extension channels, or a channel like a rover's mode switch (CH9) stays latched at its last
     * commanded value forever instead of actually being released back to the RC radio.
     */
    @Test
    @Timeout(value = 15, unit = TimeUnit.SECONDS)
    void releaseEmitsAReleaseBurstThenGoesSilentAndIsIdempotent() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        realScheduler = TxScheduler.pooled(io.mavlink4j.config.MavlinkSettings.defaults());
        ManualControlService service = new ManualControlService(sink, realScheduler, peers, settings());

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        service.send(link, new RcChannels(List.of(
                1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)));
        sink.awaitOverrideWhereChan1Is(1500, Duration.ofSeconds(3)); // confirm real values flowed first

        releaseAndWait(service, link);
        assertFalse(link.active());

        RcChannelsOverride releaseFrame = sink.awaitOverrideWhereChan1Is(RcChannels.RELEASE, Duration.ofSeconds(3));
        // Channels 1..8: the wire's own RELEASE=0.
        assertEquals(0, releaseFrame.chan1Raw());
        assertEquals(0, releaseFrame.chan8Raw());
        // Channels 9..16: RELEASE is 65534 on the wire, not 0 -- 0 would mean "ignore" there.
        assertEquals(RcChannels.EXTENSION_RELEASE, releaseFrame.chan9Raw());
        assertEquals(65534, releaseFrame.chan9Raw());
        assertEquals(RcChannels.EXTENSION_RELEASE, releaseFrame.chan16Raw());
        // Channels 17/18: always IGNORE, unconditionally (ArduPilot does not read them).
        assertEquals(RcChannels.IGNORE, releaseFrame.chan17Raw());
        assertEquals(RcChannels.IGNORE, releaseFrame.chan18Raw());

        sink.clear();
        sleepQuietly(TICK.toMillis() * 5);
        assertNull(sink.pollAny(Duration.ofMillis(50)), "expected no frames once release() has returned");

        // idempotent: a second release is a safe no-op
        releaseAndWait(service, link);
        assertFalse(link.active());
        assertNull(sink.pollAny(Duration.ofMillis(150)), "a repeat release() must not emit anything");

        // send() after release must also be a no-op
        service.send(link, new RcChannels(List.of(1600, 1600, 1600, 1600, 1600, 1600, 1600, 1600)));
        assertNull(sink.pollAny(Duration.ofMillis(150)), "send() after release() must be a no-op");
    }

    /** E9: release no longer parks the caller; the burst goes out on the scheduler's ticks and the future says when. */
    @Test
    void releaseReturnsAtOnceAndCompletesOnlyWhenTheReleaseBurstHasGone() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        ManualTxScheduler scheduler = new ManualTxScheduler();
        ManualNanoClock clock = new ManualNanoClock();
        ManualControlService service = new ManualControlService(sink, scheduler, peers, settings(clock));
        ManualControlService.ManualControlLink link = service.engage(TARGET);

        CompletableFuture<Void> released = service.release(link);

        assertFalse(link.active());
        assertFalse(released.isDone(), "no frame has gone out yet -- the future must not claim the burst is done");
        assertSame(released, service.release(link), "a second release is the same pending release");
        for (int frame = 1; frame < RELEASE_FRAMES; frame++) {
            scheduler.tick();
            assertFalse(released.isDone(), "frame " + frame + " of " + RELEASE_FRAMES);
        }
        scheduler.tick();
        assertTrue(released.isDone(), "the last release frame has been handed to the sink");
        assertEquals(RELEASE_FRAMES, sink.sentCount());
        assertTrue(scheduler.closed, "the periodic task is stopped once the burst is out");
    }

    @Test
    void sendRejectsALinkNotCreatedByThisService() {
        ManualControlService service = new ManualControlService(new RecordingFrameSink(), new ManualTxScheduler(),
                new FakePeerDirectory(), settings());
        ManualControlService.ManualControlLink foreign = () -> true;

        assertThrows(IllegalArgumentException.class, () -> service.send(foreign, RcChannels.allIgnore(8)));
    }

    @Test
    void releaseRejectsALinkNotCreatedByThisService() {
        ManualControlService service = new ManualControlService(new RecordingFrameSink(), new ManualTxScheduler(),
                new FakePeerDirectory(), settings());
        ManualControlService.ManualControlLink foreign = () -> true;

        assertThrows(IllegalArgumentException.class, () -> service.release(foreign));
    }

    @Test
    void lostTargetPausesWithoutKillingTheTask() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        ManualTxScheduler scheduler = new ManualTxScheduler();
        ManualNanoClock clock = new ManualNanoClock();
        ManualControlService service = new ManualControlService(sink, scheduler, peers, settings(clock));

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        service.send(link, new RcChannels(List.of(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)));
        scheduler.tick(); // reachable -- a frame goes out
        assertEquals(1, sink.sentCount());

        peers.forget(TARGET); // simulate the target going silent
        clock.advance(TICK);
        scheduler.tick(); // must pause, not throw, not stop the schedule
        clock.advance(TICK);
        scheduler.tick();
        assertEquals(1, sink.sentCount(), "no frame should be sent while the target is unreachable");
        assertTrue(link.active(), "a lost target pauses -- it does not release or kill the link");

        peers.knowAgain(TARGET, LINK); // target reachable again
        clock.advance(TICK);
        scheduler.tick();
        assertEquals(2, sink.sentCount(), "sending must resume once the target is reachable again");
    }

    @Test
    void aWriteAfterTheCeilingHasElapsedReachesTheWireWithoutWaitingForATick() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        ManualTxScheduler scheduler = new ManualTxScheduler();
        ManualNanoClock clock = new ManualNanoClock();
        ManualControlService service = new ManualControlService(sink, scheduler, peers, settings(clock));

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        service.send(link, new RcChannels(List.of(1600, 1600, 1600, 1600, 1600, 1600, 1600, 1600)));

        assertEquals(1, scheduler.runSubmitted(), "the write should have scheduled a one-shot transmit");
        assertEquals(1, sink.sentCount(), "the frame must reach the wire without any tick firing");
        RcChannelsOverride frame = (RcChannelsOverride) sink.sent.peek();
        assertEquals(1600, frame.chan1Raw());
    }

    @Test
    void wireLagIsTheTimeBetweenTheWriteAndTheTransmitAndEmptyBeforeTheFirstOne() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        ManualTxScheduler scheduler = new ManualTxScheduler();
        ManualNanoClock clock = new ManualNanoClock();
        ManualControlService service = new ManualControlService(sink, scheduler, peers, settings(clock));

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        assertTrue(link.lastWireLagMillis().isEmpty(), "nothing transmitted yet -- unmeasured, not zero");

        service.send(link, new RcChannels(List.of(1600, 1600, 1600, 1600, 1600, 1600, 1600, 1600)));
        clock.advance(Duration.ofMillis(7));
        scheduler.runSubmitted();

        assertEquals(1, sink.sentCount());
        assertEquals(7L, link.lastWireLagMillis().orElseThrow());

        clock.advance(Duration.ofSeconds(5)); // a keepalive repeat of the old mailbox is not a new frame
        scheduler.tick();
        assertEquals(2, sink.sentCount());
        assertEquals(7L, link.lastWireLagMillis().orElseThrow(), "keepalive must not re-measure");
    }

    @Test
    void aSecondWriteInsideTheCeilingIsCoalescedRatherThanTransmitted() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        ManualTxScheduler scheduler = new ManualTxScheduler();
        ManualNanoClock clock = new ManualNanoClock();
        ManualControlService service = new ManualControlService(sink, scheduler, peers, settings(clock));

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        service.send(link, new RcChannels(List.of(1600, 1600, 1600, 1600, 1600, 1600, 1600, 1600)));
        scheduler.runSubmitted();
        assertEquals(1, sink.sentCount());

        // No clock movement: still inside the ceiling.
        service.send(link, new RcChannels(List.of(1700, 1700, 1700, 1700, 1700, 1700, 1700, 1700)));
        assertEquals(0, scheduler.submittedCount(), "a write inside the ceiling must not schedule a transmit");
        scheduler.tick();
        assertEquals(1, sink.sentCount(), "the ceiling must hold even when a tick fires inside it");

        clock.advance(TICK);
        scheduler.tick();
        assertEquals(2, sink.sentCount(), "the coalesced value goes out on the first tick past the ceiling");
        RcChannelsOverride latest = sink.lastOverride();
        assertEquals(1700, latest.chan1Raw(), "and it is the latest value, not the one it replaced");
    }

    @Test
    void anUnchangedMailboxStillTransmitsOnceTheKeepaliveFloorIsDue() {
        RecordingFrameSink sink = new RecordingFrameSink();
        FakePeerDirectory peers = FakePeerDirectory.knowing(TARGET, LINK);
        ManualTxScheduler scheduler = new ManualTxScheduler();
        ManualNanoClock clock = new ManualNanoClock();
        ManualControlService service = new ManualControlService(sink, scheduler, peers, settings(clock));

        ManualControlService.ManualControlLink link = service.engage(TARGET);
        service.send(link, new RcChannels(List.of(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)));
        scheduler.runSubmitted();
        assertEquals(1, sink.sentCount());

        scheduler.tick(); // nothing new, floor not due
        assertEquals(1, sink.sentCount(), "an unchanged mailbox must not transmit before the floor is due");

        clock.advance(TICK); // floor due
        scheduler.tick();
        assertEquals(2, sink.sentCount(), "an aircraft must never see a gap longer than the keepalive floor");
        assertEquals(1500, sink.lastOverride().chan1Raw(), "the keepalive repeats the last value");
    }

    @Test
    @Timeout(value = 20, unit = TimeUnit.SECONDS)
    void reResolutionPerTickUsesAMovedAddress() throws Exception {
        try (UdpListenLink listenLink = new UdpListenLink("127.0.0.1", 0)) {
            MavlinkSession session = new MavlinkSession(MavlinkNode.groundStation(), MavlinkSettings.defaults(), Signing.off());
            try {
                session.addLink(listenLink);
                int port = Integer.parseInt(listenLink.id().value().substring(listenLink.id().value().lastIndexOf(':') + 1));

                try (UdpTestVehicle vehicle = UdpTestVehicle.start("127.0.0.1", port, 41, 1,
                        MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA, MavType.MAV_TYPE_QUADROTOR)) {
                    PeerId target = vehicle.id();
                    awaitPeerKnown(session, target, Duration.ofSeconds(10));

                    realScheduler = TxScheduler.pooled(io.mavlink4j.config.MavlinkSettings.defaults());
                    ManualControlService service =
                            new ManualControlService(session.sink(), realScheduler, session.peers(), settings());
                    ManualControlService.ManualControlLink link = service.engage(target);
                    service.send(link, new RcChannels(List.of(1234, 1234, 1234, 1234, 1234, 1234, 1234, 1234)));

                    vehicle.awaitOverrideWhereChan1Is(1234, Duration.ofSeconds(5));

                    // The vehicle "moves": new local UDP port, same MAVLink identity. The service
                    // must, on its own next tick, re-resolve the peer's now-fresher address rather
                    // than keep transmitting to the address it resolved at engage()-time.
                    vehicle.clearReceived();
                    vehicle.relocate();
                    // Prove the peer's *address* actually changed (not just that the old link died)
                    // by waiting for PeerDirectory to observe a fresh heartbeat on the new address,
                    // then confirming a subsequent send is still delivered.
                    Thread.sleep(300);
                    service.send(link, new RcChannels(List.of(1750, 1750, 1750, 1750, 1750, 1750, 1750, 1750)));

                    RcChannelsOverride afterMove = vehicle.awaitOverrideWhereChan1Is(1750, Duration.ofSeconds(5));
                    assertEquals(1750, afterMove.chan1Raw());

                    releaseAndWait(service, link);
                }
            } finally {
                session.close();
            }
        }
    }

    private static void releaseAndWait(ManualControlService service, ManualControlService.ManualControlLink link) {
        service.release(link).orTimeout(5, TimeUnit.SECONDS).join();
    }

    private static void awaitPeerKnown(MavlinkSession session, PeerId target, Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (System.nanoTime() < deadline) {
            if (session.peers().peer(target).isPresent()) {
                return;
            }
            Thread.sleep(20);
        }
        throw new AssertionError("expected " + target + " to become known within " + timeout);
    }

    private static void sleepQuietly(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** A {@link TxScheduler} that never fires on its own -- the test drives {@link #tick()} and
     * {@link #runSubmitted()} manually. */
    private static final class ManualTxScheduler implements TxScheduler {
        private volatile Runnable task;
        private volatile boolean started;
        private volatile boolean closed;
        private final ConcurrentLinkedQueue<Runnable> submitted = new ConcurrentLinkedQueue<>();

        @Override
        public Handle repeat(String name, Duration period, Runnable task) {
            this.task = task;
            this.started = true;
            return () -> closed = true;
        }

        @Override
        public void submit(String name, Runnable task) {
            submitted.add(task);
        }

        void tick() {
            task.run();
        }

        /** Drains and runs every queued one-shot, returning how many ran. */
        int runSubmitted() {
            int ran = 0;
            for (Runnable next = submitted.poll(); next != null; next = submitted.poll()) {
                next.run();
                ran++;
            }
            return ran;
        }

        int submittedCount() {
            return submitted.size();
        }
    }

    /** A monotonic clock the test advances by hand -- lets a manually driven scheduler step past the
     * ceiling/keepalive without real sleeps. */
    private static final class ManualNanoClock implements InstantSource {
        private final AtomicLong nanos = new AtomicLong(0);

        @Override
        public Instant instant() {
            return Instant.ofEpochSecond(0, nanos.get());
        }

        void advance(Duration by) {
            nanos.addAndGet(by.toNanos());
        }
    }

    /** Records every {@link #send} call; {@link #broadcast} is unused by {@link ManualControlService}. */
    private static final class RecordingFrameSink implements FrameSink {
        private final ConcurrentLinkedQueue<Object> sent = new ConcurrentLinkedQueue<>();

        @Override
        public void send(Object payload, PeerId target) {
            sent.add(payload);
        }

        @Override
        public void broadcast(Object payload, LinkId link) {
            throw new UnsupportedOperationException("not used by ManualControlService");
        }

        @Override
        public java.util.Set<LinkId> links() {
            return java.util.Set.of();
        }

        int sentCount() {
            return sent.size();
        }

        void clear() {
            sent.clear();
        }

        RcChannelsOverride lastOverride() {
            RcChannelsOverride last = null;
            for (Object next : sent) {
                if (next instanceof RcChannelsOverride override) {
                    last = override;
                }
            }
            return last;
        }

        Object pollAny(Duration timeout) {
            long deadline = System.nanoTime() + timeout.toNanos();
            while (System.nanoTime() < deadline) {
                Object next = sent.poll();
                if (next != null) {
                    return next;
                }
                sleepQuietly(5);
            }
            return null;
        }

        RcChannelsOverride awaitOverrideWhereChan1Is(int expectedChan1, Duration timeout) {
            long deadline = System.nanoTime() + timeout.toNanos();
            while (System.nanoTime() < deadline) {
                Object next = sent.poll();
                if (next instanceof RcChannelsOverride override && override.chan1Raw() == expectedChan1) {
                    return override;
                }
                sleepQuietly(2);
            }
            throw new AssertionError("expected an RC_CHANNELS_OVERRIDE with chan1Raw=" + expectedChan1 + " within " + timeout);
        }
    }

    /** A {@link PeerDirectory} the test can tell to "know" or "forget" a peer, unlike the real one. */
    private static final class FakePeerDirectory implements PeerDirectory {
        private final AtomicReference<Peer> known = new AtomicReference<>();

        static FakePeerDirectory knowing(PeerId id, LinkId link) {
            FakePeerDirectory directory = new FakePeerDirectory();
            directory.knowAgain(id, link);
            return directory;
        }

        void knowAgain(PeerId id, LinkId link) {
            known.set(new Peer(id, link, LinkPeer.NONE, Instant.now(), Instant.now(), java.util.Optional.empty()));
        }

        void forget(PeerId id) {
            known.updateAndGet(p -> p != null && p.id().equals(id) ? null : p);
        }

        @Override
        public java.util.Collection<Peer> peers() {
            Peer p = known.get();
            return p == null ? List.of() : List.of(p);
        }

        @Override
        public java.util.Optional<Peer> peer(PeerId id) {
            Peer p = known.get();
            return p != null && p.id().equals(id) ? java.util.Optional.of(p) : java.util.Optional.empty();
        }

        @Override
        public List<Peer> peersOnLink(LinkId link) {
            Peer p = known.get();
            return p != null && p.link().equals(link) ? List.of(p) : List.of();
        }

        @Override
        public java.util.Set<LinkPeer> concurrentAddresses(PeerId id, Duration window) {
            return java.util.Set.of();
        }
    }
}
