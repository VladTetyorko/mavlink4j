/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.session;

import io.mavlink4j.MavlinkNode;

import io.mavlink4j.CompId;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.codec.SigningKeyResolver;
import io.mavlink4j.codec.SigningPolicy;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.LinkId;
import io.mavlink4j.transport.LinkPeer;
import io.mavlink4j.transport.MavlinkLink;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * M6 D5: link health is a link-layer measurement. A frame the signing policy refuses (UNSIGNED under
 * REQUIRED) was still received over the radio, so it must count as received -- not as a sequence gap
 * that reads as loss. Before the fix only accepted frames were recorded, so a REQUIRED vehicle that
 * emitted unsigned frames showed a steady 19-25 % "drop rate" on a loopback link.
 */
class LinkHealthCountsPolicyDroppedFramesTest {

    private static final PeerId VEHICLE = new PeerId(new SysId(42), new CompId(1));
    private static final int FRAMES = 20;

    @Test
    void policyDroppedFramesAreReceivedNotLost() throws Exception {
        QueueLink link = new QueueLink();
        for (byte[] frame : unsignedFrames(link.id(), FRAMES)) {
            link.offer(frame);
        }
        AtomicInteger decisions = new AtomicInteger();
        // refuses every second frame, like REQUIRED refusing a vehicle that signs only part of its traffic
        SigningPolicy refuseEverySecond = (sysid, status) -> decisions.incrementAndGet() % 2 == 1;
        Signing signing = new Signing(SigningKeyResolver.NONE, refuseEverySecond,
                Signing.off().clocks());
        List<Object> rejected = new ArrayList<>();
        try (MavlinkSession session = new MavlinkSession(MavlinkNode.groundStation(), MavlinkSettings.defaults(), signing)) {
            session.onFrameRejected(rejected::add);
            session.addLink(link);

            awaitReceived(session, FRAMES);

            LinkHealth.Health health = session.health().of(VEHICLE);
            assertEquals(FRAMES, health.received(), "every decoded frame is a received frame, accepted or not");
            assertEquals(0, health.lost(), "a policy drop is not radio loss");
            assertEquals(FRAMES / 2, rejected.size());
        }
    }

    private static void awaitReceived(MavlinkSession session, int expected) throws InterruptedException {
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(5);
        while (System.nanoTime() < deadline && session.health().of(VEHICLE).received() < expected) {
            Thread.sleep(20);
        }
    }

    private static List<byte[]> unsignedFrames(LinkId linkId, int count) {
        List<byte[]> frames = new ArrayList<>();
        MavlinkLink capture = new MavlinkLink() {
            @Override public LinkId id() { return linkId; }
            @Override public boolean preservesMessageBoundaries() { return true; }
            @Override public ByteChunk poll(Duration timeout) { return null; }
            @Override public void send(byte[] frame, int off, int len, LinkPeer target) {
                frames.add(Arrays.copyOfRange(frame, off, off + len));
            }
            @Override public LinkPeer defaultTarget() { return LinkPeer.NONE; }
            @Override public void close() { }
        };
        FrameWriter writer = new FrameWriter(VEHICLE.system(), VEHICLE.component(), Signing.off());
        writer.addLink(capture);
        Heartbeat heartbeat = Heartbeat.builder().type(MavType.MAV_TYPE_GROUND_ROVER)
                .autopilot(MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA).systemStatus(MavState.MAV_STATE_ACTIVE)
                .mavlinkVersion(3).build();
        for (int i = 0; i < count; i++) {
            writer.broadcast(heartbeat, linkId);
        }
        return frames;
    }

    private static final class QueueLink implements MavlinkLink {
        private final LinkId id = new LinkId("queue-link");
        private final BlockingQueue<byte[]> queue = new LinkedBlockingQueue<>();

        void offer(byte[] frame) {
            queue.add(frame);
        }

        @Override public LinkId id() { return id; }
        @Override public boolean preservesMessageBoundaries() { return true; }
        @Override public ByteChunk poll(Duration timeout) throws java.io.IOException {
            try {
                byte[] frame = queue.poll(timeout.toMillis(), TimeUnit.MILLISECONDS);
                return frame == null ? null
                        : new ByteChunk(frame, frame.length, new LinkPeer("127.0.0.1", 40_000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            }
        }
        @Override public void send(byte[] frame, int off, int len, LinkPeer target) { }
        @Override public LinkPeer defaultTarget() { return LinkPeer.NONE; }
        @Override public void close() { }
    }
}
