/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.MavlinkNode;
import io.mavlink4j.PeerId;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.codec.SigningClocks;
import io.mavlink4j.codec.SigningPolicy;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.service.Command;
import io.mavlink4j.service.CommandResult;
import io.mavlink4j.service.CommandService;
import io.mavlink4j.service.ParameterOutcome;
import io.mavlink4j.service.ParameterService;
import io.mavlink4j.session.MavlinkSession;
import io.mavlink4j.session.MessageFilter;
import io.mavlink4j.transport.UdpListenLink;

import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavParamType;
import io.dronefleet.mavlink.common.MavResult;
import io.dronefleet.mavlink.minimal.Heartbeat;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.time.Duration;
import java.time.InstantSource;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.TimeUnit;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** {@link FakeVehicle} against a real {@link MavlinkSession} over UDP loopback. */
@Timeout(value = 30, unit = TimeUnit.SECONDS)
class FakeVehicleTest {

    private static final Duration AWAIT = Duration.ofSeconds(10);
    private static final MavlinkSettings FAST = MavlinkSettings.defaults().toBuilder()
            .commandTimeout(Duration.ofMillis(300)).commandRetries(1)
            .parameter(new MavlinkSettings.Parameter(Duration.ofMillis(300), 1, 8)).build();

    private static final class Rig implements AutoCloseable {
        final MavlinkSession session;
        final int port;

        Rig(Signing signing) throws Exception {
            UdpListenLink link = new UdpListenLink("127.0.0.1", 0);
            session = new MavlinkSession(MavlinkNode.groundStation(), FAST, signing);
            session.addLink(link);
            port = Integer.parseInt(link.id().value().substring(link.id().value().lastIndexOf(':') + 1));
        }

        void awaitKnown(PeerId id) throws InterruptedException {
            long deadline = System.nanoTime() + AWAIT.toNanos();
            while (session.peers().peer(id).isEmpty()) {
                if (System.nanoTime() > deadline) {
                    throw new AssertionError(id + " never heard");
                }
                Thread.sleep(20);
            }
        }

        @Override
        public void close() {
            session.close();
        }
    }

    /** The first frame a subscriber received, waiting up to {@link #AWAIT} for it to arrive. */
    private static Frame firstOf(List<Frame> received) throws InterruptedException {
        long deadline = System.nanoTime() + AWAIT.toNanos();
        while (received.isEmpty()) {
            if (System.nanoTime() > deadline) {
                throw new AssertionError("no frame reached the subscriber");
            }
            Thread.sleep(20);
        }
        return received.get(0);
    }

    @Test
    void aScriptedCommandIsAckedAndAnUnscriptedOneIsRecordedButLeftUnanswered() throws Exception {
        try (Rig rig = new Rig(Signing.off());
             FakeVehicle vehicle = FakeVehicle.builder("127.0.0.1", rig.port).sysid(21)
                     .onCommand(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM, CommandReply.ack(MavResult.MAV_RESULT_ACCEPTED))
                     .build()) {
            rig.awaitKnown(vehicle.id());
            CommandService commands = new CommandService(rig.session.sink(), rig.session.correlator(), FAST);

            CommandResult armed = commands.send(vehicle.id(), Command.of(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM).param1(1f))
                    .get(AWAIT.toSeconds(), TimeUnit.SECONDS);
            assertEquals(CommandResult.Status.ACCEPTED, armed.status());

            vehicle.clearReceived();
            CommandResult ignored = commands.send(vehicle.id(), Command.of(MavCmd.MAV_CMD_DO_SET_MODE).param1(1f))
                    .get(AWAIT.toSeconds(), TimeUnit.SECONDS);
            assertEquals(CommandResult.Status.NO_ACK, ignored.status());
            assertEquals(MavCmd.MAV_CMD_DO_SET_MODE,
                    vehicle.awaitFrame(CommandLong.class, AWAIT).command().entry());
        }
    }

    @Test
    void onCommandCanBeScriptedAfterStartAndInProgressIsFollowedByTheFinalAck() throws Exception {
        try (Rig rig = new Rig(Signing.off());
             FakeVehicle vehicle = FakeVehicle.builder("127.0.0.1", rig.port).sysid(22).build()) {
            rig.awaitKnown(vehicle.id());
            vehicle.onCommand(MavCmd.MAV_CMD_NAV_TAKEOFF, CommandReply.inProgressThen(40, MavResult.MAV_RESULT_ACCEPTED));
            CommandService commands = new CommandService(rig.session.sink(), rig.session.correlator(), FAST);

            CommandResult result = commands.send(vehicle.id(), Command.of(MavCmd.MAV_CMD_NAV_TAKEOFF))
                    .get(AWAIT.toSeconds(), TimeUnit.SECONDS);
            assertEquals(CommandResult.Status.ACCEPTED, result.status());
        }
    }

    @Test
    void scriptedParametersAnswerReadsAndWritesAndAnUnknownNameIsSilent() throws Exception {
        try (Rig rig = new Rig(Signing.off());
             FakeVehicle vehicle = FakeVehicle.builder("127.0.0.1", rig.port).sysid(23)
                     .parameter("WPNAV_SPEED", 500f, MavParamType.MAV_PARAM_TYPE_REAL32).build()) {
            vehicle.parameter("ARMING_CHECK", 1f, MavParamType.MAV_PARAM_TYPE_INT32);
            rig.awaitKnown(vehicle.id());
            ParameterService params = new ParameterService(rig.session.sink(), rig.session.correlator(), FAST);

            ParameterOutcome read = params.read(vehicle.id(), "WPNAV_SPEED").get(AWAIT.toSeconds(), TimeUnit.SECONDS);
            assertEquals(ParameterOutcome.Status.OK, read.status());
            assertEquals(500f, read.value().value());

            ParameterOutcome written = params.write(vehicle.id(), "ARMING_CHECK", 0f, MavParamType.MAV_PARAM_TYPE_INT32)
                    .get(AWAIT.toSeconds(), TimeUnit.SECONDS);
            assertEquals(ParameterOutcome.Status.OK, written.status());
            assertEquals(0f, params.read(vehicle.id(), "ARMING_CHECK").get(AWAIT.toSeconds(), TimeUnit.SECONDS).value().value());

            ParameterOutcome unknown = params.read(vehicle.id(), "NO_SUCH_PARAM").get(AWAIT.toSeconds(), TimeUnit.SECONDS);
            assertEquals(ParameterOutcome.Status.NO_REPLY, unknown.status());
        }
    }

    @Test
    void aSignedVehicleSendsSignedFramesTheSessionVerifies() throws Exception {
        byte[] key = new byte[32];
        java.util.Arrays.fill(key, (byte) 0x2A);
        Signing signing = new Signing(sysid -> Optional.of(key), SigningPolicy.PERMISSIVE,
                SigningClocks.inMemory(InstantSource.system(), Duration.ofMinutes(1)));
        try (Rig rig = new Rig(signing)) {
            List<Frame> heartbeats = new CopyOnWriteArrayList<>();
            rig.session.dispatcher().subscribe(MessageFilter.type(Heartbeat.class), heartbeats::add);
            try (FakeVehicle vehicle = FakeVehicle.builder("127.0.0.1", rig.port).sysid(24).signedWith(key).build()) {
                assertTrue(firstOf(heartbeats).header().signed(), "the signed heartbeat was signed on the wire");
            }
        }
    }

    @Test
    void anUnsignedVehicleIsNotSigned() throws Exception {
        try (Rig rig = new Rig(Signing.off())) {
            List<Frame> heartbeats = new CopyOnWriteArrayList<>();
            rig.session.dispatcher().subscribe(MessageFilter.type(Heartbeat.class), heartbeats::add);
            try (FakeVehicle vehicle = FakeVehicle.builder("127.0.0.1", rig.port).sysid(25).build()) {
                // Not awaitKnown: the session registers the peer before dispatching the frame to subscribers.
                assertFalse(firstOf(heartbeats).header().signed());
            }
        }
    }

    @Test
    void aSigningKeyOfTheWrongLengthIsRefused() {
        assertThrows(IllegalArgumentException.class,
                () -> FakeVehicle.builder("127.0.0.1", 1).signedWith(new byte[5]));
    }

    @Test
    void relocateKeepsTheIdentityButMovesTheAddress() throws Exception {
        try (Rig rig = new Rig(Signing.off());
             FakeVehicle vehicle = FakeVehicle.builder("127.0.0.1", rig.port).sysid(26)
                     .onCommand(MavCmd.MAV_CMD_DO_SET_MODE, CommandReply.ack(MavResult.MAV_RESULT_ACCEPTED)).build()) {
            rig.awaitKnown(vehicle.id());
            CommandService commands = new CommandService(rig.session.sink(), rig.session.correlator(), FAST);
            vehicle.relocate();
            Thread.sleep(500);
            CompletableFuture<CommandResult> after = commands.send(vehicle.id(), Command.of(MavCmd.MAV_CMD_DO_SET_MODE));
            assertEquals(CommandResult.Status.ACCEPTED, after.get(AWAIT.toSeconds(), TimeUnit.SECONDS).status());
        }
    }
}
