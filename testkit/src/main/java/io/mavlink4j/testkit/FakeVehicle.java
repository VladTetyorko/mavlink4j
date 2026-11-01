/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.testkit;

import io.mavlink4j.CompId;
import io.mavlink4j.MavlinkNode;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.Frame;
import io.mavlink4j.codec.FrameReader;
import io.mavlink4j.codec.FrameWriter;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.codec.SigningClocks;
import io.mavlink4j.codec.SigningPolicy;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.transport.ByteChunk;
import io.mavlink4j.transport.UdpTargetLink;

import io.dronefleet.mavlink.common.AutopilotVersion;
import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.CommandInt;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavParamType;
import io.dronefleet.mavlink.common.MavResult;
import io.dronefleet.mavlink.common.ParamRequestList;
import io.dronefleet.mavlink.common.ParamRequestRead;
import io.dronefleet.mavlink.common.ParamSet;
import io.dronefleet.mavlink.common.ParamValue;
import io.dronefleet.mavlink.common.RcChannelsOverride;
import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.minimal.MavType;
import io.dronefleet.mavlink.util.EnumValue;

import java.io.IOException;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * A scripted aircraft over real UDP loopback: it dials the system under test, sends a heartbeat every
 * {@link Builder#heartbeatPeriod 200 ms} by default (one heartbeat can be lost on plain UDP while the
 * far side is still binding, so the stream never stops), records every frame it receives, and answers
 * on demand or by script.
 *
 * <h2>Scripting</h2>
 * <ul>
 *   <li>{@link #onCommand(MavCmd, CommandReply)} &mdash; what to do when that command arrives; an
 *       unscripted command is recorded and left unanswered.</li>
 *   <li>{@link #parameter(String, float, MavParamType)} &mdash; makes a parameter exist; the vehicle then
 *       answers {@code PARAM_REQUEST_READ}, {@code PARAM_REQUEST_LIST} and {@code PARAM_SET} for it. An
 *       unknown name gets no reply, as on a real autopilot.</li>
 *   <li>{@link Builder#signedWith(byte[])} &mdash; the vehicle signs what it sends and verifies what it
 *       receives with that 32-byte key.</li>
 * </ul>
 * The {@code reply*} methods send one reply by hand, for cases a script cannot express (a wrong
 * parameter name, an out-of-order ack).
 *
 * <p><b>Thread-safety:</b> every public method may be called from the test thread while the vehicle's own
 * heartbeat and reader threads run.
 */
public final class FakeVehicle implements AutoCloseable {

    private static final Duration POLL_TIMEOUT = Duration.ofMillis(200);
    private static final Duration JOIN_TIMEOUT = Duration.ofSeconds(2);

    private final String gatewayHost;
    private final int gatewayPort;
    private final SysId sysid;
    private final CompId compid;
    private final MavAutopilot autopilot;
    private final MavType mavType;
    private final Duration heartbeatPeriod;
    private final Signing signing;
    private final boolean signed;

    private final Map<MavCmd, CommandReply> commandScript = new ConcurrentHashMap<>();
    private final List<StoredParam> parameters = new CopyOnWriteArrayList<>();

    /** (link, writer, reader) always swapped together so a reader never sees a torn combination. */
    private record LinkBundle(UdpTargetLink link, FrameWriter writer, FrameReader reader) {
    }

    private static final class StoredParam {
        private final String name;
        private volatile float value;
        private volatile MavParamType type;

        private StoredParam(String name, float value, MavParamType type) {
            this.name = name;
            this.value = value;
            this.type = type;
        }
    }

    private volatile LinkBundle current;
    private final BlockingQueue<Frame> received = new LinkedBlockingQueue<>();
    private final AtomicBoolean stopped = new AtomicBoolean(false);
    private volatile Thread heartbeatThread;
    private volatile Thread readerThread;

    private FakeVehicle(Builder b) {
        this.gatewayHost = b.host;
        this.gatewayPort = b.port;
        this.sysid = new SysId(b.sysid);
        this.compid = new CompId(b.compid);
        this.autopilot = b.autopilot;
        this.mavType = b.type;
        this.heartbeatPeriod = b.heartbeatPeriod;
        this.signed = b.key != null;
        this.signing = signed ? signingFor(b.key) : Signing.off();
    }

    /**
     * Starts describing a vehicle that will send to {@code host:port}.
     *
     * @param host the address the system under test listens on
     * @param port its UDP port
     * @return a builder with sysid 1, compid 1, an ArduPilot quadrotor and a 200 ms heartbeat
     */
    public static Builder builder(String host, int port) {
        return new Builder(host, port);
    }

    /** Describes a {@link FakeVehicle}; {@link #build()} starts it. */
    public static final class Builder {
        private final String host;
        private final int port;
        private int sysid = 1;
        private int compid = 1;
        private MavAutopilot autopilot = MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA;
        private MavType type = MavType.MAV_TYPE_QUADROTOR;
        private Duration heartbeatPeriod = Duration.ofMillis(200);
        private byte[] key;
        private final Map<MavCmd, CommandReply> commands = new ConcurrentHashMap<>();
        private final List<StoredParam> params = new ArrayList<>();

        private Builder(String host, int port) {
            this.host = Objects.requireNonNull(host, "host");
            this.port = port;
        }

        /**
         * Sets the MAVLink system id.
         *
         * @param sysid 1..255
         * @return this
         */
        public Builder sysid(int sysid) {
            this.sysid = sysid;
            return this;
        }

        /**
         * Sets the MAVLink component id.
         *
         * @param compid 1..255
         * @return this
         */
        public Builder compid(int compid) {
            this.compid = compid;
            return this;
        }

        /**
         * Sets the autopilot reported in the heartbeat.
         *
         * @param autopilot the autopilot
         * @return this
         */
        public Builder autopilot(MavAutopilot autopilot) {
            this.autopilot = Objects.requireNonNull(autopilot, "autopilot");
            return this;
        }

        /**
         * Sets the airframe reported in the heartbeat.
         *
         * @param type the vehicle type
         * @return this
         */
        public Builder type(MavType type) {
            this.type = Objects.requireNonNull(type, "type");
            return this;
        }

        /**
         * Sets how often the heartbeat is sent.
         *
         * @param period a positive period
         * @return this
         */
        public Builder heartbeatPeriod(Duration period) {
            if (period.isZero() || period.isNegative()) {
                throw new IllegalArgumentException("heartbeatPeriod must be positive, got " + period);
            }
            this.heartbeatPeriod = period;
            return this;
        }

        /**
         * Makes the vehicle sign every frame it sends, and verify every frame it receives, with
         * {@code key}.
         *
         * @param key the 32-byte MAVLink 2 signing secret
         * @return this
         */
        public Builder signedWith(byte[] key) {
            this.key = signingKey(key);
            return this;
        }

        /**
         * Scripts a command before the vehicle starts; see {@link FakeVehicle#onCommand}.
         *
         * @param command the command to answer
         * @param reply what to do when it arrives
         * @return this
         */
        public Builder onCommand(MavCmd command, CommandReply reply) {
            commands.put(Objects.requireNonNull(command, "command"), Objects.requireNonNull(reply, "reply"));
            return this;
        }

        /**
         * Adds a parameter before the vehicle starts; see {@link FakeVehicle#parameter}.
         *
         * @param name the parameter name
         * @param value its value
         * @param type its wire type
         * @return this
         */
        public Builder parameter(String name, float value, MavParamType type) {
            params.add(new StoredParam(name, value, type));
            return this;
        }

        /**
         * Starts the vehicle: opens its link, then begins heartbeating and reading.
         *
         * @return the running vehicle; close it when done
         * @throws IOException if the UDP socket cannot be opened
         */
        public FakeVehicle build() throws IOException {
            FakeVehicle vehicle = new FakeVehicle(this);
            vehicle.commandScript.putAll(commands);
            vehicle.parameters.addAll(params);
            vehicle.openLink();
            vehicle.heartbeatThread = new Thread(vehicle::heartbeatLoop, "fake-vehicle-heartbeat-" + sysid);
            vehicle.heartbeatThread.setDaemon(true);
            vehicle.heartbeatThread.start();
            return vehicle;
        }
    }

    private static byte[] signingKey(byte[] key) {
        Objects.requireNonNull(key, "key");
        if (key.length != 32) {
            throw new IllegalArgumentException("a MAVLink 2 signing key is 32 bytes, got " + key.length);
        }
        return key.clone();
    }

    private static Signing signingFor(byte[] key) {
        return new Signing(sysid -> Optional.of(key), SigningPolicy.PERMISSIVE,
                SigningClocks.inMemory(java.time.InstantSource.system(), Duration.ofMinutes(1)));
    }

    /** @return this vehicle's MAVLink identity */
    public PeerId id() {
        return new PeerId(sysid, compid);
    }

    /**
     * Scripts the reply to a command. Replaces an earlier script for the same command.
     *
     * @param command the {@code MAV_CMD} to answer
     * @param reply what to do when it arrives, for example {@code CommandReply.ack(MAV_RESULT_ACCEPTED)}
     * @return this
     */
    public FakeVehicle onCommand(MavCmd command, CommandReply reply) {
        commandScript.put(Objects.requireNonNull(command, "command"), Objects.requireNonNull(reply, "reply"));
        return this;
    }

    /**
     * Makes a parameter exist, or changes the value of one that does.
     *
     * @param name the parameter name (at most 16 characters on the wire)
     * @param value its value
     * @param type its wire type
     * @return this
     */
    public FakeVehicle parameter(String name, float value, MavParamType type) {
        Objects.requireNonNull(name, "name");
        for (StoredParam existing : parameters) {
            if (existing.name.equals(name)) {
                existing.value = value;
                existing.type = type;
                return this;
            }
        }
        parameters.add(new StoredParam(name, value, type));
        return this;
    }

    private void openLink() throws IOException {
        UdpTargetLink newLink = new UdpTargetLink(gatewayHost, gatewayPort);
        FrameWriter newWriter = new FrameWriter(sysid, compid, signing);
        newWriter.addLink(newLink);
        FrameReader newReader = new FrameReader(newLink, MavlinkSettings.defaults(), signing, rejected -> { });
        LinkBundle bundle = new LinkBundle(newLink, newWriter, newReader);
        this.current = bundle;
        Thread newReaderThread = new Thread(() -> readerLoop(bundle), "fake-vehicle-reader-" + sysid.value());
        newReaderThread.setDaemon(true);
        this.readerThread = newReaderThread;
        newReaderThread.start();
    }

    /**
     * Simulates this vehicle's transport address changing (a re-elected claim, a NAT rebind): closes the
     * current link and opens a fresh one to the same gateway under the identical MAVLink identity. Proves
     * a caller re-resolves a peer's address on every send rather than caching it.
     *
     * @throws IOException if the new socket cannot be opened
     * @throws InterruptedException if interrupted while the old reader stops
     */
    public void relocate() throws IOException, InterruptedException {
        LinkBundle old = this.current;
        Thread oldReaderThread = this.readerThread;
        openLink();
        old.link().close();
        oldReaderThread.join(JOIN_TIMEOUT.toMillis());
    }

    private void heartbeatLoop() {
        while (!stopped.get()) {
            try {
                Heartbeat heartbeat = Heartbeat.builder()
                        .type(mavType)
                        .autopilot(autopilot)
                        .baseMode(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED, MavModeFlag.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                        .customMode(0)
                        .systemStatus(MavState.MAV_STATE_ACTIVE)
                        .mavlinkVersion(3)
                        .build();
                emit(heartbeat);
                Thread.sleep(heartbeatPeriod.toMillis());
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            } catch (RuntimeException e) {
                // A stale link mid-relocate, or the link closing: skip this beat, the next reads a fresh bundle.
            }
        }
    }

    private void readerLoop(LinkBundle bundle) {
        while (!stopped.get() && this.current == bundle) {
            try {
                ByteChunk chunk = bundle.link().poll(POLL_TIMEOUT);
                if (chunk != null) {
                    bundle.reader().offer(chunk, this::onFrame);
                }
            } catch (IOException e) {
                return; // link closing -- stop quietly
            }
        }
    }

    private void onFrame(Frame frame) {
        received.add(frame);
        try {
            script(frame);
        } catch (RuntimeException e) {
            System.getLogger(FakeVehicle.class.getName())
                    .log(System.Logger.Level.WARNING, "scripted reply failed", e);
        }
    }

    private void script(Frame frame) {
        if (frame.is(CommandLong.class)) {
            command(frame.as(CommandLong.class).command(), frame.payload());
        } else if (frame.is(CommandInt.class)) {
            command(frame.as(CommandInt.class).command(), frame.payload());
        } else if (frame.is(ParamRequestRead.class)) {
            ParamRequestRead request = frame.as(ParamRequestRead.class);
            StoredParam found = request.paramIndex() >= 0
                    ? byIndex(request.paramIndex()) : byName(request.paramId());
            if (found != null) {
                replyParam(found);
            }
        } else if (frame.is(ParamRequestList.class)) {
            parameters.forEach(this::replyParam);
        } else if (frame.is(ParamSet.class)) {
            ParamSet set = frame.as(ParamSet.class);
            StoredParam found = byName(set.paramId());
            if (found != null) {
                found.value = set.paramValue();
                found.type = set.paramType().entry() != null ? set.paramType().entry() : found.type;
                replyParam(found);
            }
        }
    }

    private void command(EnumValue<MavCmd> raw, Object message) {
        MavCmd command = raw.entry();
        CommandReply reply = command == null ? null : commandScript.get(command);
        if (reply != null) {
            reply.answer(this, new CommandRequest(command, message));
        }
    }

    private StoredParam byName(String name) {
        for (StoredParam param : parameters) {
            if (param.name.equals(name)) {
                return param;
            }
        }
        return null;
    }

    private StoredParam byIndex(int index) {
        return index < parameters.size() ? parameters.get(index) : null;
    }

    private void replyParam(StoredParam param) {
        replyParamValue(param.name, param.value, param.type, parameters.indexOf(param), parameters.size());
    }

    private void emit(Object message) {
        LinkBundle bundle = this.current;
        if (signed) {
            MavlinkNode gcs = MavlinkNode.groundStation();
            bundle.writer().sendTo(message, bundle.link().id(), bundle.link().defaultTarget(),
                    new PeerId(gcs.system(), gcs.component()));
        } else {
            bundle.writer().broadcast(message, bundle.link().id());
        }
    }

    /**
     * Blocks until a frame whose payload is a {@code T} arrives, skipping anything else.
     *
     * @param type the payload class to wait for
     * @param timeout how long to wait
     * @param <T> the payload type
     * @return the payload
     * @throws InterruptedException if interrupted while waiting
     * @throws AssertionError if none arrives within {@code timeout}
     */
    public <T> T awaitFrame(Class<T> type, Duration timeout) throws InterruptedException {
        long deadlineNanos = System.nanoTime() + timeout.toNanos();
        while (true) {
            long remainingMillis = (deadlineNanos - System.nanoTime()) / 1_000_000L;
            if (remainingMillis <= 0) {
                break;
            }
            Frame frame = received.poll(remainingMillis, TimeUnit.MILLISECONDS);
            if (frame != null && frame.is(type)) {
                return frame.as(type);
            }
        }
        throw new AssertionError("expected a " + type.getSimpleName() + " within " + timeout);
    }

    /**
     * Like {@link #awaitFrame} for {@code RC_CHANNELS_OVERRIDE}, but keeps polling until channel 1
     * matches, skipping earlier ticks.
     *
     * @param expectedChan1 the channel 1 value to wait for
     * @param timeout how long to wait
     * @return the matching override
     * @throws InterruptedException if interrupted while waiting
     * @throws AssertionError if none matches within {@code timeout}
     */
    public RcChannelsOverride awaitOverrideWhereChan1Is(int expectedChan1, Duration timeout) throws InterruptedException {
        long deadlineNanos = System.nanoTime() + timeout.toNanos();
        while (true) {
            long remainingMillis = (deadlineNanos - System.nanoTime()) / 1_000_000L;
            if (remainingMillis <= 0) {
                break;
            }
            Frame frame = received.poll(remainingMillis, TimeUnit.MILLISECONDS);
            if (frame != null && frame.is(RcChannelsOverride.class)) {
                RcChannelsOverride override = frame.as(RcChannelsOverride.class);
                if (override.chan1Raw() == expectedChan1) {
                    return override;
                }
            }
        }
        throw new AssertionError("expected an RC_CHANNELS_OVERRIDE with chan1Raw=" + expectedChan1 + " within " + timeout);
    }

    /**
     * Polls for the next queued frame of any kind.
     *
     * @param timeout how long to wait
     * @return the frame, or {@code null} if none arrived
     * @throws InterruptedException if interrupted while waiting
     */
    public Frame pollAny(Duration timeout) throws InterruptedException {
        return received.poll(timeout.toMillis(), TimeUnit.MILLISECONDS);
    }

    /** Forgets every frame received so far. */
    public void clearReceived() {
        received.clear();
    }

    /**
     * Sends a {@code COMMAND_ACK}.
     *
     * @param command the command being acknowledged
     * @param result the result to report
     */
    public void replyAck(MavCmd command, MavResult result) {
        emit(CommandAck.builder().command(command).result(result).build());
    }

    /**
     * Sends a {@code COMMAND_ACK} with {@code IN_PROGRESS}.
     *
     * @param command the command being acknowledged
     * @param progress percent complete, 0..100
     */
    public void replyAckInProgress(MavCmd command, int progress) {
        emit(CommandAck.builder().command(command).result(MavResult.MAV_RESULT_IN_PROGRESS).progress(progress).build());
    }

    /**
     * Sends a {@code PARAM_VALUE} with the name taken verbatim rather than echoing the request, so a test
     * can answer a <i>different</i> parameter than the one asked for.
     *
     * @param paramId the parameter name to report
     * @param value its value
     * @param type its wire type
     * @param index its index
     * @param count the total parameter count to report
     */
    public void replyParamValue(String paramId, float value, MavParamType type, int index, int count) {
        emit(ParamValue.builder()
                .paramId(paramId)
                .paramValue(value)
                .paramType(type)
                .paramIndex(index)
                .paramCount(count)
                .build());
    }

    /**
     * Sends an {@code AUTOPILOT_VERSION}.
     *
     * @param flightSwVersion the packed firmware version
     * @param capabilityBits the raw capability bitmask, as the wire carries it
     * @param boardVersion the board version
     * @param vendorId the vendor id
     * @param productId the product id
     */
    public void replyAutopilotVersion(long flightSwVersion, int capabilityBits, long boardVersion,
                                      int vendorId, int productId) {
        emit(AutopilotVersion.builder()
                .capabilities(EnumValue.create(capabilityBits))
                .flightSwVersion(flightSwVersion)
                .boardVersion(boardVersion)
                .vendorId(vendorId)
                .productId(productId)
                .build());
    }

    /** Stops the heartbeat and reader threads and closes the socket. */
    @Override
    public void close() {
        stopped.set(true);
        current.link().close();
        try {
            if (heartbeatThread != null) {
                heartbeatThread.join(JOIN_TIMEOUT.toMillis());
            }
            if (readerThread != null) {
                readerThread.join(JOIN_TIMEOUT.toMillis());
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
