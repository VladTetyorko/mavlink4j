/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.sitl;

import io.mavlink4j.CompId;
import io.mavlink4j.MavlinkNode;
import io.mavlink4j.PeerId;
import io.mavlink4j.SysId;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.service.CapabilityService;
import io.mavlink4j.service.CommandService;
import io.mavlink4j.service.ParameterService;
import io.mavlink4j.session.MavlinkSession;
import io.mavlink4j.transport.UdpListenLink;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A session listening on a host UDP port plus the SITL containers pushing to it, with the services
 * wired the way an application would. Closing stops the containers first, then the session.
 */
final class SitlRig implements AutoCloseable {

    /** SITL needs to boot and settle before it heartbeats; generous because CI runners are slow. */
    static final Duration BOOT = Duration.ofSeconds(90);
    static final Duration CALL = Duration.ofSeconds(30);

    private static final Duration POLL = Duration.ofMillis(200);

    final MavlinkSession session;
    final CommandService commands;
    final ParameterService parameters;
    final CapabilityService capabilities;

    private final int port;
    private final List<SitlContainer> containers = new ArrayList<>();

    private SitlRig(MavlinkSession session, int port, MavlinkSettings settings) {
        this.session = session;
        this.port = port;
        this.commands = new CommandService(session.sink(), session.correlator(), settings);
        this.parameters = new ParameterService(session.sink(), session.correlator(), settings);
        this.capabilities = new CapabilityService(session.sink(), session.correlator(), settings);
    }

    /** Binds an ephemeral UDP port on all interfaces, so containers can reach it through the host gateway. */
    static SitlRig open() throws Exception {
        UdpListenLink link = new UdpListenLink("0.0.0.0", 0);
        MavlinkSettings settings = MavlinkSettings.defaults();
        MavlinkSession session = new MavlinkSession(MavlinkNode.groundStation(), settings, Signing.off());
        session.addLink(link);
        String id = link.id().value();
        return new SitlRig(session, Integer.parseInt(id.substring(id.lastIndexOf(':') + 1)), settings);
    }

    /** Starts one vehicle with the given sysid; instance numbers follow start order. */
    SitlRig withVehicle(int sysid) {
        containers.add(SitlContainer.start(port, sysid, containers.size()));
        return this;
    }

    /** The autopilot component of a vehicle, which is what a heartbeat identifies. */
    static PeerId autopilot(int sysid) {
        return new PeerId(new SysId(sysid), new CompId(1));
    }

    /** Waits until the vehicle's heartbeat has been heard; fails the test if it never is. */
    PeerId awaitVehicle(int sysid) throws InterruptedException {
        PeerId id = autopilot(sysid);
        long deadline = System.nanoTime() + BOOT.toNanos();
        while (System.nanoTime() < deadline) {
            Optional<?> heard = session.peers().peer(id).flatMap(peer -> peer.heartbeat());
            if (heard.isPresent()) {
                return id;
            }
            Thread.sleep(POLL.toMillis());
        }
        throw new AssertionError("no heartbeat from sysid " + sysid + " within " + BOOT);
    }

    @Override
    public void close() {
        containers.forEach(SitlContainer::close);
        session.close();
    }
}
