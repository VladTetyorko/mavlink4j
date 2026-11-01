/*
 * Copyright 2026 Vlad Tetyorko and mavlink4j contributors
 * SPDX-License-Identifier: Apache-2.0
 */
package io.mavlink4j.sitl;

import com.github.dockerjava.api.model.HostConfig;
import org.testcontainers.DockerClientFactory;
import org.testcontainers.containers.GenericContainer;
import org.testcontainers.images.builder.ImageFromDockerfile;
import org.testcontainers.utility.DockerImageName;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Duration;

/**
 * One running ArduPilot SITL container, built from {@code sitl/docker/Dockerfile}.
 *
 * <p>SITL is the UDP client: it pushes MAVLink 2 to a socket the test binds on the host, reached as
 * {@code host.docker.internal} (mapped to the Docker host gateway so it works on Linux runners too).
 * Instances differ by sysid and by {@code INSTANCE}, which keeps their internal ports apart.
 *
 * <p>The image is taken from the local daemon when CI has already built and tagged it
 * ({@value #PREBUILT_TAG}, layer-cached there); otherwise Testcontainers builds it from the Dockerfile.
 * Nothing builds ArduPilot — the Dockerfile downloads the project's prebuilt SITL binary.
 */
final class SitlContainer implements AutoCloseable {

    /** Tag the CI job gives the image it builds with a layer cache. */
    static final String PREBUILT_TAG = "mavlink4j-sitl:4.7.0";

    private static final Path DOCKER_DIR = Paths.get("docker");
    private static final Duration START_TIMEOUT = Duration.ofSeconds(60);

    private final GenericContainer<?> container;

    private SitlContainer(GenericContainer<?> container) {
        this.container = container;
    }

    /** True when a Docker daemon answers; tests {@code assumeTrue} on it so they skip rather than fail. */
    static boolean dockerAvailable() {
        try {
            return DockerClientFactory.instance().isDockerAvailable();
        } catch (RuntimeException | LinkageError e) {
            return false;
        }
    }

    /**
     * Starts a copter SITL that pushes to {@code hostPort} on the Docker host.
     *
     * @param sysid    MAVLink system id the vehicle uses
     * @param instance SITL instance number; distinct per concurrently running container
     */
    static SitlContainer start(int hostPort, int sysid, int instance) {
        GenericContainer<?> container = new GenericContainer<>(imageName())
                .withEnv("VEHICLE", "copter")
                .withEnv("SYSID_THISMAV", Integer.toString(sysid))
                .withEnv("INSTANCE", Integer.toString(instance))
                .withEnv("MAVLINK_TARGET_HOST", "host.docker.internal")
                .withEnv("MAVLINK_TARGET_PORT", Integer.toString(hostPort))
                .withCreateContainerCmdModifier(cmd -> {
                    HostConfig hostConfig = cmd.getHostConfig() == null ? HostConfig.newHostConfig() : cmd.getHostConfig();
                    cmd.withHostConfig(hostConfig.withExtraHosts("host.docker.internal:host-gateway"));
                })
                .withStartupTimeout(START_TIMEOUT);
        container.start();
        return new SitlContainer(container);
    }

    private static DockerImageName imageName() {
        return DockerImageName.parse(prebuiltPresent() ? PREBUILT_TAG : buildImage());
    }

    private static boolean prebuiltPresent() {
        try {
            return !DockerClientFactory.lazyClient().listImagesCmd().withImageNameFilter(PREBUILT_TAG).exec().isEmpty();
        } catch (RuntimeException e) {
            return false;
        }
    }

    private static String buildImage() {
        return new ImageFromDockerfile("mavlink4j-sitl-local", false)
                .withFileFromPath("Dockerfile", DOCKER_DIR.resolve("Dockerfile"))
                .withFileFromPath("entrypoint.sh", DOCKER_DIR.resolve("entrypoint.sh"))
                .withFileFromPath("autofly.py", DOCKER_DIR.resolve("autofly.py"))
                .get();
    }

    @Override
    public void close() {
        container.stop();
    }
}
