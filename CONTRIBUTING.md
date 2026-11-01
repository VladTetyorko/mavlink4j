# Contributing

## Build

Java 17+ and the bundled wrapper:

    ./mvnw -B verify

This runs unit tests, ArchUnit layer rules, the JaCoCo gate (85 % line / 75 % branch on `core`), javadoc with
doclint, maven-enforcer (dependency convergence) and a Spotless check that every `.java` file starts with the
SPDX license header (`./mvnw spotless:apply` adds it; nothing else is reformatted). Two serial tests need `socat` on PATH and are skipped
otherwise (`apt-get install socat`).

## The layer rule

`io.mavlink4j` (L0 ids) <- `transport` (L1) <- `codec` (L2) <- `session` (L3) <- `service` (L4); `config` is
plain values. A lower layer never imports a higher one; `Default*` classes are package-private; time comes from an
injected `InstantSource` and threads from the configured `ThreadFactory`. `ArchitectureTest` enforces this.

## Pull requests

A PR needs a test (one that fails when the code is broken) and a line in `CHANGELOG.md`. No DCO or CLA. By
contributing you agree your work is licensed under Apache-2.0.

## Test tooling note

The optional SITL smoke tests (`sitl/`, `-Psitl`) run ArduPilot SITL in a container. ArduPilot is GPL-3.0; it is a
CI-only test tool and is not distributed with this library.
