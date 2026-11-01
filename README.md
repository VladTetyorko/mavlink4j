# mavlink4j

> **Pre-release (0.1.0-SNAPSHOT).** The API may change in any `0.x` minor; every break is listed in
> [CHANGELOG.md](CHANGELOG.md). Nothing is published to Maven Central yet.

A framework-free MAVLink 2 library for the JVM: UDP, TCP and serial transports, frame codec with message
signing, a session that routes by `(sysid, compid)`, and services for the command, parameter, heartbeat,
manual-control and message-interval microservices. Apache-2.0. Java 17 target. Message classes come from
[`io.dronefleet:mavlink`](https://github.com/dronefleet/mavlink); everything else is this library.

**Naming note:** the Maven groupId is `io.github.vladtetyorko` (a GitHub-verified Central namespace) while the Java
packages are rooted at `io.mavlink4j`. The mismatch is deliberate; a bought domain may replace the groupId at 1.0.

## Install

```xml
<dependency>
  <groupId>io.github.vladtetyorko</groupId>
  <artifactId>mavlink4j-core</artifactId>
  <version>0.1.0-SNAPSHOT</version>
</dependency>
<!-- optional: serial ports via jSerialComm (brings native code) -->
<dependency>
  <groupId>io.github.vladtetyorko</groupId>
  <artifactId>mavlink4j-serial</artifactId>
  <version>0.1.0-SNAPSHOT</version>
</dependency>
```

| Artifact | Contents |
|---|---|
| `mavlink4j-core` | transports (UDP, TCP, serial SPI), codec with signing, session, services; no natives |
| `mavlink4j-serial` | `SerialLinks.open(port, baud)` over jSerialComm |
| `mavlink4j-testkit` | test doubles: scripted `FakeVehicle`, `RecordingLink`, `LoopbackLinkPair`, `ManualInstantSource`, `DirectTxScheduler` |

Build from source: `./mvnw -B verify` (Java 17+).

## Hello world

Listen on UDP 14550, print heartbeats, arm vehicle 1.

```java
import io.mavlink4j.*;
import io.mavlink4j.config.MavlinkSettings;
import io.mavlink4j.codec.Signing;
import io.mavlink4j.service.*;
import io.mavlink4j.session.*;
import io.mavlink4j.transport.UdpListenLink;
import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.common.MavCmd;

MavlinkSettings settings = MavlinkSettings.defaults();
try (UdpListenLink link = new UdpListenLink("0.0.0.0", 14550);
     MavlinkSession session = new MavlinkSession(MavlinkNode.groundStation(), settings, Signing.off())) {

    session.addLink(link);                       // starts the reader thread for this link

    session.dispatcher().subscribe(
            MessageFilter.type(Heartbeat.class),
            frame -> System.out.println("heartbeat from " + frame.header().system()));

    CommandService commands = new CommandService(session.sink(), session.correlator(), settings);
    PeerId vehicle = new PeerId(new SysId(1), new CompId(1));
    CommandResult result = commands
            .send(vehicle, Command.of(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM).param1(1))
            .join();                             // fails with PeerUnreachableException if never heard
    System.out.println(result.status());         // ACCEPTED, DENIED, NO_ACK, ...
}
```

`session.close()` stops the reader threads but does not close the links you registered; the
try-with-resources above closes both. Handlers run on the reader thread, so keep them fast; use
`subscribeAsync` for slow consumers.

## Examples

Five runnable `main`s in `examples/` (package `io.mavlink4j.examples`). Each takes `--fake`, which starts a
testkit `FakeVehicle` in-process: no hardware, no Docker, bounded runtime, exit 0 on success. Without
`--fake` it talks to a real endpoint. Common options: `--bind host:port` (default `0.0.0.0:14550`) and
`--timeout seconds` (default 10).

```bash
./mvnw -B -pl examples -am install -DskipTests
./mvnw -q -pl examples exec:java -Dexec.mainClass=io.mavlink4j.examples.ListenHeartbeats -Dexec.args=--fake
```

| Example | Shows | Extra options (real endpoint) |
|---|---|---|
| `ListenHeartbeats` | `(sysid, compid, VehicleKind, mode)` per heartbeat via `MessageFilter.type` | `--count N` (default 2) |
| `ArmWithAck` | `COMMAND_LONG` arm, `CommandResult` incl. the `IN_PROGRESS` extension and `NO_ACK` | `--sysid N`; refuses without `--confirm-arm` because it really arms |
| `ReadWriteParameter` | read `MAV_SYSID`, write `RC1_MIN` and show `MISMATCH` on a clamp | `--sysid N`, `--read NAME`, `--write NAME=VALUE` (REAL32, default `RC1_MIN=900`) |
| `TwoVehiclesOneSocket` | two vehicles on one `UdpListenLink`, a targeted command to each reaches only that vehicle | `--sysids A,B` (default `1,2`; sends a disarm to each) |
| `SignedLink` | key resolver + a policy requiring valid signatures; `SignatureRejection` for an unsigned frame and a replay | `--key` (64 hex chars, a 32-byte key); listens for `--timeout` s and prints rejections |

`examples/` is never published (`maven.deploy.skip` and `skipPublishing`), and `ExamplesTest` runs every
example with `--fake` as part of `verify`.

## Documentation

- [Architecture](docs/architecture.md): layers and data path
- [Threading](docs/threading.md): which thread runs what
- [Signing](docs/signing.md): MAVLink 2 message signing
- [Transports](docs/transports.md): UDP, TCP, serial, writing your own link
- [Testing](docs/testing.md): deterministic time and the fake vehicle
- [API.md](docs/API.md): the layer contract, per-layer types and the rules each seam must honour

## Status

Implemented: transports (UDP listen/target, TCP client, serial), codec, MAVLink 2 signing, session,
command, parameter read/write-with-readback, heartbeat, manual control (`RC_CHANNELS_OVERRIDE`),
message interval, capability probe (`AUTOPILOT_VERSION`). Not implemented: mission protocol, file
transfer protocol.

## License

Apache-2.0, see [LICENSE](LICENSE) and [NOTICE](NOTICE).
