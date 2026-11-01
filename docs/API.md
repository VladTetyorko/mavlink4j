# mavlink4j-core — API contract

This file is the layer contract; the per-type signatures are the javadoc (the source of truth).
Narrative docs: [architecture](architecture.md), [threading](threading.md), [signing](signing.md),
[transports](transports.md), [testing](testing.md).

A reusable, framework-free MAVLink 2 library with no knowledge of any host application's domain model.
One third-party dependency: `io.dronefleet.mavlink:mavlink`. Package root `io.mavlink4j`.

## Levels

| Level | Package | Depends on |
|---|---|---|
| L0 kernel | `io.mavlink4j` (root) | nothing — `SysId`, `CompId`, `PeerId`, `VehicleKind` |
| L1 transport | `io.mavlink4j.transport` | L0 |
| L2 codec | `io.mavlink4j.codec` | L0, L1 |
| L3 session | `io.mavlink4j.session` | L0, L1, L2 |
| L4 service | `io.mavlink4j.service` | L0, L3 |
| — config | `io.mavlink4j.config` | nothing |

Above L0, no level may reach two levels down. L0 and `config` are universal and may be depended on by
anything — they are this module's own kernel. Enforced by ArchUnit.

**L0 — identity value types.** `SysId(int value)` and `CompId(int value)`, both validated 1..255 in their
compact constructors, and `PeerId(SysId system, CompId component)`. They live in the root package rather
than under `session` because L2's `FrameHeader` and `FrameSink` need them: a message's origin is a wire-level
fact, not a session-level one.

**L0 — `VehicleKind`.** `enum VehicleKind { COPTER, PLANE, ROVER, SUBMARINE, OTHER_VEHICLE, NOT_A_VEHICLE,
UNKNOWN }` — the one `HEARTBEAT.type` (`MAV_TYPE`) → family table, `static VehicleKind of(int mavType)` only
(display labels are the host's concern). It belongs at L0, not `service` or `config`, for the same reason
`SysId`/`PeerId` do: it is a wire-level protocol fact (what kind of airframe a `MAV_TYPE` number denotes)
usable by any level, not session or service state.

---

## L1 — transport

Bytes in, bytes out. Knows nothing about frames.

| Type | Kind | Signature / fields |
|---|---|---|
| `LinkId` | record | `String value` — e.g. `"udp-listen:0.0.0.0:14550"`. Stable across the link's life. |
| `LinkPeer` | record | `String host, int port`. `LinkPeer.NONE` for links with no addressable peer (serial). |
| `ByteChunk` | record | `byte[] data, int length, LinkPeer source` — no timestamp (L1 holds no clock; `FrameReader` stamps `Frame.receivedAt` from `MavlinkSettings.clock()`) |
| `MavlinkLink` | interface | `LinkId id()` · `boolean preservesMessageBoundaries()` · `ByteChunk poll(Duration timeout)` · `void send(byte[] frame, int off, int len, LinkPeer target)` · `LinkPeer defaultTarget()` · `void close()` |
| `LinkIntake` | record | `long datagramsReceived, long bytesReceived` (the last-datagram time is `MavlinkSession.lastReceivedAt(LinkId)`) (see the note below the table) |

**`poll` returns `null` on timeout** — named for `BlockingQueue.poll`'s established contract, so the return
value carries its own documentation. It throws `IOException` only on genuine socket failure; a closed link
returns `null` rather than throwing.

**`preservesMessageBoundaries()` is the LSP hinge.** The interface contract is *"a byte stream with
buffered resync"*, which a datagram link satisfies trivially and a stream link satisfies correctly. It must
**never** be read as "one poll = one message" — that promise would make `TcpClientLink` an unimplementable
subtype. The flag exists only so `FrameReader` can decide how many resync buffers to keep, never so a
caller can skip resync.

Implementations: `UdpListenLink` (bind, learn peers from inbound), `UdpTargetLink` (fixed destination,
for the TX simulator), `TcpClientLink`. `SerialLink(SerialChannel)`; the jSerialComm backend lives in the `mavlink-serial` artifact (`io.mavlink4j.serial.SerialLinks.open(port, baud)`).

**Send is thread-safe on every implementation.** Multiple services share one link concurrently
(`DatagramSocket#send` already is; TCP needs a write lock).

**`UdpListenLink.intake(): LinkIntake`.** A dead UDP port
never errors — `DatagramSocket#receive` on a port nothing transmits to blocks forever, no timeout, no
log (a genuine silent failure this module cannot fix at the socket, only report honestly around). Both
UDP link implementations share the counting (in package-private `UdpSocketIo`, incremented on every
successful `receive`, **pre-parse** — before `FrameReader`/`MavlinkConnection` ever see the bytes), but
only `UdpListenLink` exposes it publicly, since it is the shape a standing lobby/dead-port diagnostic
needs; `UdpTargetLink`/`TcpClientLink` gained no new public method. `datagramsReceived`/`bytesReceived`
are monotonic counters, never reset; Safe to call
from any thread (the counters are `AtomicLong`/`AtomicReference`, independent of the one-reader-thread
`poll()` contract). No behavior change to `poll()`/`send()`/`defaultTarget()` — this is bookkeeping
only, added at the one point ("just received these bytes off the wire") that is true regardless of
whether they ever decode into a valid frame. See `LinkIntake`'s own javadoc for why
`datagramsReceived` vs. a higher layer's `framesDecoded` is the actual diagnostic value.

---

## L2 — codec

Frames in, frames out. Knows nothing about who is out there.

| Type | Kind | Signature / fields |
|---|---|---|
| `FrameHeader` | record | `int version, int sequence, SysId system, CompId component, int messageId, int incompatFlags, int compatFlags, boolean signed` |
| `Frame` | record | `FrameHeader header, Object payload, LinkId link, LinkPeer source, Instant receivedAt` + `boolean is(Class<?>)` · `<T> T as(Class<T>)` |
| `FrameReader` | class | `void offer(ByteChunk chunk, Consumer<Frame> out)` |
| `FrameSink` | interface | `void send(Object message, PeerId target)` (throws unchecked `PeerUnreachableException` if the peer was never heard) · `void broadcast(Object message, LinkId link)` · `Set<LinkId> links()` · `default void broadcastAll(Object message)` (best-effort per link) |
| `FrameWriter` | class | explicit-address `sendTo(message, link, address, target)` and `broadcast(message, link)`; the **only** class in the module allowed to construct a `MavlinkConnection`. Not a `FrameSink` — `RoutingFrameSink` is |

**`Object payload` is a known wart**, accepted as a design decision (library message types flow through L2–L4).
**Settled — do not re-investigate.** Every generated message class in `io.dronefleet.mavlink` is a
plain `public final class` with no common interface or superclass; the only shared marker is the
class-level `@MavlinkMessageInfo` annotation, which cannot occupy a type position. `Object` stays.
`FrameWriter` uses that annotation defensively instead: a payload lacking it is rejected with a clear
`IllegalArgumentException` rather than an obscure NPE deep in `MavlinkConnection.send2`.

`FrameHeader.version` is the **semantic protocol number (1 or 2)**, not the raw magic byte.

**Routing lives in L3, not here.** `FrameWriter` is **link-scoped**: it owns one
persistent `MavlinkConnection` per link, which is what makes per-link sequence numbering work at all (a
fresh connection per send resets `seq` to 0 every time). It cannot answer "which link is this `PeerId`
reachable on" — that needs `PeerDirectory`. `RoutingFrameSink` (L3) resolves the peer's link through
`PeerDirectory` and delegates to that link's `FrameWriter`; `FrameWriter` keeps its narrow link-scoped
contract and is not a `FrameSink` in its own right.

**`FrameReader` keeps one resync buffer per source.** For a boundary-preserving link that means one buffer
per `LinkPeer`; for a stream link, exactly one. This is not an optimisation — concatenating datagrams from
different vehicles into one byte stream is the classic flaw of reading several vehicles through one
stream decoder. Buffers are bounded and evicted LRU
(`maxResyncBuffers`, default 64) so a spoofing or scanning source cannot grow memory without bound.

`FrameWriter` owns sequence-number allocation, **per `(link, our sysid, our compid)`**, wrapping at 255.

---

## L3 — session

Who is out there, what am I waiting for, what runs periodically.

(`SysId`, `CompId`, `PeerId` are L0 — see the level table above.)

| Type | Kind | Signature / fields |
|---|---|---|
| `HeartbeatInfo` | record | `int autopilot, int mavType, int baseMode, long customMode, int systemStatus` — `null` until the first `HEARTBEAT` |
| `Peer` | record | `PeerId id, LinkId link, LinkPeer address, Instant firstHeard, Instant lastHeard, Optional<HeartbeatInfo> heartbeat` (empty until the first `HEARTBEAT`) |
| `PeerDirectory` | interface | `Collection<Peer> peers()` · `Optional<Peer> peer(PeerId)` · `List<Peer> peersOnLink(LinkId)` — read-only facts |
| `MessageFilter` | interface | `boolean test(Frame)`; statics `any()`, `messageId(int)`, `type(Class<?>)`, `fromSystem(SysId)`, `and(...)` |
| `Dispatcher` | interface | `Subscription subscribe(MessageFilter, Consumer<Frame>)` · `AsyncSubscription subscribeAsync(MessageFilter, Consumer<Frame>)` (own drain thread, bounded drop-oldest queue; `AsyncSubscription.droppedCount()`) |
| `Subscription` | interface | `void close()` — idempotent |
| `MatchKey` | record | `SysId system, int messageId, long discriminator` |
| `CorrelationKeys` | class | the one place a correlated reply's key is defined, for **both** directions. Request side: `forCommandAck(SysId,int)` · `forParamValue(SysId,String)` · `forAutopilotVersion(SysId)`; reply side: a package-local class→`MatchKey` table. Also `normalizeParamId(String)` · `paramDiscriminator(String)` |
| `Correlator` | interface | `CompletableFuture<Frame> await(MatchKey, Duration)` · `void cancel(MatchKey)` |
| `TxScheduler` | interface | `Handle repeat(String name, Duration period, Runnable)` · `void submit(String, Runnable)` · `static TxScheduler pooled(MavlinkSettings)` · `close()` (no-op by default) · `Handle` has `void close()` |
| `LinkHealth` | interface | `Health of(PeerId)`; `record Health(PeerId peerId, boolean connected, Instant lastHeard, long received, long lost, double dropRate)` (see the note below the table) |
| `RoutingFrameSink` | class (package-private) | the `FrameSink` implementation that resolves a `PeerId`'s link and address via `PeerDirectory` and delegates to `FrameWriter.sendTo`; reached through `MavlinkSession.sink()` |
| `MavlinkNode` | record | in the root package `io.mavlink4j`; `SysId system, CompId component`; `groundStation()` returns the conventional ground-control identity, **255 / 190** |
| `MavlinkSession` | class | `MavlinkSession(MavlinkNode, MavlinkSettings, Signing)` · `LinkQuality linkQuality()` · `TxScheduler scheduler()` · `Set<LinkId> links()` · `Optional<Instant> lastReceivedAt(LinkId)` · `void addLink(MavlinkLink)` · `void removeLink(LinkId)` · `PeerDirectory peers()` · `Dispatcher dispatcher()` · `Correlator correlator()` · `FrameSink sink()` · `LinkHealth health()` · `void onLinkFailure(BiConsumer<LinkId, IOException> listener)` (see the note below the table) · `void close()` |

`MavlinkSession` is the composition root: it owns one reader thread per link, drives `FrameReader`, updates
`PeerDirectory` and `LinkHealth` from every frame, offers each frame to `Correlator` and then to
`Dispatcher`, in that order.

### Contracts that must be honoured exactly

- **`Dispatcher` handlers run on the RX thread and MUST NOT block.** The dispatcher does not queue and does
  not retry. A consumer that needs to buffer wraps itself in `BoundedSubscriber` (provided, drop-oldest,
  bounded capacity, counts drops). A message-broker producer subscribes exactly
  like any other consumer does, and a broker stall degrades the broker path only, never the RX loop.
- **`Correlator.await` must be registered before the request is sent**, and always released in a `finally`
  via `cancel`. A waiter registered and never removed both leaks and permanently shadows future replies for
  the same key — a classic sharp edge of ack registries.
- **`MatchKey.discriminator`** is the command id for `COMMAND_ACK`, the item seq for mission transfers, the
  session/seq for FTP, and a hash of the `param_id` for `PARAM_VALUE`. `COMMAND_ACK` is matched on
  `(origin sysid, command id)` only — `targetSystem`/`targetComponent` are wire extension fields and are
  **not** reliably populated; never match on them.
- **A new correlated message type is a row in `CorrelationKeys`, never an edit to `DefaultCorrelator`**
  The correlator owns registry mechanics only; key extraction is table-driven and
  compiled in — not a runtime plugin registry, which nothing needs.
- **`PARAM_VALUE` routes by hash and is verified by name.** A 16-character `param_id` does not fit in a
  64-bit discriminator, so `CorrelationKeys.paramDiscriminator` (FNV-1a 64 over the normalised name) routes
  the reply and `ParameterService` then compares the name exactly. **Both halves are required**: hashing
  alone would silently accept a colliding parameter's value. FNV-1a specifically because its constants can
  never drift, unlike `String#hashCode` (32-bit) or `Objects#hash` (no stability contract).
- **`PeerDirectory` records protocol facts only.** Which vehicle a host application pins, claims or
  re-elects is host policy, not library behaviour.
- **Peer identity is `(sysid, compid)`, never the transport address.** One link multiplexes several compids
  and NAT can move an address between packets from one system.
- **`LinkHealth` drop rate** is expected-vs-received `seq` accounting per `(PeerId, LinkId)`, handling the
  8-bit wrap. The MAVLink spec defines no formula; ours is documented, not standard.
- **`LinkHealth.Health` carries `PeerId peerId` as its first field.** A caller aggregating several peers'
  `Health` records (for example into a map keyed by its own device id) can then recover which peer a record
  belongs to without threading the `PeerId` alongside by hand.
- **`MavlinkSession.onLinkFailure(BiConsumer<LinkId, IOException> listener)`.**
  Reports a genuine `poll()` `IOException` on a link's reader thread: the transport itself has failed, as
  distinct from the far end merely going quiet (which is not a transport failure and correctly reports through
  `LinkHealth`/`PeerDirectory` staleness instead). The listener **fires synchronously on the dying reader
  thread and must not block or throw** (a throwing listener is caught and logged, not propagated) — the same
  threading discipline `Dispatcher` handlers already live under. It **never fires for a shutdown-triggered
  failure** (a poll unblocked by an intentional `close()`/`removeLink()` racing with a genuine socket error):
  `LinkRuntime` checks its own pre-existing `running` flag, already flipped by `stop()` before the join,
  rather than adding a second "are we shutting down" signal. It **never fires for a `RuntimeException`**
  from frame processing either — that error means one frame misbehaved, not that the transport is gone, and
  the reader loop continues to the next `poll()`. Default listener is a no-op, not `null`.

---

## L4 — services

One class per MAVLink microservice. Adding one must touch nothing below.

| Service | Responsibility | Family |
|---|---|---|
| `HeartbeatService` | emit our `HEARTBEAT` at a fixed rate; mark peers connected/disconnected | B — streaming |
| `CommandService` | `send(PeerId, Command)` — `Command.of(MavCmd)…` is a `COMMAND_LONG`, `Command.positioned(…)` a `COMMAND_INT`; `COMMAND_ACK` await, retry with `confirmation` increment, `IN_PROGRESS` extends the deadline. Returns `CommandResult(status, mavResult, resultCode, ack)` | A — request/response |
| `ManualControlService` | fixed-rate `RC_CHANNELS_OVERRIDE` relay, latest-wins mailbox, release burst | B — streaming |
| `MessageIntervalService` | `MAV_CMD_SET_MESSAGE_INTERVAL` (µs) / `MAV_CMD_REQUEST_MESSAGE`, over `CommandService` | A, via Command |
| `ParameterService` | `read`/`readAll` by **name** (`PARAM_REQUEST_READ`, `param_index = -1`), `write` as `PARAM_SET` **plus a mandatory read-back**. Returns `ParameterOutcome(OK\|MISMATCH\|NO_REPLY, ParameterValue, detail)` | A — request/response |
| `CapabilityService` | `MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)`, awaiting the **message**, not its ack. Returns `CapabilityReport(OK\|NO_REPLY, firmwareVersion, maturity, capabilities, …)` | A — request/response |

Services depend on **small role interfaces only** — `FrameSink`, `Correlator`, `PeerDirectory`,
`TxScheduler` — never on `MavlinkSession` as a whole, and never on `MavlinkConnection`. `ManualControlService`
must not even be able to *express* an ack wait: it gets `FrameSink` + `TxScheduler` and nothing else.

Family A's shared machine — *send X, expect Y matching key K within T, retry N times, duplicate response is
idempotent* — is one class, `RequestResponse`, parameterised per service. Future mission and FTP services reuse it.

**L4 result records.** Every service returns a value for every protocol-level outcome; a thrown exception
means a *send-level* fault only (unreachable target), never a "no" from the aircraft.

| Type | Kind | Signature / fields |
|---|---|---|
| `ParameterValue` | record | `String name, float value, MavParamType type, int index, int count`; `boolean isNamed(String)`. `name` is normalised; `index` is informational — never correlate on it |
| `ParameterOutcome` | record | `Status status, ParameterValue value, String detail`; `Status` = `OK \| MISMATCH \| NO_REPLY`; `boolean ok()` |
| `CapabilityReport` | record | `Status status, String firmwareVersion, Maturity maturity, Set<MavProtocolCapability> capabilities, long boardVersion, int vendorId, int productId, AutopilotVersion raw`; `Status` = `OK \| NO_REPLY`; `Maturity` = `DEV \| ALPHA \| BETA \| RC \| OFFICIAL \| UNKNOWN`; `boolean supports(MavProtocolCapability)` |

Three rules these records encode, so no caller re-derives them:

- **`MISMATCH` is why a write is a read-back.** An autopilot clamps an out-of-range value and truncates a
  float into the parameter's real integer width **without refusing the write**. The echoed `PARAM_VALUE` is
  the only evidence of what it now holds, so the comparison is part of the exchange — exact float compare,
  deliberately no epsilon (an epsilon hides exactly the small clamps that matter).
- **`NO_REPLY` is also how "I have no such parameter" arrives.** The protocol gives an autopilot no way to
  refuse a name it does not recognise. Unsupported and unreachable are one observation and are not guessed
  apart here.
- **`AUTOPILOT_VERSION` silence is a value, not a fault.** Betaflight and older firmware never implement it;
  report an incomplete profile, never a fabricated one.

---

## Configuration

`MavlinkSettings` — one framework-free record, compact-constructor validated, `static defaults()`,
`builder()`/`toBuilder()` (no `with*`). Every value here is configuration because the **MAVLink spec itself declines to
specify it**, not because we were being cautious.

| Field | Default | Source of the default |
|---|---|---|
| `heartbeatPeriod` | 1 s | convention; spec mandates no rate |
| `peerTimeout` | 5 s | convention "4–5 missed heartbeats" |
| `commandTimeout` | 2 s | ours; long enough for a telemetry-radio round trip |
| `commandRetries` | 2 | spec: "a flight-specific number of times" — implementation-defined |
| `rc.overrideHz` / `min` / `max` | 33 / 10 / 50 | ours; RC override pacing |
| `rc.releaseFrames` | 3 | ours; RC override pacing |
| `closeJoinTimeout` | 5 s | ours; bounded join on shutdown |
| `maxResyncBuffers` | 64 | ours; L2 eviction bound |
| `dispatchQueueCapacity` | 256 | ours; `BoundedSubscriber` default |
| `commandMaxInProgressExtensions` | 20 | ours; bounds `COMMAND_ACK` `IN_PROGRESS` deadline extensions |
| `pollTimeout` | 200 ms | ours; reader-thread `poll` slice |
| `txPoolSize` | 2 | ours; `TxScheduler` pool threads |
| `peer.flapWindow` / `warnInterval` | 3 s / 30 s | ours; address-flap warning |
| `parameter.maxForeignReplies` | 8 | ours |
| `clock` / `threadFactory` / `callbackExecutor` | system / `Thread::new` / direct | injected seams — the only place `Instant.now()`/`new Thread`/`Executors` appear |
| `parameter.timeout` / `retries` | 1 s / 3 | **spec numbers** — the parameter-protocol page states its own retry policy |

No `System.getenv`, no `System.getProperty`, no Spring anywhere in this module. Callers construct the
record themselves, from whatever configuration source they use.

---

## Non-goals — do not build these

Our own serializer (no CRC_EXTRA tables, no field reordering, no truncation logic — the underlying library does
it for ~500 messages). A general MAVLink router. A DI framework. Our own message DTOs. Any broker client.
Mission and FTP services (not yet built).

One more, deliberate:

- **The full parameter download (`PARAM_REQUEST_LIST`)** — the stream-me-all-~1200 protocol with its own
  missing-index gap detection and re-request loop. Typical callers (probes, readiness checks,
  remediation) name their parameters in advance, and a full download costs minutes of airtime on a
  2.4 kB/s link.
