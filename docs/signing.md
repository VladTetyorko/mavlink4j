# MAVLink 2 signing

Implements [MAVLink 2 message signing](https://mavlink.io/en/guide/message_signing.html): a 13-byte
signature (link id, 48-bit timestamp, 6-byte HMAC-SHA256 prefix) appended to a frame, keyed by a shared
32-byte secret. Signing gives authenticity and replay protection, never confidentiality.

## Pieces

`Signing` bundles three host-supplied collaborators; `Signing.off()` disables the feature.

| Piece | Role | Implementer must guarantee |
|---|---|---|
| `SigningKeyResolver` | `Optional<byte[]> keyFor(int sysid)`: the key for a remote system | fast, non-blocking, thread-safe, never throws; empty means "no key known" |
| `SigningPolicy` | `boolean accept(int sysid, SignatureStatus)`: dispatch or drop | fast, thread-safe, never throws; never accepts `INVALID` |
| `SigningClocks` / `SigningClock` | per-remote-system timestamp counter | monotonic; durable if replay protection must survive restarts |

Numeric tuning (windows, stale tolerance, persist interval, stream bound) lives in
`MavlinkSettings.SigningSettings`.

## Outbound

`FrameWriter` signs a frame addressed to a system when the resolver has a key for it, taking the
timestamp from that system's `SigningClock.next()`. A broadcast on a link that carries several keyed
systems cannot carry one signature valid for all of them, so `RoutingFrameSink` signs it only when the
link carries exactly one keyed system and sends it plain otherwise.

## Inbound

`FrameReader` hands every signed frame to `SignatureVerifier`, which yields a `SignatureStatus`:

| Status | Meaning |
|---|---|
| `UNSIGNED` | signed bit not set (never a rejection by itself) |
| `VALID` | signed, key known, HMAC and timestamp accepted |
| `UNVERIFIABLE` | signed, but no key known for the origin yet |
| `INVALID` | signed and rejected |

The `SigningPolicy` then decides whether the frame is dispatched. `SigningPolicy.PERMISSIVE` accepts
everything except `INVALID`.

## Rejection reasons

Every drop is reported as a `SignatureRejection(link, system, component, messageId, status, reason, ...)`
to the listener set with `MavlinkSession.onFrameRejected`; key material is never logged.

| `SignatureRejection.Reason` | Cause |
|---|---|
| `BAD_SIGNATURE` | the HMAC did not verify |
| `REPLAY` | the timestamp did not increase for this (system, component, signing link id) stream |
| `STALE_TIMESTAMP` | the timestamp was further behind the freshness reference than the tolerance |
| `POLICY` | the verdict was acceptable on its face but the `SigningPolicy` refused it |

## Timestamp clock

The timestamp counts 10 microsecond ticks since 2015-01-01, not the Unix
epoch. The spec requires it to be monotonic, advanced on every send, persisted, and on startup the
maximum of the clock-implied value and the stored one. `SigningClock` implements exactly that through
`loadPersisted` / `persist` callbacks (the library has no storage of its own).

Two values, two jobs: `current()` is this side's outbound sequence (seeded from the maximum of system
time and the stored value); `floor()` is the freshness reference for inbound frames (the highest
timestamp accepted from, or handed to, that system, seeded from stored state only). A vehicle without
a real-time clock can only advance its clock from frames sent to it, so judging it against wall time
after a long restart would reject it forever. `SigningClocks.inMemory` persists nothing: replay windows
then reset on restart. A host needing durable protection supplies its own `SigningClocks`.
