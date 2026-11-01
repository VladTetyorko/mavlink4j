# Security Policy

## Reporting a vulnerability

Report privately through GitHub: *Security -> Report a vulnerability* (private security advisory) on this
repository. Do not open a public issue. Expect an acknowledgement within a week.

## Supported versions

Pre-release (`0.x`): only the latest release receives fixes.

## Threat model in one paragraph

MAVLink 2 signing gives message **authenticity and replay protection**, never confidentiality: frames are
readable by anyone on the link. Replay windows rely on timestamps the host must persist across restarts
(`SigningClock`); a host that forgets them reopens the window. Unsigned links trust every sender on the wire.
Signing keys are supplied by the host and are never logged or stored by the library.
