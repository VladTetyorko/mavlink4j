# Changelog

All notable changes are recorded here. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning: [SemVer](https://semver.org/). While the version is `0.x`, a minor bump may break the public API.

## [Unreleased]

### Added
- Repository bootstrap: `core` (`mavlink4j-core`) and `serial` (`mavlink4j-serial`) imported from the originating
  project (see `PROVENANCE`); empty `testkit`, `examples`, `sitl` modules.
- `mavlink4j-testkit`: public `FakeVehicle` (scripted `onCommand`, `parameter`, `signedWith`), `RecordingLink`, `LoopbackLinkPair`, `ManualInstantSource`, `DirectTxScheduler`.
- Build gates: JaCoCo (85 % line / 75 % branch on core), javadoc doclint, maven-enforcer, ArchUnit layer rules.
- SPDX license header (`Apache-2.0`) on every Java source, enforced by Spotless in `verify`.

### Changed
- Javadoc and test comments no longer reference the originating project's internal plans and modules.
- Core-internal test doubles renamed (`UdpTestVehicle`, `SendCapturingLink`) to avoid confusion with the public testkit classes.
