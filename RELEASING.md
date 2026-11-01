# Releasing

A release is a tag. `release.yml` runs on `v*`, signs, publishes to Maven Central through the Central
Portal, pushes the aggregate javadoc to `gh-pages`, and creates the GitHub Release from `CHANGELOG.md`.

## One-time owner setup (B1)

1. **Central Portal account** at <https://central.sonatype.com>, and verify the namespace
   `io.github.vladtetyorko` (the portal asks for a temporary public repository named with a token).
2. **Portal user token** (Account, Generate User Token): gives a username and a password.
3. **GPG key** for signing; publish its public half to a keyserver (`keys.openpgp.org`) so Central can
   verify signatures.
   `gpg --armor --export-secret-keys <KEYID>` is the value of `MAVEN_GPG_PRIVATE_KEY`.
4. **GitHub Pages**: Settings, Pages, source `gh-pages` branch (created by the first release).

## Repository secrets

| Secret | Value |
|---|---|
| `MAVEN_GPG_PRIVATE_KEY` | ASCII-armored private key |
| `MAVEN_GPG_PASSPHRASE` | its passphrase |
| `CENTRAL_TOKEN_USERNAME` | portal user-token username |
| `CENTRAL_TOKEN_PASSWORD` | portal user-token password |

`GITHUB_TOKEN` is provided by Actions; nothing else is needed.

## Cutting a release

1. Move the `CHANGELOG.md` entries under a `## [x.y.z]` heading (the release notes are cut from it).
2. `git tag vx.y.z && git push origin vx.y.z`. The workflow sets every module version from the tag.
3. The deployment lands in the portal **un-published** (`central.autoPublish=false`); review it at
   <https://central.sonatype.com/publishing> and press Publish. Pass `-Dcentral.autoPublish=true` in
   `release.yml` once the first release has gone through cleanly.

Published modules: `mavlink4j-core`, `mavlink4j-serial`, `mavlink4j-testkit`. `mavlink4j-sitl` and the
examples are never published (`maven.deploy.skip`; `sitl` also sets `skipPublishing`).

## Local dry run

`./mvnw -B -Prelease verify` signs locally and needs the key in your keyring; without it, run plain
`./mvnw -B verify`, which already builds the sources and javadoc jars Central requires.
