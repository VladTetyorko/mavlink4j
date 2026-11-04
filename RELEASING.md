# Releasing

A release is a tag on the `release` branch. `release.yml` signs, publishes to Maven Central through the Central
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

## Release gates

Nothing reaches Maven Central unless every gate passes, in this order:

| Gate | Where | Stops |
|---|---|---|
| `release` accepts only PRs, merge commits only, no force push or deletion | ruleset `release branch` | direct pushes, rewritten history |
| A PR into `release` must come from `main` of this repository | CI job `release-pr-source` | feature branches and forks going straight to a release |
| Full test matrix (Linux JDK 17/21, Windows, macOS) and SITL | CI jobs `verify`, `sitl`, required on the PR | broken or platform-specific code |
| `release-preflight.sh`: Central bundle has exactly parent + core/serial/testkit with sources and javadoc, no `skipPublishing`, aggregate javadoc builds | CI job `release-preflight`, required on the PR | an upload that silently skips, ships sitl/examples, or lacks javadoc; a broken gh-pages step |
| `actionlint` on all workflows | CI job `actionlint`, required on the PR | workflow typos, bad expressions, shell mistakes |
| The release workflow runs only on `release` (or a `v*` tag on a `release` commit) | `release.yml`, first step | releases from any other branch |
| A green `ci` run exists for the exact commit being released | `release.yml` | releasing a merge commit CI has not passed (or is still running) |
| Preflight again with the real version, before anything is signed or uploaded | `release.yml` | version-specific surprises |
| `v*` tags cannot be moved or deleted | ruleset `release tags` | rewriting a published version |

The tag is pushed only after the upload succeeded, and the plugin waits until the Portal has validated
the deployment, so a rejected bundle fails the run instead of passing silently.

Run the preflight locally before opening a PR to `release` (it works on a temporary copy):

    .github/scripts/release-preflight.sh

### Installing the rulesets (one time)

Settings, Rules, Rulesets, New ruleset, **Import a ruleset**, once per file in `.github/rulesets/`:
`release-branch.json`, `release-tags.json`, `main-branch.json`. The required check names in
`release-branch.json` are the CI job names; rename a job and you must update the ruleset too.

## Cutting a release

1. Move the `CHANGELOG.md` entries under a `## [x.y.z]` heading on `main` (the release notes are cut from it).
2. Open a PR from `main` into `release`. Wait for every required check, then merge it (merge commit).
3. Wait for the `ci` run on the `release` push to finish green.
4. Run **Actions, release, Run workflow** on `release` and pick `patch`/`minor`/`major` (or type an exact
   version). The workflow bumps the latest `v*` tag and pushes the new tag itself, after the upload.
5. The deployment lands in the portal **un-published** (`central.autoPublish=false`); review it at
   <https://central.sonatype.com/publishing/deployments> and press Publish.

Published modules: `mavlink4j-core`, `mavlink4j-serial`, `mavlink4j-testkit`. `mavlink4j-sitl` and the
examples are never published: the root pom's `release` profile lists them in the central plugin's `excludeArtifacts`. Do not set `skipPublishing` in a module: the bundle is uploaded from the last module (`sitl`), so it would cancel the whole upload.

## Local dry run

`.github/scripts/release-preflight.sh` builds the exact Central bundle without signing or uploading.
`./mvnw -B -Prelease verify` additionally signs, and needs the key in your keyring.
