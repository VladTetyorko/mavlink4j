# Naming check: `mavlink4j`

Date: 2026-10-28. Result: **no collision found; name is free.**

| Registry | Query | Result |
|---|---|---|
| Maven Central search | `https://search.maven.org/solrsearch/select?q=a:mavlink4j*&rows=20&wt=json` | `numFound: 0` |
| Maven Central search | `...?q=mavlink4j&rows=20&wt=json` (free text, group/artifact/text) | `numFound: 0` |
| Maven Central repo listing | `repo.maven.apache.org/maven2/io/mavlink4j/` | HTTP 404 (no such group) |
| Maven Central repo listing | `repo.maven.apache.org/maven2/io/github/vladtetyorko/` | HTTP 404 (our groupId is unclaimed, namespace not yet registered) |
| Central Portal | `central.sonatype.com/api/internal/browse/components?name=mavlink4j` | HTTP 405 (internal endpoint not usable by curl); the Solr results above are authoritative |
| GitHub repository search | `api.github.com/search/repositories?q=mavlink4j+in:name` | `total_count: 0` |
| GitHub repository search | `...?q=mavlink4j` (name, description, topics, readme) | `total_count: 0` |
| GitHub | `api.github.com/repos/VladTetyorko/mavlink4j` | HTTP 404 (repo not yet created) |
| GitHub code search | `api.github.com/search/code?q=mavlink4j` | 401, needs authentication; not checked |

Caveats: Solr search indexes lag new uploads by hours; GitHub unauthenticated search may omit
very new or private repositories. Re-run both queries immediately before the 0.1.0 release.
