// SPDX-License-Identifier: MulanPSL-2.0
//! Facts about this build, recorded by build.rs through vergen. A fact vergen
//! could not determine (git facts of a build without `.git`) is `None`.

use serde::Serialize;

use crate::output::short_sha;

#[derive(Serialize)]
pub struct BuildInfo {
    pub version: &'static str,
    pub git_sha: Option<&'static str>,
    pub git_describe: Option<&'static str>,
    pub git_commit_date: Option<&'static str>,
    pub git_dirty: Option<bool>,
    pub build_date: Option<&'static str>,
    pub rustc: Option<&'static str>,
    pub target: Option<&'static str>,
}

/// vergen's placeholder for a fact it could not determine.
fn known(value: &'static str) -> Option<&'static str> {
    (value != "VERGEN_IDEMPOTENT_OUTPUT").then_some(value)
}

impl BuildInfo {
    pub fn get() -> Self {
        Self {
            version: env!("CARGO_PKG_VERSION"),
            git_sha: known(env!("VERGEN_GIT_SHA")),
            git_describe: known(env!("VERGEN_GIT_DESCRIBE")),
            git_commit_date: known(env!("VERGEN_GIT_COMMIT_DATE")),
            git_dirty: known(env!("VERGEN_GIT_DIRTY")).map(|d| d == "true"),
            build_date: known(env!("VERGEN_BUILD_DATE")),
            rustc: known(env!("VERGEN_RUSTC_SEMVER")),
            target: known(env!("VERGEN_CARGO_TARGET_TRIPLE")),
        }
    }

    /// `0.1.0 (d9459885 2026-09-28, dirty, built 2026-09-30, rustc 1.95.0, x86_64-unknown-linux-gnu)`
    pub fn summary(&self) -> String {
        let or_unknown = |v: Option<&'static str>| v.unwrap_or("unknown");
        let dirty = if self.git_dirty == Some(true) {
            ", dirty"
        } else {
            ""
        };
        format!(
            "{} ({} {}{dirty}, built {}, rustc {}, {})",
            self.version,
            or_unknown(self.git_sha.map(short_sha)),
            or_unknown(self.git_commit_date),
            or_unknown(self.build_date),
            or_unknown(self.rustc),
            or_unknown(self.target),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn summary_shows_known_and_unknown_facts() {
        let mut info = BuildInfo {
            version: "0.1.0",
            git_sha: None,
            git_describe: None,
            git_commit_date: None,
            git_dirty: None,
            build_date: Some("2026-09-30"),
            rustc: Some("1.95.0"),
            target: Some("x86_64-unknown-linux-gnu"),
        };
        assert_eq!(
            info.summary(),
            "0.1.0 (unknown unknown, built 2026-09-30, rustc 1.95.0, x86_64-unknown-linux-gnu)"
        );
        info.git_sha = Some("d94598851f2a0c3e");
        info.git_commit_date = Some("2026-09-28");
        info.git_dirty = Some(true);
        assert_eq!(
            info.summary(),
            "0.1.0 (d9459885 2026-09-28, dirty, built 2026-09-30, rustc 1.95.0, x86_64-unknown-linux-gnu)"
        );
    }
}
