// SPDX-License-Identifier: MulanPSL-2.0
//! Remote-provider freshness check.
//!
//! Deploy manifests pull some providers from a git `url:` into
//! `rbnx-boot/cache/<name>/`. Once cloned, that checkout is reused on every
//! boot/build and never advances on its own — so a teammate can be running
//! a stale copy of an upstream package without noticing. This module compares
//! each cloned provider against its remote branch tip and reports how far
//! behind it is.
//!
//! `rbnx boot` and `rbnx build` call [`report_outdated`] (print-only, never
//! fatal — a missing network must not block a deploy). `rbnx update` calls
//! [`collect_remote_providers`] + [`status_of`] to act on the same data.

use anyhow::{Context, Result};
use std::path::{Path, PathBuf};
use std::process::Command;

use robonix_cli::output;

/// A remote-backed provider declared in a deploy manifest.
#[derive(Debug, Clone)]
pub struct RemoteProvider {
    pub name: String,
    pub branch: Option<String>,
    /// Local cache checkout (`rbnx-boot/cache/<name>`); may not exist yet.
    pub dir: PathBuf,
}

/// How a local checkout compares to its remote branch tip.
#[derive(Debug, Clone)]
pub struct RemoteStatus {
    pub name: String,
    pub dir: PathBuf,
    /// Commits the local checkout is behind the remote tip. `None` when the
    /// count could not be determined (e.g. shallow history, fetch failed).
    pub behind: Option<u32>,
    pub local_short: String,
    pub remote_short: String,
    pub remote_date: String,
    pub remote_subject: String,
    /// Non-fatal explanation when the check is incomplete.
    pub note: Option<String>,
}

impl RemoteStatus {
    /// True when the remote tip differs from the local HEAD (i.e. an update
    /// is available). Behind-count may be unknown but still outdated.
    pub fn outdated(&self) -> bool {
        self.behind.map(|b| b > 0).unwrap_or(false)
            || (!self.remote_short.is_empty() && self.remote_short != self.local_short)
    }
}

/// Run a git command in `dir`, returning trimmed stdout on success.
fn git(dir: &Path, args: &[&str]) -> Option<String> {
    let out = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(args)
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let s = String::from_utf8(out.stdout).ok()?.trim().to_string();
    Some(s)
}

fn origin_url(dir: &Path) -> Option<String> {
    git(dir, &["config", "--get", "remote.origin.url"]).filter(|s| !s.trim().is_empty())
}

/// Parse a deploy manifest and list its `url:` providers (cloned or not).
pub fn collect_remote_providers(manifest_path: &Path) -> Result<Vec<RemoteProvider>> {
    let raw = std::fs::read_to_string(manifest_path)
        .with_context(|| format!("read deploy manifest {}", manifest_path.display()))?;
    let doc: serde_yaml::Value = serde_yaml::from_str(&raw)
        .with_context(|| format!("parse deploy manifest {}", manifest_path.display()))?;

    let manifest_dir = manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let cache_root = manifest_dir.join("rbnx-boot").join("cache");

    let mut providers = Vec::new();
    for section in ["primitive", "service", "skill"] {
        let Some(entries) = doc.get(section).and_then(|v| v.as_sequence()) else {
            continue;
        };
        for entry in entries {
            let Some(url) = entry.get("url").and_then(|v| v.as_str()) else {
                continue; // local `path:` provider — nothing to track
            };
            let name = entry
                .get("name")
                .and_then(|v| v.as_str())
                .map(|s| s.to_string())
                .filter(|s| !s.is_empty())
                .unwrap_or_else(|| {
                    url.trim_end_matches(".git")
                        .rsplit('/')
                        .next()
                        .unwrap_or("pkg")
                        .to_string()
                });
            let branch = entry
                .get("branch")
                .and_then(|v| v.as_str())
                .map(|s| s.to_string());
            providers.push(RemoteProvider {
                // Cache dir = git repo name (one clone per repo), not the
                // per-instance provider id. See deploy::repo_dir_name.
                dir: cache_root.join(super::deploy::repo_dir_name(url)),
                name,
                branch,
            });
        }
    }
    Ok(providers)
}

/// Resolve the branch to compare against — the manifest's `branch:` or the
/// remote's default branch (`origin/HEAD`).
fn target_branch(p: &RemoteProvider) -> String {
    if let Some(b) = &p.branch {
        return b.clone();
    }
    git(&p.dir, &["rev-parse", "--abbrev-ref", "origin/HEAD"])
        .and_then(|s| s.rsplit('/').next().map(|s| s.to_string()))
        .unwrap_or_else(|| "main".to_string())
}

/// Fetch the remote branch and compare it to the local checkout. Best-effort:
/// network / shallow-history problems surface as a `note`, not an error.
pub fn status_of(p: &RemoteProvider) -> RemoteStatus {
    let mut st = RemoteStatus {
        name: p.name.clone(),
        dir: p.dir.clone(),
        behind: None,
        local_short: String::new(),
        remote_short: String::new(),
        remote_date: String::new(),
        remote_subject: String::new(),
        note: None,
    };

    if !p.dir.join(".git").is_dir() {
        st.note = Some("not cloned yet (will clone on next boot/build)".into());
        return st;
    }

    st.local_short = git(&p.dir, &["rev-parse", "--short", "HEAD"]).unwrap_or_default();
    if origin_url(&p.dir).is_none() {
        st.note = Some("local checkout has no origin remote — skipped".into());
        return st;
    }
    let branch = target_branch(p);

    // Deepen the fetch so HEAD..origin/branch is computable even on the
    // --depth 1 clones boot creates. 200 covers any realistic drift.
    //
    // HARD time bound: this freshness check is a best-effort, non-fatal
    // notice, but a plain `git fetch` against an unreachable/slow remote
    // can block on TCP connect for ~130s — and since boot/build call this
    // for EVERY url-provider, an offline or throttled github stalls the
    // whole boot before atlas even starts. Wrap in `timeout(1)` so a
    // wedged fetch is abandoned in seconds and the check just reports
    // "skipped". `http.lowSpeedLimit/Time` additionally kills a transfer
    // that connects but then stalls mid-stream. If the `timeout` binary
    // isn't present (e.g. non-coreutils host), fall back to a bare fetch.
    let fetch_args = [
        "-C",
        p.dir.to_str().unwrap_or("."),
        "-c",
        "http.lowSpeedLimit=1000",
        "-c",
        "http.lowSpeedTime=8",
        "fetch",
        "--quiet",
        "--depth",
        "200",
        "origin",
        &branch,
    ];
    // Keep this short: the freshness notice is cosmetic and must never
    // noticeably delay boot. 6s is plenty for a reachable remote; an
    // unreachable one fails fast and the check is skipped.
    let timed = Command::new("timeout")
        .arg("6")
        .arg("git")
        .args(fetch_args)
        .status();
    let fetched = match timed {
        Ok(s) => s.success(),
        // `timeout` unavailable — fall back to a direct fetch.
        Err(_) => Command::new("git")
            .args(fetch_args)
            .status()
            .map(|s| s.success())
            .unwrap_or(false),
    };
    if !fetched {
        st.note = Some("git fetch timed out / failed (offline?) — skipped".into());
        return st;
    }

    let remote_ref = "FETCH_HEAD";
    st.remote_short = git(&p.dir, &["rev-parse", "--short", remote_ref]).unwrap_or_default();
    st.remote_date = git(
        &p.dir,
        &["log", "-1", "--format=%cd", "--date=relative", remote_ref],
    )
    .unwrap_or_default();
    st.remote_subject = git(&p.dir, &["log", "-1", "--format=%s", remote_ref]).unwrap_or_default();
    st.behind = git(
        &p.dir,
        &["rev-list", "--count", &format!("HEAD..{remote_ref}")],
    )
    .and_then(|s| s.parse::<u32>().ok());
    st
}

/// boot/build hook: print a one-line notice for every outdated remote provider
/// plus how to update. Never fails the caller.
pub fn report_outdated(manifest_path: &Path) {
    let providers = match collect_remote_providers(manifest_path) {
        Ok(p) => p,
        Err(_) => return,
    };
    let remote: Vec<_> = providers
        .into_iter()
        .filter(|p| p.dir.join(".git").is_dir())
        .collect();
    if remote.is_empty() {
        return;
    }

    // Each provider triggers a real `git fetch` (up to 6s on a slow/offline
    // remote), serially. Show an in-place spinner per package while its fetch
    // runs, then overwrite it with a definite verdict — `[ OK ] up to date`
    // or `[ ↑ ] N behind …` — so nothing is left dangling on a `…`. Skip the
    // whole phase with `rbnx boot --no-update-check`.
    output::boot_section("checking remote providers for updates (skip: --no-update-check)");
    let mut outdated = Vec::new();
    for p in &remote {
        if origin_url(&p.dir).is_none() {
            let st = status_of(p);
            output::boot_skip(
                &p.name,
                st.note
                    .as_deref()
                    .unwrap_or("local checkout has no origin remote — skipped"),
            );
            continue;
        }
        output::boot_progress(&p.name, "fetching origin…", 0);
        let st = status_of(p);
        if st.outdated() {
            let behind = match st.behind {
                Some(n) => format!("{n} behind"),
                None => "behind".to_string(),
            };
            output::boot_update_avail(
                &p.name,
                &format!(
                    "{behind}; remote {} ({}): {}",
                    st.remote_short, st.remote_date, st.remote_subject
                ),
            );
            outdated.push(st);
        } else if let Some(note) = &st.note {
            // Couldn't determine (offline / shallow / fetch failed) — say so
            // plainly rather than implying it's up to date.
            output::boot_skip(&p.name, note);
        } else {
            output::boot_ok(&p.name, "up to date");
        }
    }

    // Final, unambiguous verdict line.
    if outdated.is_empty() {
        output::sub_step(&format!(
            "{} remote provider(s) up to date — nothing to update",
            remote.len()
        ));
        return;
    }
    output::sub_step(&format!(
        "{} of {} provider(s) have updates available:",
        outdated.len(),
        remote.len()
    ));
    output::info(&format!(
        "update all:  rbnx update -f {}",
        manifest_path.display()
    ));
    output::info("update one:  cd <pkg dir> && rbnx update   (or: rbnx update -p <pkg dir>)");
}
