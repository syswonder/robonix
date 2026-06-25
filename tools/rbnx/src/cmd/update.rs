// SPDX-License-Identifier: MulanPSL-2.0
//! `rbnx update` — pull remote (`url:`) providers to their latest upstream
//! commit. Two modes, both gated on a y/N confirmation after an overview:
//!
//!   * deploy dir (cwd has `robonix_manifest.yaml`, or `-f <manifest>`):
//!     update every cloned `url:` provider in that deploy.
//!   * package dir (`-p <dir>`, or cwd is itself a git checkout): update just
//!     that one repo.
//!
//! Equivalent to a `git pull` (fast-forward) of each checkout. Diverged
//! checkouts are reported and skipped, never force-reset.

use anyhow::Result;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::Command;

use robonix_cli::{Config, output};

use super::check_remotes::{self, RemoteProvider, RemoteStatus};

/// Entry point for `rbnx update [-p <dir>] [-f <manifest>]`.
pub async fn execute(_config: Config, path: Option<PathBuf>, file: Option<PathBuf>) -> Result<()> {
    if let Some(p) = path {
        return update_single(&p);
    }
    let manifest = match file {
        Some(f) => f,
        None => {
            let cwd = std::env::current_dir()?;
            let m = cwd.join("robonix_manifest.yaml");
            if m.is_file() {
                m
            } else if cwd.join(".git").is_dir() {
                return update_single(&cwd);
            } else {
                anyhow::bail!(
                    "no robonix_manifest.yaml in {} and cwd is not a git checkout.\n\
                     Run from a deploy dir, or pass -f <manifest> / -p <package dir>.",
                    cwd.display()
                );
            }
        }
    };
    update_deploy(&manifest)
}

/// Run git in `dir`, returning trimmed stdout on success.
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
    Some(String::from_utf8_lossy(&out.stdout).trim().to_string())
}

/// Fetch the remote branch tip into FETCH_HEAD (deepened so behind-counts work
/// on the shallow clones boot creates). Returns false when fetch fails.
fn fetch(dir: &Path, branch: &str) -> bool {
    Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(["fetch", "--quiet", "--depth", "200", "origin", branch])
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn prompt_yes(question: &str) -> Result<bool> {
    print!("{question} [y/N]: ");
    io::stdout().flush()?;
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let a = input.trim().to_lowercase();
    Ok(a == "y" || a == "yes")
}

/// Fast-forward `dir` to FETCH_HEAD. Returns Ok(false) when it cannot (diverged).
fn fast_forward(dir: &Path) -> Result<bool> {
    let ok = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(["merge", "--ff-only", "FETCH_HEAD"])
        .status()?
        .success();
    Ok(ok)
}

/// Update a single package checkout (overview → confirm → fast-forward).
fn update_single(dir: &Path) -> Result<()> {
    if !dir.join(".git").is_dir() {
        anyhow::bail!("{} is not a git checkout", dir.display());
    }
    let branch = git(dir, &["rev-parse", "--abbrev-ref", "HEAD"]).unwrap_or_else(|| "HEAD".into());
    let local = git(dir, &["rev-parse", "--short", "HEAD"]).unwrap_or_default();

    output::boot_section(&format!("update {}", dir.display()));
    output::sub_step(&format!("branch {branch}  local {local}"));
    output::action("fetch", "origin");
    if !fetch(dir, &branch) {
        anyhow::bail!("git fetch failed (offline?)");
    }

    let remote = git(dir, &["rev-parse", "--short", "FETCH_HEAD"]).unwrap_or_default();
    if remote.is_empty() || remote == local {
        output::success("already up to date");
        return Ok(());
    }
    let behind = git(dir, &["rev-list", "--count", "HEAD..FETCH_HEAD"]);
    let subject = git(dir, &["log", "-1", "--format=%s", "FETCH_HEAD"]).unwrap_or_default();
    let date = git(
        dir,
        &["log", "-1", "--format=%cd", "--date=relative", "FETCH_HEAD"],
    )
    .unwrap_or_default();
    output::sub_step(&format!(
        "remote {remote} ({date}): {subject}{}",
        behind
            .map(|n| format!("   [{n} commit(s) behind]"))
            .unwrap_or_default()
    ));

    if !prompt_yes("Pull to latest?")? {
        output::info("skipped");
        return Ok(());
    }
    if fast_forward(dir)? {
        output::success(&format!("updated → {remote}"));
    } else {
        anyhow::bail!("fast-forward failed — local checkout diverged; resolve manually");
    }
    Ok(())
}

/// Update every outdated cloned `url:` provider in a deploy manifest.
fn update_deploy(manifest: &Path) -> Result<()> {
    let cloned: Vec<RemoteProvider> = check_remotes::collect_remote_providers(manifest)?
        .into_iter()
        .filter(|p| p.dir.join(".git").is_dir())
        .collect();
    if cloned.is_empty() {
        output::info("no cloned remote providers in this deploy — nothing to update");
        return Ok(());
    }

    output::boot_section(&format!("update remote providers — {}", manifest.display()));
    let statuses: Vec<RemoteStatus> = cloned.iter().map(check_remotes::status_of).collect();
    for st in &statuses {
        let detail = match (&st.note, st.behind) {
            (Some(note), _) => note.clone(),
            (None, Some(0)) => "up to date".to_string(),
            (None, Some(n)) => format!(
                "{n} behind → {} ({}): {}",
                st.remote_short, st.remote_date, st.remote_subject
            ),
            (None, None) if st.outdated() => format!(
                "behind → {} ({}): {}",
                st.remote_short, st.remote_date, st.remote_subject
            ),
            (None, None) => "up to date".to_string(),
        };
        output::boot_note(&st.name, &detail);
    }

    let outdated: Vec<&RemoteStatus> = statuses.iter().filter(|s| s.outdated()).collect();
    if outdated.is_empty() {
        output::success("all remote providers up to date");
        return Ok(());
    }
    if !prompt_yes(&format!("Pull {} package(s) to latest?", outdated.len()))? {
        output::info("skipped");
        return Ok(());
    }
    for st in outdated {
        output::action("pull", &st.name);
        match fast_forward(&st.dir) {
            Ok(true) => output::success(&format!("{} → {}", st.name, st.remote_short)),
            Ok(false) => output::warning(&format!(
                "{}: fast-forward failed (diverged) — skipped",
                st.name
            )),
            Err(e) => output::warning(&format!("{}: {e}", st.name)),
        }
    }
    Ok(())
}
