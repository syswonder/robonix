// SPDX-License-Identifier: MulanPSL-2.0
//! `rbnx update` — pull remote (`url:`) providers to their latest upstream
//! commit. Two modes, both gated on a y/N confirmation (or `--yes`) after an
//! overview; `--check` prints the overview of a deploy and pulls nothing:
//!
//!   * deploy dir (cwd has `robonix_manifest.yaml`, or `-f <manifest>`):
//!     update every cloned `url:` provider in that deploy.
//!   * package dir (`-p <dir>`, or cwd is itself a git checkout): update just
//!     that one repo.
//!
//! Equivalent to a `git pull` (fast-forward) of each checkout. Diverged
//! checkouts are reported and skipped, never force-reset.

use anyhow::Result;
use std::io::{self, IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::process::Command;

use robonix_cli::{Config, output};

use super::check_remotes::{self, RemoteProvider, RemoteStatus};

/// Entry point for `rbnx update [-p <dir>] [-f <manifest>] [-y] [--check [--json]]`.
pub async fn execute(
    _config: Config,
    path: Option<PathBuf>,
    file: Option<PathBuf>,
    yes: bool,
    check: bool,
    json: bool,
) -> Result<()> {
    if let Some(p) = path {
        return update_single(&p, yes);
    }
    let manifest = match file {
        Some(f) => f,
        None => {
            let cwd = std::env::current_dir()?;
            let m = cwd.join("robonix_manifest.yaml");
            if m.is_file() {
                m
            } else if check {
                anyhow::bail!(
                    "--check needs a deploy manifest: no robonix_manifest.yaml in {}; pass -f <manifest>",
                    cwd.display()
                );
            } else if cwd.join(".git").is_dir() {
                return update_single(&cwd, yes);
            } else {
                anyhow::bail!(
                    "no robonix_manifest.yaml in {} and cwd is not a git checkout.\n\
                     Run from a deploy dir, or pass -f <manifest> / -p <package dir>.",
                    cwd.display()
                );
            }
        }
    };
    if json {
        println!("{}", check_json(&manifest)?);
        return Ok(());
    }
    update_deploy(&manifest, yes, check)
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

/// Ask before pulling. Without a terminal nobody can answer, so refuse rather
/// than read whatever stdin holds.
fn prompt_yes(question: &str, yes: bool) -> Result<bool> {
    if yes {
        return Ok(true);
    }
    if !io::stdin().is_terminal() {
        anyhow::bail!("cannot ask for confirmation: stdin is not a terminal (pass --yes to pull)");
    }
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
fn update_single(dir: &Path, yes: bool) -> Result<()> {
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

    if !prompt_yes("Pull to latest?", yes)? {
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
fn update_deploy(manifest: &Path, yes: bool, check: bool) -> Result<()> {
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
                output::short_sha(&st.remote_sha),
                st.remote_date,
                st.remote_subject
            ),
            (None, None) if st.outdated() => format!(
                "behind → {} ({}): {}",
                output::short_sha(&st.remote_sha),
                st.remote_date,
                st.remote_subject
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
    if check {
        return Ok(());
    }
    if !prompt_yes(
        &format!("Pull {} package(s) to latest?", outdated.len()),
        yes,
    )? {
        output::info("skipped");
        return Ok(());
    }
    for st in outdated {
        output::action("pull", &st.name);
        match fast_forward(&st.dir) {
            Ok(true) => output::success(&format!(
                "{} → {}",
                st.name,
                output::short_sha(&st.remote_sha)
            )),
            Ok(false) => output::warning(&format!(
                "{}: fast-forward failed (diverged) — skipped",
                st.name
            )),
            Err(e) => output::warning(&format!("{}: {e}", st.name)),
        }
    }
    Ok(())
}

/// `--check --json`: every `url:` provider in the deploy, cloned or not.
/// Unknown values are null.
fn check_json(manifest: &Path) -> Result<serde_json::Value> {
    let known = |s: &str| (!s.is_empty()).then(|| s.to_string());
    let packages: Vec<_> = check_remotes::collect_remote_providers(manifest)?
        .iter()
        .map(|p| {
            let st = check_remotes::status_of(p);
            serde_json::json!({
                "name": p.name,
                "kind": p.kind,
                "dir": p.dir,
                "url": p.url,
                "branch": p.branch,
                "cloned": p.dir.join(".git").is_dir(),
                "local_sha": known(&st.local_sha),
                "remote_sha": known(&st.remote_sha),
                "behind": st.behind,
                "remote_date": known(&st.remote_date),
                "remote_subject": known(&st.remote_subject),
                "note": st.note,
            })
        })
        .collect();
    Ok(serde_json::json!({ "manifest": manifest, "packages": packages }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_json_lists_uncloned_provider() {
        let dir = std::env::temp_dir().join(format!("rbnx-update-check-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let manifest = dir.join("robonix_manifest.yaml");
        std::fs::write(
            &manifest,
            "service:\n  - name: map\n    url: https://example.com/org/service-map-rbnx.git\n",
        )
        .unwrap();

        let report = check_json(&manifest).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();

        let pkg = &report["packages"][0];
        assert_eq!(report["packages"].as_array().unwrap().len(), 1);
        assert_eq!(pkg["name"], "map");
        assert_eq!(pkg["kind"], "service");
        assert_eq!(pkg["url"], "https://example.com/org/service-map-rbnx.git");
        assert_eq!(pkg["cloned"], false);
        assert!(pkg["branch"].is_null());
        assert!(pkg["local_sha"].is_null());
        assert!(pkg["behind"].is_null());
        assert!(pkg["note"].as_str().unwrap().contains("not cloned"));
    }
}
