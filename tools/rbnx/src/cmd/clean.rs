// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx clean` — drop build artifacts.
//
//   rbnx clean [-p <pkg>]      remove <pkg>/rbnx-build/. Defaults to the
//                              package containing the current directory.
//   rbnx clean -f <manifest>   recursively clean every package the manifest
//                              references (path: + url: + system/*) plus the
//                              deploy's rbnx-boot/{logs,state.json}.
//   rbnx clean -f <m> --cache  also wipe rbnx-boot/cache/ (force re-clone).

use anyhow::{Context, Result};
use robonix_cli::config::Config;
use robonix_cli::output;
use serde_yaml::Value;
use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};

use super::run_package;

/// Ask the user (y/N) whether to retry the failed paths with `sudo rm -rf`.
/// Returns Ok(()) if the retry succeeded or the user opted out cleanly.
/// Returns Err if sudo failed or the user declined and we want to surface
/// that as a non-zero exit.
fn prompt_sudo_retry(failed: Vec<(PathBuf, std::io::Error)>) -> Result<()> {
    if failed.is_empty() {
        return Ok(());
    }
    eprintln!();
    eprintln!("These paths could not be removed (likely docker-build root-owned files):");
    for (p, e) in &failed {
        eprintln!("  ✗ {} : {}", p.display(), e);
    }
    eprintln!();
    eprint!(
        "Retry with `sudo rm -rf` for the {} path(s) above? [y/N] ",
        failed.len()
    );
    io::stderr().flush().ok();
    let mut line = String::new();
    io::stdin().lock().read_line(&mut line)?;
    let yes = matches!(line.trim().to_ascii_lowercase().as_str(), "y" | "yes");
    if !yes {
        anyhow::bail!(
            "{} path(s) skipped (run `sudo rm -rf <path>` manually if needed)",
            failed.len()
        );
    }
    let mut cmd = std::process::Command::new("sudo");
    cmd.arg("rm").arg("-rf");
    for (p, _) in &failed {
        cmd.arg(p);
    }
    let status = cmd
        .status()
        .context("failed to spawn sudo (is it installed?)")?;
    if !status.success() {
        anyhow::bail!("sudo rm exited with {:?}", status.code());
    }
    Ok(())
}

pub async fn execute(
    config: Config,
    package: Option<PathBuf>,
    file: Option<PathBuf>,
    cache: bool,
) -> Result<()> {
    match (package, file) {
        (Some(_), Some(_)) => {
            anyhow::bail!("pass one of -p / -f, not both")
        }
        (Some(pkg), None) => clean_package(&pkg),
        (None, Some(f)) => clean_deploy(&config, &f, cache),
        (None, None) => {
            // Default-mode resolution. cwd-local hints in priority order:
            //   1. ./robonix_manifest.yaml   → deploy clean (sibling of `rbnx boot -f`)
            //   2. ./package_manifest.yaml or ancestor → package clean
            //   3. neither → bail with both options surfaced
            let cwd = std::env::current_dir().context("get cwd")?;
            let deploy = cwd.join("robonix_manifest.yaml");
            if deploy.is_file() {
                return clean_deploy(&config, &deploy, cache);
            }
            match run_package::find_package_from_cwd() {
                Ok(pkg) => clean_package(&pkg),
                Err(_) => anyhow::bail!(
                    "no robonix_manifest.yaml in {} and no package_manifest.yaml in any parent. \
                     Pass `-f <manifest>` (deploy) or `-p <pkg>` (package).",
                    cwd.display()
                ),
            }
        }
    }
}

fn clean_package(pkg: &Path) -> Result<()> {
    let pkg = pkg
        .canonicalize()
        .with_context(|| format!("not a directory: {}", pkg.display()))?;
    let build = pkg.join("rbnx-build");
    if !build.exists() {
        output::sub_step(&format!(
            "nothing to clean: {} (no rbnx-build/)",
            pkg.display()
        ));
        return Ok(());
    }
    output::action("Cleaning", &build.display().to_string());
    if let Err(e) = std::fs::remove_dir_all(&build) {
        // Common case: docker-build left root-owned files. Ask the user
        // whether to retry with sudo rather than bailing or silently
        // running sudo on their behalf.
        return prompt_sudo_retry(vec![(build, e)]);
    }
    Ok(())
}

fn clean_deploy(config: &Config, manifest_path: &Path, also_cache: bool) -> Result<()> {
    let manifest_path = manifest_path
        .canonicalize()
        .with_context(|| format!("manifest not found: {}", manifest_path.display()))?;
    let manifest_dir = manifest_path
        .parent()
        .context("deploy manifest has no parent directory")?
        .to_path_buf();
    let raw = std::fs::read_to_string(&manifest_path)
        .with_context(|| format!("read {}", manifest_path.display()))?;
    let root: Value =
        serde_yaml::from_str(&raw).with_context(|| format!("parse {}", manifest_path.display()))?;
    let cache_root = manifest_dir.join("rbnx-boot").join("cache");

    output::action("Cleaning deploy", &manifest_path.display().to_string());

    // Collect every package directory referenced by the manifest.
    let mut pkgs: Vec<PathBuf> = Vec::new();
    for section in &["primitive", "service", "skill"] {
        let Some(seq) = root.get(*section).and_then(|v| v.as_sequence()) else {
            continue;
        };
        for entry in seq {
            let local = entry.get("path").and_then(|v| v.as_str());
            let url = entry.get("url").and_then(|v| v.as_str());
            match (local, url) {
                (Some(p), _) => pkgs.push(manifest_dir.join(p)),
                // Cache dir = git repo name (one clone per repo), not the
                // per-instance provider id. See deploy::repo_dir_name.
                (None, Some(u)) => pkgs.push(cache_root.join(super::deploy::repo_dir_name(u))),
                _ => {}
            }
        }
    }
    // system: section — non-builtin entries are real packages under
    // `<robonix_source>/system/<key>/` (memory/scene/speech/…).
    const SYSTEM_BUILTINS: &[&str] = &["atlas", "executor", "pilot", "liaison", "soma", "vitals"];
    if let Some(map) = root.get("system").and_then(|v| v.as_mapping())
        && let Some(source_root) = config.robonix_source_path.as_ref()
    {
        for (key, _) in map {
            let Some(k) = key.as_str() else { continue };
            if SYSTEM_BUILTINS.contains(&k) {
                continue;
            }
            let pkg = source_root.join("system").join(k);
            if pkg.exists() {
                pkgs.push(pkg);
            }
        }
    }

    // Per-package cleanup. Tolerate per-package failures (docker-build
    // packages often leave root-owned rbnx-build/ that requires sudo).
    output::step("packages", &format!("{} to inspect", pkgs.len()));
    let mut failed: Vec<(PathBuf, std::io::Error)> = Vec::new();
    for p in &pkgs {
        if !p.exists() {
            continue;
        }
        let build = p.join("rbnx-build");
        if build.exists() {
            output::sub_step(&format!("rm -rf {}", build.display()));
            if let Err(e) = std::fs::remove_dir_all(&build) {
                failed.push((build, e));
            }
        }
    }

    // Deploy-level cleanup. Walk every entry under <manifest>/rbnx-boot/
    // and remove it. Skip cache/ unless `--cache` was given (re-cloning
    // url-fetched packages is expensive).
    let rbnx_boot = manifest_dir.join("rbnx-boot");
    if rbnx_boot.exists() {
        if let Ok(entries) = std::fs::read_dir(&rbnx_boot) {
            for entry in entries.flatten() {
                let path = entry.path();
                let name = entry.file_name();
                let is_cache = name == "cache";
                if is_cache && !also_cache {
                    output::sub_step(&format!("keep {} (use --cache to wipe)", path.display()));
                    continue;
                }
                let label = if is_cache { " (cache)" } else { "" };
                let kind = entry.file_type().ok();
                if kind.map(|k| k.is_dir()).unwrap_or(false) {
                    output::sub_step(&format!("rm -rf {}{}", path.display(), label));
                    if let Err(e) = std::fs::remove_dir_all(&path) {
                        failed.push((path, e));
                    }
                } else {
                    output::sub_step(&format!("rm {}{}", path.display(), label));
                    if let Err(e) = std::fs::remove_file(&path) {
                        failed.push((path, e));
                    }
                }
            }
        }
        // If rbnx-boot/ is empty (or only cache/ left when !also_cache),
        // try to rmdir it too. Best-effort — no failure surface.
        let _ = std::fs::remove_dir(&rbnx_boot);
    }

    prompt_sudo_retry(failed)
}
