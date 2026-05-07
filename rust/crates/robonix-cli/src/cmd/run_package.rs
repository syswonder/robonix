// SPDX-License-Identifier: MulanPSL-2.0
// Run package commands: build, start (start blocks until process exits).
//
// Dev-packaging contract: one package has ONE top-level `start` shell body
// (not a list of nodes). `rbnx start` just executes that body at the
// package root — the body itself is responsible for spawning processes
// and registering capabilities with atlas. No node-id flag.

use super::build;
use anyhow::{Context, Result};
use robonix_cli::Config;
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::process::ProcessManager;
use std::path::{Path, PathBuf};

/// Directory against which relative `-p` is resolved: **the pwd of the command invocation**.
/// When `cargo run` runs from `robonix/rust`, the process cwd is not the user's shell cwd — wrappers
/// should `export RBNX_INVOCATION_CWD="$(pwd)"` before `cd`+`cargo run`. If unset, `std::env::current_dir()` is used.
pub(crate) const RBNX_INVOCATION_CWD: &str = "RBNX_INVOCATION_CWD";

/// POSIX-shell single-quoted escape, used when we synthesise `export FOO=...`
/// fragments to inject into a package's `start` body.
fn shell_escape(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\"'\"'"))
}

fn path_base_for_dash_p() -> Result<PathBuf> {
    if let Ok(s) = std::env::var(RBNX_INVOCATION_CWD) {
        Ok(PathBuf::from(s))
    } else {
        std::env::current_dir().context("Failed to get current directory")
    }
}

/// Resolve `-p` to a filesystem path before `canonicalize`: relative paths and `.` use
/// [`path_base_for_dash_p`] as the prefix (invocation pwd, or process cwd).
pub(crate) fn resolve_local_path_for_filesystem(p: &Path) -> Result<PathBuf> {
    if p.as_os_str() == "." || p.as_os_str() == "./" {
        return path_base_for_dash_p();
    }
    if p.is_absolute() {
        return Ok(p.to_path_buf());
    }
    Ok(path_base_for_dash_p()?.join(p))
}

/// Walk up from the invocation cwd looking for a directory that contains
/// a `package_manifest.yaml`. Returns the first match.
pub(crate) fn find_package_from_cwd() -> Result<PathBuf> {
    let start = path_base_for_dash_p()?;
    let mut cur: Option<&Path> = Some(&start);
    while let Some(d) = cur {
        if d.join(manifest::MANIFEST_FILE).is_file() {
            return d
                .canonicalize()
                .with_context(|| format!("Failed to canonicalize: {}", d.display()));
        }
        cur = d.parent();
    }
    anyhow::bail!(
        "no {} found in {} or any parent; pass -p <path> or `cd` into a package directory",
        manifest::MANIFEST_FILE,
        start.display()
    )
}

/// Resolve package path from -p (local path) or -g (system-installed name).
/// When neither is given, walk up from cwd to find a package manifest.
fn resolve_package_path(
    config: &Config,
    path: Option<PathBuf>,
    global: Option<String>,
) -> Result<PathBuf> {
    if let Some(p) = path {
        let p = resolve_local_path_for_filesystem(&p)?;
        let canonical = p
            .canonicalize()
            .with_context(|| format!("Failed to canonicalize: {}", p.display()))?;
        if canonical.join(manifest::MANIFEST_FILE).exists() {
            return Ok(canonical);
        }
        anyhow::bail!(
            "Path {} does not contain {}",
            canonical.display(),
            manifest::MANIFEST_FILE
        );
    }

    if let Some(name) = global {
        let db = robonix_cli::PackageDatabase::load(&config.package_storage_path)?;
        if let Some(pkg) = db.get_package(&name) {
            return Ok(pkg.path.clone());
        }
        anyhow::bail!(
            "Package '{}' not found in system storage ({})",
            name,
            config.package_storage_path.display()
        );
    }

    find_package_from_cwd()
}

/// Resolve package path for `start`: same `-p` rules as `build`, then system-installed name fallback.
fn resolve_package_path_for_start(config: &Config, spec: &str) -> Result<PathBuf> {
    let path = resolve_local_path_for_filesystem(Path::new(spec))?;
    if path.join(manifest::MANIFEST_FILE).is_file()
        || path.join(manifest::LEGACY_MANIFEST_FILE).is_file()
    {
        return path
            .canonicalize()
            .with_context(|| format!("Failed to canonicalize: {}", path.display()));
    }

    let db = robonix_cli::PackageDatabase::load(&config.package_storage_path)?;
    if let Some(pkg) = db.get_package(spec) {
        return Ok(pkg.path.clone());
    }

    anyhow::bail!(
        "Package '{}' not found at {} (relative -p uses {} or process cwd). Try -g <installed name> or export {}=\"$(pwd)\" before cargo run.",
        spec,
        path.display(),
        RBNX_INVOCATION_CWD,
        RBNX_INVOCATION_CWD
    )
}

pub async fn execute_build(
    config: Config,
    path: Option<PathBuf>,
    global: Option<String>,
    clean: bool,
) -> Result<()> {
    // Deploy-manifest mode: if `path` (or cwd, when -p is omitted)
    // contains a `robonix_manifest.yaml`, build every primitive /
    // service / skill entry it lists. This lets the user run
    //   `cd examples/webots && rbnx build`
    // and get all packages built in one shot rather than chasing
    // each package directory by hand. The corresponding lookup for
    // `package_manifest.yaml` (single-package mode) stays as the
    // fallback below.
    let candidate_dir = match &path {
        Some(p) => Some(p.clone()),
        None => std::env::current_dir().ok(),
    };
    if let Some(dir) = candidate_dir {
        let deploy_manifest = dir.join("robonix_manifest.yaml");
        if deploy_manifest.is_file() {
            return build_deploy_manifest(&deploy_manifest, &config, clean);
        }
    }
    let package_root = resolve_package_path(&config, path, global)?;
    build::execute_local(package_root, clean).await
}

/// Build every package referenced by a top-level `robonix_manifest.yaml`.
/// Two phases:
///   1. **fetch** — `path:` entries already on disk; `url:` entries
///      get `git clone --depth 1` into `rbnx-boot/cache/<name>/`
///      (idempotent — skipped when the cache dir already exists).
///   2. **build** — for each resolved package, run its `build.sh`.
///
/// `rbnx boot` deliberately does NOT do either; it just verifies
/// both phases happened (warns + remediates if not). This lets
/// "fetch → build" be a controlled offline step the user can run
/// when they have network / time, then `rbnx boot` is a fast,
/// online-optional bring-up.
fn build_deploy_manifest(manifest_path: &Path, config: &Config, clean: bool) -> Result<()> {
    use serde_yaml::Value;
    let manifest_dir = manifest_path
        .parent()
        .context("deploy manifest has no parent directory")?
        .to_path_buf();
    let raw = std::fs::read_to_string(manifest_path)
        .with_context(|| format!("read {}", manifest_path.display()))?;
    let root: Value =
        serde_yaml::from_str(&raw).with_context(|| format!("parse {}", manifest_path.display()))?;
    let cache_root = manifest_dir.join("rbnx-boot").join("cache");

    output::action(
        "Building",
        &format!("packages declared in {}", manifest_path.display()),
    );

    // Collect (section, name, pkg_dir, url_to_clone) for every entry.
    struct Resolved {
        section: &'static str,
        name: String,
        pkg_dir: PathBuf,
        url_to_clone: Option<(String, Option<String>)>, // (url, branch)
    }
    let mut entries: Vec<Resolved> = Vec::new();
    for section in &["primitive", "service", "skill"] {
        let Some(seq) = root.get(*section).and_then(|v| v.as_sequence()) else {
            continue;
        };
        for entry in seq {
            let name = entry
                .get("name")
                .and_then(|v| v.as_str())
                .unwrap_or("(unnamed)")
                .to_string();
            let local_path = entry.get("path").and_then(|v| v.as_str());
            let url = entry.get("url").and_then(|v| v.as_str());
            let branch = entry
                .get("branch")
                .and_then(|v| v.as_str())
                .map(String::from);
            match (local_path, url) {
                (Some(p), _) => entries.push(Resolved {
                    section,
                    name,
                    pkg_dir: manifest_dir.join(p),
                    url_to_clone: None,
                }),
                (None, Some(u)) => entries.push(Resolved {
                    section,
                    name: name.clone(),
                    pkg_dir: cache_root.join(&name),
                    url_to_clone: Some((u.to_string(), branch)),
                }),
                (None, None) => {
                    output::warning(&format!(
                        "skipping {section}/{name}: entry has neither `path` nor `url`"
                    ));
                }
            }
        }
    }
    // `system:` non-builtin entries are real packages too (memory / scene
    // / speech / nexus / …), they just live under `<robonix_source>/system/<key>/`
    // instead of being declared with an explicit `path:` / `url:`. The
    // builtin Rust binaries (atlas / executor / pilot / liaison) are
    // shipped via `cargo install` and skipped here.
    const SYSTEM_BUILTINS: &[&str] = &["atlas", "executor", "pilot", "liaison"];
    if let Some(map) = root.get("system").and_then(|v| v.as_mapping()) {
        let source_root = config.robonix_source_path.as_ref();
        for (key, _value) in map {
            let Some(key_str) = key.as_str() else {
                continue;
            };
            if SYSTEM_BUILTINS.contains(&key_str) {
                continue;
            }
            let Some(source_root) = source_root else {
                output::warning(&format!(
                    "skipping system/{key_str}: robonix_source_path unset \
                     (run `rbnx setup` from the repo root once)"
                ));
                continue;
            };
            let pkg_dir = source_root.join("system").join(key_str);
            if !pkg_dir.exists() {
                output::warning(&format!(
                    "skipping system/{key_str}: not on disk at {}",
                    pkg_dir.display()
                ));
                continue;
            }
            entries.push(Resolved {
                section: "system",
                name: key_str.to_string(),
                pkg_dir,
                url_to_clone: None,
            });
        }
    }

    // Phase 1: fetch. git clone url-remote pkgs into cache.
    let to_clone: Vec<&Resolved> = entries
        .iter()
        .filter(|e| e.url_to_clone.is_some() && !e.pkg_dir.exists())
        .collect();
    if !to_clone.is_empty() {
        output::step("fetch", &format!("{} package(s)", to_clone.len()));
        std::fs::create_dir_all(&cache_root)?;
        for r in &to_clone {
            let (url, branch) = r.url_to_clone.as_ref().unwrap();
            output::sub_step(&format!("git clone {url} -> {}", r.pkg_dir.display()));
            let mut clone = std::process::Command::new("git");
            clone.arg("clone").arg("--depth").arg("1");
            if let Some(b) = branch {
                clone.arg("--branch").arg(b);
            }
            clone.arg(url).arg(&r.pkg_dir);
            let status = clone
                .status()
                .with_context(|| format!("git clone {url} failed to spawn"))?;
            if !status.success() {
                anyhow::bail!("git clone {url} exited with {:?}", status.code());
            }
        }
    }

    // Phase 2: build. Run build.sh for each resolved pkg.
    let mut built = 0usize;
    let mut skipped = 0usize;
    for r in &entries {
        if !r.pkg_dir.join("package_manifest.yaml").is_file() {
            output::warning(&format!(
                "skipping {}/{}: no package_manifest.yaml at {}",
                r.section,
                r.name,
                r.pkg_dir.display()
            ));
            skipped += 1;
            continue;
        }
        let canon = r
            .pkg_dir
            .canonicalize()
            .with_context(|| format!("canonicalize {}", r.pkg_dir.display()))?;
        output::step(r.section, &r.name);
        build::build_local_package(&canon, clean)?;
        built += 1;
    }
    output::success(&format!(
        "fetched {} / built {built} / skipped {skipped} / total {} from {}",
        to_clone.len(),
        entries.len(),
        manifest_path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("manifest")
    ));
    Ok(())
}

pub async fn execute_start(
    config: &Config,
    spec: Option<&str>,
    registry_endpoint: Option<&str>,
) -> Result<()> {
    let package_root = match spec {
        Some(s) => resolve_package_path_for_start(config, s)?,
        None => find_package_from_cwd()?,
    };
    let detected = manifest::detect_and_load(&package_root)?;
    let manifest = &detected.manifest;
    manifest.validate_and_summarize()?;

    let endpoint = registry_endpoint
        .map(String::from)
        .unwrap_or_else(|| "127.0.0.1:50051".to_string());

    let run_root = package_root
        .parent()
        .context("Package root has no parent")?;
    let log_dir = run_root.join("rbnx-boot").join("logs");
    let process_manager = ProcessManager::new(log_dir)?;

    output::action("Running", &manifest.package.name);
    output::sub_step(&format!("Atlas endpoint: {}", endpoint));
    if !manifest.capabilities.is_empty() {
        output::sub_step(&format!(
            "Capabilities: {}",
            manifest
                .capabilities
                .iter()
                .map(|c| c.name.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }

    let mut env = std::collections::HashMap::new();
    env.insert("ROBONIX_ATLAS".to_string(), endpoint.clone());
    // Force unbuffered stdout/stderr in any Python child the package's
    // start body launches. Without this, Python block-buffers stdout
    // when it's a pipe (which `rbnx boot` always makes it), and a
    // primitive whose driver is still alive never flushes its
    // `Driver(cmd=0) received` line until the buffer fills or the
    // process exits — so a 60-second boot full of "what is happening"
    // looks like the package wedged at "ready - awaiting Driver".
    // See `examples/webots/rbnx-boot/logs/primitive_tiago_camera.log`
    // for the diagnostic this turned up. Override with PYTHONUNBUFFERED=
    // (empty) in the manifest if a package really wants buffered output.
    env.entry("PYTHONUNBUFFERED".to_string())
        .or_insert_with(|| "1".to_string());

    if !manifest.build.trim().is_empty() && !build::build_stamp_path(&package_root).exists() {
        output::sub_step("No rbnx-build/.rbnx-built — running package build first");
        build::build_local_package(&package_root, false)?;
    }

    let exports = env
        .iter()
        .map(|(k, v)| format!("export {}={}", k, shell_escape(v)))
        .collect::<Vec<_>>()
        .join("; ");
    let start_body = manifest.start.trim();
    let setup_bash = package_root
        .join("rbnx-build")
        .join("ws")
        .join("install")
        .join("setup.bash");
    let setup_source = if setup_bash.exists() {
        format!("source {}", shell_escape(&setup_bash.display().to_string()))
    } else {
        String::new()
    };
    let prefix_parts: Vec<String> = [setup_source, exports]
        .into_iter()
        .filter(|s| !s.is_empty())
        .collect();
    let start_command = if prefix_parts.is_empty() {
        start_body.to_string()
    } else {
        format!("{}; {start_body}", prefix_parts.join("; "))
    };

    let result = process_manager
        .start_process(
            &manifest.package.name,
            &manifest.package.name,
            "package",
            &package_root,
            &start_command,
        )
        .await?;
    output::check(&format!(
        "{} exited (PID {})",
        manifest.package.name, result.pid
    ));

    output::success(&format!("Package {} finished", manifest.package.name));
    Ok(())
}
