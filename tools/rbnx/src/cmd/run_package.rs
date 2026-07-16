// SPDX-License-Identifier: MulanPSL-2.0
// Run package commands: build, start (start blocks until process exits).
//
// Dev-packaging contract: one package has ONE top-level `start` shell body
// (not a list of nodes). `rbnx start` just executes that body at the
// package root — the body itself is responsible for spawning processes
// and registering capabilities with atlas. No node-id flag.

use super::build;
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_cli::Config;
use robonix_cli::launch::{
    CMD_ACTIVATE, CMD_INIT, call_driver_cmd, snapshot_provider_ids, wait_for_registration_core,
};
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::process::ProcessManager;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

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
    file: Option<PathBuf>,
    path: Option<PathBuf>,
    global: Option<String>,
    clean: bool,
    no_update_check: bool,
) -> Result<()> {
    if let Some(file) = file {
        let manifest_path = resolve_local_path_for_filesystem(&file)?;
        if !manifest_path.is_file() {
            anyhow::bail!("deployment manifest not found: {}", manifest_path.display());
        }
        return build_deploy_manifest(&manifest_path, &config, clean, no_update_check);
    }

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
            return build_deploy_manifest(&deploy_manifest, &config, clean, no_update_check);
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
fn build_deploy_manifest(
    manifest_path: &Path,
    config: &Config,
    clean: bool,
    no_update_check: bool,
) -> Result<()> {
    use serde_yaml::Value;
    let manifest_dir = manifest_path
        .parent()
        .context("deploy manifest has no parent directory")?
        .to_path_buf();
    let raw = std::fs::read_to_string(manifest_path)
        .with_context(|| format!("read {}", manifest_path.display()))?;
    let root: Value =
        serde_yaml::from_str(&raw).with_context(|| format!("parse {}", manifest_path.display()))?;
    let root = super::deploy::prepare_manifest(root, config.robonix_source_path.as_deref())
        .with_context(|| format!("prepare {}", manifest_path.display()))?;
    let cache_root = manifest_dir.join("rbnx-boot").join("cache");

    output::action(
        "Building",
        &format!("packages declared in {}", manifest_path.display()),
    );
    // Notice (non-fatal) if any cloned remote provider is behind upstream.
    if !no_update_check {
        super::check_remotes::report_outdated(manifest_path);
    }

    // Collect (section, name, pkg_dir, url_to_clone) for every entry.
    struct Resolved {
        section: &'static str,
        name: String,
        pkg_dir: PathBuf,
        url_to_clone: Option<(String, Option<String>)>, // (url, branch)
        // Deploy `manifest:` override — selects a per-target package
        // manifest variant (e.g. package_manifest.jetson-native.yaml) so
        // the right build path runs. None = default package_manifest.yaml.
        manifest_override: Option<String>,
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
            let manifest_override = entry
                .get("manifest")
                .and_then(|v| v.as_str())
                .map(String::from);
            match (local_path, url) {
                (Some(p), _) => entries.push(Resolved {
                    section,
                    name,
                    pkg_dir: manifest_dir.join(p),
                    url_to_clone: None,
                    manifest_override,
                }),
                (None, Some(u)) => entries.push(Resolved {
                    section,
                    name: name.clone(),
                    // Cache dir = git repo name (one clone per repo), not the
                    // per-instance provider id. See deploy::repo_dir_name.
                    pkg_dir: cache_root.join(super::deploy::repo_dir_name(u)),
                    url_to_clone: Some((u.to_string(), branch)),
                    manifest_override,
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
    // / speech / …), they just live under `<robonix_source>/system/<key>/`
    // instead of being declared with an explicit `path:` / `url:`. The
    // builtin Rust binaries (atlas / executor / pilot / liaison / soma / vitals) are
    // shipped via `cargo install` and skipped here.
    const SYSTEM_BUILTINS: &[&str] = &["atlas", "executor", "pilot", "liaison", "soma", "vitals"];
    if let Some(map) = root.get("system").and_then(|v| v.as_mapping()) {
        let source_root = config.robonix_source_path.as_ref();
        for (key, value) in map {
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
            let (manifest_override, _runtime_config) = manifest::split_system_package_config(value)
                .with_context(|| format!("parse system/{key_str} package selector"))?;
            entries.push(Resolved {
                section: "system",
                name: key_str.to_string(),
                pkg_dir,
                url_to_clone: None,
                manifest_override,
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
    struct Row {
        section: &'static str,
        name: String,
        pkg_name: String, // reverse-domain `package.name` from package_manifest.yaml
        version: String,
        location: String, // path relative to manifest_dir, or absolute when outside
        source: Option<(String, Option<String>)>, // (git url, branch) for url-fetched
    }
    let mut built: Vec<Row> = Vec::new();
    let mut skipped: Vec<(&'static str, String, String)> = Vec::new(); // (section, name, reason)
    let mut failed: Vec<(&'static str, String, anyhow::Error)> = Vec::new();

    fn read_pkg_meta(pkg_dir: &Path) -> (String, String) {
        // Best-effort: parse package.name + package.version from manifest.
        let manifest = pkg_dir.join("package_manifest.yaml");
        let raw = match std::fs::read_to_string(&manifest) {
            Ok(s) => s,
            Err(_) => return (String::new(), String::new()),
        };
        let v: serde_yaml::Value = match serde_yaml::from_str(&raw) {
            Ok(v) => v,
            Err(_) => return (String::new(), String::new()),
        };
        let pkg = v.get("package");
        let name = pkg
            .and_then(|p| p.get("name").and_then(|n| n.as_str()))
            .unwrap_or("")
            .to_string();
        let ver = pkg
            .and_then(|p| p.get("version").and_then(|n| n.as_str()))
            .unwrap_or("")
            .to_string();
        (name, ver)
    }

    fn rel_to(_base: &Path, p: &Path) -> String {
        // Always show absolute (realpath) so the user can copy-paste straight
        // into a shell. The pkg_dir we get is already canonicalize()'d below.
        p.display().to_string()
    }

    for r in &entries {
        if !r.pkg_dir.join("package_manifest.yaml").is_file() {
            let reason = format!("no package_manifest.yaml at {}", r.pkg_dir.display());
            output::warning(&format!("skipping {}/{}: {}", r.section, r.name, reason));
            skipped.push((r.section, r.name.clone(), reason));
            continue;
        }
        let canon = r
            .pkg_dir
            .canonicalize()
            .with_context(|| format!("canonicalize {}", r.pkg_dir.display()))?;
        output::step(r.section, &r.name);
        let (pkg_name, version) = read_pkg_meta(&canon);
        let location = rel_to(&manifest_dir, &canon);
        match build::build_local_package(&canon, clean, r.manifest_override.as_deref()) {
            Ok(()) => built.push(Row {
                section: r.section,
                name: r.name.clone(),
                pkg_name,
                version,
                location,
                source: r.url_to_clone.clone(),
            }),
            Err(e) => failed.push((r.section, r.name.clone(), e)),
        }
    }

    // ── Summary ─────────────────────────────────────────────────────────────
    let manifest_label = manifest_path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("manifest");
    let term_w = crossterm::terminal::size()
        .map(|(c, _)| c as usize)
        .unwrap_or(120);

    fn center_title(width: usize, title: &str) -> String {
        let t = format!(" {title} ");
        if width <= t.len() {
            return "═".repeat(width);
        }
        let left = (width - t.len()) / 2;
        let right = width - t.len() - left;
        format!("{}{t}{}", "═".repeat(left), "═".repeat(right))
    }

    let h_status = "";
    let h_sec = "section";
    let h_name = "name";
    let h_pkg = "package.name";
    let h_ver = "version";
    let h_loc = "location";
    let w_status = 1;
    let w_sec = built
        .iter()
        .map(|r| r.section.len())
        .max()
        .unwrap_or(0)
        .max(h_sec.len());
    let w_name = built
        .iter()
        .map(|r| r.name.len())
        .max()
        .unwrap_or(0)
        .max(h_name.len());
    let w_pkg = built
        .iter()
        .map(|r| r.pkg_name.len())
        .max()
        .unwrap_or(0)
        .max(h_pkg.len());
    let w_ver = built
        .iter()
        .map(|r| r.version.len())
        .max()
        .unwrap_or(0)
        .max(h_ver.len());
    // Location: take its natural width so realpaths don't get truncated. The
    // table simply ends up wider than the terminal — better that the user can
    // copy-paste a full path than read a half-truncated one.
    let nat_loc = built
        .iter()
        .map(|r| r.location.len())
        .max()
        .unwrap_or(0)
        .max(h_loc.len());
    let w_loc = nat_loc;
    let table_w = if built.is_empty() {
        term_w
    } else {
        2 + w_status + 2 + w_sec + 2 + w_name + 2 + w_pkg + 2 + w_ver + 2 + w_loc
    };
    let bar_w = table_w.max(term_w);
    let bar = "═".repeat(bar_w);

    println!();
    println!("{}", center_title(bar_w, "Build summary"));
    println!("  Manifest: {}", manifest_path.display());
    println!(
        "  Built: {}   Fetched: {}   Skipped: {}   Failed: {}   Total: {}",
        built.len(),
        to_clone.len(),
        skipped.len(),
        failed.len(),
        entries.len()
    );

    if !built.is_empty() {
        println!();
        println!(
            "  {:<ws$}  {:<wsec$}  {:<wn$}  {:<wp$}  {:<wv$}  {:<wl$}",
            h_status,
            h_sec,
            h_name,
            h_pkg,
            h_ver,
            h_loc,
            ws = w_status,
            wsec = w_sec,
            wn = w_name,
            wp = w_pkg,
            wv = w_ver,
            wl = w_loc,
        );
        let rule = |w: usize| "─".repeat(w);
        println!(
            "  {}  {}  {}  {}  {}  {}",
            rule(w_status),
            rule(w_sec),
            rule(w_name),
            rule(w_pkg),
            rule(w_ver),
            rule(w_loc),
        );
        let cont_indent = 2 + w_status + 2 + w_sec + 2 + w_name + 2 + w_pkg + 2 + w_ver + 2;
        for r in &built {
            println!(
                "  {:<ws$}  {:<wsec$}  {:<wn$}  {:<wp$}  {:<wv$}  {}",
                "✓",
                r.section,
                r.name,
                r.pkg_name,
                r.version,
                r.location,
                ws = w_status,
                wsec = w_sec,
                wn = w_name,
                wp = w_pkg,
                wv = w_ver,
            );
            if let Some((url, branch)) = &r.source {
                let suffix = match branch {
                    Some(b) => format!("↳ {url} (branch={b})"),
                    None => format!("↳ {url}"),
                };
                println!("{}{suffix}", " ".repeat(cont_indent));
            }
        }
    }
    if !skipped.is_empty() {
        println!();
        for (section, name, reason) in &skipped {
            println!("  - {section}/{name}: {reason}");
        }
    }
    if !failed.is_empty() {
        println!();
        for (section, name, e) in &failed {
            println!("  ✗ {section}/{name}: {e:#}");
        }
    }
    println!("{bar}");

    if !failed.is_empty() {
        anyhow::bail!(
            "{} package(s) failed to build from {manifest_label}",
            failed.len()
        );
    }
    Ok(())
}

pub async fn execute_start(
    config: &Config,
    spec: Option<&str>,
    registry_endpoint: Option<&str>,
    config_file: Option<&Path>,
    set_overrides: &[String],
    manifest_override: Option<&str>,
) -> Result<()> {
    let package_root = match spec {
        Some(s) => resolve_package_path_for_start(config, s)?,
        None => find_package_from_cwd()?,
    };
    let detected = manifest::detect_and_load(&package_root, manifest_override)?;
    let manifest = &detected.manifest;
    manifest.validate_and_summarize()?;

    let endpoint = registry_endpoint
        .map(String::from)
        .unwrap_or_else(|| "127.0.0.1:50051".to_string());

    // Materialize per-instance config from --config + --set overrides
    // entirely in memory. The provider process never sees the file — config
    // is delivered via Driver(CMD_INIT, config_json) only (post-spawn
    // task below). Empty inputs still deliver Driver(CMD_INIT, "{}") so a
    // standalone `rbnx start` follows the same lifecycle as `rbnx boot`.
    let has_explicit_config = config_file.is_some() || !set_overrides.is_empty();
    let driver_contracts = manifest
        .capabilities
        .iter()
        .filter(|capability| capability.name.ends_with("/driver"))
        .map(|capability| capability.name.clone())
        .collect::<Vec<_>>();
    if driver_contracts.len() > 1 {
        anyhow::bail!(
            "package '{}' declares multiple */driver capabilities: {}; standalone lifecycle requires exactly one",
            manifest.package.name,
            driver_contracts.join(", ")
        );
    }
    let expected_driver_contract = driver_contracts.first().cloned();
    let has_driver_capability = expected_driver_contract.is_some();
    let deploy_managed = std::env::var_os("RBNX_DEPLOY_MANAGED").is_some();
    let materialized_cfg_json = build_start_config_json(config_file, set_overrides)?;

    // Per-package run logs live under <pkg>/rbnx-build/logs (gitignored,
    // owned by the package itself).  When `rbnx boot` spawns us, it sets
    // $SCRIBE_LOG_DIR to the deploy log dir — respect that so boot-time
    // logs stay under `rbnx-boot/logs/` for `rbnx logs` to find.
    let log_dir = std::env::var("SCRIBE_LOG_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| package_root.join("rbnx-build").join("logs"));
    let process_manager = Arc::new(ProcessManager::new(log_dir.clone())?);

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
    env.insert("SCRIBE_LOG_DIR".to_string(), log_dir.display().to_string());
    if has_explicit_config && should_drive_standalone_init(has_driver_capability, deploy_managed) {
        output::sub_step("Config: will deliver via Driver(CMD_INIT) post-register");
    } else if has_explicit_config && deploy_managed {
        output::sub_step("Config: deployment owner will deliver Driver(CMD_INIT)");
    } else if has_explicit_config {
        output::warning(
            "Config supplied, but the package manifest declares no */driver capability; config cannot be delivered",
        );
    }
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
        build::build_local_package(&package_root, false, manifest_override)?;
    }

    let exports = env
        .iter()
        .map(|(k, v)| format!("export {}={}", k, shell_escape(v)))
        .collect::<Vec<_>>()
        .join("; ");
    let pythonpath_export = generated_pythonpath_export(&package_root);
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
    let prefix_parts: Vec<String> = [setup_source, exports, pythonpath_export]
        .into_iter()
        .filter(|s| !s.is_empty())
        .collect();
    let start_command = if prefix_parts.is_empty() {
        start_body.to_string()
    } else {
        format!("{}; {start_body}", prefix_parts.join("; "))
    };

    // A standalone lifecycle owner snapshots Atlas before spawning. Snapshot
    // failures are fatal: treating them as an empty set could select an
    // unrelated pre-existing provider and deliver this package's config to it.
    // `rbnx boot` sets RBNX_DEPLOY_MANAGED and owns this sequence itself.
    let standalone_lifecycle =
        if should_drive_standalone_init(has_driver_capability, deploy_managed) {
            let json = materialized_cfg_json
                .expect("start config materialization always returns a JSON object");
            let normalized = normalize_atlas_endpoint(&endpoint);
            let mut atlas = AtlasClient::connect(&normalized).await.with_context(|| {
                format!("connect Atlas at {normalized} before standalone spawn")
            })?;
            let before_snapshot = snapshot_provider_ids(&mut atlas)
                .await
                .context("standalone pre-spawn Atlas snapshot")?;
            Some((
                atlas,
                before_snapshot,
                json,
                expected_driver_contract
                    .clone()
                    .expect("standalone lifecycle requires a driver contract"),
            ))
        } else {
            None
        };

    // Scribe tag = the per-INSTANCE provider id, never the package name. A
    // single package (one `package.name`) can be deployed as N instances, each
    // with a distinct provider id; tagging by package.name would collide them
    // all into one log. `rbnx boot` passes the instance's provider id via
    // RBNX_INSTANCE_NAME (the deploy manifest entry's `name`); fall back to
    // package.name only for a bare standalone `rbnx start` with no instance.
    let instance_name =
        std::env::var("RBNX_INSTANCE_NAME").unwrap_or_else(|_| manifest.package.name.clone());
    let result = if let Some((mut atlas, before, json, expected_contract)) = standalone_lifecycle {
        // `start_process` blocks for the package lifetime, so run it alongside
        // registration/lifecycle driving. On lifecycle failure, stop the exact
        // package process group instead of leaving a REGISTERED/ERROR provider.
        let manager_for_start = Arc::clone(&process_manager);
        let package_root_for_start = package_root.clone();
        let start_command_for_start = start_command.clone();
        let instance_for_start = instance_name.clone();
        let mut process_task = tokio::spawn(async move {
            manager_for_start
                .start_process(
                    &instance_for_start,
                    &instance_for_start,
                    "package",
                    &package_root_for_start,
                    &start_command_for_start,
                )
                .await
        });

        // Wait until ProcessManager has recorded the child so every lifecycle
        // failure path can terminate it. Surface an early child exit directly.
        let recorded_deadline = tokio::time::Instant::now() + Duration::from_secs(5);
        while !process_manager.has_process_record(&instance_name, "package") {
            if process_task.is_finished() {
                let process_result = process_task.await.context("package process task failed")?;
                return match process_result {
                    Ok(result) => anyhow::bail!(
                        "package exited before registering with Atlas (PID {})",
                        result.pid
                    ),
                    Err(error) => Err(error),
                };
            }
            if tokio::time::Instant::now() >= recorded_deadline {
                process_task.abort();
                anyhow::bail!("package process was not recorded within 5s after spawn");
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }

        let lifecycle = drive_standalone_lifecycle(&mut atlas, &before, &expected_contract, json);
        tokio::pin!(lifecycle);
        tokio::select! {
            lifecycle_result = &mut lifecycle => {
                if let Err(error) = lifecycle_result {
                    output::warning(&format!("standalone lifecycle failed: {error:#}"));
                    if let Err(stop_error) = process_manager.stop_process(&instance_name, "package").await {
                        output::warning(&format!("failed to stop package after lifecycle error: {stop_error:#}"));
                    }
                    let _ = process_task.await;
                    return Err(error);
                }
            }
            process_result = &mut process_task => {
                let process_result = process_result.context("package process task failed")?;
                return match process_result {
                    Ok(result) => anyhow::bail!(
                        "package exited before completing standalone lifecycle (PID {})",
                        result.pid
                    ),
                    Err(error) => Err(error),
                };
            }
        }
        process_task
            .await
            .context("package process task failed")??
    } else {
        process_manager
            .start_process(
                &instance_name,
                &instance_name,
                "package",
                &package_root,
                &start_command,
            )
            .await?
    };
    output::check(&format!(
        "{} exited (PID {})",
        manifest.package.name, result.pid
    ));

    output::success(&format!("Package {} finished", manifest.package.name));
    Ok(())
}

/// Build the package-local Python import path without touching a colcon
/// workspace. A real `rbnx-build/ws/install/setup.bash`, when present, is
/// sourced first; this export then prepends generated stubs while preserving
/// every Python path contributed by that overlay and the parent environment.
fn generated_pythonpath_export(package_root: &Path) -> String {
    let codegen_root = package_root.join("rbnx-build").join("codegen");
    let mut paths = vec![package_root.to_path_buf()];
    for path in [
        codegen_root.join("proto_gen"),
        codegen_root.join("robonix_mcp_types"),
    ] {
        if path.is_dir() {
            paths.push(path);
        }
    }
    let joined = paths
        .iter()
        .map(|path| path.display().to_string())
        .collect::<Vec<_>>()
        .join(":");
    format!(
        "export PYTHONPATH={}:${{PYTHONPATH:-}}",
        shell_escape(&joined)
    )
}

fn should_drive_standalone_init(has_driver_capability: bool, deploy_managed: bool) -> bool {
    has_driver_capability && !deploy_managed
}

fn normalize_atlas_endpoint(endpoint: &str) -> String {
    if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    }
}

/// Wait for exactly one new provider, verify that it declares the driver
/// contract from this package manifest, then drive INIT and (except for
/// skills) ACTIVATE. Contract verification is mandatory before any config is
/// sent, so a provider exposing a different lifecycle cannot receive this
/// package's configuration. Concurrent starts of two instances with the same
/// driver contract still require a future registration token for full identity
/// correlation.
async fn drive_standalone_lifecycle(
    atlas: &mut AtlasClient,
    before: &std::collections::HashSet<String>,
    expected_driver_contract: &str,
    config_json: String,
) -> Result<()> {
    let outcome = wait_for_registration_core(atlas, before, "rbnx start").await?;
    let driver_contract = require_expected_driver_contract(
        &outcome.provider_id,
        outcome.driver_contract.as_deref(),
        expected_driver_contract,
    )?;

    let init_state = call_driver_cmd(
        atlas,
        &outcome.provider_id,
        driver_contract,
        CMD_INIT,
        config_json.clone(),
        "rbnx start",
    )
    .await?;
    output::sub_step(&format!(
        "Driver(CMD_INIT) → {} ok (state={init_state})",
        outcome.provider_id
    ));
    if should_activate_standalone_provider(outcome.provider_kind) {
        let state = call_driver_cmd(
            atlas,
            &outcome.provider_id,
            driver_contract,
            CMD_ACTIVATE,
            config_json,
            "rbnx start",
        )
        .await?;
        output::sub_step(&format!(
            "Driver(CMD_ACTIVATE) → {} ok (state={state})",
            outcome.provider_id
        ));
    }
    Ok(())
}

fn should_activate_standalone_provider(provider_kind: i32) -> bool {
    provider_kind != atlas_pb::Kind::Skill as i32
}

fn require_expected_driver_contract<'a>(
    provider_id: &str,
    observed: Option<&'a str>,
    expected: &str,
) -> Result<&'a str> {
    let Some(observed) = observed else {
        anyhow::bail!(
            "provider '{provider_id}' registered without required driver contract '{expected}'"
        );
    };
    if observed != expected {
        anyhow::bail!(
            "provider '{provider_id}' declared driver contract '{observed}', expected '{expected}' from the package manifest"
        );
    }
    Ok(observed)
}

/// Materialize a per-instance config from `--config <file>` plus
/// repeatable `--set k.v=val` overrides. Returns the merged JSON
/// string. When neither input was provided, returns an empty JSON object so
/// `rbnx start` still performs the provider lifecycle initialization.
///
/// Layering: load file (json or yaml) → overlay each `--set` on the
/// tree → serialise to a single JSON string. The string is delivered
/// to the provider exclusively via Driver(CMD_INIT, config_json). The provider
/// process MUST NOT read this through env / disk — that's the v0.1
/// invariant `rbnx start` and `rbnx boot` both honour.
fn build_start_config_json(config_file: Option<&Path>, sets: &[String]) -> Result<Option<String>> {
    if config_file.is_none() && sets.is_empty() {
        return Ok(Some("{}".to_string()));
    }

    let mut value: serde_json::Value = match config_file {
        Some(p) => {
            let raw = std::fs::read_to_string(p)
                .with_context(|| format!("read config file {}", p.display()))?;
            // Try JSON first; fall through to YAML.
            match serde_json::from_str::<serde_json::Value>(&raw) {
                Ok(v) => v,
                Err(_) => {
                    let y: serde_yaml::Value = serde_yaml::from_str(&raw)
                        .with_context(|| format!("parse config {} as JSON or YAML", p.display()))?;
                    serde_json::to_value(y)
                        .with_context(|| format!("convert {} YAML→JSON", p.display()))?
                }
            }
        }
        None => serde_json::Value::Object(serde_json::Map::new()),
    };

    for s in sets {
        let (key, raw_val) = s
            .split_once('=')
            .with_context(|| format!("--set {s:?}: expected KEY=VALUE"))?;
        let parsed: serde_json::Value = serde_json::from_str(raw_val)
            .unwrap_or_else(|_| serde_json::Value::String(raw_val.into()));
        merge_dotted(&mut value, key, parsed)?;
    }

    Ok(Some(
        serde_json::to_string(&value).unwrap_or_else(|_| "{}".into()),
    ))
}

/// Set `obj[a][b][c] = v` for a dotted key like `"a.b.c"`. Creates
/// intermediate objects as needed; bails on a non-object collision.
fn merge_dotted(root: &mut serde_json::Value, key: &str, v: serde_json::Value) -> Result<()> {
    let parts: Vec<&str> = key.split('.').filter(|p| !p.is_empty()).collect();
    if parts.is_empty() {
        anyhow::bail!("--set: empty key");
    }
    if !root.is_object() {
        *root = serde_json::Value::Object(serde_json::Map::new());
    }
    let mut cur = root;
    for p in &parts[..parts.len() - 1] {
        let map = cur.as_object_mut().ok_or_else(|| {
            anyhow::anyhow!("--set {key}: cannot descend into non-object at '{p}'")
        })?;
        let entry = map
            .entry((*p).to_string())
            .or_insert_with(|| serde_json::Value::Object(serde_json::Map::new()));
        if !entry.is_object() {
            *entry = serde_json::Value::Object(serde_json::Map::new());
        }
        cur = entry;
    }
    let last = parts[parts.len() - 1];
    cur.as_object_mut()
        .ok_or_else(|| anyhow::anyhow!("--set {key}: parent is not an object"))?
        .insert(last.to_string(), v);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_root(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!("rbnx-start-{label}-{}-{nonce}", std::process::id()))
    }

    #[test]
    fn generated_pythonpath_is_injected_without_a_setup_stub() {
        let root = temp_root("pythonpath");
        let proto = root.join("rbnx-build/codegen/proto_gen");
        let mcp = root.join("rbnx-build/codegen/robonix_mcp_types");
        fs::create_dir_all(&proto).unwrap();
        fs::create_dir_all(&mcp).unwrap();

        let export = generated_pythonpath_export(&root);

        assert!(export.contains(&root.display().to_string()));
        assert!(export.contains(&proto.display().to_string()));
        assert!(export.contains(&mcp.display().to_string()));
        assert!(export.ends_with(":${PYTHONPATH:-}"));
        assert!(!root.join("rbnx-build/ws/install/setup.bash").exists());
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn start_without_config_still_materializes_empty_init_config() {
        let config = build_start_config_json(None, &[]).unwrap();
        assert_eq!(config.as_deref(), Some("{}"));
    }

    #[test]
    fn standalone_driver_start_owns_init_but_deploy_managed_start_does_not() {
        assert!(should_drive_standalone_init(true, false));
        assert!(!should_drive_standalone_init(true, true));
        assert!(!should_drive_standalone_init(false, false));
        assert!(!should_drive_standalone_init(false, true));
    }

    #[test]
    fn standalone_start_activates_primitive_and_service_but_not_skill() {
        assert!(should_activate_standalone_provider(
            atlas_pb::Kind::Primitive as i32
        ));
        assert!(should_activate_standalone_provider(
            atlas_pb::Kind::Service as i32
        ));
        assert!(!should_activate_standalone_provider(
            atlas_pb::Kind::Skill as i32
        ));
    }

    #[test]
    fn standalone_lifecycle_requires_the_manifest_driver_contract() {
        assert_eq!(
            require_expected_driver_contract(
                "camera-a",
                Some("robonix/primitive/camera/driver"),
                "robonix/primitive/camera/driver",
            )
            .unwrap(),
            "robonix/primitive/camera/driver"
        );
        let missing =
            require_expected_driver_contract("camera-a", None, "robonix/primitive/camera/driver")
                .unwrap_err()
                .to_string();
        assert!(missing.contains("registered without required driver contract"));
        let mismatch = require_expected_driver_contract(
            "lidar-b",
            Some("robonix/primitive/lidar/driver"),
            "robonix/primitive/camera/driver",
        )
        .unwrap_err()
        .to_string();
        assert!(mismatch.contains("expected 'robonix/primitive/camera/driver'"));
    }

    #[test]
    fn system_package_uses_the_deploy_selected_manifest() {
        let root = temp_root("system-manifest");
        let source_root = root.join("source");
        let scene_root = source_root.join("system/scene");
        let deploy_root = root.join("deploy");
        fs::create_dir_all(&scene_root).unwrap();
        fs::create_dir_all(&deploy_root).unwrap();
        fs::write(
            scene_root.join("package_manifest.yaml"),
            r#"manifestVersion: 1
package:
  name: test.system.scene
  version: 0.1.0
  vendor: test
  description: default target
  license: Apache-2.0
build: touch default-selected
start: "true"
"#,
        )
        .unwrap();
        fs::write(
            scene_root.join("package_manifest.jetson-native.yaml"),
            r#"manifestVersion: 1
package:
  name: test.system.scene
  version: 0.1.0
  vendor: test
  description: Jetson native target
  license: Apache-2.0
build: touch jetson-selected
start: "true"
"#,
        )
        .unwrap();
        let deploy_manifest = deploy_root.join("robonix_manifest.yaml");
        fs::write(
            &deploy_manifest,
            r#"manifestVersion: 1
name: system-target-test
system:
  scene:
    manifest: package_manifest.jetson-native.yaml
    camera_provider_id: front_camera
"#,
        )
        .unwrap();
        let config = Config {
            package_storage_path: root.join("packages"),
            robonix_source_path: Some(source_root),
        };

        build_deploy_manifest(&deploy_manifest, &config, false, true).unwrap();

        assert!(scene_root.join("jetson-selected").is_file());
        assert!(!scene_root.join("default-selected").exists());
        fs::remove_dir_all(root).unwrap();
    }
}
