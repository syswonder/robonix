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
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::process::ProcessManager;
use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};
use tonic::Request;
use tonic::transport::Endpoint;

use crate::pb::lifecycle::{DriverRequest, DriverResponse};

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
    // task below). Empty inputs → no CMD_INIT push (start body still
    // gets a default Driver(CMD_INIT, "{}") call so it can advance to
    // INACTIVE, just with an empty config dict).
    let materialized_cfg_json = build_start_config_json(config_file, set_overrides)?;

    // Per-package run logs live under <pkg>/rbnx-build/logs (gitignored,
    // owned by the package itself).  When `rbnx boot` spawns us, it sets
    // $SCRIBE_LOG_DIR to the deploy log dir — respect that so boot-time
    // logs stay under `rbnx-boot/logs/` for `rbnx logs` to find.
    let log_dir = std::env::var("SCRIBE_LOG_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| package_root.join("rbnx-build").join("logs"));
    let process_manager = ProcessManager::new(log_dir.clone())?;

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
    if materialized_cfg_json.is_some() {
        output::sub_step("Config: will deliver via Driver(CMD_INIT) post-register");
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

    // Post-spawn task: wait for the provider to register with atlas, then
    // send Driver(CMD_INIT, config_json=<materialized>). The provider
    // process never sees the JSON anywhere — atlas + this gRPC call
    // is the only delivery path. Skipped when no --config / --set was
    // given (rbnx start without config = run package as-is, init with
    // default empty config).
    let init_task = if let Some(json) = materialized_cfg_json {
        // One package = one provider. Snapshot atlas, then post-spawn diff
        // gives the new provider_id.
        let endpoint_for_task = endpoint.clone();
        let before_snapshot = match AtlasClient::connect(&endpoint).await {
            Ok(mut a) => a
                .query_capabilities("", "", atlas_pb::Transport::Unspecified)
                .await
                .map(|providers| providers.into_iter().map(|r| r.id).collect::<HashSet<_>>())
                .unwrap_or_default(),
            Err(_) => HashSet::new(),
        };
        Some(tokio::spawn(async move {
            drive_cmd_init_after_register(&endpoint_for_task, &before_snapshot, json).await
        }))
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
    let result = process_manager
        .start_process(
            &instance_name,
            &instance_name,
            "package",
            &package_root,
            &start_command,
        )
        .await?;

    if let Some(handle) = init_task {
        handle.abort(); // package exited; no point still polling
    }
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

const CMD_INIT_DELIVERY: u32 = 0;
const DEFAULT_DRIVER_INIT_TIMEOUT: Duration = Duration::from_secs(90);
const CAP_REGISTER_TIMEOUT: Duration = Duration::from_secs(60);
const POLL_INTERVAL: Duration = Duration::from_millis(500);

fn driver_init_timeout() -> Duration {
    std::env::var("ROBONIX_DRIVER_INIT_TIMEOUT_S")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .filter(|secs| *secs > 0)
        .map(Duration::from_secs)
        .unwrap_or(DEFAULT_DRIVER_INIT_TIMEOUT)
}

/// Wait for the new provider (any provider not in `before`) to appear in atlas with a
/// `*/driver` gRPC capability, then call Driver(CMD_INIT, config_json). One
/// package = one provider. Gives up after 60s; `rbnx start` keeps the package
/// running regardless.
async fn drive_cmd_init_after_register(
    atlas_endpoint: &str,
    before: &HashSet<String>,
    config_json: String,
) {
    let normalized = if atlas_endpoint.starts_with("http") {
        atlas_endpoint.to_string()
    } else {
        format!("http://{atlas_endpoint}")
    };
    let mut atlas = match AtlasClient::connect(&normalized).await {
        Ok(c) => c,
        Err(e) => {
            output::warning(&format!("CMD_INIT delivery: connect atlas failed: {e:#}"));
            return;
        }
    };
    let started = Instant::now();
    loop {
        if started.elapsed() > CAP_REGISTER_TIMEOUT {
            output::warning(&format!(
                "CMD_INIT delivery: no new provider registered within {:?}; config not delivered",
                CAP_REGISTER_TIMEOUT
            ));
            return;
        }
        let providers = match atlas
            .query_capabilities("", "", atlas_pb::Transport::Unspecified)
            .await
        {
            Ok(r) => r,
            Err(_) => {
                tokio::time::sleep(POLL_INTERVAL).await;
                continue;
            }
        };
        let new_provider = providers.iter().find(|r| !before.contains(&r.id));
        let Some(provider) = new_provider else {
            tokio::time::sleep(POLL_INTERVAL).await;
            continue;
        };
        let driver_cap = provider.capabilities.iter().find(|c| {
            c.transport == atlas_pb::Transport::Grpc as i32 && c.contract_id.ends_with("/driver")
        });
        let Some(driver) = driver_cap else {
            tokio::time::sleep(POLL_INTERVAL).await;
            continue;
        };
        let driver_contract = driver.contract_id.clone();
        let provider_id = provider.id.clone();
        match call_driver_init(
            &mut atlas,
            &provider_id,
            &driver_contract,
            config_json.clone(),
        )
        .await
        {
            Ok(state) => {
                output::sub_step(&format!(
                    "Driver(CMD_INIT) → {provider_id} ok (state={state})"
                ));
            }
            Err(e) => {
                output::warning(&format!("Driver(CMD_INIT) → {provider_id} failed: {e:#}"));
            }
        }
        return;
    }
}

/// Send Driver(CMD_INIT, config_json) to a known provider's `*/driver`
/// gRPC capability. Mirrors deploy.rs's call_driver_cmd but inlined to
/// keep run_package.rs free of cross-module coupling.
async fn call_driver_init(
    atlas: &mut AtlasClient,
    provider_id: &str,
    driver_contract: &str,
    config_json: String,
) -> Result<String> {
    let (channel_id, endpoint, _params) = atlas
        .connect_capability(
            "rbnx-cli/start",
            provider_id,
            driver_contract,
            atlas_pb::Transport::Grpc,
        )
        .await
        .with_context(|| format!("ConnectCapability({driver_contract})"))?;
    let normalized = if endpoint.starts_with("http") {
        endpoint
    } else {
        format!("http://{endpoint}")
    };
    let result = async {
        let driver_timeout = driver_init_timeout();
        let channel = Endpoint::new(normalized.clone())
            .with_context(|| format!("invalid driver endpoint '{normalized}'"))?
            .connect()
            .await
            .with_context(|| format!("dial driver at '{normalized}'"))?;
        let svc = contract_id_to_service_name(driver_contract);
        let path: tonic::codegen::http::uri::PathAndQuery =
            format!("/robonix.contracts.{svc}/Driver")
                .parse()
                .with_context(|| format!("build gRPC path for '{driver_contract}'"))?;
        let mut grpc = tonic::client::Grpc::new(channel);
        grpc.ready().await.context("gRPC ready")?;
        let codec: tonic_prost::ProstCodec<DriverRequest, DriverResponse> = Default::default();
        let resp = tokio::time::timeout(
            driver_timeout,
            grpc.unary(
                Request::new(DriverRequest {
                    command: CMD_INIT_DELIVERY,
                    config_json,
                }),
                path,
                codec,
            ),
        )
        .await
        .map_err(|_| {
            anyhow::anyhow!(
                "Driver(CMD_INIT) timed out after {}s",
                driver_timeout.as_secs()
            )
        })?
        .context("Driver(CMD_INIT) RPC failed")?;
        Ok::<_, anyhow::Error>(resp.into_inner())
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    let r = result?;
    if !r.ok {
        anyhow::bail!(
            "Driver(CMD_INIT) returned ok=false (state={}, error={})",
            r.state,
            r.error
        );
    }
    Ok(r.state)
}

/// `robonix/primitive/chassis/move` → `RobonixPrimitiveChassisMove`.
/// `mycomp/a/b/c`                   → `MycompABC`.
/// Uniform PascalCase per `/`-segment; no prefix stripping.
fn contract_id_to_service_name(contract_id: &str) -> String {
    let mut out = String::new();
    for seg in contract_id.split('/').filter(|s| !s.is_empty()) {
        for part in seg.split('_').filter(|s| !s.is_empty()) {
            let mut ch = part.chars();
            if let Some(c) = ch.next() {
                out.push(c.to_ascii_uppercase());
                out.extend(ch);
            }
        }
    }
    out
}

/// Materialize a per-instance config from `--config <file>` plus
/// repeatable `--set k.v=val` overrides. Returns the merged JSON
/// string, or `None` when neither input was provided.
///
/// Layering: load file (json or yaml) → overlay each `--set` on the
/// tree → serialise to a single JSON string. The string is delivered
/// to the provider exclusively via Driver(CMD_INIT, config_json). The provider
/// process MUST NOT read this through env / disk — that's the v0.1
/// invariant `rbnx start` and `rbnx boot` both honour.
fn build_start_config_json(config_file: Option<&Path>, sets: &[String]) -> Result<Option<String>> {
    if config_file.is_none() && sets.is_empty() {
        return Ok(None);
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
