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
        match build::build_local_package(&canon, clean) {
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
    let w_sec = built.iter().map(|r| r.section.len()).max().unwrap_or(0).max(h_sec.len());
    let w_name = built.iter().map(|r| r.name.len()).max().unwrap_or(0).max(h_name.len());
    let w_pkg = built.iter().map(|r| r.pkg_name.len()).max().unwrap_or(0).max(h_pkg.len());
    let w_ver = built.iter().map(|r| r.version.len()).max().unwrap_or(0).max(h_ver.len());
    // Location: take its natural width so realpaths don't get truncated. The
    // table simply ends up wider than the terminal — better that the user can
    // copy-paste a full path than read a half-truncated one.
    let nat_loc = built.iter().map(|r| r.location.len()).max().unwrap_or(0).max(h_loc.len());
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
            h_status, h_sec, h_name, h_pkg, h_ver, h_loc,
            ws=w_status, wsec=w_sec, wn=w_name, wp=w_pkg, wv=w_ver, wl=w_loc,
        );
        let rule = |w: usize| "─".repeat(w);
        println!(
            "  {}  {}  {}  {}  {}  {}",
            rule(w_status), rule(w_sec), rule(w_name), rule(w_pkg), rule(w_ver), rule(w_loc),
        );
        let cont_indent = 2 + w_status + 2 + w_sec + 2 + w_name + 2 + w_pkg + 2 + w_ver + 2;
        for r in &built {
            println!(
                "  {:<ws$}  {:<wsec$}  {:<wn$}  {:<wp$}  {:<wv$}  {}",
                "✓", r.section, r.name, r.pkg_name, r.version, r.location,
                ws=w_status, wsec=w_sec, wn=w_name, wp=w_pkg, wv=w_ver,
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

    // Materialize per-instance config from --config + --set overrides.
    // Same RBNX_CONFIG_FILE shape rbnx boot writes per package, just
    // sourced from the user's CLI invocation instead of a deploy
    // manifest. Empty inputs → no RBNX_CONFIG_FILE export at all
    // (start body falls back to defaults / env-only).
    let materialized_cfg = build_start_config(&package_root, config_file, set_overrides)?;

    // Per-package run logs live under <pkg>/rbnx-build/logs (gitignored,
    // owned by the package itself). Earlier code put them in the parent
    // dir's rbnx-boot/logs, which created stray empty `rbnx-boot/`
    // directories sibling to the package whenever `rbnx start` ran from
    // outside.
    let log_dir = package_root.join("rbnx-build").join("logs");
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
    if let Some(cfg_path) = &materialized_cfg {
        env.insert(
            "RBNX_CONFIG_FILE".to_string(),
            cfg_path.display().to_string(),
        );
        output::sub_step(&format!("Config: {}", cfg_path.display()));
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

/// Materialize a per-instance config file from `--config <file>` plus
/// repeatable `--set k.v=val` overrides. Returns the path the start
/// body should read via `RBNX_CONFIG_FILE`, or `None` when neither
/// input was provided (start body falls back to its own defaults).
///
/// Order of operations: load file (json or yaml) → overlay each
/// `--set` on the resulting tree → write JSON to
/// `<pkg>/rbnx-build/instances/cli.json`. Same shape `rbnx boot`
/// already writes per package, just sourced from CLI flags.
fn build_start_config(
    pkg_root: &Path,
    config_file: Option<&Path>,
    sets: &[String],
) -> Result<Option<PathBuf>> {
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
        // Try JSON parse for typed values (`true` / `42` / `"x"` / `[1,2]`);
        // fall back to a bare string when that fails so `--set algo=rtabmap`
        // does the obvious thing without quoting.
        let parsed: serde_json::Value =
            serde_json::from_str(raw_val).unwrap_or_else(|_| serde_json::Value::String(raw_val.into()));
        merge_dotted(&mut value, key, parsed)?;
    }

    let out_dir = pkg_root.join("rbnx-build").join("instances");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("create {}", out_dir.display()))?;
    let out_path = out_dir.join("cli.json");
    let pretty = serde_json::to_string_pretty(&value).unwrap_or_else(|_| "{}".into());
    std::fs::write(&out_path, &pretty)
        .with_context(|| format!("write {}", out_path.display()))?;
    Ok(Some(out_path))
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
