// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx deploy` — bring up a whole robonix deployment from a top-level
// `robonix_manifest.yaml`. No shell scripts required: every tunable lives
// in the manifest, each package declares its own `start` body, and the
// config block per entry flows through as `RBNX_CAP_CONFIG_JSON`.
//
// What this does, in order:
//   1. Parse the deployment manifest (env expansion on scalar strings).
//   2. Launch `system:` components (atlas, executor, pilot, liaison, vlm,
//      memory) via `cargo run -p <binary>` from the robonix source tree.
//      Skipped when `--skip-system` is passed or the section is empty.
//   3. For each entry in `primitive`/`service`/`skill`, spawn `rbnx start
//      -p <path>` in a subprocess with the entry's `config:` block in
//      `RBNX_CAP_CONFIG_JSON`.
//   4. Redirect every child's stdout/stderr to `<log_dir>/<component>.log`.
//   5. Block on SIGINT/SIGTERM; propagate to all children and wait.
//
// Deliberately simple — no crash-restart, no health checks, no dependency
// ordering beyond `sleep 1.5` between boots. A proper supervisor is P3
// work; this is the "rbnx deploy == one-shot bring-up" foundation.

use anyhow::{Context, Result};
use robonix_cli::output;
use serde::Deserialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use tokio::process::{Child, Command};
use tokio::signal::unix::{signal, SignalKind};

// ── Deploy manifest schema (subset used by this orchestrator) ───────────

#[derive(Debug, Clone, Deserialize, Default)]
struct DeployManifest {
    #[serde(default)]
    name: String,
    #[serde(default)]
    env: HashMap<String, String>,
    #[serde(default)]
    system: HashMap<String, serde_yaml::Value>,
    #[serde(default)]
    primitive: Vec<PackageEntry>,
    #[serde(default)]
    service: Vec<PackageEntry>,
    #[serde(default)]
    skill: Vec<PackageEntry>,
}

#[derive(Debug, Clone, Deserialize)]
struct PackageEntry {
    /// Package identifier for logs (falls back to the directory basename).
    #[serde(default)]
    name: String,
    /// Local filesystem path (relative to the manifest dir). Mutually
    /// exclusive with `url`.
    #[serde(default)]
    path: Option<String>,
    /// Git URL for remote packages (e.g. the standalone mapping or nav
    /// repos too big to ship inside `examples/`). `rbnx deploy` clones
    /// into `<manifest-dir>/rbnx-deploy/cache/<name>/` on first run and
    /// reuses that checkout on subsequent runs. Mutually exclusive with
    /// `path`.
    #[serde(default)]
    url: Option<String>,
    /// Git branch / tag / commit to check out. Defaults to the default
    /// branch at clone time. Ignored when `path` is used.
    #[serde(default)]
    branch: Option<String>,
    /// Opaque config block; serialised to JSON and handed to the package's
    /// `start` body as `RBNX_CAP_CONFIG_JSON`.
    #[serde(default)]
    config: serde_yaml::Value,
}

/// Resolve a `PackageEntry` to a filesystem path, cloning from `url` into
/// `<cache_root>/<name>/` if necessary.
fn resolve_entry_path(entry: &PackageEntry, cache_root: &Path, manifest_dir: &Path) -> Result<PathBuf> {
    match (&entry.path, &entry.url) {
        (Some(p), None) => Ok(manifest_dir.join(p)),
        (None, Some(url)) => {
            let name = if entry.name.is_empty() {
                url.trim_end_matches(".git")
                    .rsplit('/')
                    .next()
                    .unwrap_or("pkg")
                    .to_string()
            } else {
                entry.name.clone()
            };
            let dest = cache_root.join(&name);
            if !dest.exists() {
                std::fs::create_dir_all(cache_root)?;
                output::sub_step(&format!("cloning {url} -> {}", dest.display()));
                let mut clone = std::process::Command::new("git");
                clone.arg("clone").arg("--depth").arg("1");
                if let Some(b) = &entry.branch {
                    clone.arg("--branch").arg(b);
                }
                clone.arg(url).arg(&dest);
                let status = clone.status().with_context(|| {
                    format!("git clone {url} failed to spawn (is git installed?)")
                })?;
                if !status.success() {
                    anyhow::bail!("git clone {url} exited with {:?}", status.code());
                }
            } else {
                output::sub_step(&format!(
                    "[cache hit] {} -> {}",
                    name,
                    dest.display()
                ));
            }
            Ok(dest)
        }
        (Some(_), Some(_)) => {
            anyhow::bail!("package entry has both `path` and `url`; pick one")
        }
        (None, None) => {
            anyhow::bail!("package entry has neither `path` nor `url`")
        }
    }
}

// ── env expansion — replace ${VAR} / $VAR in scalar strings ─────────────

fn expand_env_in_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'$' && i + 1 < bytes.len() {
            if bytes[i + 1] == b'{' {
                if let Some(end) = s[i + 2..].find('}') {
                    let var = &s[i + 2..i + 2 + end];
                    out.push_str(&std::env::var(var).unwrap_or_default());
                    i = i + 2 + end + 1;
                    continue;
                }
            } else if bytes[i + 1].is_ascii_alphabetic() || bytes[i + 1] == b'_' {
                let start = i + 1;
                let mut end = start;
                while end < bytes.len()
                    && (bytes[end].is_ascii_alphanumeric() || bytes[end] == b'_')
                {
                    end += 1;
                }
                let var = &s[start..end];
                out.push_str(&std::env::var(var).unwrap_or_default());
                i = end;
                continue;
            }
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

fn expand_yaml(v: &mut serde_yaml::Value) {
    use serde_yaml::Value;
    match v {
        Value::String(s) => *s = expand_env_in_str(s),
        Value::Sequence(seq) => {
            for item in seq {
                expand_yaml(item);
            }
        }
        Value::Mapping(map) => {
            for (_, val) in map.iter_mut() {
                expand_yaml(val);
            }
        }
        _ => {}
    }
}

// ── child-process helpers ───────────────────────────────────────────────

struct Spawned {
    name: String,
    child: Child,
}

fn log_path(log_dir: &Path, name: &str) -> PathBuf {
    log_dir.join(format!("{name}.log"))
}

async fn spawn_system_binary(
    rust_root: &Path,
    log_dir: &Path,
    name: &str,
    bin: &str,
    args: &[&str],
    extra_env: &HashMap<String, String>,
) -> Result<Spawned> {
    let log = std::fs::File::create(log_path(log_dir, name))
        .with_context(|| format!("failed to open log file for {name}"))?;
    let err = log.try_clone()?;
    let mut cmd = Command::new("cargo");
    cmd.current_dir(rust_root)
        .arg("run")
        .arg("-p")
        .arg(bin)
        .arg("--");
    for a in args {
        cmd.arg(a);
    }
    for (k, v) in extra_env {
        cmd.env(k, v);
    }
    cmd.stdin(Stdio::null())
        .stdout(Stdio::from(log))
        .stderr(Stdio::from(err));
    let child = cmd
        .spawn()
        .with_context(|| format!("failed to spawn system binary {bin}"))?;
    output::sub_step(&format!(
        "[system] {name} -> {}",
        log_path(log_dir, name).display()
    ));
    Ok(Spawned { name: name.to_string(), child })
}

async fn spawn_package(
    rust_root: &Path,
    log_dir: &Path,
    cache_root: &Path,
    component: &str,
    entry: &PackageEntry,
    manifest_dir: &Path,
) -> Result<Spawned> {
    let pkg_path = resolve_entry_path(entry, cache_root, manifest_dir)?;
    let pkg_path = pkg_path
        .canonicalize()
        .with_context(|| format!("package path not found: {}", pkg_path.display()))?;
    let cfg_json = serde_json::to_string(&entry.config).unwrap_or_else(|_| "{}".to_string());

    let name = if entry.name.is_empty() {
        pkg_path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("package")
            .to_string()
    } else {
        entry.name.clone()
    };
    let log_name = format!("{component}_{name}");

    let log = std::fs::File::create(log_path(log_dir, &log_name))
        .with_context(|| format!("failed to open log for {log_name}"))?;
    let err = log.try_clone()?;

    let mut cmd = Command::new("cargo");
    cmd.current_dir(rust_root)
        .arg("run")
        .arg("-p")
        .arg("robonix-cli")
        .arg("--")
        .arg("start")
        .arg("-p")
        .arg(pkg_path.as_os_str())
        .env("RBNX_CAP_CONFIG_JSON", cfg_json)
        .env("RBNX_INVOCATION_CWD", manifest_dir)
        .stdin(Stdio::null())
        .stdout(Stdio::from(log))
        .stderr(Stdio::from(err));
    let child = cmd
        .spawn()
        .with_context(|| format!("failed to spawn package {name}"))?;
    output::sub_step(&format!(
        "[{component}] {name} -> {}",
        log_path(log_dir, &log_name).display()
    ));
    Ok(Spawned { name: log_name, child })
}

// ── entry point ─────────────────────────────────────────────────────────

pub async fn execute(
    _config: robonix_cli::Config,
    manifest_path: PathBuf,
    log_dir: Option<PathBuf>,
    skip_system: bool,
) -> Result<()> {
    let manifest_path = manifest_path
        .canonicalize()
        .with_context(|| format!("manifest not found: {}", manifest_path.display()))?;
    let manifest_dir = manifest_path
        .parent()
        .context("manifest has no parent directory")?
        .to_path_buf();

    let raw = std::fs::read_to_string(&manifest_path)
        .with_context(|| format!("failed to read {}", manifest_path.display()))?;
    let mut deploy: DeployManifest = serde_yaml::from_str(&raw)
        .with_context(|| format!("failed to parse {}", manifest_path.display()))?;
    // Env expansion applies to both the top-level env block and all nested
    // scalar strings in system / primitive / service / skill configs.
    for v in deploy.system.values_mut() {
        expand_yaml(v);
    }
    for e in deploy
        .primitive
        .iter_mut()
        .chain(deploy.service.iter_mut())
        .chain(deploy.skill.iter_mut())
    {
        expand_yaml(&mut e.config);
    }

    let log_dir = log_dir.unwrap_or_else(|| manifest_dir.join("rbnx-deploy").join("logs"));
    std::fs::create_dir_all(&log_dir)
        .with_context(|| format!("failed to create log dir {}", log_dir.display()))?;
    let cache_root = manifest_dir.join("rbnx-deploy").join("cache");

    // `cargo run -p <bin>` requires we cd into a cargo workspace. We assume
    // the `robonix-cli` binary is itself running out of the robonix source
    // tree — walk up from the current exe to find the workspace root.
    let rust_root = find_rust_root()?;

    // Propagate the manifest's `env:` block into our own env so child
    // processes (which inherit) see it.
    // set_var is unsafe on edition 2024 (other threads may race). We call
    // it before spawning any children, so no races in practice.
    for (k, v) in &deploy.env {
        unsafe {
            std::env::set_var(k, expand_env_in_str(v));
        }
    }

    output::action(
        "Deploying",
        &format!(
            "{} (manifest: {})",
            if deploy.name.is_empty() { "robonix" } else { &deploy.name },
            manifest_path.display()
        ),
    );

    let mut children: Vec<Spawned> = Vec::new();

    if !skip_system {
        // Minimal system bring-up. Each component's config block is passed
        // as env vars whose exact keys are determined by the component's
        // own code (TODO #25: make them accept the manifest-derived config
        // uniformly). For now we just launch binaries with no extra args —
        // they use their default ports.
        let bin_map: &[(&str, &str)] = &[
            ("atlas", "robonix-atlas"),
            ("executor", "robonix-executor"),
            ("pilot", "robonix-pilot"),
            ("liaison", "robonix-liaison"),
        ];
        for (name, bin) in bin_map {
            if !deploy.system.contains_key(*name) {
                continue;
            }
            let extra = system_env(*name, deploy.system.get(*name));
            let sp = spawn_system_binary(&rust_root, &log_dir, name, bin, &[], &extra).await?;
            children.push(sp);
            tokio::time::sleep(std::time::Duration::from_millis(1500)).await;
        }
    } else {
        output::sub_step("Skipping system bring-up (--skip-system)");
    }

    for e in &deploy.primitive {
        let sp = spawn_package(&rust_root, &log_dir, &cache_root, "primitive", e, &manifest_dir).await?;
        children.push(sp);
    }
    for e in &deploy.service {
        let sp = spawn_package(&rust_root, &log_dir, &cache_root, "service", e, &manifest_dir).await?;
        children.push(sp);
    }
    for e in &deploy.skill {
        let sp = spawn_package(&rust_root, &log_dir, &cache_root, "skill", e, &manifest_dir).await?;
        children.push(sp);
    }

    output::success(&format!(
        "{} component(s) up; logs under {}",
        children.len(),
        log_dir.display()
    ));
    output::sub_step("Ctrl-C to tear down.");

    // Wait for SIGINT / SIGTERM, then shut children down.
    let mut sigint = signal(SignalKind::interrupt())?;
    let mut sigterm = signal(SignalKind::terminate())?;
    tokio::select! {
        _ = sigint.recv() => {}
        _ = sigterm.recv() => {}
    }
    output::action("Stopping", &format!("{} child(ren)", children.len()));
    for sp in &mut children {
        let _ = sp.child.start_kill();
    }
    for sp in &mut children {
        let _ = sp.child.wait().await;
        output::sub_step(&format!("{} stopped", sp.name));
    }
    Ok(())
}

/// Walk up from the running executable's directory until a `Cargo.toml`
/// containing `[workspace]` is found (the robonix rust/ root). This is
/// how we locate the cargo workspace when called as an installed binary
/// or from `cargo run` alike.
fn find_rust_root() -> Result<PathBuf> {
    let exe = std::env::current_exe()?;
    let mut cur: Option<&Path> = exe.parent();
    while let Some(d) = cur {
        let cargo = d.join("Cargo.toml");
        if cargo.is_file() {
            if let Ok(text) = std::fs::read_to_string(&cargo) {
                if text.contains("[workspace]") {
                    return Ok(d.to_path_buf());
                }
            }
        }
        cur = d.parent();
    }
    // Fallback: current working dir (useful in dev).
    std::env::current_dir().context("could not locate rust workspace root")
}

/// Convert a system component's config block into the env-var set its
/// binary expects. Intentionally narrow — keeps the mapping close to the
/// call site so a missing key is obvious. Extend as components grow.
fn system_env(name: &str, cfg: Option<&serde_yaml::Value>) -> HashMap<String, String> {
    let mut out = HashMap::new();
    let cfg = match cfg.and_then(|v| v.as_mapping()) {
        Some(m) => m,
        None => return out,
    };
    let s = |k: &str| -> Option<String> {
        cfg.get(serde_yaml::Value::String(k.into()))
            .and_then(|v| v.as_str())
            .map(|s| s.to_string())
    };
    match name {
        "atlas" => {
            if let Some(l) = s("listen") {
                out.insert("ROBONIX_ATLAS".into(), l);
            }
        }
        "executor" => {
            if let Some(l) = s("listen") {
                out.insert("ROBONIX_EXECUTOR_LISTEN".into(), l);
            }
        }
        "pilot" => {
            if let Some(l) = s("listen") {
                out.insert("ROBONIX_PILOT_LISTEN".into(), l);
            }
            if let Some(v) = s("vlm_service") {
                out.insert("ROBONIX_VLM_SERVICE".into(), v);
            }
        }
        "liaison" => {
            if let Some(l) = s("listen") {
                out.insert("ROBONIX_LIAISON_LISTEN".into(), l);
            }
        }
        _ => {}
    }
    out
}
