// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx deploy` — bring up a whole robonix deployment from a top-level
// `robonix_manifest.yaml`.
//
// Conventions:
//   - `system:` Rust binaries (atlas / pilot / executor) are launched with
//     CLI arguments translated from the manifest block (`--listen`,
//     `--log`, `--vlm-*`, …). No env-var translation, no YAML config files.
//   - Package entries (`primitive` / `service`) are launched serially:
//     spawn → wait for the package to register a cap with a `*/driver`
//     interface on atlas → call Driver(CMD_INIT, config_json) → wait for
//     `ok=true`. Only after every primitive's driver returns ok do we move
//     on to `service:` (which can depend on primitive data being ready).
//     The package's `config:` block is JSON-encoded and ALSO handed to
//     the start body via `RBNX_CONFIG_FILE`; whether the init logic reads
//     it from the env file or from the Driver call's `config_json` arg
//     is the package's choice.
//   - `skill:` entries are NOT started at deploy time — the executor
//     invokes them on demand. We just log them so the user can see they
//     were registered with the manifest.
//
// Out of scope: crash-restart, health checks beyond Driver(INIT).

use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_cli::output;
use serde::Deserialize;
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::time::{Duration, Instant};
use tokio::process::{Child, Command};
use tokio::signal::unix::{SignalKind, signal};
use tonic::Request;
use tonic::transport::Endpoint;

use crate::pb::contracts::lifecycle_driver_client::LifecycleDriverClient;
use crate::pb::lifecycle::DriverRequest;

// Driver.srv command discriminator.
const CMD_INIT: u32 = 0;
// How long to wait for a freshly spawned package to register its driver
// interface with atlas before giving up.
const DRIVER_REGISTER_TIMEOUT: Duration = Duration::from_secs(60);
const DRIVER_POLL_INTERVAL: Duration = Duration::from_millis(500);
// How long Driver(CMD_INIT) is given to return.
const DRIVER_INIT_TIMEOUT: Duration = Duration::from_secs(60);
const DEPLOY_CONSUMER_ID: &str = "rbnx-cli/deploy";

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
fn resolve_entry_path(
    entry: &PackageEntry,
    cache_root: &Path,
    manifest_dir: &Path,
) -> Result<PathBuf> {
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
                output::sub_step(&format!("[cache hit] {} -> {}", name, dest.display()));
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
    args: &[String],
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
    cmd.stdin(Stdio::null())
        .stdout(Stdio::from(log))
        .stderr(Stdio::from(err));
    let child = cmd
        .spawn()
        .with_context(|| format!("failed to spawn system binary {bin}"))?;
    let arg_preview = if args.is_empty() {
        String::new()
    } else {
        format!(" ({})", args.join(" "))
    };
    output::sub_step(&format!(
        "[system] {name}{arg_preview} -> {}",
        log_path(log_dir, name).display()
    ));
    Ok(Spawned {
        name: name.to_string(),
        child,
    })
}

async fn spawn_package(
    rust_root: &Path,
    log_dir: &Path,
    cache_root: &Path,
    instances_dir: &Path,
    component: &str,
    entry: &PackageEntry,
    manifest_dir: &Path,
) -> Result<Spawned> {
    let pkg_path = resolve_entry_path(entry, cache_root, manifest_dir)?;
    let pkg_path = pkg_path
        .canonicalize()
        .with_context(|| format!("package path not found: {}", pkg_path.display()))?;

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

    // Write this instance's config to a per-instance JSON file. Passing a
    // file path (rather than the JSON itself) in an env var sidesteps
    // three env-var landmines: bash escaping of quotes/newlines, ARG_MAX
    // blowing up on large configs, and `printenv`-only debugging. The
    // package's start body just does `jq ... < "$RBNX_CONFIG_FILE"`.
    let cfg_json = serde_json::to_value(&entry.config).unwrap_or(serde_json::Value::Null);
    let cfg_pretty = serde_json::to_string_pretty(&cfg_json).unwrap_or_else(|_| "{}".into());
    let cfg_file = instances_dir.join(format!("{name}.json"));
    std::fs::write(&cfg_file, &cfg_pretty)
        .with_context(|| format!("failed to write {}", cfg_file.display()))?;

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
        .env("RBNX_CONFIG_FILE", &cfg_file)
        .env("RBNX_INSTANCE_NAME", &name)
        .env("RBNX_INVOCATION_CWD", manifest_dir)
        .stdin(Stdio::null())
        .stdout(Stdio::from(log))
        .stderr(Stdio::from(err));
    let child = cmd
        .spawn()
        .with_context(|| format!("failed to spawn package {name}"))?;
    output::sub_step(&format!(
        "[{component}] {name} -> {} (config: {})",
        log_path(log_dir, &log_name).display(),
        cfg_file.display(),
    ));
    Ok(Spawned {
        name: log_name,
        child,
    })
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
    let instances_dir = manifest_dir.join("rbnx-deploy").join("instances");
    std::fs::create_dir_all(&instances_dir)
        .with_context(|| format!("failed to create instances dir {}", instances_dir.display()))?;

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
            if deploy.name.is_empty() {
                "robonix"
            } else {
                &deploy.name
            },
            manifest_path.display()
        ),
    );

    let mut children: Vec<Spawned> = Vec::new();

    if !skip_system {
        // System Rust binaries: launched in atlas → executor → pilot order.
        // Each is fed CLI flags translated from `system.<name>:` block.
        // executor + pilot inherit `--atlas` from `system.atlas.listen`
        // unless they declare their own `atlas:` (rare).
        let atlas_listen = deploy
            .system
            .get("atlas")
            .and_then(|v| v.as_mapping())
            .and_then(|m| m.get(serde_yaml::Value::String("listen".into())))
            .and_then(|v| v.as_str())
            .map(str::to_string);
        let bin_map: &[(&str, &str)] = &[
            ("atlas", "robonix-atlas"),
            ("executor", "robonix-executor"),
            ("pilot", "robonix-pilot"),
        ];
        for (name, bin) in bin_map {
            if !deploy.system.contains_key(*name) {
                continue;
            }
            let args = system_cli_args(name, deploy.system.get(*name), atlas_listen.as_deref());
            let sp = spawn_system_binary(&rust_root, &log_dir, name, bin, &args).await?;
            children.push(sp);
            tokio::time::sleep(std::time::Duration::from_millis(1500)).await;
        }
        // Warn (don't fail) on system entries we don't know how to bring up
        // yet (e.g. liaison while it's still being ported off robonix-sdk).
        for key in deploy.system.keys() {
            if !bin_map.iter().any(|(n, _)| *n == key) {
                output::sub_step(&format!(
                    "[system] {key}: no built-in launcher; ignored (declare it as a service package if needed)"
                ));
            }
        }
    } else {
        output::sub_step("Skipping system bring-up (--skip-system)");
    }

    // Connect to atlas once; reuse for every primitive/service init dance.
    let atlas_endpoint = deploy
        .system
        .get("atlas")
        .and_then(|v| v.as_mapping())
        .and_then(|m| m.get(serde_yaml::Value::String("listen".into())))
        .and_then(|v| v.as_str())
        .unwrap_or("127.0.0.1:50051")
        .to_string();
    let mut atlas =
        AtlasClient::connect_with_retry(&atlas_endpoint, 20, Duration::from_millis(500))
            .await
            .with_context(|| {
                format!("connect to atlas at '{atlas_endpoint}' for lifecycle init")
            })?;

    for e in &deploy.primitive {
        let sp = spawn_and_init(
            "primitive",
            e,
            &rust_root,
            &log_dir,
            &cache_root,
            &instances_dir,
            &manifest_dir,
            &mut atlas,
        )
        .await?;
        children.push(sp);
    }
    for e in &deploy.service {
        let sp = spawn_and_init(
            "service",
            e,
            &rust_root,
            &log_dir,
            &cache_root,
            &instances_dir,
            &manifest_dir,
            &mut atlas,
        )
        .await?;
        children.push(sp);
    }
    // Skills are NOT spawned at deploy — the executor invokes them on demand.
    // Just report what was registered so the user has visibility.
    for e in &deploy.skill {
        let label = if e.name.is_empty() {
            e.path
                .as_deref()
                .or(e.url.as_deref())
                .unwrap_or("(unnamed)")
                .to_string()
        } else {
            e.name.clone()
        };
        output::sub_step(&format!(
            "[skill] {label}: registered (invoked on demand by executor — not spawned at deploy)"
        ));
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

/// Translate a `system.<name>:` block into CLI args for the corresponding
/// Rust binary. Per-binary mapping kept narrow — adding a new flag means
/// touching exactly this function plus the binary's clap struct.
///
/// `atlas_listen` is the value of `system.atlas.listen` (already resolved
/// elsewhere). Consumers that don't carry their own `atlas:` field inherit
/// from this so the manifest doesn't have to repeat the address. An
/// explicit per-block `atlas:` still wins.
fn system_cli_args(
    name: &str,
    cfg: Option<&serde_yaml::Value>,
    atlas_listen: Option<&str>,
) -> Vec<String> {
    let mut out = Vec::new();
    let map = cfg.and_then(|v| v.as_mapping());
    let s = |k: &str| -> Option<String> {
        map.and_then(|m| {
            m.get(serde_yaml::Value::String(k.into()))
                .and_then(|v| v.as_str())
                .map(|s| s.to_string())
        })
    };
    let nested_str = |outer: &str, inner: &str| -> Option<String> {
        map.and_then(|m| m.get(serde_yaml::Value::String(outer.into())))
            .and_then(|v| v.as_mapping())
            .and_then(|m| m.get(serde_yaml::Value::String(inner.into())))
            .and_then(|v| v.as_str())
            .map(|s| s.to_string())
    };
    let push_pair = |out: &mut Vec<String>, flag: &str, val: Option<String>| {
        if let Some(v) = val {
            out.push(flag.into());
            out.push(v);
        }
    };
    match name {
        "atlas" => {
            push_pair(&mut out, "--listen", s("listen"));
            push_pair(&mut out, "--log", s("log"));
        }
        "executor" => {
            push_pair(&mut out, "--listen", s("listen"));
            push_pair(
                &mut out,
                "--atlas",
                s("atlas").or_else(|| atlas_listen.map(str::to_string)),
            );
            push_pair(&mut out, "--log", s("log"));
        }
        "pilot" => {
            push_pair(&mut out, "--listen", s("listen"));
            push_pair(
                &mut out,
                "--atlas",
                s("atlas").or_else(|| atlas_listen.map(str::to_string)),
            );
            push_pair(&mut out, "--log", s("log"));
            // Embedded VLM block.
            push_pair(&mut out, "--vlm-upstream", nested_str("vlm", "upstream"));
            push_pair(&mut out, "--vlm-api-key", nested_str("vlm", "api_key"));
            push_pair(&mut out, "--vlm-model", nested_str("vlm", "model"));
            push_pair(&mut out, "--vlm-format", nested_str("vlm", "api_format"));
        }
        _ => {}
    }
    out
}

/// Spawn one primitive / service package, then drive it through Driver(CMD_INIT).
///
/// Flow:
///   1. Snapshot atlas's known cap_id set
///   2. Spawn `rbnx start -p <path>` (the package boots and `register_capability`
///      itself, then declares its driver interface)
///   3. Poll atlas until a NEW cap_id appears that has at least one
///      `*/driver` interface declared over gRPC
///   4. ConnectCapability → tonic Channel → LifecycleDriver.Driver(CMD_INIT, config_json)
///   5. Verify response.ok; on failure return Err and let the caller abort
///   6. DisconnectCapability and continue
///
/// The package's process handle is returned so the deploy loop can teardown
/// it on Ctrl-C.
async fn spawn_and_init(
    component: &str,
    entry: &PackageEntry,
    rust_root: &Path,
    log_dir: &Path,
    cache_root: &Path,
    instances_dir: &Path,
    manifest_dir: &Path,
    atlas: &mut AtlasClient,
) -> Result<Spawned> {
    let before: HashSet<String> = atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await
        .with_context(|| format!("[{component}] pre-spawn atlas snapshot"))?
        .into_iter()
        .map(|r| r.capability_id)
        .collect();

    let sp = spawn_package(
        rust_root,
        log_dir,
        cache_root,
        instances_dir,
        component,
        entry,
        manifest_dir,
    )
    .await?;
    let pkg_label = sp.name.clone();

    // 1. Wait for the new cap to register + declare its driver interface.
    let (cap_id, driver_contract) = wait_for_driver(atlas, &before, &pkg_label, component).await?;

    // 2. ConnectCapability to get the driver endpoint.
    let (channel_id, endpoint, _params) = atlas
        .connect_capability(
            DEPLOY_CONSUMER_ID,
            &cap_id,
            &driver_contract,
            atlas_pb::Transport::Grpc,
        )
        .await
        .with_context(|| {
            format!("[{component}/{pkg_label}] ConnectCapability for {driver_contract}")
        })?;

    // 3. Driver(CMD_INIT, config_json).
    let config_json = serde_json::to_string(&entry.config).unwrap_or_else(|_| "{}".into());
    let normalized = if endpoint.starts_with("http") {
        endpoint.clone()
    } else {
        format!("http://{}", endpoint)
    };
    let init_result = async {
        let channel = Endpoint::new(normalized.clone())
            .with_context(|| format!("invalid driver endpoint '{normalized}'"))?
            .connect()
            .await
            .with_context(|| format!("dial driver at '{normalized}'"))?;
        let mut client = LifecycleDriverClient::new(channel);
        let resp = tokio::time::timeout(
            DRIVER_INIT_TIMEOUT,
            client.driver(Request::new(DriverRequest {
                command: CMD_INIT,
                config_json,
            })),
        )
        .await
        .map_err(|_| anyhow::anyhow!("Driver(CMD_INIT) timed out after {:?}", DRIVER_INIT_TIMEOUT))?
        .with_context(|| "Driver(CMD_INIT) RPC failed")?;
        Ok::<_, anyhow::Error>(resp.into_inner())
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;

    match init_result {
        Ok(r) if r.ok => {
            output::sub_step(&format!(
                "[{component}/{pkg_label}] Driver(INIT) ok (state={})",
                r.state
            ));
        }
        Ok(r) => {
            anyhow::bail!(
                "[{component}/{pkg_label}] Driver(INIT) returned ok=false (state={}, error={})",
                r.state,
                r.error
            );
        }
        Err(e) => {
            anyhow::bail!("[{component}/{pkg_label}] Driver(INIT) failed: {e:#}");
        }
    }

    Ok(sp)
}

/// Poll atlas every DRIVER_POLL_INTERVAL until a cap NOT in `before` appears
/// with a `*/driver` gRPC interface. Returns (cap_id, driver_contract_id).
async fn wait_for_driver(
    atlas: &mut AtlasClient,
    before: &HashSet<String>,
    pkg_label: &str,
    component: &str,
) -> Result<(String, String)> {
    output::sub_step(&format!(
        "[{component}/{pkg_label}] waiting for driver registration..."
    ));
    let deadline = Instant::now() + DRIVER_REGISTER_TIMEOUT;
    loop {
        let records = atlas
            .query_capabilities("", "", atlas_pb::Transport::Unspecified)
            .await
            .with_context(|| format!("[{component}/{pkg_label}] poll atlas for driver"))?;
        for rec in records {
            if before.contains(&rec.capability_id) {
                continue;
            }
            // New cap! Check if it's declared a driver interface yet.
            for iface in &rec.interfaces {
                if iface.transport != atlas_pb::Transport::Grpc as i32 {
                    continue;
                }
                if iface.contract_id.ends_with("/driver") {
                    return Ok((rec.capability_id.clone(), iface.contract_id.clone()));
                }
            }
        }
        if Instant::now() >= deadline {
            anyhow::bail!(
                "[{component}/{pkg_label}] timed out after {:?} waiting for the package to register a `*/driver` gRPC interface — check the package's stdout/stderr log",
                DRIVER_REGISTER_TIMEOUT
            );
        }
        tokio::time::sleep(DRIVER_POLL_INTERVAL).await;
    }
}
