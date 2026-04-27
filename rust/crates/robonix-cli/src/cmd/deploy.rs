// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx boot` — bring up the whole robonix stack from a top-level
// `robonix_manifest.yaml`. (`rbnx boot` is a back-compat alias.)
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

use crate::pb::lifecycle::{DriverRequest, DriverResponse};

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
    /// repos too big to ship inside `examples/`). `rbnx boot` clones
    /// into `<manifest-dir>/rbnx-boot/cache/<name>/` on first run and
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
    log_dir: &Path,
    name: &str,
    bin: &str,
    args: &[String],
) -> Result<Spawned> {
    let log = std::fs::File::create(log_path(log_dir, name))
        .with_context(|| format!("failed to open log file for {name}"))?;
    let err = log.try_clone()?;
    // Run the installed binary directly. `cargo install` puts atlas /
    // executor / pilot in $CARGO_HOME/bin (via `make install`); the user
    // is expected to have run that. No `cargo run` here — that requires
    // the source tree on disk and needlessly slows down deploy.
    let mut cmd = Command::new(bin);
    for a in args {
        cmd.arg(a);
    }
    cmd.stdin(Stdio::null())
        .stdout(Stdio::from(log))
        .stderr(Stdio::from(err));
    let child = cmd.spawn().with_context(|| {
        format!(
            "failed to spawn system binary `{bin}` — is it installed (try `make install` from the rust/ workspace)?"
        )
    })?;
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

    // Spawn `rbnx start -p <pkg>` via the currently-running rbnx binary
    // itself — i.e. argv[0] of the deploy process. This way deploy doesn't
    // need a cargo workspace on disk and version-skew is impossible.
    let rbnx_bin = std::env::current_exe()
        .context("could not resolve current rbnx binary path for `start` re-exec")?;
    let mut cmd = Command::new(&rbnx_bin);
    cmd.arg("start")
        .arg("-p")
        .arg(pkg_path.as_os_str())
        .env("RBNX_CONFIG_FILE", &cfg_file)
        .env("RBNX_INSTANCE_NAME", &name)
        .env("RBNX_INVOCATION_CWD", manifest_dir)
        .stdin(Stdio::null())
        .stdout(Stdio::from(log))
        .stderr(Stdio::from(err));
    let child = cmd.spawn().with_context(|| {
        format!(
            "failed to spawn package {name} via `{} start`",
            rbnx_bin.display()
        )
    })?;
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

    let log_dir = log_dir.unwrap_or_else(|| manifest_dir.join("rbnx-boot").join("logs"));
    std::fs::create_dir_all(&log_dir)
        .with_context(|| format!("failed to create log dir {}", log_dir.display()))?;
    let cache_root = manifest_dir.join("rbnx-boot").join("cache");
    let instances_dir = manifest_dir.join("rbnx-boot").join("instances");
    std::fs::create_dir_all(&instances_dir)
        .with_context(|| format!("failed to create instances dir {}", instances_dir.display()))?;

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
            let sp = spawn_system_binary(&log_dir, name, bin, &args).await?;
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

/// Spawn one primitive / service package and wait for it to register
/// at least one capability with atlas. If the new cap has a `*/driver`
/// gRPC interface, also drive Driver(CMD_INIT) and pass the entry's
/// `config:` as `config_json`. Packages that don't declare a driver
/// (legacy RegisterNode + DeclareInterface, or new packages that just
/// don't need init-time wiring) are deployed as-is once their first cap
/// appears in atlas.
async fn spawn_and_init(
    component: &str,
    entry: &PackageEntry,
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
        log_dir,
        cache_root,
        instances_dir,
        component,
        entry,
        manifest_dir,
    )
    .await?;
    let pkg_label = sp.name.clone();

    let (cap_id, driver_contract) =
        wait_for_registration(atlas, &before, &pkg_label, component).await?;

    let Some(driver_contract) = driver_contract else {
        // Legacy / no-lifecycle package: registration alone is enough.
        output::sub_step(&format!(
            "[{component}/{pkg_label}] cap '{cap_id}' registered (no `*/driver` interface — skipping Driver(INIT))"
        ));
        return Ok(sp);
    };

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
        // Each per-area driver TOML (`primitive/<area>/driver`,
        // `service/<area>/driver`) generates its own gRPC service
        // (`PrimitiveChassisDriver`, `ServiceNavigationDriver`, …) —
        // they all share the lifecycle/srv/Driver wire shape but live
        // at distinct gRPC paths. Build the path from the contract_id
        // and call it directly via tonic's low-level Grpc client, so
        // boot doesn't need to know which area it's talking to.
        let svc_name = contract_id_to_service_name(&driver_contract);
        let path: tonic::codegen::http::uri::PathAndQuery =
            format!("/robonix.contracts.{svc_name}/Driver")
                .parse()
                .with_context(|| format!("build gRPC path for '{driver_contract}'"))?;
        let mut grpc = tonic::client::Grpc::new(channel);
        grpc.ready().await.with_context(|| "gRPC ready")?;
        let codec: tonic_prost::ProstCodec<DriverRequest, DriverResponse> = Default::default();
        let resp = tokio::time::timeout(
            DRIVER_INIT_TIMEOUT,
            grpc.unary(
                Request::new(DriverRequest {
                    command: CMD_INIT,
                    config_json,
                }),
                path,
                codec,
            ),
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

/// Mirrors `robonix_codegen::contract_gen::contract_id_to_service_name`.
/// Strips the `robonix/` prefix and concatenates the remaining path
/// segments in UpperCamelCase: `robonix/primitive/chassis/driver` →
/// `PrimitiveChassisDriver`. The full gRPC service path is then
/// `/robonix.contracts.<this>/Driver`, matching what the per-area driver
/// TOMLs codegen emits in `robonix_contracts.proto`.
fn contract_id_to_service_name(id: &str) -> String {
    let body = id.strip_prefix("robonix/").unwrap_or(id);
    body.split('/')
        .filter(|x| !x.is_empty())
        .map(|seg| {
            seg.split('_')
                .filter(|p| !p.is_empty())
                .map(|p| {
                    let mut c = p.chars();
                    match c.next() {
                        None => String::new(),
                        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                    }
                })
                .collect::<String>()
        })
        .collect::<String>()
}

/// Poll atlas until a cap NOT in `before` appears. Returns the new
/// `cap_id` plus an optional `driver_contract_id` if the new cap
/// declared a `*/driver` gRPC interface (signal to the caller that
/// Driver(CMD_INIT) lifecycle should run).
async fn wait_for_registration(
    atlas: &mut AtlasClient,
    before: &HashSet<String>,
    pkg_label: &str,
    component: &str,
) -> Result<(String, Option<String>)> {
    output::sub_step(&format!(
        "[{component}/{pkg_label}] waiting for cap registration..."
    ));
    let deadline = Instant::now() + DRIVER_REGISTER_TIMEOUT;
    loop {
        let records = atlas
            .query_capabilities("", "", atlas_pb::Transport::Unspecified)
            .await
            .with_context(|| format!("[{component}/{pkg_label}] poll atlas"))?;
        for rec in records {
            if before.contains(&rec.capability_id) {
                continue;
            }
            // Found a new cap. If it has a `*/driver` gRPC interface,
            // the caller should run Driver(CMD_INIT); otherwise it's a
            // legacy / no-lifecycle package and we just record the cap.
            let driver = rec.interfaces.iter().find(|iface| {
                iface.transport == atlas_pb::Transport::Grpc as i32
                    && iface.contract_id.ends_with("/driver")
            });
            return Ok((
                rec.capability_id.clone(),
                driver.map(|i| i.contract_id.clone()),
            ));
        }
        if Instant::now() >= deadline {
            anyhow::bail!(
                "[{component}/{pkg_label}] timed out after {:?} — package never registered a cap with atlas. Check its log.",
                DRIVER_REGISTER_TIMEOUT
            );
        }
        tokio::time::sleep(DRIVER_POLL_INTERVAL).await;
    }
}
