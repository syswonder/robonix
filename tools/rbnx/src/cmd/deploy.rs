// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx boot` — bring up the whole robonix stack from a top-level
// `robonix_manifest.yaml`. (`rbnx boot` is a back-compat alias.)
//
// Conventions:
//   - `system:` Rust binaries (atlas / pilot / executor) are launched with
//     CLI arguments translated from the manifest block (`--listen`,
//     `--log`, `--vlm-*`, …). No env-var translation, no YAML config files.
//   - Package entries (`primitive` / `service`) are launched serially:
//     spawn → wait for the package to register a provider with a `*/driver`
//     capability on atlas → call Driver(CMD_INIT, config_json) → wait for
//     `ok=true`. Only after every primitive's driver returns ok do we move
//     on to `service:` (which can depend on primitive data being ready).
//     The package's `config:` block is JSON-encoded and delivered ONLY via
//     Driver(CMD_INIT)'s config_json field. The provider process never sees a
//     config file or env var — that's the v0.1 layering invariant.
//   - `skill:` entries are spawned identically to `service:` — they
//     need a long-lived process for their MCP tools to be registered
//     on atlas. The semantic difference (skill = atomic intent
//     invokable by pilot, service = always-on capability) lives in
//     the contract namespace (`robonix/skill/*` vs `robonix/service/*`),
//     not in the lifecycle. The earlier "skill is registered but not
//     spawned" model lied about what was actually running and forced
//     manifest authors to put skills like explore in `service:` as a
//     workaround.
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
use tokio::io::AsyncBufReadExt;
use tokio::process::{Child, Command};
use tokio::signal::unix::{SignalKind, signal};
use tonic::Request;
use tonic::transport::Endpoint;

use robonix_scribe as scribe;

use crate::pb::lifecycle::{DriverRequest, DriverResponse};

use super::teardown;

// Driver.srv command discriminators (mirrors lifecycle/srv/Driver.srv).
const CMD_INIT: u32 = 0;
const CMD_ACTIVATE: u32 = 1;
#[allow(dead_code)]
const CMD_DEACTIVATE: u32 = 2;
#[allow(dead_code)]
const CMD_SHUTDOWN: u32 = 3;
// How long to wait for a freshly spawned package to register its driver
// capability with atlas before giving up.
const DRIVER_REGISTER_TIMEOUT: Duration = Duration::from_secs(60);
// Default Driver(CMD_INIT) deadline. Webots CI can override this with
// ROBONIX_DRIVER_INIT_TIMEOUT_S for real stacks whose lifecycle bringup may
// exceed 90s on a cold self-hosted runner.
const DEFAULT_DRIVER_INIT_TIMEOUT: Duration = Duration::from_secs(90);
const DEPLOY_CONSUMER_ID: &str = "rbnx-cli/deploy";

fn driver_init_timeout() -> Duration {
    std::env::var("ROBONIX_DRIVER_INIT_TIMEOUT_S")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .filter(|secs| *secs > 0)
        .map(Duration::from_secs)
        .unwrap_or(DEFAULT_DRIVER_INIT_TIMEOUT)
}

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
    /// Optional package-manifest filename override. A package may ship
    /// per-deployment-target manifests (e.g. `package_manifest.yaml` for
    /// x86+docker, `package_manifest.jetson-native.yaml`,
    /// `package_manifest.jetson-docker.yaml`), each with its own build/start.
    /// This selects which one `rbnx build`/`boot` uses for THIS deployment;
    /// the package-manifest schema itself is unchanged. Defaults to
    /// `package_manifest.yaml`.
    #[serde(default)]
    manifest: Option<String>,
}

/// Compute a `PackageEntry`'s expected on-disk path. PURE — no I/O,
/// no logging, no cloning. `path:` entries land at `manifest_dir/path`;
/// `url:` entries land at `cache_root/<name>` (whether or not it's
/// been cloned yet). Use `entry_path_exists_on_disk` to check
/// presence; use the public `cmd::fetch::clone_remote_packages`
/// (called from `rbnx build`) to actually populate the cache.
/// Cache directory name for a url-remote package: the git REPO name (last path
/// segment of the url, minus `.git`), NOT the per-instance provider id.
///
/// A single repo can back several providers/instances in one manifest (each
/// with its own `name`/provider_id); they must share ONE clone. Keying the
/// cache dir by `name` would clone the same repo once per instance — and the
/// directory wouldn't reflect what was actually cloned. Key it by the repo.
pub(crate) fn repo_dir_name(url: &str) -> String {
    url.trim_end_matches('/')
        .trim_end_matches(".git")
        .rsplit('/')
        .next()
        .filter(|s| !s.is_empty())
        .unwrap_or("pkg")
        .to_string()
}

fn resolve_entry_path(
    entry: &PackageEntry,
    cache_root: &Path,
    manifest_dir: &Path,
) -> Result<PathBuf> {
    match (&entry.path, &entry.url) {
        (Some(p), None) => Ok(manifest_dir.join(p)),
        (None, Some(url)) => Ok(cache_root.join(repo_dir_name(url))),
        (Some(_), Some(_)) => {
            anyhow::bail!("package entry has both `path` and `url`; pick one")
        }
        (None, None) => {
            anyhow::bail!("package entry has neither `path` nor `url`")
        }
    }
}

/// Boot-time prerequisites check:
///   - any url-remote package whose cache dir doesn't exist → warn,
///     clone it inline (so the user isn't blocked) and tell them to
///     run `rbnx build` for proper bring-up.
///   - any package whose `rbnx-build/.rbnx-built` sentinel is missing
///     → warn and run its build.sh inline.
///
/// Boot's job is to spawn and atlas-register; fetching and building
/// belong to `rbnx build`. We do the inline remediation here ONLY so
/// the user isn't stuck after a fresh clone with no build done — the
/// warnings are deliberately loud so the right path (build first,
/// then boot) stays visible.
fn check_prerequisites(
    deploy: &DeployManifest,
    cache_root: &Path,
    manifest_dir: &Path,
) -> Result<()> {
    use std::collections::BTreeMap;
    // value: (url, branch, manifest_override)
    let mut needs_clone: BTreeMap<String, (String, Option<String>, Option<String>)> =
        BTreeMap::new();
    // value: (pkg_path, manifest_override)
    let mut needs_build: BTreeMap<String, (PathBuf, Option<String>)> = BTreeMap::new();
    for entry in deploy
        .primitive
        .iter()
        .chain(deploy.service.iter())
        .chain(deploy.skill.iter())
    {
        let pkg_path = match resolve_entry_path(entry, cache_root, manifest_dir) {
            Ok(p) => p,
            Err(_) => continue, // bad manifest entry; later steps will surface it
        };
        let name = if entry.name.is_empty() {
            pkg_path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("(unnamed)")
                .to_string()
        } else {
            entry.name.clone()
        };
        if !pkg_path.exists()
            && let Some(url) = entry.url.as_ref()
        {
            needs_clone.insert(
                name.clone(),
                (url.clone(), entry.branch.clone(), entry.manifest.clone()),
            );
            continue;
        }
        let stamp = pkg_path.join("rbnx-build").join(".rbnx-built");
        if !stamp.exists() {
            needs_build.insert(name, (pkg_path, entry.manifest.clone()));
        }
    }
    if needs_clone.is_empty() && needs_build.is_empty() {
        return Ok(());
    }
    output::boot_section("prerequisites");
    for (name, (url, branch, manifest_ov)) in &needs_clone {
        output::warning(&format!(
            "{name}: not in cache — `rbnx build` should run before `rbnx boot`. cloning inline."
        ));
        let dest = cache_root.join(repo_dir_name(url));
        std::fs::create_dir_all(cache_root)?;
        let mut clone = std::process::Command::new("git");
        clone.arg("clone").arg("--depth").arg("1");
        if let Some(b) = branch {
            clone.arg("--branch").arg(b);
        }
        clone.arg(url).arg(&dest);
        let status = clone
            .status()
            .with_context(|| format!("git clone {url} failed to spawn"))?;
        if !status.success() {
            anyhow::bail!("git clone {url} exited with {:?}", status.code());
        }
        // Newly-cloned package needs a build too.
        let stamp = dest.join("rbnx-build").join(".rbnx-built");
        if !stamp.exists() {
            needs_build.insert(name.clone(), (dest, manifest_ov.clone()));
        }
    }
    for (name, (pkg_path, manifest_ov)) in &needs_build {
        output::warning(&format!(
            "{name}: not built — `rbnx build` should run before `rbnx boot`. building inline."
        ));
        crate::cmd::build::build_local_package(pkg_path, false, manifest_ov.as_deref())
            .with_context(|| format!("inline build of {name} at {} failed", pkg_path.display()))?;
    }
    Ok(())
}

// ── env expansion — replace ${VAR} / $VAR in scalar strings ─────────────

fn expand_env_in_str(s: &str) -> String {
    // We scan the source as bytes (cheap to index) but only care about ASCII
    // sigils — `$`, `{`, `}`, alphanumerics, underscore — which are all
    // single-byte in UTF-8. Multi-byte chars are passed through via
    // `s[..].chars()` so non-ASCII (Chinese paths, en-dashes pasted from
    // chat, …) stays intact.
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
        // Walk one full UTF-8 char from the current byte offset, not one
        // byte. `bytes[i] as char` is a 7-bit cast that would corrupt any
        // continuation byte of a multi-byte scalar.
        let ch = s[i..]
            .chars()
            .next()
            .expect("non-empty by while-loop guard");
        out.push(ch);
        i += ch.len_utf8();
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
    /// "system_builtin" | "system_package" | "primitive" | "service"
    kind: String,
    child: Child,
    pid: u32,
    /// Process group id. Each child is spawned with `process_group(0)` so
    /// it becomes the leader of a new PGID == its own PID. Tear-down
    /// signals `-PGID` to take the whole subtree (rbnx start wrapper +
    /// inner interpreter + any docker-exec wrappers it forked).
    pgid: u32,
}

fn log_path(log_dir: &Path, name: &str) -> PathBuf {
    // `name` is the provider_id — the exact Scribe tag, so `<name>.log` is the
    // real file rbnx should point at. No name mangling.
    log_dir.join(format!("{name}.log"))
}

async fn spawn_system_binary(
    log_dir: &Path,
    name: &str,
    bin: &str,
    args: &[String],
) -> Result<Spawned> {
    // Run the installed binary directly.  Stdout / stderr are piped
    // through Scribe (tag = binary name, e.g. "executor") so nothing
    // escapes to the terminal.  Structured logs from within the binary
    // also go through Scribe via the `log` facade auto-init.
    let mut cmd = Command::new(bin);
    for a in args {
        cmd.arg(a);
    }
    cmd.stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .env("SCRIBE_LOG_DIR", log_dir)
        .process_group(0);
    let mut child = cmd.spawn().with_context(|| {
        format!(
            "failed to spawn system binary `{bin}` — is it installed (try `make install` from the rust/ workspace)?"
        )
    })?;
    let pid = child
        .id()
        .ok_or_else(|| anyhow::anyhow!("spawned `{bin}` but it had no pid"))?;

    // Pipe stdout / stderr into Scribe so raw println!/eprintln! from the
    // binary are captured alongside its structured logs.
    let stdout = child.stdout.take().expect("stdout not piped");
    let stderr = child.stderr.take().expect("stderr not piped");
    let tag_out = name.to_string();
    let tag_err = name.to_string();
    tokio::spawn(async move {
        let reader = tokio::io::BufReader::new(stdout);
        let mut lines = reader.lines();
        while let Ok(Some(line)) = lines.next_line().await {
            scribe::ingest(&tag_out, &line);
        }
    });
    tokio::spawn(async move {
        let reader = tokio::io::BufReader::new(stderr);
        let mut lines = reader.lines();
        while let Ok(Some(line)) = lines.next_line().await {
            // stderr is not always errors — Python logging defaults to
            // stderr for INFO too.  Use `info` to avoid misrepresenting
            // the actual severity.
            scribe::ingest(&tag_err, &line);
        }
    });
    // Salient detail per builtin: port + role, redact long flag soup
    // (--capabilities path lists, --vlm-api-key, …). Full args are
    // available in the log file; the boot line stays terse so users
    // can scan the bring-up sequence at a glance.
    let detail = system_boot_detail(name, args);
    output::boot_ok(name, &detail);
    Ok(Spawned {
        name: name.to_string(),
        kind: "system_builtin".to_string(),
        child,
        pid,
        pgid: pid,
    })
}

struct PackageSpawnEnv<'a> {
    log_dir: &'a Path,
    cache_root: &'a Path,
    instances_dir: &'a Path,
    manifest_dir: &'a Path,
    atlas_endpoint: &'a str,
}

async fn spawn_package(
    component: &str,
    entry: &PackageEntry,
    env: &PackageSpawnEnv<'_>,
) -> Result<Spawned> {
    let pkg_path = resolve_entry_path(entry, env.cache_root, env.manifest_dir)?;
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
    // Scribe tag + log-file stem = the provider_id (`entry.name`) verbatim.
    // provider_id is unique per deploy (atlas enforces it), so no kind prefix
    // is needed for disambiguation — `rbnx logs -t <provider_id>` and the file
    // `<provider_id>.log` both key on the same name the user wrote.
    let log_name = name.clone();

    // Write this instance's config to disk for boot's own bookkeeping
    // (debugging via `cat <instances>/<name>.json`, post-mortem
    // inspection). Boot itself reads `entry.config` in-memory and
    // pushes it via Driver(CMD_INIT, config_json) — see call_driver_cmd
    // below. The provider process MUST NOT see this path; we do not export
    // it as an env var to the spawned `rbnx start`.
    let cfg_json = serde_json::to_value(&entry.config).unwrap_or(serde_json::Value::Null);
    let cfg_pretty = serde_json::to_string_pretty(&cfg_json).unwrap_or_else(|_| "{}".into());
    let cfg_file = env.instances_dir.join(format!("{name}.json"));
    std::fs::write(&cfg_file, &cfg_pretty)
        .with_context(|| format!("failed to write {}", cfg_file.display()))?;

    // Spawn `rbnx start -p <pkg>` via the currently-running rbnx binary
    // itself — i.e. argv[0] of the deploy process. This way deploy doesn't
    // need a cargo workspace on disk and version-skew is impossible.
    // Stdout / stderr are piped through Scribe (tag = log_name, e.g.
    // "service_mapping") so boot-time display stays clean.
    let rbnx_bin = std::env::current_exe()
        .context("could not resolve current rbnx binary path for `start` re-exec")?;
    // Per v0.1 layering: do NOT pass the config file path to the
    // spawned `rbnx start` (which would propagate to the provider process
    // env). rbnx boot itself drives Driver(CMD_INIT, config_json) over
    // gRPC after the provider registers (see `call_driver_cmd` below). The
    // cfg_file on disk is for boot's own use — we read it back via
    // `entry.config` higher in this module — and atlas-side bookkeeping;
    // the provider never sees it.
    let _ = &cfg_file; // kept for debug / inspection; not exported
    // Tell the provider which atlas to register with — derived from the
    // manifest's `system.atlas.listen`, NOT the hard default 127.0.0.1:50051.
    // Without this an alt-port deploy (e.g. an isolated CI run) leaves every
    // provider dialing 50051 and failing to register. A bind-all listen
    // (0.0.0.0) is rewritten to a dialable loopback for the provider; an
    // in-container driver further overrides this via ROBONIX_SIM_ATLAS.
    let provider_atlas = env.atlas_endpoint.replacen("0.0.0.0", "127.0.0.1", 1);
    let mut cmd = Command::new(&rbnx_bin);
    cmd.arg("start")
        .arg("-p")
        .arg(pkg_path.as_os_str())
        .arg("--endpoint")
        .arg(&provider_atlas)
        .env("RBNX_INSTANCE_NAME", &name)
        .env("RBNX_INVOCATION_CWD", env.manifest_dir)
        .env("SCRIBE_LOG_DIR", env.log_dir)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .process_group(0);
    // Per-deployment-target package manifest selector (deploy entry's
    // `manifest:` field) — `rbnx start` loads this file instead of the
    // default package_manifest.yaml so the right start path runs.
    if let Some(m) = entry.manifest.as_deref() {
        cmd.arg("--manifest").arg(m);
    }
    let mut child = cmd.spawn().with_context(|| {
        format!(
            "failed to spawn package {name} via `{} start`",
            rbnx_bin.display()
        )
    })?;
    let pid = child
        .id()
        .ok_or_else(|| anyhow::anyhow!("spawned package '{name}' but it had no pid"))?;

    // Pipe stdout / stderr into Scribe — tag = provider_id, so the file is
    // `<provider_id>.log` (e.g. "mapping.log").
    let stdout = child.stdout.take().expect("stdout not piped");
    let stderr = child.stderr.take().expect("stderr not piped");
    let tag_out = log_name.clone();
    let tag_err = log_name.clone();
    tokio::spawn(async move {
        let reader = tokio::io::BufReader::new(stdout);
        let mut lines = reader.lines();
        while let Ok(Some(line)) = lines.next_line().await {
            scribe::ingest(&tag_out, &line);
        }
    });
    tokio::spawn(async move {
        let reader = tokio::io::BufReader::new(stderr);
        let mut lines = reader.lines();
        while let Ok(Some(line)) = lines.next_line().await {
            // stderr is not always errors — Python logging defaults to
            // stderr for INFO too.  Use `info` to avoid misrepresenting
            // the actual severity.
            scribe::ingest(&tag_err, &line);
        }
    });
    // No spawn line here — wait until provider registration and emit one
    // boot_ok with the provider_id so each component takes ONE line in the
    // boot log instead of three (spawn + waiting + registered).
    let kind = match component {
        "system" => "system_package",
        other => other,
    }
    .to_string();
    Ok(Spawned {
        name: log_name,
        kind,
        child,
        pid,
        pgid: pid,
    })
}

// ── entry point ─────────────────────────────────────────────────────────

pub async fn execute(
    config: robonix_cli::Config,
    manifest_path: PathBuf,
    log_dir: Option<PathBuf>,
    skip_system: bool,
    no_update_check: bool,
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
    // Banner + boot header FIRST, so the logo/version and what we're booting
    // lead the output — before the (possibly slow) remote freshness check.
    output::boot_banner();
    output::boot_start(
        if deploy.name.is_empty() {
            "robonix"
        } else {
            &deploy.name
        },
        &manifest_path.display().to_string(),
    );
    scribe::info(
        "bootstrap",
        &format!(
            "booting {} from {}",
            if deploy.name.is_empty() {
                "robonix"
            } else {
                &deploy.name
            },
            manifest_path.display()
        ),
    );

    // Notice (non-fatal) if any cloned remote provider is behind upstream.
    // `--no-update-check` skips the per-package `git fetch` pass entirely.
    if !no_update_check {
        super::check_remotes::report_outdated(&manifest_path);
    }
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
    // Wipe stale per-component logs from prior runs — without this you
    // can't tell whether `system_speech.log` is from THIS boot or one
    // ten `rbnx boot` retries ago. Only `*.log` files at the top level
    // get removed; nested directories (if a future package wants its
    // own subdir) are left alone.
    if log_dir.is_dir()
        && let Ok(entries) = std::fs::read_dir(&log_dir)
    {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.extension().and_then(|s| s.to_str()) == Some("log") {
                let _ = std::fs::remove_file(&p);
            }
        }
    }
    std::fs::create_dir_all(&log_dir)
        .with_context(|| format!("failed to create log dir {}", log_dir.display()))?;

    // SCRIBE_CONSOLE_LEVEL is set in main.rs before any scribe call.
    // Set SCRIBE_LOG_DIR so boot-time scribe messages (bootstrap,
    // child-process pipe forwarding) land in the deploy log dir rather
    // than the default ./logs.
    // Safety: called before any child spawns, no concurrent access.
    unsafe {
        std::env::set_var("SCRIBE_LOG_DIR", log_dir.as_os_str());
    }

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

    let mut children: Vec<Spawned> = Vec::new();
    let state_path = teardown::state_path(&manifest_dir);
    let started_at_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    let atlas_endpoint = deploy
        .system
        .get("atlas")
        .and_then(|v| v.as_mapping())
        .and_then(|m| m.get(serde_yaml::Value::String("listen".into())))
        .and_then(|v| v.as_str())
        .unwrap_or("127.0.0.1:50051")
        .to_string();
    let spawn_env = PackageSpawnEnv {
        log_dir: &log_dir,
        cache_root: &cache_root,
        instances_dir: &instances_dir,
        manifest_dir: &manifest_dir,
        atlas_endpoint: &atlas_endpoint,
    };

    // Boot is responsible for spawning + atlas registration ONLY.
    // Fetching (git clone of url-remote pkgs) and building are
    // `rbnx build`'s job. We just verify both have happened; if
    // not, warn loudly and remediate inline so the user isn't
    // stuck on a fresh clone.
    check_prerequisites(&deploy, &cache_root, &manifest_dir)?;

    // Install the SIGINT/SIGTERM handlers BEFORE bringup begins, not after.
    // Bringup takes many seconds (git, spawns, waiting for ACTIVE); a Ctrl-C
    // in that window used to hit the default disposition and kill rbnx
    // outright, orphaning every child already spawned. Racing the bringup
    // future against these streams lets us tear the partial stack down
    // instead. (SIGKILL can't be trapped — only SIGINT/SIGTERM.) The same
    // streams are reused for the post-boot idle wait further down.
    let mut sigint = signal(SignalKind::interrupt())?;
    let mut sigterm = signal(SignalKind::terminate())?;

    let bringup = async {
        if !skip_system {
            output::boot_section("system");
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
            // Atlas's contract registry walks every dir in
            // --capabilities at startup. We seed it with:
            //   1. <robonix_source>/capabilities — the global tree
            //   2. <pkg>/capabilities for every primitive/service/skill
            //      package whose source dir is on disk and contains a
            //      `capabilities/` subdir
            // Roots are merged in order; later wins on duplicate id, so
            // a package can re-declare a global contract for itself.
            // A manifest-level override `system.atlas.capabilities`
            // still wins via system_cli_args (clobbers the auto list).
            let mut atlas_caps_roots: Vec<String> = Vec::new();
            if let Some(root) = config.robonix_source_path.as_ref() {
                atlas_caps_roots.push(root.join("capabilities").to_string_lossy().into_owned());
            }
            for entry in deploy
                .primitive
                .iter()
                .chain(deploy.service.iter())
                .chain(deploy.skill.iter())
            {
                if let Ok(pkg_path) = resolve_entry_path(entry, &cache_root, &manifest_dir) {
                    let providers = pkg_path.join("capabilities");
                    if providers.is_dir() {
                        atlas_caps_roots.push(providers.to_string_lossy().into_owned());
                    }
                }
            }
            let atlas_caps_default: Option<String> = if atlas_caps_roots.is_empty() {
                None
            } else {
                Some(atlas_caps_roots.join(","))
            };
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
                let mut args =
                    system_cli_args(name, deploy.system.get(*name), atlas_listen.as_deref());
                if *name == "atlas"
                    && !args.iter().any(|a| a == "--capabilities")
                    && let Some(p) = atlas_caps_default.as_ref()
                {
                    args.push("--capabilities".into());
                    args.push(p.clone());
                }
                // Refuse to spawn if the listen port is already taken — without
                // this, the spawned binary silently dies on bind() failure but
                // boot keeps going against whoever already owns the port (often
                // a stale debug-build atlas/executor/etc from a prior aborted
                // run). The fallout is mysterious: register_capability hits an
                // atlas that doesn't have your takeover/state-push fixes,
                // endpoints route to dead orphan gRPC servers, …
                if let Some(listen) = system_listen(name, deploy.system.get(*name))
                    && let Err(e) = port_is_free(&listen)
                {
                    output::boot_fail(
                        name,
                        &format!(
                            "listen address '{listen}' is taken: {e:#}. \
                                  Stop the running process (try `bash sim/stop.sh` \
                                  or `pkill -f robonix-{name}`) and retry."
                        ),
                    );
                    anyhow::bail!(
                        "system/{name}: listen address '{listen}' is already in use; \
                         refusing to spawn (would shadow the existing process)"
                    );
                }

                // Required-arg validation before spawn. Without this, an empty
                // `${VLM_BASE_URL}` (forgot to source the env file) makes pilot
                // start, register_capability briefly, then die with `missing
                // required field 'vlm.upstream'`. Boot still printed `[ OK ]`
                // because we never re-checked. Fail fast at spawn time and tell
                // the user exactly what's missing.
                if let Err(e) = require_system_args(name, &args) {
                    output::boot_fail(name, &e);
                    anyhow::bail!("system/{name}: {e}");
                }

                let sp = spawn_system_binary(&log_dir, name, bin, &args).await?;
                children.push(sp);
                persist_state(
                    &state_path,
                    &manifest_path,
                    &atlas_endpoint,
                    started_at_ms,
                    &children,
                );
                tokio::time::sleep(std::time::Duration::from_millis(1500)).await;
            }
        } else {
            output::sub_step("Skipping system bring-up (--skip-system)");
        }

        // Connect to atlas once; reuse for every primitive/service init dance.
        let mut atlas =
            AtlasClient::connect_with_retry(&atlas_endpoint, 20, Duration::from_millis(500))
                .await
                .with_context(|| {
                    format!("connect to atlas at '{atlas_endpoint}' for lifecycle init")
                })?;

        // Non-builtin `system:` keys (memory / speech / …) are real robonix
        // packages — same start/init/register flow as primitive/service, just
        // resolved by name against `<robonix_source>/system/<key>/`. Builtin
        // Rust binaries (atlas/executor/pilot) were spawned above and skipped
        // here. A key whose package directory is missing on disk is warned
        // and skipped, not fatal — manifests can declare optional services
        // that aren't installed yet (e.g. liaison while it's being ported).
        // Best-effort boot: a failure on any non-system-builtin package is
        // recorded but does NOT bail the whole bring-up. Goal is to get
        // atlas + executor + pilot + liaison up so `rbnx chat` can still
        // be poked at even when scene / memory / mapping is broken — the
        // alternative (the previous fail-fast model) means a single
        // package's milvus lock or sensor-init quirk gates every other
        // component the operator wants to test.
        //
        // System builtins (atlas/executor/pilot/liaison) are still
        // bail-on-error: nothing else makes sense without those.
        let mut failures: Vec<(String, String, String)> = Vec::new(); // (component, name, err)

        if !skip_system {
            let builtin_names: &[&str] = &["atlas", "executor", "pilot", "liaison"];
            for (key, value) in &deploy.system {
                if builtin_names.contains(&key.as_str()) {
                    continue;
                }
                let pkg_dir = match config.robonix_source_path.as_ref() {
                    Some(root) => root.join("system").join(key),
                    None => {
                        output::boot_skip(
                            key,
                            "robonix_source_path unset (`rbnx setup` from repo root)",
                        );
                        continue;
                    }
                };
                if !pkg_dir.exists() {
                    output::boot_skip(key, "not on disk");
                    continue;
                }
                let entry = PackageEntry {
                    name: key.clone(),
                    path: Some(pkg_dir.to_string_lossy().into_owned()),
                    url: None,
                    branch: None,
                    config: value.clone(),
                    manifest: None,
                };
                match spawn_and_init("system", &entry, &spawn_env, &mut atlas).await {
                    Ok(sp) => {
                        children.push(sp);
                        persist_state(
                            &state_path,
                            &manifest_path,
                            &atlas_endpoint,
                            started_at_ms,
                            &children,
                        );
                    }
                    Err(e) => {
                        failures.push(("system".to_string(), key.clone(), format!("{e:#}")));
                    }
                }
            }
        }

        if !deploy.primitive.is_empty() {
            output::boot_section("primitive");
        }
        for e in &deploy.primitive {
            match spawn_and_init("primitive", e, &spawn_env, &mut atlas).await {
                Ok(sp) => {
                    children.push(sp);
                    persist_state(
                        &state_path,
                        &manifest_path,
                        &atlas_endpoint,
                        started_at_ms,
                        &children,
                    );
                }
                Err(err) => {
                    failures.push(("primitive".to_string(), e.name.clone(), format!("{err:#}")));
                }
            }
        }
        if !deploy.service.is_empty() {
            output::boot_section("service");
        }
        for e in &deploy.service {
            match spawn_and_init("service", e, &spawn_env, &mut atlas).await {
                Ok(sp) => {
                    children.push(sp);
                    persist_state(
                        &state_path,
                        &manifest_path,
                        &atlas_endpoint,
                        started_at_ms,
                        &children,
                    );
                }
                Err(err) => {
                    failures.push(("service".to_string(), e.name.clone(), format!("{err:#}")));
                }
            }
        }
        // Skills are spawned at deploy time, same as services. The
        // semantic distinction (skill = atomic intent invokable by
        // pilot, service = always-on capability) is in the contract
        // namespace (`robonix/skill/*` vs `robonix/service/*`), not
        // in the lifecycle. Skills still need a long-lived process
        // so their MCP tools are registered with atlas; pilot calls
        // those tools on demand. Earlier the manifest had to put
        // `explore` in `service:` as a workaround because skill: was
        // "registered, not spawned" — that lied to consumers about
        // what was actually running. Now skill: spawns the same way
        // service: does, just kept distinct for documentation.
        if !deploy.skill.is_empty() {
            output::boot_section("skill");
        }
        for e in &deploy.skill {
            match spawn_and_init("skill", e, &spawn_env, &mut atlas).await {
                Ok(sp) => {
                    children.push(sp);
                    persist_state(
                        &state_path,
                        &manifest_path,
                        &atlas_endpoint,
                        started_at_ms,
                        &children,
                    );
                }
                Err(err) => {
                    failures.push(("skill".to_string(), e.name.clone(), format!("{err:#}")));
                }
            }
        }
        Ok(failures)
    };

    // Race bringup against the signal streams. On a mid-boot signal the
    // pinned future is dropped at the end of this scope (cancelling bringup
    // at its current await point), which releases its `&mut children` borrow
    // so we can tear down whatever was spawned so far.
    let mut interrupted_during_boot = false;
    let outcome: Result<Vec<(String, String, String)>> = {
        tokio::pin!(bringup);
        tokio::select! {
            o = &mut bringup => o,
            _ = sigint.recv() => { interrupted_during_boot = true; Ok(Vec::new()) }
            _ = sigterm.recv() => { interrupted_during_boot = true; Ok(Vec::new()) }
        }
    };

    if interrupted_during_boot {
        output::action(
            "Interrupted",
            &format!("tearing down {} partial child(ren)", children.len()),
        );
        scribe::info(
            "bootstrap",
            &format!(
                "boot interrupted by signal — tearing down {} partial children",
                children.len()
            ),
        );
        persist_state(
            &state_path,
            &manifest_path,
            &atlas_endpoint,
            started_at_ms,
            &children,
        );
        let providers = component_records(&children);
        teardown::teardown(&providers).await;
        for sp in &mut children {
            let _ = sp.child.wait().await;
        }
        let _ = std::fs::remove_file(&state_path);
        return Ok(());
    }

    let failures = match outcome {
        Ok(failures) => failures,
        Err(e) => {
            output::action("Boot failed", &format!("{e:#}"));
            // System-builtin failure is still terminal — no point
            // pretending the deploy is usable when atlas itself didn't come
            // up. Reap whatever we did spawn before bailing.
            persist_state(
                &state_path,
                &manifest_path,
                &atlas_endpoint,
                started_at_ms,
                &children,
            );
            let providers = component_records(&children);
            teardown::teardown(&providers).await;
            let _ = std::fs::remove_file(&state_path);
            return Err(e);
        }
    };

    if !failures.is_empty() {
        output::boot_section("failures");
        for (component, name, err) in &failures {
            // Trim the err to a single line — the full stack already lives
            // in the per-package log file we listed in the FAIL line.
            let one_line = err.lines().next().unwrap_or(err.as_str());
            output::boot_fail(name, &format!("[{component}] {one_line}"));
        }
        eprintln!();
        eprintln!(
            "  {} of {} packages failed to start; the rest are running. \
             `rbnx caps` to inspect, `rbnx shutdown` to tear down.",
            failures.len(),
            failures.len() + children.len(),
        );
    }

    output::success(&format!(
        "{} component(s) up; logs under {}",
        children.len(),
        log_dir.display()
    ));
    scribe::info("bootstrap", "all components up — waiting for signal");
    output::sub_step("Ctrl-C to tear down (or run `rbnx shutdown` from another shell).");

    // Wait for SIGINT / SIGTERM (reusing the streams installed before
    // bringup), then shut children down.
    tokio::select! {
        _ = sigint.recv() => {}
        _ = sigterm.recv() => {}
    }
    output::action("Stopping", &format!("{} child(ren)", children.len()));
    scribe::info(
        "bootstrap",
        &format!(
            "shutdown signal received, tearing down {} children",
            children.len()
        ),
    );
    let providers = component_records(&children);
    teardown::teardown(&providers).await;
    // Best-effort wait so we get clean "exited" lines in our own log.
    for sp in &mut children {
        let _ = sp.child.wait().await;
    }
    let _ = std::fs::remove_file(&state_path);
    Ok(())
}

fn component_records(children: &[Spawned]) -> Vec<teardown::ComponentRecord> {
    children
        .iter()
        .map(|s| teardown::ComponentRecord {
            name: s.name.clone(),
            kind: s.kind.clone(),
            pid: s.pid,
            pgid: s.pgid,
        })
        .collect()
}

fn persist_state(
    state_path: &Path,
    manifest_path: &Path,
    atlas_endpoint: &str,
    started_at_ms: u64,
    children: &[Spawned],
) {
    let state = teardown::BootState {
        manifest_path: manifest_path.display().to_string(),
        boot_pid: std::process::id(),
        started_at_ms,
        atlas_endpoint: atlas_endpoint.to_string(),
        components: component_records(children),
    };
    if let Err(e) = teardown::write_state(state_path, &state) {
        output::sub_step(&format!(
            "[boot] warning: failed to persist boot state to {}: {e:#}",
            state_path.display()
        ));
    }
}

/// Render a one-line "what is this binary doing" string for the boot
/// log. Pulls out the high-signal flags (port, vlm model+host) and
/// drops noisy ones (--capabilities, --log, raw API keys).
/// Per-binary required-arg sanity check, run before spawning.
///
/// Pilot needs all three VLM fields non-empty. The manifest renders
/// `${VLM_BASE_URL}` etc. literally when the env var isn't set, which
/// produces `--vlm-upstream ""` — pilot then registers briefly, dies
/// with `missing required field 'vlm.upstream'`, and boot reports
/// `[ OK ]` because the failure happens after spawn-and-register. Catch
/// it here so the user sees a `[FAIL]` line naming the bad keys.
fn require_system_args(name: &str, args: &[String]) -> std::result::Result<(), String> {
    if name != "pilot" {
        return Ok(());
    }
    let need = [
        ("--vlm-upstream", "vlm.upstream / VLM_BASE_URL"),
        ("--vlm-api-key", "vlm.api_key / VLM_API_KEY"),
        ("--vlm-model", "vlm.model / VLM_MODEL"),
    ];
    let mut missing: Vec<&str> = Vec::new();
    for (flag, label) in need {
        let val = args
            .iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1));
        match val {
            Some(v) if !v.is_empty() => {}
            _ => missing.push(label),
        }
    }
    if missing.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "missing required pilot config: {}. Set in manifest under \
             system: pilot: vlm: {{...}} or via env (source your .zshrc / \
             inline-prepend VLM_BASE_URL=… VLM_API_KEY=… VLM_MODEL=…)",
            missing.join(", "),
        ))
    }
}

fn system_boot_detail(name: &str, args: &[String]) -> String {
    let mut listen: Option<&str> = None;
    let mut vlm_upstream: Option<&str> = None;
    let mut vlm_model: Option<&str> = None;
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        let next = args.get(i + 1).map(|s| s.as_str());
        match (a, next) {
            ("--listen", Some(v)) => {
                listen = Some(v);
                i += 2;
            }
            ("--vlm-upstream", Some(v)) => {
                vlm_upstream = Some(v);
                i += 2;
            }
            ("--vlm-model", Some(v)) => {
                vlm_model = Some(v);
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }
    let port = listen
        .and_then(|s| s.rsplit(':').next())
        .map(|p| format!(":{p}"))
        .unwrap_or_default();
    if name == "pilot" {
        let host = vlm_upstream
            .and_then(|u| {
                u.trim_start_matches("https://")
                    .trim_start_matches("http://")
                    .split('/')
                    .next()
            })
            .unwrap_or("?");
        let model = vlm_model.unwrap_or("?");
        format!("{port}  vlm={model}@{host}")
    } else {
        port
    }
}

/// Translate a `system.<name>:` block into CLI args for the corresponding
/// Rust binary. Per-binary mapping kept narrow — adding a new flag means
/// touching exactly this function plus the binary's clap struct.
///
/// `atlas_listen` is the value of `system.atlas.listen` (already resolved
/// elsewhere). Consumers that don't carry their own `atlas:` field inherit
/// from this so the manifest doesn't have to repeat the address. An
/// explicit per-block `atlas:` still wins.
/// Extract the `host:port` string each system binary will try to bind.
/// Used by the pre-spawn port-availability check. Returns None for
/// services we don't gate on (or whose listen field is absent — caller
/// then doesn't pre-check).
fn system_listen(name: &str, cfg: Option<&serde_yaml::Value>) -> Option<String> {
    let map = cfg?.as_mapping()?;
    let s = map
        .get(serde_yaml::Value::String("listen".into()))?
        .as_str()?;
    let trimmed = s.trim();
    if trimmed.is_empty() || !matches!(name, "atlas" | "executor" | "pilot" | "liaison") {
        return None;
    }
    Some(trimmed.to_string())
}

/// Probe a host:port. Returns `Ok(())` when nothing is listening (we can
/// safely bind), `Err` describing the live owner otherwise. This is a
/// race-prone pre-check (someone else can grab the port between probe
/// and spawn) but in practice the failure mode it catches — a stale
/// previous-boot daemon — has been alive for minutes, not seconds, so
/// a single connect attempt is enough.
fn port_is_free(listen: &str) -> std::result::Result<(), anyhow::Error> {
    use std::net::{TcpStream, ToSocketAddrs};
    let addrs: Vec<_> = listen
        .to_socket_addrs()
        .with_context(|| format!("parse listen='{listen}' as socket addr"))?
        .collect();
    for addr in &addrs {
        // 200 ms is enough for a local connect; if a daemon is alive on
        // 127.0.0.1 the SYN-ACK is sub-ms.
        if TcpStream::connect_timeout(addr, std::time::Duration::from_millis(200)).is_ok() {
            return Err(anyhow::anyhow!("something is already listening on {addr}"));
        }
    }
    Ok(())
}

fn system_cli_args(
    name: &str,
    cfg: Option<&serde_yaml::Value>,
    atlas_listen: Option<&str>,
) -> Vec<String> {
    let mut out = Vec::new();
    let map = cfg.and_then(|v| v.as_mapping());

    // Pass the component's whole manifest config block as one JSON arg. The
    // binary parses the keys it needs (e.g. scribe reads `log` via
    // robonix_scribe::init_from_config), so new manifest keys flow through
    // without per-key plumbing here. The typed flags below remain for the
    // fields binaries still read individually.
    if let Some(v) = cfg
        && let Ok(json) = serde_json::to_string(v)
    {
        out.push("--config-json".into());
        out.push(json);
    }

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
            // Atlas walks `<root>/capabilities/**/*.toml` at startup to
            // build the contract registry. Honour an explicit override
            // from the manifest, otherwise let atlas fall back to its
            // own ROBONIX_SOURCE_PATH-derived default (we don't pass
            // --capabilities here from rbnx; deploy.rs sets the env var
            // on the spawned process so the default path stays correct
            // even when manifests don't mention atlas at all).
            push_pair(&mut out, "--capabilities", s("capabilities"));
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
        "liaison" => {
            push_pair(&mut out, "--listen", s("listen"));
            push_pair(
                &mut out,
                "--atlas",
                s("atlas").or_else(|| atlas_listen.map(str::to_string)),
            );
            push_pair(&mut out, "--pilot-endpoint", s("pilot_endpoint"));
            push_pair(&mut out, "--log", s("log"));
        }
        _ => {}
    }
    out
}

/// Spawn one primitive / service package and wait for it to register
/// at least one capability with atlas. If the new provider has a `*/driver`
/// gRPC capability, also drive Driver(CMD_INIT) and pass the entry's
/// `config:` as `config_json`. Packages that don't declare a driver
/// (e.g. system packages or new packages that just
/// don't need init-time wiring) are deployed as-is once their first provider
/// appears in atlas.
async fn spawn_and_init(
    component: &str,
    entry: &PackageEntry,
    spawn_env: &PackageSpawnEnv<'_>,
    atlas: &mut AtlasClient,
) -> Result<Spawned> {
    let before: HashSet<String> = atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await
        .with_context(|| format!("[{component}] pre-spawn atlas snapshot"))?
        .into_iter()
        .map(|r| r.id)
        .collect();

    let sp = spawn_package(component, entry, spawn_env).await?;
    let pkg_label = sp.name.clone();

    // One package = one provider. After spawn, the new provider_id is whatever
    // atlas saw register that wasn't in `before`.

    // Once the wrapper is up, every error path below must SIGKILL the
    // PGID before bailing — otherwise `?` returns the spawned process to
    // a dead Spawned (which itself has no killing Drop), the caller's
    // teardown loop never sees it (`children.push(sp)` only runs after
    // this fn succeeds), and the orphan keeps holding whatever the
    // package opened (e.g. memsearch's milvus DB lock, executor's gRPC
    // port, …). Reaping here keeps boot recoverable: a fresh `rbnx boot`
    // immediately after a failed one finds a clean process table.
    let pgid = sp.pgid;
    let reap = || {
        let _ = nix::sys::signal::killpg(
            nix::unistd::Pid::from_raw(pgid as i32),
            nix::sys::signal::Signal::SIGKILL,
        );
    };

    let (provider_id, driver_contract) =
        match wait_for_registration(atlas, &before, &pkg_label, component, spawn_env.log_dir).await
        {
            Ok(v) => v,
            Err(e) => {
                reap();
                return Err(e);
            }
        };

    // Spec: the provider_id this process registers (Python's
    // `Capability(id=...)`) MUST equal robonix_manifest.yaml's `name:`
    // for this entry. Mismatch is a deploy bug — surfacing it here
    // beats letting downstream consumers fail with cryptic
    // "no provider for X" errors.
    if provider_id != entry.name {
        let log_file = log_path(spawn_env.log_dir, &pkg_label);
        output::boot_fail(
            short_label(&pkg_label, component),
            &format!(
                "provider_id mismatch: manifest says name='{}' but Capability(id='{}') registered. \
                 Fix python source to match manifest. Log: {}",
                entry.name,
                provider_id,
                log_file.display()
            ),
        );
        reap();
        anyhow::bail!(
            "[{component}/{pkg_label}] provider_id mismatch: manifest name='{}' vs Capability(id='{}')",
            entry.name,
            provider_id,
        );
    }

    let Some(driver_contract) = driver_contract else {
        // No driver contract — system providers auto-promote to ACTIVE on
        // their own once gRPC + MCP are listening. We don't drive INIT
        // / ACTIVATE for them.
        output::boot_ok(short_label(&pkg_label, component), "ACTIVE  (no driver)");
        return Ok(sp);
    };

    let config_json = serde_json::to_string(&entry.config).unwrap_or_else(|_| "{}".into());

    let display_label = short_label(&pkg_label, component);
    let init_state = match with_spinner(
        display_label,
        "driver(INIT)…",
        call_driver_cmd(
            atlas,
            &provider_id,
            &driver_contract,
            component,
            &pkg_label,
            CMD_INIT,
            config_json.clone(),
        ),
    )
    .await
    {
        Ok(v) => v,
        Err(e) => {
            reap();
            return Err(e);
        }
    };

    if component == "skill" {
        // Skills stop at INACTIVE post-INIT; the executor sends
        // CMD_ACTIVATE on first MCP call (lazy-activate).
        output::boot_ok(
            display_label,
            &format!(
                "{}  (skill — awaits executor activate)",
                init_state.to_uppercase()
            ),
        );
        return Ok(sp);
    }

    let activate_state = match with_spinner(
        display_label,
        "driver(ACTIVATE)…",
        call_driver_cmd(
            atlas,
            &provider_id,
            &driver_contract,
            component,
            &pkg_label,
            CMD_ACTIVATE,
            config_json,
        ),
    )
    .await
    {
        Ok(v) => v,
        Err(e) => {
            reap();
            return Err(e);
        }
    };
    // Boot succeeded: provider walked REGISTERED → INACTIVE → ACTIVE. Show
    // only the final state — the two intermediate driver calls already
    // got their own spinner lines and OK ticks above. provider_id is the
    // leftmost label so we don't repeat it here.
    let _ = init_state; // intermediate, only kept for the assertion below
    output::boot_ok(display_label, &activate_state.to_uppercase());

    Ok(sp)
}

/// Run `fut` while animating the boot spinner so the user sees the
/// `[ ⠙ ] name  msg_prefix N.Ns` line update steadily even when the
/// underlying RPC takes a while (Driver(CMD_INIT) for sensor-warm-up
/// packages routinely sits at 30+ seconds). Without this the line goes
/// silent right after `wait_for_registration` finishes and rbnx looks
/// hung between OK lines.
async fn with_spinner<F, T>(label: &str, msg_prefix: &str, fut: F) -> T
where
    F: std::future::Future<Output = T>,
{
    use std::time::Instant;
    let started = Instant::now();
    let mut tick = tokio::time::interval(Duration::from_millis(100));
    tick.tick().await; // first tick fires immediately; consume so the
    // first redraw is delayed by 100 ms (no double-frame at t=0).
    tokio::pin!(fut);
    let mut frame: usize = 0;
    loop {
        tokio::select! {
            res = &mut fut => return res,
            _ = tick.tick() => {
                let elapsed = started.elapsed().as_secs_f32();
                output::boot_progress(
                    label,
                    &format!("{msg_prefix} {elapsed:>4.1}s"),
                    frame,
                );
                frame = frame.wrapping_add(1);
            }
        }
    }
}

/// Issue one Driver(cmd) RPC against a freshly-connected channel, then
/// release the channel. Returns the response's `state` string on success;
/// bail-errors when ok=false or the RPC itself fails. Used by the boot
/// path for both CMD_INIT and CMD_ACTIVATE, with identical timeout / channel
/// hygiene.
async fn call_driver_cmd(
    atlas: &mut AtlasClient,
    provider_id: &str,
    driver_contract: &str,
    component: &str,
    pkg_label: &str,
    cmd: u32,
    config_json: String,
) -> Result<String> {
    let cmd_name = match cmd {
        CMD_INIT => "INIT",
        CMD_ACTIVATE => "ACTIVATE",
        CMD_DEACTIVATE => "DEACTIVATE",
        CMD_SHUTDOWN => "SHUTDOWN",
        _ => "?",
    };
    let (channel_id, endpoint, _params) = atlas
        .connect_capability(
            DEPLOY_CONSUMER_ID,
            provider_id,
            driver_contract,
            atlas_pb::Transport::Grpc,
        )
        .await
        .with_context(|| {
            format!("[{component}/{pkg_label}] ConnectCapability for {driver_contract}")
        })?;
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
        let svc_name = contract_id_to_service_name(driver_contract);
        let path: tonic::codegen::http::uri::PathAndQuery =
            format!("/robonix.contracts.{svc_name}/Driver")
                .parse()
                .with_context(|| format!("build gRPC path for '{driver_contract}'"))?;
        let mut grpc = tonic::client::Grpc::new(channel);
        grpc.ready().await.with_context(|| "gRPC ready")?;
        let codec: tonic_prost::ProstCodec<DriverRequest, DriverResponse> = Default::default();
        let resp = tokio::time::timeout(
            driver_timeout,
            grpc.unary(
                Request::new(DriverRequest {
                    command: cmd,
                    config_json,
                }),
                path,
                codec,
            ),
        )
        .await
        .map_err(|_| {
            anyhow::anyhow!(
                "Driver(CMD_{cmd_name}) timed out after {}s",
                driver_timeout.as_secs()
            )
        })?
        .with_context(|| format!("Driver(CMD_{cmd_name}) RPC failed"))?;
        Ok::<_, anyhow::Error>(resp.into_inner())
    }
    .await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    let r = result
        .map_err(|e| anyhow::anyhow!("[{component}/{pkg_label}] Driver(CMD_{cmd_name}): {e:#}"))?;
    if !r.ok {
        anyhow::bail!(
            "[{component}/{pkg_label}] Driver(CMD_{cmd_name}) returned ok=false (state={}, error={})",
            r.state,
            r.error
        );
    }
    Ok(r.state)
}

/// Mirrors `robonix_codegen::contract_gen::contract_id_to_service_name`.
/// Uniform PascalCase: `robonix/primitive/chassis/driver` →
/// `RobonixPrimitiveChassisDriver`. No prefix stripping. Full gRPC
/// service path: `/robonix.contracts.<this>/Driver`.
fn contract_id_to_service_name(id: &str) -> String {
    id.split('/')
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

/// Poll atlas until a provider NOT in `before` appears. Returns the new
/// `provider_id` plus an optional `driver_contract_id` if the new provider
/// declared a `*/driver` gRPC capability (signal to the caller that
/// Driver(CMD_INIT) lifecycle should run).
/// Strip the leading `<component>_` from the boot-log pkg_label.
/// `system_memory` → `memory`; `primitive_tiago_chassis` → `tiago_chassis`.
/// Keeps boot-output columns narrow (the section header above already
/// said which class the entry belongs to).
fn short_label<'a>(pkg_label: &'a str, component: &str) -> &'a str {
    pkg_label
        .strip_prefix(&format!("{component}_"))
        .unwrap_or(pkg_label)
}

async fn wait_for_registration(
    atlas: &mut AtlasClient,
    before: &HashSet<String>,
    pkg_label: &str,
    component: &str,
    log_dir: &Path,
) -> Result<(String, Option<String>)> {
    // One package = one provider. Find the provider that wasn't in `before`.
    // Multiple new providers = deploy bug, fail loud. No heartbeat-based
    // freshness fallback — every existing ACTIVE provider heartbeats
    // periodically and would falsely match.
    const SPINNER_TICK: Duration = Duration::from_millis(100);
    const POLLS_PER_TICK: u32 = 2; // poll atlas every 200 ms
    let started = Instant::now();
    let deadline = started + DRIVER_REGISTER_TIMEOUT;
    let mut frame: usize = 0;
    let display_label = short_label(pkg_label, component);
    loop {
        let elapsed_s = started.elapsed().as_secs_f32();
        output::boot_progress(
            display_label,
            &format!("registering with atlas… {elapsed_s:>4.1}s"),
            frame,
        );
        if frame.is_multiple_of(POLLS_PER_TICK as usize) {
            let providers = atlas
                .query_capabilities("", "", atlas_pb::Transport::Unspecified)
                .await
                .with_context(|| format!("[{component}/{pkg_label}] poll atlas"))?;
            let matches: Vec<&atlas_pb::CapabilityProvider> = providers
                .iter()
                .filter(|provider| !before.contains(&provider.id))
                .collect();
            if matches.len() > 1 {
                let log_file = log_path(log_dir, pkg_label);
                let cap_ids: Vec<&str> = matches.iter().map(|r| r.id.as_str()).collect();
                output::boot_fail(
                    display_label,
                    &format!(
                        "multiple new providers appeared from one spawn ({}) — \
                         package start must register exactly one Capability. Log: {}",
                        cap_ids.join(", "),
                        log_file.display()
                    ),
                );
                anyhow::bail!(
                    "[{component}/{pkg_label}] multiple new providers from one spawn: {} \
                     — spec is one package start = one Capability(id=...). Log: {}",
                    cap_ids.join(", "),
                    log_file.display()
                );
            }
            if let Some(first) = matches.first() {
                let provider_id = first.id.clone();
                // RegisterPrimitive/Service/Skill and DeclareCapability are
                // two separate RPCs from the package side — Register lands
                // first, declares follow within a few hundred ms. Give it
                // up to a 1 s settle window so we don't false-fire the
                // "no driver" path on a fast poll. Capped by the outer
                // `deadline` so we never exceed user-facing timeout.
                let settle_until = Instant::now()
                    .checked_add(Duration::from_millis(1000))
                    .map(|t| t.min(deadline))
                    .unwrap_or(deadline);
                let mut current: atlas_pb::CapabilityProvider = (*first).clone();
                let driver_contract_id = loop {
                    let driver = current.capabilities.iter().find(|cap| {
                        cap.transport == atlas_pb::Transport::Grpc as i32
                            && cap.contract_id.ends_with("/driver")
                    });
                    if driver.is_some() {
                        break driver.map(|c| c.contract_id.clone());
                    }
                    if Instant::now() >= settle_until {
                        break None;
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                    let providers = atlas
                        .query_capabilities(&provider_id, "", atlas_pb::Transport::Unspecified)
                        .await
                        .with_context(|| format!("[{component}/{pkg_label}] re-poll for driver"))?;
                    match providers.into_iter().find(|p| p.id == provider_id) {
                        Some(p) => current = p,
                        None => {
                            // Provider vanished between the original match
                            // and now (crashed mid-settle, atlas evicted,
                            // heartbeat lapsed). Report loudly — silently
                            // returning "no driver" would let downstream
                            // boot logic march on against a dead process.
                            let log_file = log_path(log_dir, pkg_label);
                            output::boot_fail(
                                display_label,
                                &format!(
                                    "provider '{provider_id}' disappeared during settle — see {}",
                                    log_file.display()
                                ),
                            );
                            anyhow::bail!(
                                "[{component}/{pkg_label}] provider '{provider_id}' \
                                 unregistered during settle window. Log: {}",
                                log_file.display()
                            );
                        }
                    }
                };
                return Ok((provider_id, driver_contract_id));
            }
        }
        if Instant::now() >= deadline {
            let log_file = log_path(log_dir, pkg_label);
            output::boot_fail(
                display_label,
                &format!(
                    "registration timeout after {:?} — see {}",
                    DRIVER_REGISTER_TIMEOUT,
                    log_file.display()
                ),
            );
            anyhow::bail!(
                "[{component}/{pkg_label}] timed out after {:?} — package never registered a provider with atlas. Log: {}",
                DRIVER_REGISTER_TIMEOUT,
                log_file.display()
            );
        }
        tokio::time::sleep(SPINNER_TICK).await;
        frame = frame.wrapping_add(1);
    }
}
