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
//     Driver(CMD_INIT)'s config_json field. Omitting Driver is the canonical
//     way to select the shared lifecycle service. An exact legacy manifest may
//     use a current shared runtime Driver while it is migrated. A provider
//     without exactly one lifecycle Driver fails startup.
//     The provider process never sees a config file or env var.
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
use robonix_cli::launch::{
    PackageRuntimeRecord, ProviderRegistrationSnapshot, RegistrationOutcome,
    resolve_runtime_driver_contract, snapshot_provider_ids, terminate_process_group,
};
use robonix_cli::output;
use serde::Deserialize;
use std::collections::{HashMap, HashSet};
use std::os::fd::RawFd;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::time::{Duration, Instant};
use tokio::io::AsyncBufReadExt;
use tokio::process::{Child, Command};
use tokio::signal::unix::{SignalKind, signal};
use tonic::Request;
use tonic::transport::Endpoint;
use uuid::Uuid;

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
    /// Opaque config block; serialised to JSON and delivered through
    /// Driver(CMD_INIT). Startup fails if the provider does not declare its
    /// selected shared or exact compatible legacy lifecycle Driver.
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
///
/// Compatibility forwarding entry for sibling commands. The implementation
/// lives in `robonix_cli::manifest` so Soma and rbnx use one rule.
pub(crate) fn repo_dir_name(url: &str) -> String {
    robonix_cli::manifest::deploy_repo_dir_name(url)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boot_prerequisites_build_non_builtin_system_packages() {
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock")
            .as_nanos();
        let temp = std::env::temp_dir().join(format!(
            "rbnx-system-prerequisite-{}-{nonce}",
            std::process::id()
        ));
        let scene = temp.join("system/scene");
        std::fs::create_dir_all(&scene).expect("scene package directory");
        std::fs::write(
            scene.join("package_manifest.yaml"),
            r#"manifestVersion: 1
package:
  name: com.robonix.system.scene.test
  version: 0.1.0
  description: test system package
  license: MulanPSL-2.0
build: mkdir -p rbnx-build && touch rbnx-build/proof
start: "true"
stop: "true"
"#,
        )
        .expect("test package manifest");

        let deploy = DeployManifest {
            system: HashMap::from([("scene".to_string(), serde_yaml::Value::Null)]),
            ..Default::default()
        };
        let manifest_dir = temp.join("deployment");
        let cache_root = manifest_dir.join("rbnx-boot/cache");
        std::fs::create_dir_all(&cache_root).expect("cache directory");

        check_prerequisites(&deploy, &cache_root, &manifest_dir, Some(&temp))
            .expect("system-package prerequisite build");

        assert!(scene.join("rbnx-build/proof").is_file());
        assert!(scene.join("rbnx-build/.rbnx-built").is_file());
        std::fs::remove_dir_all(temp).expect("remove test directory");
    }

    #[test]
    fn soma_always_receives_the_selected_boot_manifest() {
        use serde_yaml::{Mapping, Value};

        let manifest_dir = PathBuf::from("/tmp/ranger-deploy");
        let selected = manifest_dir.join("robonix_manifest.arm.yaml");
        let mut soma = Mapping::new();
        // A stale local value must not make Soma boot the default profile
        // after `rbnx boot -f <arm-profile>`.
        soma.insert(
            Value::String("deployment_manifest".into()),
            Value::String("robonix_manifest.yaml".into()),
        );
        let mut system = HashMap::from([("soma".to_string(), Value::Mapping(soma))]);

        ensure_soma_defaults(&mut system, &manifest_dir, &selected);
        let args = system_cli_args("soma", system.get("soma"), None);
        let manifest_arg = args
            .windows(2)
            .find(|pair| pair[0] == "--deployment-manifest")
            .map(|pair| pair[1].as_str());

        assert_eq!(manifest_arg, Some(selected.to_string_lossy().as_ref()));
    }

    #[test]
    fn vitals_receives_typed_manifest_fields() {
        let cfg: serde_yaml::Value = serde_yaml::from_str(
            r#"
listen: 0.0.0.0:50093
provider_id: vitals
thresholds_path: config/vitals.yaml
soma_endpoint: 127.0.0.1:50091
"#,
        )
        .unwrap();
        let args = system_cli_args("vitals", Some(&cfg), Some("0.0.0.0:50051"));

        for expected in [
            ["--listen", "0.0.0.0:50093"],
            ["--atlas", "0.0.0.0:50051"],
            ["--id", "vitals"],
            ["--thresholds-path", "config/vitals.yaml"],
            ["--soma-endpoint", "127.0.0.1:50091"],
        ] {
            assert!(
                args.windows(2)
                    .any(|pair| pair[0] == expected[0] && pair[1] == expected[1]),
                "missing {:?} in {:?}",
                expected,
                args
            );
        }
    }

    #[test]
    fn provider_failure_ignores_later_shutdown_noise() {
        let path = std::env::temp_dir().join(format!(
            "rbnx-provider-failure-{}.log",
            uuid::Uuid::new_v4()
        ));
        std::fs::write(
            &path,
            concat!(
                "{\"level\":\"info\",\"msg\":\"ready -- awaiting Driver(CMD_INIT)\"}\n",
                "{\"level\":\"info\",\"msg\":\"[ranger_chassis] state REGISTERED -> ERROR (CAN setup failed: sudo password required)\"}\n",
                "{\"level\":\"info\",\"msg\":\"shutdown hook completed\"}\n",
            ),
        )
        .unwrap();

        assert_eq!(
            read_provider_failure(&path).as_deref(),
            Some("CAN setup failed: sudo password required")
        );
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn provider_failure_accepts_error_level_records() {
        let path = std::env::temp_dir().join(format!(
            "rbnx-provider-error-level-{}.log",
            uuid::Uuid::new_v4()
        ));
        std::fs::write(
            &path,
            "{\"level\":\"error\",\"msg\":\"camera device disconnected\"}\n",
        )
        .unwrap();

        assert_eq!(
            read_provider_failure(&path).as_deref(),
            Some("camera device disconnected")
        );
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn provider_exit_summary_uses_last_structured_message() {
        let path = std::env::temp_dir().join(format!(
            "rbnx-provider-exit-summary-{}.log",
            uuid::Uuid::new_v4()
        ));
        std::fs::write(
            &path,
            concat!(
                "{\"level\":\"info\",\"msg\":\"Traceback (most recent call last):\"}\n",
                "{\"level\":\"info\",\"msg\":\"ImportError: generated contract is missing\"}\n",
                "{\"level\":\"info\",\"msg\":\"Error: scene process exited with status 1\"}\n",
            ),
        )
        .unwrap();

        assert_eq!(
            read_provider_exit_summary(&path).as_deref(),
            Some("Error: scene process exited with status 1")
        );
        let _ = std::fs::remove_file(path);
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
    robonix_source_path: Option<&Path>,
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

    // Non-builtin `system:` entries are packages too.  They are resolved
    // from the configured Robonix source tree rather than from an explicit
    // deployment `path:`, so the package loop above cannot see them.  Build
    // them during prerequisites just like primitive/service/skill packages;
    // otherwise `rbnx start` performs the build after spawn and the provider
    // registration timeout can kill a legitimate first build (Scene model
    // downloads are a common example).
    const SYSTEM_BUILTINS: &[&str] = &["atlas", "executor", "pilot", "liaison", "soma"];
    if let Some(source_root) = robonix_source_path {
        for name in deploy.system.keys() {
            if SYSTEM_BUILTINS.contains(&name.as_str()) {
                continue;
            }
            let pkg_path = source_root.join("system").join(name);
            if !pkg_path.exists() {
                continue; // the later system-package loop reports optional packages
            }
            let stamp = pkg_path.join("rbnx-build").join(".rbnx-built");
            if !stamp.exists() {
                needs_build.insert(name.clone(), (pkg_path, None));
            }
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

/// Apply top-level deployment variables, then expand every scalar in the
/// manifest. Build and boot share this preparation path so package locations,
/// target-manifest selectors, system settings, and package config all resolve
/// against the same environment.
pub(super) fn prepare_manifest(
    root: serde_yaml::Value,
    robonix_source_path: Option<&Path>,
) -> Result<serde_yaml::Value> {
    let prepared = robonix_cli::manifest::prepare_deployment_manifest(root, robonix_source_path)?;
    robonix_cli::manifest::validate_deployment_instance_names(&prepared)?;
    Ok(prepared)
}

/// Make sure `system.soma` exists in the manifest map as a mapping,
/// and resolve any relative file paths inside it against `manifest_dir`.
///
/// v2 soma has four flat config keys — `atlas_endpoint`, `listen`,
/// `provider_id`, `robot_yaml` — and rbnx forwards them via CLI.
///
/// This helper handles the two "soma implied but not spelled out"
/// manifest patterns:
///   * `system.soma:` with no body — a bare tag or null.
///   * no `system.soma:` at all, but `primitive:` / `skill:` present.
///
/// In both cases we promote / insert an empty mapping so the rest of
/// deploy.rs (system_cli_args, the builtin loop, the stage 2 pipe)
/// sees a populated entry. Existing operator-supplied values are
/// NEVER overwritten.
///
/// It also normalises the two path-valued fields inside `system.soma`
/// — `robot_yaml` and `config` — from possibly relative to always
/// absolute. rbnx passes both straight through to `robonix-soma` as
/// CLI flags without chdir-ing, and soma itself only resolves paths
/// relative to its `--config` file's parent dir (not the manifest's
/// dir), so any relative value written by the operator has to be
/// pinned here or soma will try to open it from its own cwd — which
/// on systemd-launched Jetsons is `/`, producing errors like
/// `read Soma config '/soma_config.local.yaml' … No such file`.
///
/// Semantics: if the value is absolute it is left untouched (operator
/// escape hatch for bind-mounts, /opt paths, etc.); if it's relative
/// (including bare filenames like `soma.yaml`) it is joined onto the
/// manifest's own directory. Non-string values are ignored — malformed
/// manifests will surface the type mismatch at soma CLI parse time
/// rather than being silently rewritten here.
///
/// `robot_yaml` auto-injection: soma refuses to boot without a
/// `--robot-yaml` — it needs the robot description before it can
/// spawn any primitive in stage 1. Previously we left this to the
/// operator, but the failure mode ("missing robot_yaml" → soma exits
/// → rbnx sits in `wait_for_soma_stage1` for the full 180s timeout
/// before reporting failure) is disproportionately painful for a
/// missing default. So: if the operator hasn't set `robot_yaml` and
/// a file literally named `soma.yaml` sits next to the manifest, we
/// inject that path. If neither is present, we leave the slot empty
/// — soma will bail on config parse with a clear error, and the
/// stage-1 waiter (see `wait_for_soma_stage1`) will surface soma's
/// early exit instead of waiting for the timeout. Operators who
/// want a different name still just set `robot_yaml:` explicitly.
fn ensure_soma_defaults(
    system: &mut HashMap<String, serde_yaml::Value>,
    manifest_dir: &Path,
    manifest_path: &Path,
) {
    use serde_yaml::{Mapping, Value};
    let entry = system
        .entry("soma".to_string())
        .or_insert_with(|| Value::Mapping(Mapping::new()));
    // Promote a non-mapping value (`soma: ~`, `soma: true`, ...) to an
    // empty mapping so operators who wrote `soma:` with no body get a
    // usable slot rather than a parse-time surprise.
    if !entry.is_mapping() {
        *entry = Value::Mapping(Mapping::new());
    }
    let map = entry
        .as_mapping_mut()
        .expect("promoted to mapping just above");

    // Soma launches primitive and skill packages itself. It therefore must
    // read the exact deployment file selected by `rbnx boot -f`, not infer a
    // sibling default manifest from robot_yaml. This is intentionally owned
    // by the boot command: a stale manifest-local value must not make rbnx
    // build one profile while Soma starts another.
    map.insert(
        Value::String("deployment_manifest".to_string()),
        Value::String(manifest_path.to_string_lossy().into_owned()),
    );

    // Auto-inject `robot_yaml: <manifest_dir>/soma.yaml` when the
    // operator didn't set it AND the file exists. We check for the
    // key's presence-and-non-emptiness rather than presence alone so
    // an unset `${ROBOT_YAML}` (which manifest preparation turns into
    // "") still triggers the sidecar lookup. Missing sidecar → leave
    // absent (soma's own config-resolve error is the right signal;
    // stage-1 waiter surfaces the early exit fast).
    let robot_yaml_key = Value::String("robot_yaml".to_string());
    let robot_yaml_missing = match map.get(&robot_yaml_key) {
        None => true,
        Some(Value::Null) => true,
        Some(Value::String(s)) if s.is_empty() => true,
        _ => false,
    };
    if robot_yaml_missing {
        let sidecar = manifest_dir.join("soma.yaml");
        if sidecar.is_file() {
            map.insert(
                robot_yaml_key,
                Value::String(sidecar.to_string_lossy().into_owned()),
            );
        }
    }

    for key in ["robot_yaml", "deployment_manifest", "config"] {
        let k = Value::String(key.to_string());
        let Some(v) = map.get_mut(&k) else { continue };
        let Some(s) = v.as_str() else { continue };
        // Empty string usually means "${SOME_UNSET_VAR}" got expanded
        // away by manifest preparation. Don't paper over that by turning
        // it into `manifest_dir/` — leave it empty so soma's own
        // "read Soma config '' … No such file" error still fires and
        // the operator gets a signal instead of a mystery success.
        if s.is_empty() {
            continue;
        }
        let p = Path::new(s);
        if p.is_absolute() {
            continue;
        }
        let joined = manifest_dir.join(p);
        *v = Value::String(joined.to_string_lossy().into_owned());
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
    /// it becomes the leader of a new PGID == its own PID.
    pgid: u32,
    provider_id: Option<String>,
    driver_contract: Option<String>,
    /// Lifecycle contract selected by this package's exact manifest. Builtin
    /// system processes are not package-managed and leave this unset.
    expected_driver_contract: Option<String>,
    /// True only for an explicit legacy selection, permitting a current shared
    /// runtime Driver while the manifest is migrated.
    allow_shared_driver_upgrade: bool,
    config_json: Option<String>,
    package_dir: Option<PathBuf>,
    stop: Option<String>,
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
        provider_id: None,
        driver_contract: None,
        expected_driver_contract: None,
        allow_shared_driver_upgrade: false,
        config_json: None,
        package_dir: None,
        stop: None,
    })
}

/// Fixed fd number rbnx exports as `ROBONIX_SOMA_STAGE_FD` in soma's
/// environment. Any fd ≥ 3 works — we pick 3 because it's the first
/// non-stdio slot, which keeps `ls /proc/<soma>/fd` readable at a
/// glance. Soma reads the trigger line, then closes it.
const SOMA_STAGE_FD: RawFd = 3;

/// Spawn soma with an inherited pipe on `SOMA_STAGE_FD`. The parent
/// keeps the write end and later writes `stage2\n` to it (see
/// `write_stage2_trigger` below). Layered on `spawn_system_binary`'s
/// stdio+scribe pattern but adds:
///   * pipe() to create the trigger channel
///   * pre_exec dup2 to move the child's end onto SOMA_STAGE_FD
///     (the natural fd from pipe() is unpredictable — some later
///     lib open() call could grab it — so we pin it to a known
///     number)
///   * ROBONIX_SOMA_STAGE_FD env so soma finds it
///   * close-on-exec cleared on the child fd (dup2 clears it by
///     default, which is what we want)
///
/// Returns the Spawned handle and the parent's write-end File. Drop
/// the File to close the pipe (soma sees EOF and continues without
/// stage 2 — matches the `no fd` env-absent path).
async fn spawn_soma_binary(
    log_dir: &Path,
    name: &str,
    bin: &str,
    args: &[String],
) -> Result<(Spawned, std::fs::File)> {
    use std::os::fd::{AsRawFd, IntoRawFd, OwnedFd};

    // Create the trigger pipe. Parent owns the write end for the
    // lifetime of the boot; child inherits the read end. We keep
    // the OwnedFd wrappers so an early error path drops the fds
    // rather than leaking them.
    let (read_fd, write_fd): (OwnedFd, OwnedFd) =
        nix::unistd::pipe().context("pipe() for soma stage-2 trigger")?;
    let child_raw = read_fd.as_raw_fd();

    // tokio::process::Command is a thin wrapper over std::process,
    // but pre_exec lives on the std side. We prime the std Command
    // via .as_std_mut() below.
    let mut cmd = Command::new(bin);
    for a in args {
        cmd.arg(a);
    }
    cmd.stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .env("SCRIBE_LOG_DIR", log_dir)
        .env("ROBONIX_SOMA_STAGE_FD", SOMA_STAGE_FD.to_string())
        .process_group(0);

    // Hand the child's raw read fd into the closure. We can NOT let
    // `read_fd` (the OwnedFd) run its Drop in the parent before the
    // child inherits it — that would close the fd. Move ownership
    // into the closure and leak/consume it there.
    let read_owned = read_fd; // captured
    let child_target = SOMA_STAGE_FD;
    // Safety: pre_exec runs in the forked child before exec. Only
    // async-signal-safe syscalls are permitted; dup2 and close are
    // both on that list.
    unsafe {
        cmd.pre_exec(move || {
            // dup2(oldfd, newfd) atomically closes newfd (if open)
            // and duplicates oldfd onto it. The new fd has
            // CLOEXEC=0 by default, which is what we want (soma
            // needs to see it after exec).
            let old = read_owned.as_raw_fd();
            if old != child_target {
                let ret = libc_dup2(old, child_target);
                if ret < 0 {
                    return Err(std::io::Error::last_os_error());
                }
                // Original fd number is no longer needed in the
                // child; close it so it doesn't linger.
                let _ = libc_close(old);
            }
            Ok(())
        });
    }

    let mut child = cmd.spawn().with_context(|| {
        format!(
            "failed to spawn system binary `{bin}` — is it installed (try `make install` from the rust/ workspace)?"
        )
    })?;
    let pid = child
        .id()
        .ok_or_else(|| anyhow::anyhow!("spawned `{bin}` but it had no pid"))?;

    // fork() + our pre_exec dup2 have run; the child has its own
    // copy on fd 3 and the fork copy of read_owned (whatever number
    // pipe() picked). The parent's read_owned has been consumed by
    // the closure — the value inside the parent process is a dead
    // OwnedFd shell that will drop at the end of pre_exec's scope.
    // Nothing left to close on this side; child_raw is only used
    // for the debug/log line below.
    let _ = child_raw;

    // Turn the parent's write-end OwnedFd into a std File so the
    // caller can write! into it. into_raw_fd releases ownership;
    // File::from_raw_fd takes it back.
    let write_raw = write_fd.into_raw_fd();
    // Safety: write_raw is a valid, open, owned fd we just released
    // from OwnedFd — we're transferring ownership one hop over.
    let writer = unsafe { <std::fs::File as std::os::fd::FromRawFd>::from_raw_fd(write_raw) };

    // Pipe stdout / stderr into Scribe. (Same pattern as
    // spawn_system_binary — kept inline rather than extracted so
    // both call sites stay readable.)
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
            scribe::ingest(&tag_err, &line);
        }
    });

    let detail = system_boot_detail(name, args);
    output::boot_ok(name, &detail);
    Ok((
        Spawned {
            name: name.to_string(),
            kind: "system_builtin".to_string(),
            child,
            pid,
            pgid: pid,
            provider_id: None,
            driver_contract: None,
            expected_driver_contract: None,
            allow_shared_driver_upgrade: false,
            config_json: None,
            package_dir: None,
            stop: None,
        },
        writer,
    ))
}

// Local libc thunks to avoid pulling libc as a direct dep — nix
// exposes these via `nix::unistd::dup2` / `close`, but we need
// async-signal-safety inside pre_exec and can't rely on nix's
// wrappers not allocating on the error path. Raw syscalls are the
// safe choice.
unsafe extern "C" {
    fn dup2(oldfd: i32, newfd: i32) -> i32;
    fn close(fd: i32) -> i32;
}
#[inline]
fn libc_dup2(oldfd: RawFd, newfd: RawFd) -> i32 {
    unsafe { dup2(oldfd, newfd) }
}
#[inline]
fn libc_close(fd: RawFd) -> i32 {
    unsafe { close(fd) }
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
    let package_manifest =
        robonix_cli::manifest::detect_and_load(&pkg_path, entry.manifest.as_deref())
            .with_context(|| format!("load package manifest for {}", pkg_path.display()))?;
    package_manifest.manifest.validate_and_summarize()?;
    let explicit_driver_contract = package_manifest
        .manifest
        .explicit_lifecycle_driver_contract()?;
    let allow_shared_driver_upgrade = explicit_driver_contract.is_some_and(|contract| {
        contract != robonix_cli::manifest::SHARED_LIFECYCLE_DRIVER_CONTRACT
    });
    let expected_driver_contract = Some(
        package_manifest
            .manifest
            .selected_lifecycle_driver_contract()?
            .to_string(),
    );
    let stop = package_manifest.manifest.stop.trim().to_string();
    let stop = if stop.is_empty() { None } else { Some(stop) };
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
        // `rbnx start` must keep its package shell in this group.  Otherwise
        // ProcessManager creates a nested PGID and boot's failure teardown
        // kills only the wrapper, leaving the real package process orphaned.
        .env("RBNX_DEPLOY_MANAGED", "1")
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
        provider_id: None,
        driver_contract: None,
        expected_driver_contract,
        allow_shared_driver_upgrade,
        config_json: None,
        package_dir: Some(pkg_path),
        stop,
    })
}

// ── entry point ─────────────────────────────────────────────────────────

pub async fn execute(
    config: robonix_cli::Config,
    manifest_path: PathBuf,
    log_dir: Option<PathBuf>,
    skip_system: bool,
    no_update_check: bool,
    verbose: bool,
) -> Result<()> {
    output::set_boot_verbose(verbose);
    let manifest_path = manifest_path
        .canonicalize()
        .with_context(|| format!("manifest not found: {}", manifest_path.display()))?;
    let manifest_dir = manifest_path
        .parent()
        .context("manifest has no parent directory")?
        .to_path_buf();

    let raw = std::fs::read_to_string(&manifest_path)
        .with_context(|| format!("failed to read {}", manifest_path.display()))?;
    let root: serde_yaml::Value = serde_yaml::from_str(&raw)
        .with_context(|| format!("failed to parse {}", manifest_path.display()))?;
    let root = prepare_manifest(root, config.robonix_source_path.as_deref())
        .with_context(|| format!("failed to prepare {}", manifest_path.display()))?;
    robonix_cli::manifest::validate_deployment_instance_names(&root).with_context(|| {
        format!(
            "invalid deployment identities in {}",
            manifest_path.display()
        )
    })?;
    let mut deploy: DeployManifest = serde_yaml::from_value(root)
        .with_context(|| format!("failed to decode {}", manifest_path.display()))?;
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
    // Notice (non-fatal) if any cloned remote provider is behind upstream.
    // `--no-update-check` skips the per-package `git fetch` pass entirely.
    if !no_update_check {
        super::check_remotes::report_outdated(&manifest_path);
    }

    // soma owns primitive + skill bring-up (see
    // docs/soma_two_stage_bringup.md). If the manifest declares ANY
    // primitive or skill, we MUST start a soma — otherwise those
    // packages are silently never spawned (boot looks "OK" because rbnx
    // got through atlas/executor/pilot, but the robot stays dead).
    //
    // Two manifest patterns we want to keep working without forcing
    // every existing deploy to add a `system.soma:` block:
    //   1. manifest has primitive/skill, no system.soma at all
    //      → inject a default soma block.
    //   2. manifest has system.soma but didn't set deployments / didn't
    //      set start_packages → fill in sane defaults (deployments =
    //      [this manifest's dir], start_packages = true).
    //
    // Both branches route through `ensure_soma_defaults` so the rest of
    // deploy.rs (spawning the soma binary in the builtin loop, sending
    // the stage 2 trigger after service: bring-up) just sees a
    // populated system.soma entry like any other.
    //
    // Existing operator-supplied values are NEVER overwritten — this
    // hole-fills, it does not override intent.
    //
    // We also run this whenever `system.soma` was explicitly declared,
    // even if there are no primitives/skills that would auto-imply
    // soma. Rationale: `ensure_soma_defaults` no longer just fills in
    // a mapping shell — it also resolves `robot_yaml` / `config`
    // relative paths against manifest_dir. An operator who writes
    // `system.soma: { config: soma_config.local.yaml }` on its own
    // deserves the same path-normalisation as the auto-injected case.
    let soma_declared = deploy.system.contains_key("soma");
    let soma_implied = !deploy.primitive.is_empty() || !deploy.skill.is_empty();
    if (soma_declared || soma_implied) && !skip_system {
        ensure_soma_defaults(&mut deploy.system, &manifest_dir, &manifest_path);
    }

    let log_dir = log_dir.unwrap_or_else(|| manifest_dir.join("rbnx-boot").join("logs"));
    // The CLI prepares and clears this directory before Scribe's first log
    // call. Do not remove files here: Scribe may already hold open handles.
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

    let cache_root = manifest_dir.join("rbnx-boot").join("cache");
    let instances_dir = manifest_dir.join("rbnx-boot").join("instances");
    std::fs::create_dir_all(&instances_dir)
        .with_context(|| format!("failed to create instances dir {}", instances_dir.display()))?;

    let mut children: Vec<Spawned> = Vec::new();
    let state_path = teardown::state_path(&manifest_dir);
    let boot_id = Uuid::new_v4().to_string();
    // Every wrapper and provider inherits this marker. Persisted teardown
    // verifies it against /proc before signalling a PGID, preventing stale
    // state from killing an unrelated process after PID reuse.
    unsafe { std::env::set_var("RBNX_BOOT_ID", &boot_id) };
    let boot_start_time_ticks = robonix_cli::launch::proc_start_time_ticks(std::process::id());
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
    teardown::write_state(
        &state_path,
        &teardown::BootState {
            manifest_path: manifest_path.display().to_string(),
            boot_pid: std::process::id(),
            boot_start_time_ticks,
            boot_id: boot_id.clone(),
            started_at_ms,
            atlas_endpoint: atlas_endpoint.clone(),
            components: Vec::new(),
        },
    )?;
    super::boot_watchdog::spawn(
        &state_path,
        std::process::id(),
        boot_start_time_ticks,
        &boot_id,
    )?;
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
    check_prerequisites(
        &deploy,
        &cache_root,
        &manifest_dir,
        config.robonix_source_path.as_deref(),
    )?;

    // Install the SIGINT/SIGTERM handlers BEFORE bringup begins, not after.
    // Bringup takes many seconds (git, spawns, waiting for ACTIVE); a Ctrl-C
    // in that window used to hit the default disposition and kill rbnx
    // outright, orphaning every child already spawned. Racing the bringup
    // future against these streams lets us tear the partial stack down
    // instead. (SIGKILL can't be trapped — only SIGINT/SIGTERM.) The same
    // streams are reused for the post-boot idle wait further down.
    let mut sigint = signal(SignalKind::interrupt())?;
    let mut sigterm = signal(SignalKind::terminate())?;

    // Owns the parent side of the stage-2 trigger pipe (created inside
    // `spawn_soma_binary`, drained by `write_stage2_trigger`). Declared
    // outside `bringup` so it survives the async-block scope even
    // though we only assign into it from inside.
    let mut soma_stage_writer: Option<std::fs::File> = None;

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
            let soma_listen = deploy
                .system
                .get("soma")
                .and_then(|v| v.as_mapping())
                .and_then(|m| m.get(serde_yaml::Value::String("listen".into())))
                .and_then(|v| v.as_str())
                .map(|s| s.replacen("0.0.0.0", "127.0.0.1", 1));
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
                ("soma", "robonix-soma"),
                ("vitals", "robonix-vitals"),
                ("pilot", "robonix-pilot"),
                ("liaison", "robonix-liaison"),
            ];
            for (name, bin) in bin_map {
                if !deploy.system.contains_key(*name) {
                    continue;
                }
                let mut args =
                    system_cli_args(name, deploy.system.get(*name), atlas_listen.as_deref());
                if *name == "vitals"
                    && !args.iter().any(|arg| arg == "--soma-endpoint")
                    && let Some(endpoint) = soma_listen.as_ref()
                {
                    args.push("--soma-endpoint".into());
                    args.push(endpoint.clone());
                }
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

                let sp = if *name == "soma" {
                    // soma needs an inherited pipe fd for the stage-2
                    // trigger. Everything else uses the plain spawn path.
                    let (sp, writer) = spawn_soma_binary(&log_dir, name, bin, &args).await?;
                    soma_stage_writer = Some(writer);
                    sp
                } else {
                    spawn_system_binary(&log_dir, name, bin, &args).await?
                };
                children.push(sp);
                persist_state(
                    &state_path,
                    &manifest_path,
                    &atlas_endpoint,
                    started_at_ms,
                    &children,
                );
                tokio::time::sleep(std::time::Duration::from_millis(1500)).await;
                if *name == "soma" {
                    if !deploy.primitive.is_empty() {
                        output::boot_section("primitive");
                        if output::boot_verbose() {
                            for entry in &deploy.primitive {
                                output::boot_wait(&entry.name, "waiting for registration");
                            }
                        }
                    }
                    let mut stage1_atlas = AtlasClient::connect_with_retry(
                        &atlas_endpoint,
                        20,
                        Duration::from_millis(500),
                    )
                    .await
                    .with_context(|| {
                        format!("connect to atlas at '{atlas_endpoint}' for primitive readiness")
                    })?;
                    // We just pushed the soma `Spawned` above; grab a
                    // mutable borrow on its Child so the stage-1 waiter
                    // can `try_wait()` per tick. Without this the waiter
                    // sits for the full SOMA_STAGE1_TIMEOUT even when
                    // soma exited immediately (e.g. `missing robot_yaml`
                    // config error). The unwrap is safe: we just pushed.
                    let soma_child = &mut children
                        .last_mut()
                        .expect("soma Spawned pushed above")
                        .child;
                    wait_for_soma_stage1(
                        &mut stage1_atlas,
                        &deploy
                            .primitive
                            .iter()
                            .map(|entry| entry.name.clone())
                            .collect::<Vec<_>>(),
                        soma_child,
                        &log_dir,
                    )
                    .await?;

                    // soma just finished stage 1 (all primitives ACTIVE). The
                    // next thing the operator sees on this terminal is the
                    // remaining builtins (pilot, liaison — whichever come
                    // after soma in `bin_map`) followed by the non-builtin
                    // `system:` entries loop below (memory / scene / speech
                    // / …). Both cohorts are *system services*, NOT
                    // primitives, but without a fresh section header they
                    // visually chain onto the "primitive" header just above
                    // and readers routinely mistake pilot for a primitive
                    // — the exact confusion this header exists to prevent.
                    //
                    // Only emit the header if there's actually something
                    // downstream to label. Concretely: at least one builtin
                    // ordered after soma in `bin_map` is declared in the
                    // manifest, OR the manifest has any non-builtin
                    // `system:` key that will run in the loop after this
                    // for-loop finishes. Otherwise (e.g. an atlas-executor-
                    // soma-only deploy) skip the header — dangling section
                    // titles with nothing under them are worse than none.
                    let builtin_after_soma = bin_map
                        .iter()
                        .skip_while(|(n, _)| *n != "soma")
                        .skip(1) // drop soma itself
                        .any(|(n, _)| deploy.system.contains_key(*n));
                    let has_non_builtin_system = deploy
                        .system
                        .keys()
                        .any(|k| !bin_map.iter().any(|(n, _)| n == k));
                    if builtin_after_soma || has_non_builtin_system {
                        output::boot_section("system");
                    }
                }
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
            let builtin_names: &[&str] =
                &["atlas", "executor", "pilot", "liaison", "soma", "vitals"];
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
                let (manifest_override, runtime_config) =
                    robonix_cli::manifest::split_system_package_config(value)
                        .with_context(|| format!("parse system/{key} package selector"))?;
                let entry = PackageEntry {
                    name: key.clone(),
                    path: Some(pkg_dir.to_string_lossy().into_owned()),
                    url: None,
                    branch: None,
                    config: runtime_config,
                    manifest: manifest_override,
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

        // primitive: handled by soma's stage 1 (kicked off when soma
        //   starts up, finishes before soma declares get_yaml/get_urdf).
        // skill:     handled by soma's stage 2 (kicked off by the
        //   StageTrigger("stage2") we send below, after non-builtin
        //   system services and service: entries have finished
        //   bring-up — which is the earliest skills can safely talk
        //   to executor/pilot/memory/scene from their MCP tools).
        // The deploy.primitive / deploy.skill fields stay in the
        // manifest schema because soma reads them; rbnx just doesn't
        // launch those processes any more.
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

        // All system + service bring-up done; fire soma's stage 2
        // trigger so it spawns + INITs the skill packages. Best
        // effort: a soma that never went ACTIVE (or shut down
        // between our checks and this NotifyProvider) is a deploy
        // problem the operator already sees in earlier boot output;
        // we log and continue rather than tearing everything down
        // for the skills that never started.
        //
        // The section is labelled "skill" (not "stage 2") because
        // that's what the operator actually sees launching under the
        // header — the "stage 2" name is an internal rbnx↔soma pipe
        // protocol detail (see `STAGE2_TRIGGER` in soma/main.rs and
        // `write_stage2_trigger` below). Keeping the wire word out
        // of the terminal UI avoids operators having to learn our
        // two-stage bring-up vocabulary just to read boot output.
        if deploy.system.contains_key("soma") && !skip_system {
            output::boot_section("skill");
            if output::boot_verbose() {
                for entry in &deploy.skill {
                    output::boot_wait(&entry.name, "waiting for registration");
                }
            }
            if let Err(e) = write_stage2_trigger(&mut soma_stage_writer) {
                failures.push((
                    "system".to_string(),
                    "soma".to_string(),
                    format!("start skill packages: {e:#}"),
                ));
            } else if let Err(e) = wait_for_soma_skills(
                &mut atlas,
                &deploy
                    .skill
                    .iter()
                    .map(|entry| entry.name.clone())
                    .collect::<Vec<_>>(),
            )
            .await
            {
                failures.push(("skill".to_string(), "soma".to_string(), format!("{e:#}")));
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
        let complete = teardown::teardown(Some(&atlas_endpoint), &providers, Some(&boot_id)).await;
        if !complete {
            anyhow::bail!(
                "interrupted boot left identity-mismatched process groups; preserving {}",
                state_path.display()
            );
        }
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
            let complete =
                teardown::teardown(Some(&atlas_endpoint), &providers, Some(&boot_id)).await;
            if complete {
                let _ = std::fs::remove_file(&state_path);
            } else {
                return Err(e.context(format!(
                    "cleanup refused identity-mismatched process groups; preserving {}",
                    state_path.display()
                )));
            }
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
    if failures.is_empty() {
        scribe::info("bootstrap", "all components up — waiting for signal");
    } else {
        scribe::info(
            "bootstrap",
            &format!(
                "{} component(s) up, {} package(s) failed — waiting for signal",
                children.len(),
                failures.len()
            ),
        );
    }
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
    let complete = teardown::teardown(Some(&atlas_endpoint), &providers, Some(&boot_id)).await;
    if !complete {
        anyhow::bail!(
            "shutdown refused identity-mismatched process groups; preserving {}",
            state_path.display()
        );
    }
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
        .map(|s| PackageRuntimeRecord {
            name: s.name.clone(),
            kind: s.kind.clone(),
            pid: s.pid,
            pgid: s.pgid,
            provider_id: s.provider_id.clone(),
            driver_contract: s.driver_contract.clone(),
            config_json: s.config_json.clone(),
            package_dir: s.package_dir.as_ref().map(|p| p.display().to_string()),
            stop: s.stop.clone(),
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
        boot_start_time_ticks: robonix_cli::launch::proc_start_time_ticks(std::process::id()),
        boot_id: std::env::var("RBNX_BOOT_ID").unwrap_or_default(),
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
    if trimmed.is_empty()
        || !matches!(
            name,
            "atlas" | "executor" | "pilot" | "liaison" | "soma" | "vitals"
        )
    {
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

/// Translate supported `system:` manifest fields into CLI args for built-in binaries.
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
        "soma" => {
            // v2 flat schema: four keys, four CLI flags. rbnx no longer
            // passes `--rbnx-bin` (soma calls the on-PATH `rbnx`),
            // `--default-robot` / `--deployment` (single robot, deployment
            // path derived from --robot-yaml's parent), or
            // `--start-packages` (soma always spawns primitives + skills;
            // opting out was never actually used). Stage 2 is delivered
            // over an inherited pipe fd (see spawn_soma_binary), not
            // atlas RPC.
            push_pair(&mut out, "--listen", s("listen"));
            push_pair(
                &mut out,
                "--atlas",
                s("atlas_endpoint")
                    .or_else(|| s("atlas"))
                    .or_else(|| atlas_listen.map(str::to_string)),
            );
            push_pair(&mut out, "--provider-id", s("provider_id"));
            push_pair(&mut out, "--robot-yaml", s("robot_yaml"));
            push_pair(&mut out, "--deployment-manifest", s("deployment_manifest"));
            push_pair(&mut out, "--config", s("config"));
            push_pair(&mut out, "--log", s("log"));
        }
        "vitals" => {
            push_pair(&mut out, "--listen", s("listen"));
            push_pair(
                &mut out,
                "--atlas",
                s("atlas").or_else(|| atlas_listen.map(str::to_string)),
            );
            push_pair(&mut out, "--id", s("provider_id").or_else(|| s("id")));
            push_pair(&mut out, "--thresholds-path", s("thresholds_path"));
            push_pair(&mut out, "--soma-endpoint", s("soma_endpoint"));
            push_pair(&mut out, "--config", s("config"));
            push_pair(&mut out, "--log", s("log"));
        }
        _ => {}
    }
    out
}

/// Spawn one package and wait for its provider to register with Atlas.
///
/// The selected shared or explicit legacy Driver is verified before
/// INIT/ACTIVATE and receives the entry's config. Omission and explicit shared
/// selections stay shared-only; only an exact namespace legacy selection may
/// accept an upgraded shared runtime Driver.
async fn spawn_and_init(
    component: &str,
    entry: &PackageEntry,
    spawn_env: &PackageSpawnEnv<'_>,
    atlas: &mut AtlasClient,
) -> Result<Spawned> {
    let before = snapshot_provider_ids(atlas)
        .await
        .with_context(|| format!("[{component}] pre-spawn atlas snapshot"))?;

    let mut sp = spawn_package(component, entry, spawn_env).await?;
    let pkg_label = sp.name.clone();

    // One package = one provider. Atlas may reuse a stable provider id on
    // takeover, so registration_id (not id alone) correlates this spawn.

    // Once the wrapper is up, every error path below must terminate the
    // PGID before bailing — otherwise `?` returns the spawned process to
    // a dead Spawned (which itself has no killing Drop), the caller's
    // teardown loop never sees it (`children.push(sp)` only runs after
    // this fn succeeds), and the orphan keeps holding whatever the
    // package opened (e.g. memsearch's milvus DB lock, executor's gRPC
    // port, …). Give the provider's SIGTERM handler time to run
    // `on_shutdown` before SIGKILL fallback; providers may own ROS children
    // in their own process groups that only the handler knows about.
    let pgid = sp.pgid;

    let registration = match wait_for_registration(
        atlas,
        &before,
        &entry.name,
        &pkg_label,
        component,
        spawn_env.log_dir,
        &mut sp.child,
    )
    .await
    {
        Ok(v) => v,
        Err(e) => {
            terminate_process_group(pgid, Duration::from_secs(8)).await;
            return Err(e);
        }
    };
    let provider_id = registration.provider_id.clone();
    // The exact-id waiter makes this an internal invariant. Keep the check as
    // defense in depth so a future launcher refactor cannot deliver config to
    // a provider other than the manifest instance.
    if provider_id != entry.name {
        let log_file = log_path(spawn_env.log_dir, &pkg_label);
        output::boot_fail(
            short_label(&pkg_label, component),
            &format!(
                "deployment instance propagation failed: expected manifest name='{}', \
                 observed Capability(id='{}'). Log: {}",
                entry.name,
                provider_id,
                log_file.display()
            ),
        );
        terminate_process_group(pgid, Duration::from_secs(8)).await;
        anyhow::bail!(
            "[{component}/{pkg_label}] deployment identity invariant failed: manifest name='{}' vs Capability(id='{}')",
            entry.name,
            provider_id,
        );
    }

    sp.provider_id = Some(provider_id.clone());
    let expected_driver_contract = sp
        .expected_driver_contract
        .as_deref()
        .expect("package spawns always carry a lifecycle selection");
    let driver_contract = match resolve_runtime_driver_contract(
        &provider_id,
        &registration.provider_namespace,
        expected_driver_contract,
        &registration.driver_contracts,
        sp.allow_shared_driver_upgrade,
    ) {
        Ok(contract) => contract,
        Err(error) => {
            let log_file = log_path(spawn_env.log_dir, &pkg_label);
            output::boot_fail(
                short_label(&pkg_label, component),
                &format!("{error}; log {}", log_file.display()),
            );
            terminate_process_group(pgid, Duration::from_secs(8)).await;
            return Err(error).with_context(|| format!("[{component}/{pkg_label}] lifecycle"));
        }
    };

    if driver_contract != expected_driver_contract {
        output::warning(&format!(
            "provider '{provider_id}' publishes shared lifecycle Driver '{driver_contract}' for legacy manifest selection '{expected_driver_contract}'; remove the legacy Driver declaration to finish migration"
        ));
    }

    let config_json = serde_json::to_string(&entry.config).with_context(|| {
        format!(
            "[{component}/{pkg_label}] serialize config for deployment instance '{}'",
            entry.name
        )
    })?;
    sp.driver_contract = Some(driver_contract.clone());
    sp.config_json = Some(config_json.clone());

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
            terminate_process_group(pgid, Duration::from_secs(8)).await;
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
            terminate_process_group(pgid, Duration::from_secs(8)).await;
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
    if output::boot_verbose() {
        output::boot_wait(label, msg_prefix);
        return fut.await;
    }
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

async fn wait_for_soma_stage1(
    atlas: &mut AtlasClient,
    primitive_names: &[String],
    soma_child: &mut Child,
    log_dir: &Path,
) -> Result<()> {
    const SPINNER_TICK: Duration = Duration::from_millis(100);
    const POLLS_PER_TICK: usize = 5; // poll atlas every 500 ms
    const SOMA_STAGE1_TIMEOUT: Duration = Duration::from_secs(180);
    const SOMA_GET_YAML_CONTRACT: &str = "robonix/system/soma/get_yaml";

    let started = Instant::now();
    let deadline = started + SOMA_STAGE1_TIMEOUT;
    let mut frame: usize = 0;
    let mut observed_states: HashMap<String, i32> = HashMap::new();
    let mut active_primitives: HashSet<String> = HashSet::new();
    let mut reported_failures: HashSet<String> = HashSet::new();
    if output::boot_verbose() {
        output::boot_wait("primitive", "waiting for Soma-managed providers");
    }
    loop {
        let elapsed_s = started.elapsed().as_secs_f32();
        let detail = if primitive_names.is_empty() {
            format!("waiting for Soma gRPC readiness… {elapsed_s:>4.1}s")
        } else {
            format!(
                "starting {} primitive package(s)… {elapsed_s:>4.1}s",
                primitive_names.len()
            )
        };
        if output::boot_verbose() {
            if frame > 0 && frame.is_multiple_of(50) {
                output::boot_note("primitive", &detail);
            }
        } else {
            output::boot_progress("primitive", &detail, frame);
        }
        // Check every tick whether soma is still alive. If it exited
        // (typically: `missing robot_yaml`, `read Soma config`, port
        // bind failure), surface that immediately with the tail of
        // its own log rather than sitting on this spinner for the
        // full SOMA_STAGE1_TIMEOUT (180s) which frustrates operators
        // and blocks CI. try_wait is non-blocking; Ok(Some(_)) means
        // the child has been reaped and the OS-level status is known.
        if let Ok(Some(status)) = soma_child.try_wait() {
            let mut provider_failures = Vec::new();
            for name in primitive_names {
                let log_file = log_dir.join(format!("{name}.log"));
                if let Some(cause) = read_provider_failure(&log_file) {
                    if reported_failures.insert(name.clone()) {
                        output::boot_fail(
                            name,
                            &format!("ERROR; {cause}; log {}", log_file.display()),
                        );
                    }
                    provider_failures.push((name, cause, log_file));
                }
            }
            if !provider_failures.is_empty() {
                let names = provider_failures
                    .iter()
                    .map(|(name, _, _)| name.as_str())
                    .collect::<Vec<_>>()
                    .join(", ");
                let logs = provider_failures
                    .iter()
                    .map(|(_, _, path)| path.display().to_string())
                    .collect::<Vec<_>>()
                    .join(", ");
                output::boot_fail(
                    "primitive",
                    &format!("soma exited after provider failure(s): {names}"),
                );
                anyhow::bail!(
                    "Soma exited with {status:?} after provider failure(s): {names}; logs: {logs}"
                );
            }

            let log_file = log_dir.join("soma.log");
            let tail = read_log_tail(&log_file, 20);
            output::boot_fail(
                "primitive",
                &format!(
                    "soma exited before becoming ACTIVE (status={status:?}); see {}",
                    log_file.display()
                ),
            );
            let hint = if tail.is_empty() {
                String::new()
            } else {
                format!("\n--- soma.log tail ---\n{tail}\n--- end ---")
            };
            anyhow::bail!(
                "soma exited with {status:?} before primitive readiness; \
                 log: {}{hint}",
                log_file.display()
            );
        }
        if frame.is_multiple_of(POLLS_PER_TICK) {
            for name in primitive_names {
                let providers = atlas
                    .query_capabilities(name, "", atlas_pb::Transport::Unspecified)
                    .await
                    .with_context(|| format!("poll primitive '{name}' during Soma bring-up"))?;
                let Some(provider) = providers.into_iter().find(|provider| provider.id == *name)
                else {
                    continue;
                };
                let previous = observed_states.insert(name.clone(), provider.state);
                if previous != Some(provider.state) {
                    let state = lifecycle_state_label(provider.state);
                    if provider.state == atlas_pb::LifecycleState::StateActive as i32 {
                        output::boot_ok(name, "ACTIVE");
                        active_primitives.insert(name.clone());
                    } else if provider.state == atlas_pb::LifecycleState::StateError as i32 {
                        let log_file = log_dir.join(format!("{name}.log"));
                        let detail = read_provider_failure(&log_file).map_or_else(
                            || format!("ERROR; log {}", log_file.display()),
                            |cause| format!("ERROR; {cause}; log {}", log_file.display()),
                        );
                        output::boot_fail(name, &detail);
                        reported_failures.insert(name.clone());
                    } else if output::boot_verbose() {
                        output::boot_note(name, state);
                    }
                }
            }
            let providers = atlas
                .query_capabilities("soma", SOMA_GET_YAML_CONTRACT, atlas_pb::Transport::Grpc)
                .await
                .context("wait for Soma primitive readiness")?;
            if let Some(soma) = providers.into_iter().find(|p| p.id == "soma")
                && soma.state == atlas_pb::LifecycleState::StateActive as i32
                && soma_grpc_ready(atlas, SOMA_GET_YAML_CONTRACT).await
            {
                for name in primitive_names {
                    if !active_primitives.contains(name) {
                        output::boot_ok(name, "ACTIVE");
                    }
                }
                return Ok(());
            }
        }
        if Instant::now() >= deadline {
            output::boot_fail(
                "primitive",
                &format!(
                    "timeout after {:?}; service bring-up needs primitives ACTIVE first",
                    SOMA_STAGE1_TIMEOUT
                ),
            );
            anyhow::bail!(
                "Soma primitive bring-up did not become ready within {:?}; refusing to start service packages before primitives are ready",
                SOMA_STAGE1_TIMEOUT
            );
        }
        tokio::time::sleep(SPINNER_TICK).await;
        frame = frame.wrapping_add(1);
    }
}

fn lifecycle_state_label(state: i32) -> &'static str {
    if state == atlas_pb::LifecycleState::StateRegistered as i32 {
        "REGISTERED"
    } else if state == atlas_pb::LifecycleState::StateInactive as i32 {
        "INACTIVE"
    } else if state == atlas_pb::LifecycleState::StateActive as i32 {
        "ACTIVE"
    } else if state == atlas_pb::LifecycleState::StateError as i32 {
        "ERROR"
    } else if state == atlas_pb::LifecycleState::StateTerminated as i32 {
        "TERMINATED"
    } else {
        "STARTING"
    }
}

async fn wait_for_soma_skills(atlas: &mut AtlasClient, skill_names: &[String]) -> Result<()> {
    const TIMEOUT: Duration = Duration::from_secs(180);
    if skill_names.is_empty() {
        return Ok(());
    }
    let deadline = Instant::now() + TIMEOUT;
    let mut observed_states: HashMap<String, i32> = HashMap::new();
    let mut ready: HashSet<String> = HashSet::new();
    while Instant::now() < deadline {
        for name in skill_names {
            let providers = atlas
                .query_capabilities(name, "", atlas_pb::Transport::Unspecified)
                .await
                .with_context(|| format!("poll skill '{name}' during soma bring-up"))?;
            let Some(provider) = providers.into_iter().find(|provider| provider.id == *name) else {
                continue;
            };
            if observed_states.insert(name.clone(), provider.state) != Some(provider.state) {
                let state = lifecycle_state_label(provider.state);
                if provider.state == atlas_pb::LifecycleState::StateInactive as i32
                    || provider.state == atlas_pb::LifecycleState::StateActive as i32
                {
                    output::boot_ok(name, state);
                    ready.insert(name.clone());
                } else if provider.state == atlas_pb::LifecycleState::StateError as i32 {
                    output::boot_fail(name, "ERROR; see soma.log and provider log");
                    anyhow::bail!("skill '{name}' entered ERROR during Soma bring-up");
                } else if output::boot_verbose() {
                    output::boot_note(name, state);
                }
            }
        }
        if ready.len() == skill_names.len() {
            return Ok(());
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    let pending = skill_names
        .iter()
        .filter(|name| !ready.contains(*name))
        .cloned()
        .collect::<Vec<_>>();
    for name in &pending {
        output::boot_fail(
            name,
            "registration/INIT timeout; see soma.log and provider log",
        );
    }
    anyhow::bail!(
        "Soma skill bring-up timed out after {TIMEOUT:?}: {}",
        pending.join(", ")
    )
}

/// Read the last `max_lines` lines of a file for embedding into an
/// error message. Best-effort: an unreadable/missing file returns an
/// empty string rather than an error — the caller already reports the
/// path, we just enrich when we can. We read the whole file (soma.log
/// is scribe-managed and stays small during boot), split, and take
/// the tail — no seek-from-end acrobatics needed for the boot-time
/// use case.
fn read_log_tail(path: &Path, max_lines: usize) -> String {
    let Ok(contents) = std::fs::read_to_string(path) else {
        return String::new();
    };
    let lines: Vec<&str> = contents.lines().collect();
    let start = lines.len().saturating_sub(max_lines);
    lines[start..].join("\n")
}

/// Return the provider's actual lifecycle failure rather than whichever
/// shutdown record happened to be written last. Scribe records are JSONL;
/// providers commonly report lifecycle transitions at info level, so prefer
/// `-> ERROR (...)` messages before falling back to an error-level record.
fn read_provider_failure(path: &Path) -> Option<String> {
    let contents = std::fs::read_to_string(path).ok()?;
    let mut error_level_fallback = None;
    for line in contents.lines().rev() {
        let Ok(record) = serde_json::from_str::<serde_json::Value>(line) else {
            continue;
        };
        let Some(message) = record.get("msg").and_then(|value| value.as_str()) else {
            continue;
        };
        if let Some((_, cause)) = message.split_once(" -> ERROR (") {
            return Some(cause.strip_suffix(')').unwrap_or(cause).to_string());
        }
        if error_level_fallback.is_none()
            && record.get("level").and_then(|value| value.as_str()) == Some("error")
        {
            error_level_fallback = Some(message.to_string());
        }
    }
    error_level_fallback
}

/// Summarize a package that exited before Atlas registration. Providers often
/// forward Python tracebacks through Scribe at info level, so the lifecycle-
/// specific parser above may intentionally return None. In that case the last
/// structured message is the most useful single-line cause for boot output.
fn read_provider_exit_summary(path: &Path) -> Option<String> {
    if let Some(cause) = read_provider_failure(path) {
        return Some(cause);
    }
    let contents = std::fs::read_to_string(path).ok()?;
    for line in contents.lines().rev() {
        if let Ok(record) = serde_json::from_str::<serde_json::Value>(line)
            && let Some(message) = record.get("msg").and_then(|value| value.as_str())
            && !message.trim().is_empty()
        {
            return Some(message.trim().to_string());
        }
        if !line.trim().is_empty() {
            return Some(line.trim().to_string());
        }
    }
    None
}

async fn soma_grpc_ready(atlas: &mut AtlasClient, contract_id: &str) -> bool {
    let Ok((channel_id, endpoint, _params)) = atlas
        .connect_capability(
            DEPLOY_CONSUMER_ID,
            "soma",
            contract_id,
            atlas_pb::Transport::Grpc,
        )
        .await
    else {
        return false;
    };
    let normalized = if endpoint.starts_with("http") {
        endpoint
    } else {
        format!("http://{endpoint}")
    };
    let ready = match Endpoint::new(normalized.clone()) {
        Ok(endpoint) => tokio::time::timeout(Duration::from_secs(1), endpoint.connect())
            .await
            .is_ok_and(|r| r.is_ok()),
        Err(_) => false,
    };
    let _ = atlas.disconnect_capability(&channel_id).await;
    ready
}

/// Write the `stage2\n` trigger into the pipe rbnx and soma share
/// (see `spawn_soma_binary`). This is a one-shot: soma reads the
/// line, unblocks its skill-package launcher, and closes its read
/// end. rbnx-side we drop the writer here — no reason to hold it
/// open, and closing gives soma an immediate EOF on the (very
/// unlikely) chance it re-reads.
///
/// If we don't have a writer (soma wasn't spawned by us, e.g.
/// --skip-system), this is a no-op with a warning: someone else
/// owns soma's fd and there's nothing rbnx can meaningfully do.
fn write_stage2_trigger(writer: &mut Option<std::fs::File>) -> Result<()> {
    use std::io::Write;
    let Some(mut w) = writer.take() else {
        output::boot_skip(
            "skill",
            "start skipped: no trigger writer (Soma was not spawned by this rbnx)",
        );
        return Ok(());
    };
    w.write_all(b"stage2\n")
        .context("write 'stage2' to soma stage-trigger pipe")?;
    w.flush().context("flush soma stage-trigger pipe")?;
    // "written", not "delivered": all we know at this point is that
    // the bytes hit the pipe. Actual delivery (soma reads the line,
    // spawns skills, and their MCP tools/caps register) is verified
    // downstream by the boot-poll cap-wait loop, not here.
    Ok(())
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
/// `provider_id` plus every distinct lifecycle Driver observed after the
/// declaration settle window. The caller verifies this list before sending
/// config or lifecycle commands.
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
    before: &ProviderRegistrationSnapshot,
    expected_provider_id: &str,
    pkg_label: &str,
    component: &str,
    log_dir: &Path,
    child: &mut Child,
) -> Result<RegistrationOutcome> {
    if before.contains_key(expected_provider_id) {
        anyhow::bail!(
            "[{component}/{pkg_label}] deployment instance '{expected_provider_id}' \
             was already registered before spawn"
        );
    }

    // Wait for this manifest instance's exact id with a fresh registration
    // generation. Unrelated providers can register concurrently and must not
    // receive this instance's lifecycle config.
    const SPINNER_TICK: Duration = Duration::from_millis(100);
    const POLLS_PER_TICK: u32 = 2; // poll atlas every 200 ms
    let started = Instant::now();
    let deadline = started + DRIVER_REGISTER_TIMEOUT;
    let mut frame: usize = 0;
    let display_label = short_label(pkg_label, component);
    if output::boot_verbose() {
        output::boot_wait(display_label, "registering with atlas");
    }
    loop {
        let elapsed_s = started.elapsed().as_secs_f32();
        let detail = format!("registering with atlas… {elapsed_s:>4.1}s");
        if output::boot_verbose() {
            if frame > 0 && frame.is_multiple_of(50) {
                output::boot_note(display_label, &detail);
            }
        } else {
            output::boot_progress(display_label, &detail, frame);
        }
        // A package wrapper that exits before registering can never recover.
        // Detect it on every spinner tick instead of waiting out the full
        // registration timeout and then continuing with a misleading generic
        // timeout. The caller still terminates the package PGID so any child
        // processes left behind by a failed start hook are reaped.
        match child.try_wait() {
            Ok(Some(status)) => {
                let log_file = log_path(log_dir, pkg_label);
                let cause = read_provider_exit_summary(&log_file)
                    .unwrap_or_else(|| "no diagnostic message in provider log".to_string());
                output::boot_fail(
                    display_label,
                    &format!(
                        "start process exited ({status}); {cause}; log {}",
                        log_file.display()
                    ),
                );
                anyhow::bail!(
                    "[{component}/{pkg_label}] start process exited ({status}) before Atlas registration: {cause}. Log: {}",
                    log_file.display()
                );
            }
            Ok(None) => {}
            Err(error) => {
                anyhow::bail!(
                    "[{component}/{pkg_label}] inspect start process while waiting for Atlas registration: {error}"
                );
            }
        }
        if frame.is_multiple_of(POLLS_PER_TICK as usize) {
            let providers = atlas
                .query_capabilities("", "", atlas_pb::Transport::Unspecified)
                .await
                .with_context(|| format!("[{component}/{pkg_label}] poll atlas"))?;
            let matched = providers.iter().find(|provider| {
                robonix_cli::launch::is_expected_provider_registration(
                    provider,
                    before,
                    expected_provider_id,
                )
            });
            if let Some(first) = matched {
                let provider_id = first.id.clone();
                let registration_id = first.registration_id.clone();
                // RegisterPrimitive/Service/Skill and DeclareCapability are
                // two separate RPCs from the package side — Register lands
                // first, declares follow within a few hundred ms. Give it
                // up to a 1 s settle window so we don't false-fire a missing
                // Driver error on a fast poll. Capped by the outer
                // `deadline` so we never exceed user-facing timeout.
                let settle_until = Instant::now()
                    .checked_add(Duration::from_millis(1000))
                    .map(|t| t.min(deadline))
                    .unwrap_or(deadline);
                let mut current: atlas_pb::CapabilityProvider = (*first).clone();
                // Consume the complete settle window so a package that
                // declares both shared and legacy Drivers cannot hide the
                // second declaration behind the first successful poll.
                loop {
                    if Instant::now() >= settle_until {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                    let providers = atlas
                        .query_capabilities(&provider_id, "", atlas_pb::Transport::Unspecified)
                        .await
                        .with_context(|| format!("[{component}/{pkg_label}] re-poll for driver"))?;
                    match providers.into_iter().find(|p| p.id == provider_id) {
                        Some(p) if p.registration_id == registration_id => current = p,
                        Some(p) => {
                            let log_file = log_path(log_dir, pkg_label);
                            output::boot_fail(
                                display_label,
                                &format!(
                                    "provider '{provider_id}' registration changed during settle — see {}",
                                    log_file.display(),
                                ),
                            );
                            anyhow::bail!(
                                "[{component}/{pkg_label}] provider '{provider_id}' registration changed during settle ('{registration_id}' -> '{}'). Log: {}",
                                p.registration_id,
                                log_file.display(),
                            );
                        }
                        None => {
                            // Provider vanished between the original match
                            // and now (crashed mid-settle, atlas evicted,
                            // heartbeat lapsed). Report loudly so downstream
                            // boot logic cannot march on against a dead process.
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
                }
                let mut driver_contracts = current
                    .capabilities
                    .iter()
                    .filter(|capability| {
                        capability.transport == atlas_pb::Transport::Grpc as i32
                            && capability.contract_id.ends_with("/driver")
                    })
                    .map(|capability| capability.contract_id.clone())
                    .collect::<Vec<_>>();
                driver_contracts.sort();
                driver_contracts.dedup();
                return Ok(RegistrationOutcome {
                    provider_id,
                    provider_kind: current.kind,
                    provider_namespace: current.namespace,
                    registration_id: current.registration_id,
                    driver_contracts,
                });
            }
        }
        if Instant::now() >= deadline {
            let log_file = log_path(log_dir, pkg_label);
            output::boot_fail(
                display_label,
                &format!(
                    "registration timeout after {:?}; expected instance '{}' — see {}",
                    DRIVER_REGISTER_TIMEOUT,
                    expected_provider_id,
                    log_file.display()
                ),
            );
            anyhow::bail!(
                "[{component}/{pkg_label}] timed out after {:?} — package never registered expected deployment instance '{}' with atlas. Log: {}",
                DRIVER_REGISTER_TIMEOUT,
                expected_provider_id,
                log_file.display()
            );
        }
        tokio::time::sleep(SPINNER_TICK).await;
        frame = frame.wrapping_add(1);
    }
}
