// SPDX-License-Identifier: MulanPSL-2.0
// Deploy command: start the full stack from robonix_manifest.yaml
//
// Flow:
//   1. Load runtime config (robonix_manifest.yaml)
//   2. Start system components (atlas, executor, pilot, liaison)
//   3. Resolve & start primitives
//   4. Resolve & start services
//   5. Register available skills (on-demand, not started)
//   6. Wait for Ctrl+C, then shut down all children

use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::workspace::{self, RuntimePackageEntry, SystemConfig};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Duration;
use tokio::process::{Child, Command};

pub async fn execute(config_path: &Path) -> Result<()> {
    let config_path = if config_path.is_absolute() {
        config_path.to_path_buf()
    } else {
        std::env::current_dir()?.join(config_path)
    };
    if !config_path.exists() {
        anyhow::bail!("{} not found", config_path.display());
    }

    let config = workspace::load_runtime_config(&config_path)?;
    let project_root = config_path
        .parent()
        .unwrap_or(Path::new("."))
        .to_path_buf();

    output::action("Deploy", &format!("'{}' from {}", config.name, config_path.display()));

    // Build global env: inherit process env + overlay config env.
    let mut global_env: HashMap<String, String> = std::env::vars().collect();
    for (k, v) in &config.env {
        global_env.insert(k.clone(), v.clone());
    }

    let mut children: Vec<(String, Child)> = Vec::new();

    // ── Pre-check: has the project been built? ─────────────────────
    let stamp_file = project_root.join(workspace::BUILD_STAMP_DIR).join(".built");
    if !stamp_file.exists() {
        output::warning(
            "no build stamp found — packages may not have been built yet. \
             Consider running 'rbnx build --all' first.",
        );
    }

    // ── Step 1: Start system core components ───────────────────────
    output::action("Step 1/4", "Starting system components");
    start_system(&config.system, &global_env, &mut children).await?;

    // ── Step 2: Resolve & start primitives ─────────────────────────
    output::action("Step 2/4", &format!("Starting {} primitive(s)", config.primitives.len()));
    let sorted = resolve_and_sort_entries(&project_root, "primitives", &config.primitives)?;
    for (pkg_path, entry) in &sorted {
        start_package(pkg_path, entry, &global_env, &mut children).await?;
    }

    // ── Step 3: Resolve & start services ───────────────────────────
    output::action("Step 3/4", &format!("Starting {} service(s)", config.services.len()));
    let sorted = resolve_and_sort_entries(&project_root, "services", &config.services)?;
    for (pkg_path, entry) in &sorted {
        start_package(pkg_path, entry, &global_env, &mut children).await?;
    }

    // ── Step 4: Register skills (on-demand, not started) ───────────
    output::action("Step 4/4", &format!("Registering {} skill(s)", config.skills.len()));
    for entry in &config.skills {
        match workspace::resolve_package_path(&project_root, "skills", entry) {
            Ok(pkg_path) => {
                output::sub_step(&format!(
                    "skill '{}' available at {}",
                    entry.name,
                    pkg_path.display()
                ));
            }
            Err(e) => {
                output::warning(&format!("skill '{}': {}", entry.name, e));
            }
        }
    }

    // ── Summary ────────────────────────────────────────────────────
    output::success(&format!(
        "Deploy complete — {} component(s) running. Press Ctrl+C to shut down.",
        children.len()
    ));

    // Wait for Ctrl+C then gracefully shut down.
    tokio::signal::ctrl_c().await?;
    output::action("Shutdown", "stopping all components...");
    shutdown_children(&mut children).await;
    output::success("All components stopped.");

    Ok(())
}

// ════════════════════════════════════════════════════════════════════
// System startup
// ════════════════════════════════════════════════════════════════════

/// Default endpoints for system components (used when not specified in config).
const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
const DEFAULT_EXECUTOR_ENDPOINT: &str = "127.0.0.1:50061";
const DEFAULT_PILOT_ENDPOINT: &str = "127.0.0.1:50071";
const DEFAULT_LIAISON_ENDPOINT: &str = "127.0.0.1:50081";

async fn start_system(
    system_cfg: &SystemConfig,
    env: &HashMap<String, String>,
    children: &mut Vec<(String, Child)>,
) -> Result<()> {
    let rust_root = detect_rust_root(env)?;
    let cargo_toml = rust_root.join("Cargo.toml");

    // Build the list of components to start from the config.
    // Each component is started only if declared (even as `{}`) in the config.
    struct SystemComponent<'a> {
        name: &'static str,
        crate_name: &'static str,
        endpoint: &'a str,
        enabled: bool,
    }

    let atlas_ep = system_cfg
        .atlas
        .as_ref()
        .and_then(|c| c.endpoint.as_deref())
        .unwrap_or(DEFAULT_ATLAS_ENDPOINT);
    let executor_ep = system_cfg
        .executor
        .as_ref()
        .and_then(|c| c.endpoint.as_deref())
        .unwrap_or(DEFAULT_EXECUTOR_ENDPOINT);
    let pilot_ep = system_cfg
        .pilot
        .as_ref()
        .and_then(|c| c.endpoint.as_deref())
        .unwrap_or(DEFAULT_PILOT_ENDPOINT);
    let liaison_ep = system_cfg
        .liaison
        .as_ref()
        .and_then(|c| c.endpoint.as_deref())
        .unwrap_or(DEFAULT_LIAISON_ENDPOINT);

    let components = [
        SystemComponent {
            name: "atlas",
            crate_name: "robonix-atlas",
            endpoint: atlas_ep,
            enabled: system_cfg.atlas.is_some(),
        },
        SystemComponent {
            name: "executor",
            crate_name: "robonix-executor",
            endpoint: executor_ep,
            enabled: system_cfg.executor.is_some(),
        },
        SystemComponent {
            name: "pilot",
            crate_name: "robonix-pilot",
            endpoint: pilot_ep,
            enabled: system_cfg.pilot.is_some(),
        },
        SystemComponent {
            name: "liaison",
            crate_name: "robonix-liaison",
            endpoint: liaison_ep,
            enabled: system_cfg.liaison.is_some(),
        },
    ];

    for comp in &components {
        if !comp.enabled {
            output::sub_step(&format!("{} not declared in config, skipping", comp.name));
            continue;
        }

        output::step("Starting", &format!("{} (system)", comp.name));

        if check_port_reachable(comp.endpoint).await {
            output::check(&format!("{} already running at {}", comp.name, comp.endpoint));
            continue;
        }

        let mut cmd_env = env.clone();
        // Inject the endpoint so the component knows where to bind.
        cmd_env.insert(
            format!("ROBONIX_{}_ENDPOINT", comp.name.to_uppercase()),
            format!("http://{}", comp.endpoint),
        );
        // Atlas uses ROBONIX_ATLAS as host:port (no http://).
        if comp.name == "atlas" {
            cmd_env.insert("ROBONIX_ATLAS".to_string(), comp.endpoint.to_string());
            cmd_env.insert("ROBONIX_META_GRPC_ENDPOINT".to_string(), comp.endpoint.to_string());
        }

        output::sub_step(&format!(
            "exec: cargo run --manifest-path {} -p {}",
            cargo_toml.display(),
            comp.crate_name
        ));

        let child = Command::new("cargo")
            .args(["run", "--manifest-path"])
            .arg(&cargo_toml)
            .args(["-p", comp.crate_name])
            .envs(cmd_env.iter())
            .kill_on_drop(true)
            .spawn()
            .with_context(|| format!("failed to spawn {}", comp.crate_name))?;

        children.push((comp.name.to_string(), child));

        output::sub_step(&format!("waiting for {} ...", comp.endpoint));
        if wait_for_endpoint(comp.endpoint, Duration::from_secs(5)).await {
            output::check(&format!("{} ready at {}", comp.name, comp.endpoint));
        } else {
            output::warning(&format!(
                "{} started but {} not reachable after 5 s — continuing",
                comp.name, comp.endpoint
            ));
        }
    }
    Ok(())
}

/// Detect the `rust/` workspace root from `robonix_source_path` config or env.
fn detect_rust_root(env: &HashMap<String, String>) -> Result<PathBuf> {
    // Try the CLI config's source path first.
    if let Ok(cfg) = robonix_cli::Config::load() {
        if let Ok(p) = cfg.resolve_source_path(robonix_cli::SourcePathKey::RustRoot) {
            return Ok(p);
        }
    }
    // Fallback: RUST_ROOT env or walk up from cwd.
    if let Some(v) = env.get("RUST_ROOT") {
        let p = PathBuf::from(v);
        if p.join("Cargo.toml").exists() {
            return Ok(p);
        }
    }
    // Walk up from cwd looking for rust/Cargo.toml containing [workspace].
    let mut dir = std::env::current_dir()?;
    for _ in 0..10 {
        let candidate = dir.join("rust");
        if candidate.join("Cargo.toml").exists() {
            return Ok(candidate);
        }
        if !dir.pop() {
            break;
        }
    }
    anyhow::bail!(
        "cannot detect Rust workspace root. Run `rbnx setup` from the robonix repo, \
         or set ROBONIX_RUST_ROOT."
    )
}

// ════════════════════════════════════════════════════════════════════
// Package resolution & startup
// ════════════════════════════════════════════════════════════════════

/// Resolve paths and topologically sort entries within one layer (primitives/services/skills)
/// based on the `depends` field in each package's `package_manifest.yaml`.
fn resolve_and_sort_entries<'a>(
    project_root: &Path,
    role_dir: &str,
    entries: &'a [RuntimePackageEntry],
) -> Result<Vec<(PathBuf, &'a RuntimePackageEntry)>> {
    if entries.is_empty() {
        return Ok(Vec::new());
    }

    // Phase 1: resolve paths and load manifests to read depends.
    struct Resolved<'b> {
        pkg_path: PathBuf,
        entry: &'b RuntimePackageEntry,
        depends: Vec<String>,
    }
    let mut resolved: Vec<Resolved<'a>> = Vec::with_capacity(entries.len());
    for entry in entries {
        let pkg_path = workspace::resolve_package_path(project_root, role_dir, entry)?;
        let detected = manifest::detect_and_load(&pkg_path)?;
        let depends: Vec<String> = detected
            .manifest
            .depends
            .iter()
            .map(|d| d.name.clone())
            .collect();
        resolved.push(Resolved {
            pkg_path,
            entry,
            depends,
        });
    }

    // Phase 2: topo sort using package name as key.
    let topo_input: Vec<(&str, &[String])> = resolved
        .iter()
        .map(|r| (r.entry.package.as_str(), r.depends.as_slice()))
        .collect();
    let order = workspace::topo_sort(&topo_input)?;

    if order.len() > 1 {
        let names: Vec<&str> = order.iter().map(|&i| resolved[i].entry.name.as_str()).collect();
        output::sub_step(&format!("launch order: {}", names.join(" → ")));
    }

    // Phase 3: return sorted vec.
    Ok(order
        .into_iter()
        .map(|i| {
            let r = &resolved[i];
            (r.pkg_path.clone(), r.entry)
        })
        .collect())
}

/// Load the package manifest and start in background using top-level `start` command.
async fn start_package(
    pkg_path: &Path,
    entry: &RuntimePackageEntry,
    env: &HashMap<String, String>,
    children: &mut Vec<(String, Child)>,
) -> Result<()> {
    let detected = manifest::detect_and_load(pkg_path)?;
    let m = &detected.manifest;

    // Inject entry-level config as env vars (RBNX_CFG_<KEY>=<VALUE>).
    let mut pkg_env = env.clone();
    for (k, v) in &entry.config {
        let env_key = format!("RBNX_CFG_{}", k.to_uppercase());
        let env_val = match v {
            serde_yaml::Value::String(s) => s.clone(),
            other => serde_yaml::to_string(other).unwrap_or_default().trim().to_string(),
        };
        pkg_env.insert(env_key, env_val);
    }
    pkg_env.insert(
        "RBNX_PACKAGE_ROOT".to_string(),
        pkg_path.display().to_string(),
    );

    let start_cmd = m.start.trim();
    if start_cmd.is_empty() {
        output::warning(&format!("{} has empty start command, skipping", entry.name));
        return Ok(());
    }

    output::step("Starting", &format!("{}", entry.name));

    // Source setup.bash if present (generated by `rbnx codegen`).
    let setup_bash = pkg_path
        .join("rbnx-build")
        .join("ws")
        .join("install")
        .join("setup.bash");

    let full_cmd = if setup_bash.exists() {
        format!("source {} && {}", setup_bash.display(), start_cmd)
    } else {
        start_cmd.to_string()
    };

    let child = Command::new("bash")
        .args(["-c", &full_cmd])
        .current_dir(pkg_path)
        .envs(pkg_env.iter())
        .kill_on_drop(true)
        .spawn()
        .with_context(|| format!("failed to spawn {}", entry.name))?;

    output::sub_step(&format!("  {} (PID {})", entry.name, child.id().unwrap_or(0)));
    children.push((entry.name.clone(), child));

    tokio::time::sleep(Duration::from_secs(1)).await;

    Ok(())
}

// ════════════════════════════════════════════════════════════════════
// Network helpers
// ════════════════════════════════════════════════════════════════════

/// Try to connect to `addr` (host:port) once. Returns true if successful.
async fn check_port_reachable(addr: &str) -> bool {
    tokio::net::TcpStream::connect(addr)
        .await
        .is_ok()
}

/// Poll `addr` until it becomes reachable or `timeout` elapses.
async fn wait_for_endpoint(addr: &str, timeout: Duration) -> bool {
    let start = tokio::time::Instant::now();
    let poll_interval = Duration::from_millis(500);
    loop {
        if check_port_reachable(addr).await {
            return true;
        }
        if start.elapsed() >= timeout {
            return false;
        }
        tokio::time::sleep(poll_interval).await;
    }
}

// ════════════════════════════════════════════════════════════════════
// Shutdown
// ════════════════════════════════════════════════════════════════════

/// Send SIGTERM (unix) / kill (windows) to all children, then wait.
async fn shutdown_children(children: &mut Vec<(String, Child)>) {
    for (label, child) in children.iter_mut().rev() {
        let pid = child.id().unwrap_or(0);
        output::sub_step(&format!("stopping {} (PID {})...", label, pid));

        // Try to terminate the process gracefully.
        let _ = child.kill().await;
    }

    // Give processes time to exit gracefully.
    tokio::time::sleep(Duration::from_secs(2)).await;

    // Force-kill any remaining.
    for (label, child) in children.iter_mut() {
        match child.try_wait() {
            Ok(Some(_)) => {} // already exited
            _ => {
                output::sub_step(&format!("force-killing {}...", label));
                let _ = child.kill().await;
            }
        }
    }
}
