// SPDX-License-Identifier: MulanPSL-2.0
// Deploy command: read a config.yaml and bring up the full stack
// (runtime components, services, and packages) in dependency order.
//
// The config.yaml may reference an `upstream_config` (e.g. robonix_workspace.yaml)
// which provides runtime components, system services, and global env.
// The deploy command merges both configs, then:
//   Phase 1 — start runtime components (from upstream)
//   Phase 2 — start system services (from upstream)
//   Phase 3 — start packages (from local config), reading each package's
//             `robonix_manifest.yaml` to discover the actual start command.

use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use serde::Deserialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use tokio::process::Command;
use tokio::time::{Duration, sleep};

// ── YAML schema ─────────────────────────────────────────────────────────────

/// Top-level config.yaml (the user's project-level config).
#[derive(Debug, Deserialize)]
struct DeployConfig {
    /// Path to upstream workspace config (e.g. "robonix_workspace.yaml"),
    /// resolved relative to this config file's directory.
    #[serde(default)]
    upstream_config: Option<String>,
    /// Target platform hint (e.g. "jetson-orin-agx", "sim").
    #[serde(default)]
    target: Option<String>,
    /// Global environment variables exported to every child process.
    #[serde(default)]
    env: HashMap<String, String>,
    /// Packages to launch.
    #[serde(default)]
    packages: Vec<PackageEntry>,
}

/// Upstream workspace config (robonix_workspace.yaml).
#[derive(Debug, Deserialize, Default)]
struct UpstreamConfig {
    /// Workspace name.
    #[serde(default)]
    workspace: Option<String>,
    /// Global environment variables (merged with local, local wins).
    #[serde(default)]
    env: HashMap<String, String>,
    /// Runtime components (atlas, executor, pilot, liaison).
    #[serde(default)]
    runtime: Vec<RuntimeEntry>,
    /// System services (VLM, MemSearch, …).
    #[serde(default)]
    system: Vec<SystemEntry>,
}

/// A runtime component entry (from upstream_config).
#[derive(Debug, Deserialize, Clone)]
struct RuntimeEntry {
    name: String,
    #[serde(default)]
    endpoint: String,
    #[serde(default)]
    depends_on: Vec<String>,
    #[serde(default)]
    description: Option<String>,
}

/// A system service entry (from upstream_config).
#[derive(Debug, Deserialize, Clone)]
struct SystemEntry {
    name: String,
    #[serde(default)]
    contract_id: String,
    #[serde(default)]
    node_id: String,
    #[serde(default)]
    depends_on: Vec<String>,
    #[serde(default)]
    description: Option<String>,
}

/// A package entry (from the user's config.yaml).
#[derive(Debug, Deserialize, Clone)]
struct PackageEntry {
    name: String,
    path: String,
    #[serde(default)]
    params_file: Option<String>,
    #[serde(default)]
    params: Option<HashMap<String, serde_yaml::Value>>,
    #[serde(default)]
    depends_on: Vec<String>,
    #[serde(default)]
    transport: Option<String>,
    #[serde(default)]
    robonix: Option<RobonixIntegration>,
}

#[derive(Debug, Deserialize, Clone)]
struct RobonixIntegration {
    #[serde(default)]
    namespace: String,
    #[serde(default)]
    kind: String,
    #[serde(default)]
    interfaces: Vec<String>,
    #[serde(default)]
    skills_dir: String,
    #[serde(default)]
    tools: Vec<String>,
}

/// The merged view used during deployment.
struct MergedConfig {
    env: HashMap<String, String>,
    runtime: Vec<RuntimeEntry>,
    system: Vec<SystemEntry>,
    packages: Vec<PackageEntry>,
    #[allow(dead_code)]
    target: Option<String>,
}

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Cargo binary names for known runtime components.
fn runtime_cargo_package(name: &str) -> Option<&'static str> {
    match name {
        "robonix-atlas" => Some("robonix-atlas"),
        "robonix-executor" => Some("robonix-executor"),
        "robonix-pilot" => Some("robonix-pilot"),
        "robonix-liaison" => Some("robonix-liaison"),
        _ => None,
    }
}

/// Spawn a background process.  Returns its PID on success.
async fn spawn_background(
    label: &str,
    cmd: &str,
    args: &[&str],
    cwd: &Path,
    env: &HashMap<String, String>,
) -> Result<u32> {
    let mut command = Command::new(cmd);
    command
        .args(args)
        .current_dir(cwd)
        .envs(env)
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit());

    let child = command
        .spawn()
        .with_context(|| format!("failed to spawn {label}"))?;
    let pid = child
        .id()
        .ok_or_else(|| anyhow::anyhow!("failed to get PID for {label}"))?;
    // Don't await — let it run in the background.
    // Leak the child handle so it doesn't get killed when dropped.
    std::mem::forget(child);
    Ok(pid)
}

/// Wait for a TCP endpoint to become reachable (connect-only check).
async fn wait_for_endpoint(addr: &str, timeout: Duration) -> bool {
    let deadline = tokio::time::Instant::now() + timeout;
    while tokio::time::Instant::now() < deadline {
        if tokio::net::TcpStream::connect(addr).await.is_ok() {
            return true;
        }
        sleep(Duration::from_millis(300)).await;
    }
    false
}

/// Topological sort of packages by depends_on (works on Vec<PackageEntry>).
fn topo_sort_packages(packages: &[PackageEntry]) -> Result<Vec<usize>> {
    let name_to_idx: HashMap<&str, usize> = packages
        .iter()
        .enumerate()
        .map(|(i, p)| (p.name.as_str(), i))
        .collect();

    let n = packages.len();
    let mut in_degree = vec![0usize; n];
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];

    for (i, pkg) in packages.iter().enumerate() {
        for dep in &pkg.depends_on {
            if let Some(&dep_idx) = name_to_idx.get(dep.as_str()) {
                adj[dep_idx].push(i);
                in_degree[i] += 1;
            }
        }
    }

    // Kahn's algorithm.
    let mut queue: Vec<usize> = (0..n).filter(|&i| in_degree[i] == 0).collect();
    queue.sort_by(|a, b| packages[*a].name.cmp(&packages[*b].name));

    let mut order = Vec::with_capacity(n);
    while let Some(node) = queue.pop() {
        order.push(node);
        for &nb in &adj[node] {
            in_degree[nb] -= 1;
            if in_degree[nb] == 0 {
                queue.push(nb);
                queue.sort_by(|a, b| packages[*a].name.cmp(&packages[*b].name));
            }
        }
    }

    if order.len() != n {
        let missing: Vec<&str> = packages
            .iter()
            .enumerate()
            .filter(|(i, _)| !order.contains(i))
            .map(|(_, p)| p.name.as_str())
            .collect();
        anyhow::bail!("circular dependency detected among packages: {:?}", missing);
    }
    Ok(order)
}

/// Topological sort of runtime entries by depends_on.
fn topo_sort_runtime(entries: &[RuntimeEntry]) -> Result<Vec<usize>> {
    let name_to_idx: HashMap<&str, usize> = entries
        .iter()
        .enumerate()
        .map(|(i, e)| (e.name.as_str(), i))
        .collect();

    let n = entries.len();
    let mut in_degree = vec![0usize; n];
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];

    for (i, entry) in entries.iter().enumerate() {
        for dep in &entry.depends_on {
            if let Some(&dep_idx) = name_to_idx.get(dep.as_str()) {
                adj[dep_idx].push(i);
                in_degree[i] += 1;
            }
        }
    }

    let mut queue: Vec<usize> = (0..n).filter(|&i| in_degree[i] == 0).collect();
    queue.sort_by(|a, b| entries[*a].name.cmp(&entries[*b].name));

    let mut order = Vec::with_capacity(n);
    while let Some(node) = queue.pop() {
        order.push(node);
        for &nb in &adj[node] {
            in_degree[nb] -= 1;
            if in_degree[nb] == 0 {
                queue.push(nb);
                queue.sort_by(|a, b| entries[*a].name.cmp(&entries[*b].name));
            }
        }
    }

    if order.len() != n {
        let missing: Vec<&str> = entries
            .iter()
            .enumerate()
            .filter(|(i, _)| !order.contains(i))
            .map(|(_, e)| e.name.as_str())
            .collect();
        anyhow::bail!(
            "circular dependency detected among runtime entries: {:?}",
            missing
        );
    }
    Ok(order)
}

// ── Config loading & merging ────────────────────────────────────────────────

/// Load and merge configs: read config.yaml, optionally load upstream_config,
/// merge env (upstream defaults, local overrides), collect runtime + system + packages.
fn load_and_merge(config_path: &Path) -> Result<(MergedConfig, PathBuf)> {
    let content = std::fs::read_to_string(config_path)
        .with_context(|| format!("failed to read {}", config_path.display()))?;
    let cfg: DeployConfig = serde_yaml::from_str(&content)
        .with_context(|| format!("failed to parse {}", config_path.display()))?;

    // Resolve base directory (config.yaml's parent).
    let base_dir = config_path
        .parent()
        .map(|p| {
            if p.as_os_str().is_empty() {
                PathBuf::from(".")
            } else {
                p.to_path_buf()
            }
        })
        .unwrap_or_else(|| PathBuf::from("."));
    let base_dir = base_dir
        .canonicalize()
        .unwrap_or_else(|_| base_dir.to_path_buf());

    // Load upstream config if specified.
    let upstream = if let Some(ref upstream_path) = cfg.upstream_config {
        let resolved = base_dir.join(upstream_path);
        if resolved.exists() {
            let upstream_content = std::fs::read_to_string(&resolved)
                .with_context(|| format!("failed to read upstream config {}", resolved.display()))?;
            let up: UpstreamConfig = serde_yaml::from_str(&upstream_content).with_context(|| {
                format!("failed to parse upstream config {}", resolved.display())
            })?;
            output::sub_step(&format!(
                "loaded upstream config: {} (workspace: {})",
                resolved.display(),
                up.workspace.as_deref().unwrap_or("unnamed")
            ));
            up
        } else {
            output::warning(&format!(
                "upstream_config '{}' not found at {}, skipping",
                upstream_path,
                resolved.display()
            ));
            UpstreamConfig::default()
        }
    } else {
        UpstreamConfig::default()
    };

    // Merge env: upstream provides defaults, local config overrides.
    let mut merged_env = upstream.env.clone();
    for (k, v) in &cfg.env {
        merged_env.insert(k.clone(), v.clone());
    }

    Ok((
        MergedConfig {
            env: merged_env,
            runtime: upstream.runtime,
            system: upstream.system,
            packages: cfg.packages,
            target: cfg.target,
        },
        base_dir,
    ))
}

// ── Core deploy logic ────────────────────────────────────────────────────────

pub async fn execute(config_path: &Path) -> Result<()> {
    output::action("Deploy", &format!("from {}", config_path.display()));

    let (merged, base_dir) = load_and_merge(config_path)?;

    // Build global env map (merged config env + current process env).
    let mut global_env: HashMap<String, String> = merged.env.clone();
    for (k, v) in std::env::vars() {
        global_env.entry(k).or_insert(v);
    }

    // Inject runtime endpoints into env so packages can discover them.
    for rt in &merged.runtime {
        if !rt.endpoint.is_empty() {
            let env_key = rt.name.to_uppercase().replace('-', "_");
            global_env
                .entry(env_key)
                .or_insert_with(|| rt.endpoint.clone());
            // Also set _ENDPOINT variant.
            let env_key_ep = format!("{}_ENDPOINT", rt.name.to_uppercase().replace('-', "_"));
            global_env
                .entry(env_key_ep)
                .or_insert_with(|| format!("http://{}", rt.endpoint));
        }
    }

    if let Some(ref target) = merged.target {
        output::sub_step(&format!("target: {}", target));
    }

    // Detect Rust workspace root for cargo run.
    let rust_root = detect_rust_root(&base_dir);

    // ── Phase 1: Start runtime components ────────────────────────────────────
    if !merged.runtime.is_empty() {
        output::action("Phase 1", "Starting runtime components");

        let runtime_order = topo_sort_runtime(&merged.runtime)?;
        let runtime_names: Vec<&str> = runtime_order
            .iter()
            .map(|&i| merged.runtime[i].name.as_str())
            .collect();
        output::sub_step(&format!("boot order: {}", runtime_names.join(" → ")));

        for &idx in &runtime_order {
            let entry = &merged.runtime[idx];
            let cargo_pkg = runtime_cargo_package(&entry.name);
            if let Some(pkg) = cargo_pkg {
                output::step("Starting", &format!("{} (runtime)", entry.name));
                if let Some(desc) = &entry.description {
                    output::sub_step(&format!("  {}", desc));
                }

                // Check if already running.
                if !entry.endpoint.is_empty()
                    && wait_for_endpoint(&entry.endpoint, Duration::from_millis(500)).await
                {
                    output::check(&format!(
                        "{} already running at {}",
                        entry.name, entry.endpoint
                    ));
                    continue;
                }

                let cwd = rust_root.as_deref().unwrap_or(&base_dir);

                let pid =
                    spawn_background(&entry.name, "cargo", &["run", "-p", pkg], cwd, &global_env)
                        .await?;
                output::sub_step(&format!("PID {}", pid));

                // Wait for the endpoint to become reachable.
                if !entry.endpoint.is_empty() {
                    output::sub_step(&format!("waiting for {} ...", entry.endpoint));
                    if wait_for_endpoint(&entry.endpoint, Duration::from_secs(15)).await {
                        output::check(&format!("{} ready at {}", entry.name, entry.endpoint));
                    } else {
                        output::warning(&format!(
                            "{} started but endpoint {} not reachable after 15s — continuing",
                            entry.name, entry.endpoint
                        ));
                    }
                } else {
                    sleep(Duration::from_secs(2)).await;
                    output::check(&format!("{} started", entry.name));
                }
            } else {
                output::warning(&format!("unknown runtime '{}', skipping", entry.name));
            }
        }
    }

    // ── Phase 2: Start system services ───────────────────────────────────────
    if !merged.system.is_empty() {
        output::action("Phase 2", "Starting system services");

        let atlas_endpoint = merged
            .runtime
            .iter()
            .find(|r| r.name == "robonix-atlas")
            .map(|r| r.endpoint.clone())
            .unwrap_or_else(|| "127.0.0.1:50051".to_string());

        for entry in &merged.system {
            output::step("Starting", &format!("{} (service)", entry.name));
            if let Some(desc) = &entry.description {
                output::sub_step(&format!("  {}", desc));
            }

            if !entry.node_id.is_empty() {
                // Try to find the service package under robonix examples.
                let service_pkg_path = find_service_package(rust_root.as_deref(), &entry.name);
                if let Some(pkg_path) = service_pkg_path {
                    let cwd = rust_root.as_deref().unwrap_or(&base_dir);
                    let pkg_str = pkg_path.to_string_lossy().to_string();
                    let pid = spawn_background(
                        &entry.name,
                        "cargo",
                        &[
                            "run",
                            "-p",
                            "robonix-cli",
                            "--",
                            "start",
                            "--endpoint",
                            &atlas_endpoint,
                            "-p",
                            &pkg_str,
                            "-n",
                            &entry.node_id,
                        ],
                        cwd,
                        &global_env,
                    )
                    .await?;
                    output::sub_step(&format!("PID {} (rbnx start {})", pid, entry.node_id));
                    sleep(Duration::from_secs(2)).await;
                    output::check(&format!("{} started", entry.name));
                } else {
                    output::warning(&format!(
                        "service '{}' (node_id={}) — no package found; ensure it is started externally",
                        entry.name, entry.node_id
                    ));
                }
            } else {
                output::warning(&format!(
                    "service '{}' has no node_id — ensure it is started externally",
                    entry.name
                ));
            }
        }
    }

    // ── Phase 3: Start packages (topological order) ──────────────────────────
    if !merged.packages.is_empty() {
        output::action("Phase 3", "Starting packages");

        let launch_order = topo_sort_packages(&merged.packages)?;
        let pkg_names: Vec<&str> = launch_order
            .iter()
            .map(|&i| merged.packages[i].name.as_str())
            .collect();
        output::sub_step(&format!("launch order: {}", pkg_names.join(" → ")));

        for &idx in &launch_order {
            let entry = &merged.packages[idx];
            let pkg_path = base_dir.join(&entry.path);

            if !pkg_path.exists() {
                output::warning(&format!(
                    "package '{}' path {} does not exist, skipping",
                    entry.name,
                    pkg_path.display()
                ));
                continue;
            }

            output::step("Launching", &format!("{} ({})", entry.name, entry.path));

            // ── Read robonix_manifest.yaml from the package directory ────────
            let manifest_path = pkg_path.join(manifest::MANIFEST_FILE);
            if !manifest_path.exists() {
                output::warning(&format!(
                    "package '{}': {} not found at {}, skipping",
                    entry.name,
                    manifest::MANIFEST_FILE,
                    manifest_path.display()
                ));
                continue;
            }

            let pkg_manifest = manifest::load_from_path(&manifest_path).with_context(|| {
                format!(
                    "failed to load {} for package '{}'",
                    manifest::MANIFEST_FILE,
                    entry.name
                )
            })?;

            // Pick the first node's start command (or the only one).
            if pkg_manifest.nodes.is_empty() {
                output::warning(&format!(
                    "package '{}': {} has no nodes defined, skipping",
                    entry.name,
                    manifest::MANIFEST_FILE
                ));
                continue;
            }

            let node = &pkg_manifest.nodes[0];
            let start_cmd = node.start.trim();
            if start_cmd.is_empty() {
                output::warning(&format!(
                    "package '{}': node '{}' has empty start command, skipping",
                    entry.name, node.id
                ));
                continue;
            }

            output::sub_step(&format!(
                "manifest: {} v{} — node '{}' ({})",
                pkg_manifest.package.name,
                pkg_manifest.package.version,
                node.id,
                node.node_type.as_deref().unwrap_or("unknown")
            ));

            // Build the full launch command with optional params.
            let mut full_cmd = String::new();

            // Inject params_file if specified in config.yaml.
            if let Some(ref params_file) = entry.params_file {
                let params_abs = pkg_path.join(params_file);
                full_cmd.push_str(&format!(
                    "export RBNX_PARAMS_FILE={}; ",
                    params_abs.display()
                ));
            }

            // Inject inline params as environment variables.
            if let Some(ref params) = entry.params {
                for (k, v) in params {
                    let val_str = match v {
                        serde_yaml::Value::String(s) => s.clone(),
                        serde_yaml::Value::Bool(b) => b.to_string(),
                        serde_yaml::Value::Number(n) => n.to_string(),
                        other => format!("{:?}", other),
                    };
                    // Export as RBNX_PARAM_<KEY>=<VALUE>.
                    let env_key = format!(
                        "RBNX_PARAM_{}",
                        k.to_uppercase().replace('.', "_").replace('-', "_")
                    );
                    full_cmd.push_str(&format!("export {}={}; ", env_key, val_str));
                }
            }

            full_cmd.push_str(start_cmd);

            let pid = spawn_background(
                &entry.name,
                "bash",
                &["-c", &full_cmd],
                &pkg_path,
                &global_env,
            )
            .await?;
            output::sub_step(&format!("PID {} — {}", pid, start_cmd));

            // If this package has robonix integration, log it.
            if let Some(ref rbnx) = entry.robonix {
                output::sub_step(&format!(
                    "robonix integration: ns={}, kind={}, tools=[{}]",
                    rbnx.namespace,
                    rbnx.kind,
                    rbnx.tools.join(", ")
                ));
            }

            // Brief pause between launches for service discovery.
            sleep(Duration::from_secs(2)).await;
            output::check(&format!("{} launched", entry.name));
        }
    }

    // ── Summary ──────────────────────────────────────────────────────────────
    let total = merged.runtime.len() + merged.system.len() + merged.packages.len();
    output::success(&format!(
        "Deploy complete — {} component(s) started ({} runtime, {} service, {} package)",
        total,
        merged.runtime.len(),
        merged.system.len(),
        merged.packages.len(),
    ));
    output::info("Use 'rbnx chat' to interact with the system, or Ctrl+C to stop.");

    // Keep the main process alive so background children keep running.
    output::info("Press Ctrl+C to shut down all deployed components.");
    tokio::signal::ctrl_c()
        .await
        .context("failed to listen for Ctrl+C")?;

    output::action("Shutdown", "stopping deployed components...");
    #[cfg(unix)]
    {
        use nix::sys::signal::{Signal, killpg};
        use nix::unistd::getpgrp;
        let pgid = getpgrp();
        let _ = killpg(pgid, Signal::SIGTERM);
    }

    Ok(())
}

// ── Discovery helpers ────────────────────────────────────────────────────────

/// Walk up from `base` looking for a Cargo workspace containing robonix crates.
fn detect_rust_root(base: &Path) -> Option<PathBuf> {
    let mut dir = base.to_path_buf();
    for _ in 0..10 {
        let candidate = dir.join("Cargo.toml");
        if candidate.exists() {
            if let Ok(content) = std::fs::read_to_string(&candidate) {
                if content.contains("[workspace]") && content.contains("robonix") {
                    return Some(dir);
                }
            }
        }
        let rust_candidate = dir.join("rust").join("Cargo.toml");
        if rust_candidate.exists() {
            if let Ok(content) = std::fs::read_to_string(&rust_candidate) {
                if content.contains("[workspace]") && content.contains("robonix") {
                    return Some(dir.join("rust"));
                }
            }
        }
        if !dir.pop() {
            break;
        }
    }
    None
}

/// Try to find the service package under the robonix examples directory.
/// Supports both dotted names (robonix.sys.model.vlm → vlm_service)
/// and short names (vlm → vlm_service).
fn find_service_package(rust_root: Option<&Path>, service_name: &str) -> Option<PathBuf> {
    let root = rust_root?;
    let packages_dir = root.join("examples").join("packages");

    // Extract the short name from dotted notation:
    //   "robonix.sys.model.vlm" → "vlm"
    //   "robonix.sys.memory.search" → "memsearch" (special case)
    let short_name = service_name
        .rsplit('.')
        .next()
        .unwrap_or(service_name);

    let dir_name = match short_name {
        "vlm" => "vlm_service",
        "search" | "memsearch" => "memsearch_service",
        other => {
            // Try <name>_service as a fallback.
            let candidate = packages_dir.join(format!("{}_service", other));
            if candidate.exists() {
                return Some(candidate);
            }
            return None;
        }
    };

    let candidate = packages_dir.join(dir_name);
    if candidate.exists() {
        Some(candidate)
    } else {
        None
    }
}
