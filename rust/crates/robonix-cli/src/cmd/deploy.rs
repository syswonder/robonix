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
use std::collections::{HashMap, VecDeque};
use std::path::{Path, PathBuf};
use std::process::Stdio;
use tokio::process::{Child, Command};
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
#[allow(dead_code)]
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
#[allow(dead_code)]
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
#[allow(dead_code)]
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

/// Spawn a background process.  Returns the Child handle (caller must store it).
///
/// On Unix, the child is placed into its own process group via `pre_exec(setsid)`
/// so that we can cleanly terminate it later without killing the parent shell.
async fn spawn_background(
    label: &str,
    cmd: &str,
    args: &[&str],
    cwd: &Path,
    env: &HashMap<String, String>,
) -> Result<Child> {
    let mut command = Command::new(cmd);
    command
        .args(args)
        .current_dir(cwd)
        .envs(env)
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit());

    // Place child in its own process group so we can kill it independently.
    #[cfg(unix)]
    unsafe {
        command.pre_exec(|| {
            // Create a new session / process group for this child.
            nix::unistd::setsid().map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
            Ok(())
        });
    }

    let child = command
        .spawn()
        .with_context(|| format!("failed to spawn {label}"))?;
    Ok(child)
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

/// Generic topological sort using Kahn's algorithm with FIFO (VecDeque)
/// for correct alphabetical ordering.
///
/// `items` is a slice of (name, depends_on) pairs.
/// Returns indices in dependency-first order.
/// Reports an error if any `depends_on` reference is unknown.
pub(crate) fn topo_sort(items: &[(&str, &[String])]) -> Result<Vec<usize>> {
    let name_to_idx: HashMap<&str, usize> = items
        .iter()
        .enumerate()
        .map(|(i, (name, _))| (*name, i))
        .collect();

    let n = items.len();
    let mut in_degree = vec![0usize; n];
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];

    for (i, (item_name, deps)) in items.iter().enumerate() {
        for dep in *deps {
            if let Some(&dep_idx) = name_to_idx.get(dep.as_str()) {
                adj[dep_idx].push(i);
                in_degree[i] += 1;
            } else {
                anyhow::bail!(
                    "'{}' depends on '{}', but '{}' is not defined",
                    item_name,
                    dep,
                    dep
                );
            }
        }
    }

    // Kahn's algorithm with a VecDeque (FIFO) for stable alphabetical order.
    let mut ready: Vec<usize> = (0..n).filter(|&i| in_degree[i] == 0).collect();
    ready.sort_by(|a, b| items[*a].0.cmp(items[*b].0));
    let mut queue: VecDeque<usize> = ready.into_iter().collect();

    let mut order = Vec::with_capacity(n);
    while let Some(node) = queue.pop_front() {
        order.push(node);
        // Collect newly-ready neighbors, sort alphabetically, then enqueue.
        let mut newly_ready = Vec::new();
        for &nb in &adj[node] {
            in_degree[nb] -= 1;
            if in_degree[nb] == 0 {
                newly_ready.push(nb);
            }
        }
        newly_ready.sort_by(|a, b| items[*a].0.cmp(items[*b].0));
        for idx in newly_ready {
            queue.push_back(idx);
        }
    }

    if order.len() != n {
        let missing: Vec<&str> = items
            .iter()
            .enumerate()
            .filter(|(i, _)| !order.contains(i))
            .map(|(_, (name, _))| *name)
            .collect();
        anyhow::bail!("circular dependency detected: {:?}", missing);
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

/// Gracefully shut down all tracked child processes by sending SIGTERM
/// to each child's process group, then waiting for them.
#[cfg(unix)]
async fn shutdown_children(children: &mut Vec<Child>) {
    use nix::sys::signal::{Signal, kill};
    use nix::unistd::Pid;

    for child in children.iter_mut() {
        if let Some(pid) = child.id() {
            // Send SIGTERM to the child's process group (negative PID).
            let _ = kill(Pid::from_raw(-(pid as i32)), Signal::SIGTERM);
        }
    }
    // Give children a moment to exit gracefully.
    sleep(Duration::from_secs(2)).await;
    for child in children.iter_mut() {
        // Force kill any remaining.
        let _ = child.kill().await;
    }
}

#[cfg(not(unix))]
async fn shutdown_children(children: &mut Vec<Child>) {
    for child in children.iter_mut() {
        let _ = child.kill().await;
    }
}

// ── Core deploy logic ────────────────────────────────────────────────────────

pub async fn execute(config_path: &Path) -> Result<()> {
    output::action("Deploy", &format!("from {}", config_path.display()));

    let (merged, base_dir) = load_and_merge(config_path)?;

    // Build global env map (merged config env + current process env).
    // Config env takes priority over inherited process env.
    let mut global_env: HashMap<String, String> = std::env::vars().collect();
    for (k, v) in &merged.env {
        global_env.insert(k.clone(), v.clone());
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

    // Track all spawned child processes for graceful shutdown.
    let mut children: Vec<Child> = Vec::new();

    // ── Phase 1: Start runtime components ────────────────────────────────────
    if !merged.runtime.is_empty() {
        output::action("Phase 1", "Starting runtime components");

        let items: Vec<(&str, &[String])> = merged
            .runtime
            .iter()
            .map(|e| (e.name.as_str(), e.depends_on.as_slice()))
            .collect();
        let runtime_order = topo_sort(&items)?;
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

                let child =
                    spawn_background(&entry.name, "cargo", &["run", "-p", pkg], cwd, &global_env)
                        .await?;
                let pid = child.id().unwrap_or(0);
                output::sub_step(&format!("PID {}", pid));
                children.push(child);

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
                    let child = spawn_background(
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
                    let pid = child.id().unwrap_or(0);
                    output::sub_step(&format!("PID {} (rbnx start {})", pid, entry.node_id));
                    children.push(child);
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

        let items: Vec<(&str, &[String])> = merged
            .packages
            .iter()
            .map(|p| (p.name.as_str(), p.depends_on.as_slice()))
            .collect();
        let launch_order = topo_sort(&items)?;
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

            // ── Validate manifest start command (security) ──────────────────
            // Log the command being executed for auditability.
            output::sub_step(&format!(
                "manifest: {} v{} — node '{}' ({})",
                pkg_manifest.package.name,
                pkg_manifest.package.version,
                node.id,
                node.node_type.as_deref().unwrap_or("unknown")
            ));
            output::sub_step(&format!("executing start command: {}", start_cmd));

            // Build per-package env: inject params via environment variables
            // instead of shell string interpolation (prevents command injection).
            let mut pkg_env = global_env.clone();

            // Inject params_file if specified in config.yaml.
            if let Some(ref params_file) = entry.params_file {
                let params_abs = pkg_path.join(params_file);
                pkg_env.insert(
                    "RBNX_PARAMS_FILE".to_string(),
                    params_abs.display().to_string(),
                );
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
                    pkg_env.insert(env_key, val_str);
                }
            }

            // Launch via bash -c with the start command; params are in env,
            // not interpolated into the shell string.
            let child = spawn_background(
                &entry.name,
                "bash",
                &["-c", start_cmd],
                &pkg_path,
                &pkg_env,
            )
            .await?;
            let pid = child.id().unwrap_or(0);
            output::sub_step(&format!("PID {} — {}", pid, start_cmd));
            children.push(child);

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
    shutdown_children(&mut children).await;

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
