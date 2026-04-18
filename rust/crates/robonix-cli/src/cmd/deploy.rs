// SPDX-License-Identifier: MulanPSL-2.0
// Deploy command: read a deploy/<target>.yaml and bring up the full stack.
//
// The config file references an `upstream_config` (robonix_workspace.yaml)
// which provides workspace-level env and package declarations.
// The deploy command merges both configs, then:
//   Step 1 — start upstream system (fixed components)
//   Step 2 — check packages existence
//   Step 3 — validate packages_run declarations
//   Step 4 — start nodes in dependency order

use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::workspace::{
    DeployConfig, MergedConfig, NodeSelector, ParsedPackageRun, WorkspaceConfig,
    WorkspacePackageEntry, parse_package_run, find_workspace_root, ensure_packages_exist,
};
use std::collections::{HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::process::Stdio;
use tokio::process::{Child, Command};
use tokio::time::{Duration, sleep};

// ── Process helpers ─────────────────────────────────────────────────────────

/// Spawn a background process in its own process group.
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

    #[cfg(unix)]
    unsafe {
        command.pre_exec(|| {
            nix::unistd::setsid().map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
            Ok(())
        });
    }

    let child = command
        .spawn()
        .with_context(|| format!("failed to spawn {label}"))?;
    Ok(child)
}

/// Wait for a TCP endpoint to become reachable.
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

/// Gracefully shut down all tracked child processes.
#[cfg(unix)]
async fn shutdown_children(children: &mut Vec<Child>) {
    use nix::sys::signal::{Signal, kill};
    use nix::unistd::Pid;

    for child in children.iter_mut() {
        if let Some(pid) = child.id() {
            let _ = kill(Pid::from_raw(-(pid as i32)), Signal::SIGTERM);
        }
    }
    sleep(Duration::from_secs(2)).await;
    for child in children.iter_mut() {
        let _ = child.kill().await;
    }
}

#[cfg(not(unix))]
async fn shutdown_children(children: &mut Vec<Child>) {
    for child in children.iter_mut() {
        let _ = child.kill().await;
    }
}

// ── Topological sort ────────────────────────────────────────────────────────

/// Generic topological sort using Kahn's algorithm.
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

    let mut ready: Vec<usize> = (0..n).filter(|&i| in_degree[i] == 0).collect();
    ready.sort_by(|a, b| items[*a].0.cmp(items[*b].0));
    let mut queue: VecDeque<usize> = ready.into_iter().collect();

    let mut order = Vec::with_capacity(n);
    while let Some(node) = queue.pop_front() {
        order.push(node);
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

/// Load deploy config + upstream workspace config, merge env, parse packages_run.
fn load_and_merge(config_path: &Path) -> Result<(MergedConfig, PathBuf)> {
    let content = std::fs::read_to_string(config_path)
        .with_context(|| format!("failed to read {}", config_path.display()))?;
    let cfg: DeployConfig = serde_yaml::from_str(&content)
        .with_context(|| format!("failed to parse {}", config_path.display()))?;

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

    let upstream = if let Some(ref upstream_path) = cfg.upstream_config {
        let resolved = base_dir.join(upstream_path);
        if resolved.exists() {
            let upstream_content = std::fs::read_to_string(&resolved)
                .with_context(|| format!("failed to read upstream config {}", resolved.display()))?;
            let ws: WorkspaceConfig = serde_yaml::from_str(&upstream_content).with_context(|| {
                format!("failed to parse upstream config {}", resolved.display())
            })?;
            output::sub_step(&format!(
                "loaded upstream config: {} (workspace: {})",
                resolved.display(),
                ws.workspace.as_deref().unwrap_or("unnamed")
            ));
            ws
        } else {
            output::warning(&format!(
                "upstream_config '{}' not found at {}, skipping",
                upstream_path,
                resolved.display()
            ));
            WorkspaceConfig::default()
        }
    } else {
        WorkspaceConfig::default()
    };

    let mut merged_env = upstream.env.clone();
    for (k, v) in &cfg.env {
        merged_env.insert(k.clone(), v.clone());
    }

    let packages_run = cfg
        .packages_run
        .iter()
        .map(|e| parse_package_run(e))
        .collect::<Result<Vec<_>>>()?;

    Ok((
        MergedConfig {
            env: merged_env,
            workspace_name: upstream.workspace,
            workspace_packages: upstream.packages,
            packages_run,
            target: cfg.target,
        },
        base_dir,
    ))
}

// ── Discovery helpers ───────────────────────────────────────────────────────

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

// ── Step 1: Start upstream system ───────────────────────────────────────────

/// Start the fixed system components (atlas, executor, pilot, liaison).
async fn start_upstream_system(
    workspace_root: &Path,
    env: &HashMap<String, String>,
    children: &mut Vec<Child>,
) -> Result<()> {
    let system_components = [
        ("robonix-atlas", "127.0.0.1:50051"),
        ("robonix-executor", "127.0.0.1:50061"),
        ("robonix-pilot", "127.0.0.1:50071"),
        ("robonix-liaison", "127.0.0.1:50081"),
    ];

    let rust_root = detect_rust_root(workspace_root);
    let cwd = rust_root.as_deref().unwrap_or(workspace_root);

    for (name, endpoint) in &system_components {
        output::step("Starting", &format!("{} (system)", name));

        // Check if already running.
        if wait_for_endpoint(endpoint, Duration::from_millis(500)).await {
            output::check(&format!("{} already running at {}", name, endpoint));
            continue;
        }

        let child = spawn_background(name, "cargo", &["run", "-p", name], cwd, env).await?;
        let pid = child.id().unwrap_or(0);
        output::sub_step(&format!("PID {}", pid));
        children.push(child);

        output::sub_step(&format!("waiting for {} ...", endpoint));
        if wait_for_endpoint(endpoint, Duration::from_secs(15)).await {
            output::check(&format!("{} ready at {}", name, endpoint));
        } else {
            output::warning(&format!(
                "{} started but {} not reachable after 15s — continuing",
                name, endpoint
            ));
        }
    }
    Ok(())
}

// ── Step 3: Validate packages_run ───────────────────────────────────────────

/// A validated package run entry with resolved manifest and nodes.
struct ValidatedPackageRun {
    package_name: String,
    pkg_path: PathBuf,
    manifest: manifest::Manifest,
    nodes_to_start: Vec<manifest::Node>,
}

/// Validate packages_run entries against workspace declarations and manifests.
fn validate_packages_run(
    packages_run: &[ParsedPackageRun],
    workspace_packages: &[WorkspacePackageEntry],
    package_paths: &HashMap<String, PathBuf>,
) -> Result<Vec<ValidatedPackageRun>> {
    let ws_pkg_names: HashSet<&str> = workspace_packages.iter().map(|p| p.name.as_str()).collect();
    let mut validated = Vec::new();

    for run in packages_run {
        // 1. Check package is declared in workspace.
        if !ws_pkg_names.contains(run.package_name.as_str()) {
            anyhow::bail!(
                "packages_run references '{}' but it is not declared in workspace packages",
                run.package_name
            );
        }

        // 2. Get path and load manifest.
        let pkg_path = package_paths
            .get(&run.package_name)
            .ok_or_else(|| anyhow::anyhow!("package '{}' path not resolved", run.package_name))?;
        let manifest_path = pkg_path.join(manifest::MANIFEST_FILE);
        let pkg_manifest = manifest::load_from_path(&manifest_path).with_context(|| {
            format!(
                "failed to load {} for package '{}'",
                manifest::MANIFEST_FILE,
                run.package_name
            )
        })?;

        // 3. Validate node selector.
        let nodes_to_start: Vec<manifest::Node> = match &run.node_selector {
            NodeSelector::All => {
                if pkg_manifest.nodes.is_empty() {
                    output::warning(&format!(
                        "package '{}' has no nodes defined",
                        run.package_name
                    ));
                }
                pkg_manifest.nodes.clone()
            }
            NodeSelector::Single(node_id) => {
                let node = pkg_manifest
                    .nodes
                    .iter()
                    .find(|n| n.id == *node_id)
                    .ok_or_else(|| {
                        anyhow::anyhow!(
                            "package '{}' has no node '{}'. Available: {:?}",
                            run.package_name,
                            node_id,
                            pkg_manifest
                                .nodes
                                .iter()
                                .map(|n| &n.id)
                                .collect::<Vec<_>>()
                        )
                    })?;
                vec![node.clone()]
            }
        };

        let node_ids: Vec<&str> = nodes_to_start.iter().map(|n| n.id.as_str()).collect();
        output::check(&format!(
            "{} — {} v{}, nodes: [{}]",
            run.package_name,
            pkg_manifest.package.name,
            pkg_manifest.package.version,
            node_ids.join(", ")
        ));

        validated.push(ValidatedPackageRun {
            package_name: run.package_name.clone(),
            pkg_path: pkg_path.clone(),
            manifest: pkg_manifest,
            nodes_to_start,
        });
    }
    Ok(validated)
}

// ── Core deploy logic ────────────────────────────────────────────────────────

/// Deploy from a config file (deploy/<target>.yaml).
pub async fn execute(config_path: &Path) -> Result<()> {
    output::action("Deploy", &format!("from {}", config_path.display()));

    let (merged, base_dir) = load_and_merge(config_path)?;

    // Build global env (current process env + merged config env).
    let mut global_env: HashMap<String, String> = std::env::vars().collect();
    for (k, v) in &merged.env {
        global_env.insert(k.clone(), v.clone());
    }

    if let Some(ref target) = merged.target {
        output::sub_step(&format!("target: {}", target));
    }

    let workspace_root = find_workspace_root(&base_dir)?;
    let mut children: Vec<Child> = Vec::new();

    // ── Step 1: Check packages existence (fast-fail before heavy system startup) ─
    output::action("Step 1", "Checking packages existence");
    let package_paths = ensure_packages_exist(&workspace_root, &merged.workspace_packages)?;

    // ── Step 2: Validate packages_run declarations ──────────────────────────
    output::action("Step 2", "Validating packages_run");
    let validated = validate_packages_run(
        &merged.packages_run,
        &merged.workspace_packages,
        &package_paths,
    )?;

    if validated.is_empty() {
        output::warning("no packages_run entries — nothing to deploy");
        return Ok(());
    }

    // Build check (warning only).
    let build_dir = workspace_root.join("build");
    if !build_dir.exists() {
        output::warning("build/ directory not found; packages may not have been built yet");
    }

    // ── Step 3: Start upstream system (fixed components) ────────────────────
    output::action("Step 3", "Starting upstream system");
    start_upstream_system(&workspace_root, &global_env, &mut children).await?;

    // ── Step 4: Start nodes in dependency order ─────────────────────────────
    output::action("Step 4", "Starting nodes in dependency order");

    // Topological sort based on manifest.depend.
    let items: Vec<(&str, &[String])> = validated
        .iter()
        .map(|v| (v.package_name.as_str(), v.manifest.depend.as_slice()))
        .collect();
    let order = topo_sort(&items)?;

    let order_names: Vec<&str> = order
        .iter()
        .map(|&i| validated[i].package_name.as_str())
        .collect();
    output::sub_step(&format!("launch order: {}", order_names.join(" → ")));

    // Launch nodes in topo order.
    for &idx in &order {
        let vp = &validated[idx];
        for node in &vp.nodes_to_start {
            let start_cmd = node.start.trim();
            if start_cmd.is_empty() {
                output::warning(&format!(
                    "{}:{} has empty start command, skipping",
                    vp.package_name, node.id
                ));
                continue;
            }

            output::step("Starting", &format!("{}:{}", vp.package_name, node.id));
            output::sub_step(&format!("cmd: {}", start_cmd));

            let child = spawn_background(
                &format!("{}:{}", vp.package_name, node.id),
                "bash",
                &["-c", start_cmd],
                &vp.pkg_path,
                &global_env,
            )
            .await?;
            let pid = child.id().unwrap_or(0);
            output::sub_step(&format!("PID {}", pid));
            children.push(child);

            sleep(Duration::from_secs(2)).await;
            output::check(&format!("{}:{} launched", vp.package_name, node.id));
        }
    }

    // ── Summary + wait for Ctrl+C ───────────────────────────────────────────
    let node_count: usize = validated.iter().map(|v| v.nodes_to_start.len()).sum();
    output::success(&format!(
        "Deploy complete — {} node(s) from {} package(s) started",
        node_count,
        validated.len(),
    ));
    output::info("Press Ctrl+C to shut down all deployed components.");

    tokio::signal::ctrl_c()
        .await
        .context("failed to listen for Ctrl+C")?;

    output::action("Shutdown", "stopping deployed components...");
    shutdown_children(&mut children).await;

    Ok(())
}

/// Deploy from workspace-level config (no explicit config file).
/// Reads robonix_workspace.yaml and starts all packages with all nodes.
pub async fn execute_from_workspace() -> Result<()> {
    let ws_yaml = PathBuf::from("robonix_workspace.yaml");
    if !ws_yaml.exists() {
        anyhow::bail!(
            "no robonix_workspace.yaml found in current directory; \
             use 'rbnx deploy <target>' or 'rbnx deploy -c <config>' instead"
        );
    }

    let workspace_root = std::env::current_dir()?;
    let content = std::fs::read_to_string(&ws_yaml)?;
    let ws: WorkspaceConfig = serde_yaml::from_str(&content)
        .with_context(|| "failed to parse robonix_workspace.yaml")?;

    output::action(
        "Deploy",
        &format!(
            "from workspace {} ({} package(s))",
            ws.workspace.as_deref().unwrap_or("unnamed"),
            ws.packages.len(),
        ),
    );

    if ws.packages.is_empty() {
        output::warning("no packages declared in workspace — nothing to deploy");
        return Ok(());
    }

    // Build global env.
    let mut global_env: HashMap<String, String> = std::env::vars().collect();
    for (k, v) in &ws.env {
        global_env.insert(k.clone(), v.clone());
    }

    let mut children: Vec<Child> = Vec::new();

    // Step 1: Check packages existence (fast-fail).
    output::action("Step 1", "Checking packages existence");
    let package_paths = ensure_packages_exist(&workspace_root, &ws.packages)?;

    // Step 2: Load manifests for all packages.
    output::action("Step 2", "Loading manifests");
    let mut validated = Vec::new();
    for pkg_entry in &ws.packages {
        let pkg_path = package_paths.get(&pkg_entry.name).unwrap();
        let manifest_path = pkg_path.join(manifest::MANIFEST_FILE);
        let pkg_manifest = manifest::load_from_path(&manifest_path)?;
        let nodes = pkg_manifest.nodes.clone();
        let node_ids: Vec<&str> = nodes.iter().map(|n| n.id.as_str()).collect();
        output::check(&format!(
            "{} — {} v{}, nodes: [{}]",
            pkg_entry.name,
            pkg_manifest.package.name,
            pkg_manifest.package.version,
            node_ids.join(", ")
        ));
        validated.push(ValidatedPackageRun {
            package_name: pkg_entry.name.clone(),
            pkg_path: pkg_path.clone(),
            manifest: pkg_manifest,
            nodes_to_start: nodes,
        });
    }

    if validated.is_empty() {
        output::warning("no packages to deploy");
        return Ok(());
    }

    // Step 3: Start upstream system.
    output::action("Step 3", "Starting upstream system");
    start_upstream_system(&workspace_root, &global_env, &mut children).await?;

    // Step 4: Start nodes in dependency order.
    output::action("Step 4", "Starting nodes in dependency order");

    let items: Vec<(&str, &[String])> = validated
        .iter()
        .map(|v| (v.package_name.as_str(), v.manifest.depend.as_slice()))
        .collect();
    let order = topo_sort(&items)?;

    let order_names: Vec<&str> = order
        .iter()
        .map(|&i| validated[i].package_name.as_str())
        .collect();
    output::sub_step(&format!("launch order: {}", order_names.join(" → ")));

    for &idx in &order {
        let vp = &validated[idx];
        for node in &vp.nodes_to_start {
            let start_cmd = node.start.trim();
            if start_cmd.is_empty() {
                continue;
            }
            output::step("Starting", &format!("{}:{}", vp.package_name, node.id));
            let child = spawn_background(
                &format!("{}:{}", vp.package_name, node.id),
                "bash",
                &["-c", start_cmd],
                &vp.pkg_path,
                &global_env,
            )
            .await?;
            children.push(child);
            sleep(Duration::from_secs(2)).await;
            output::check(&format!("{}:{} launched", vp.package_name, node.id));
        }
    }

    let node_count: usize = validated.iter().map(|v| v.nodes_to_start.len()).sum();
    output::success(&format!(
        "Deploy complete — {} node(s) from {} package(s) started",
        node_count,
        validated.len(),
    ));
    output::info("Press Ctrl+C to shut down all deployed components.");

    tokio::signal::ctrl_c().await?;
    output::action("Shutdown", "stopping deployed components...");
    shutdown_children(&mut children).await;

    Ok(())
}
