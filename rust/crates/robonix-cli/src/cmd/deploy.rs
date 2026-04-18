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
use robonix_cli::output;
use robonix_cli::workspace::{
    DeployConfig, MergedConfig, WorkspaceConfig, parse_package_run,
};
use std::collections::{HashMap, VecDeque};
use std::path::{Path, PathBuf};
use std::process::Stdio;
use tokio::process::{Child, Command};
use tokio::time::{Duration, sleep};

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Spawn a background process.  Returns the Child handle (caller must store it).
///
/// On Unix, the child is placed into its own process group via `pre_exec(setsid)`
/// so that we can cleanly terminate it later without killing the parent shell.
#[allow(dead_code)]
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
#[allow(dead_code)]
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

/// Load and merge configs: read deploy/<target>.yaml, optionally load upstream
/// robonix_workspace.yaml, merge env (upstream defaults, local overrides),
/// parse packages_run entries.
fn load_and_merge(config_path: &Path) -> Result<(MergedConfig, PathBuf)> {
    let content = std::fs::read_to_string(config_path)
        .with_context(|| format!("failed to read {}", config_path.display()))?;
    let cfg: DeployConfig = serde_yaml::from_str(&content)
        .with_context(|| format!("failed to parse {}", config_path.display()))?;

    // Resolve base directory (config file's parent).
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

    // Load upstream workspace config if specified.
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

    // Merge env: upstream provides defaults, local config overrides.
    let mut merged_env = upstream.env.clone();
    for (k, v) in &cfg.env {
        merged_env.insert(k.clone(), v.clone());
    }

    // Parse packages_run entries.
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

/// Gracefully shut down all tracked child processes by sending SIGTERM
/// to each child's process group, then waiting for them.
#[cfg(unix)]
#[allow(dead_code)]
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
#[allow(dead_code)]
async fn shutdown_children(children: &mut Vec<Child>) {
    for child in children.iter_mut() {
        let _ = child.kill().await;
    }
}

// ── Discovery helpers ────────────────────────────────────────────────────────

/// Walk up from `base` looking for a Cargo workspace containing robonix crates.
#[allow(dead_code)]
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

// ── Core deploy logic ────────────────────────────────────────────────────────

/// Deploy from a config file (deploy/<target>.yaml).
///
/// Phase 1 (stage one): loads and validates config, prints summary.
/// Full execution logic (system startup, node launching) will be
/// implemented in stage three.
pub async fn execute(config_path: &Path) -> Result<()> {
    output::action("Deploy", &format!("from {}", config_path.display()));

    let (merged, _base_dir) = load_and_merge(config_path)?;

    // Print summary of loaded configuration.
    if let Some(ref target) = merged.target {
        output::sub_step(&format!("target: {}", target));
    }
    output::sub_step(&format!(
        "workspace: {}, {} workspace package(s), {} packages_run entry(ies)",
        merged.workspace_name.as_deref().unwrap_or("unnamed"),
        merged.workspace_packages.len(),
        merged.packages_run.len(),
    ));

    for pkg in &merged.workspace_packages {
        let source = if let Some(ref url) = pkg.url {
            format!("url={}", url)
        } else if let Some(ref path) = pkg.path {
            format!("path={}", path)
        } else {
            "no source".to_string()
        };
        output::sub_step(&format!("  package: {} ({})", pkg.name, source));
    }

    for run in &merged.packages_run {
        let selector = match &run.node_selector {
            robonix_cli::workspace::NodeSelector::All => "all".to_string(),
            robonix_cli::workspace::NodeSelector::Single(id) => id.clone(),
        };
        output::sub_step(&format!("  run: {}:{}", run.package_name, selector));
    }

    output::warning("deploy execution logic not yet implemented (stage 3 WIP)");

    Ok(())
}

/// Deploy from the workspace-level config (no explicit config file).
///
/// Stub implementation — will be completed in stage three.
pub async fn execute_from_workspace() -> Result<()> {
    let ws_yaml = PathBuf::from("robonix_workspace.yaml");
    if !ws_yaml.exists() {
        anyhow::bail!(
            "no robonix_workspace.yaml found in current directory; \
             use 'rbnx deploy <target>' or 'rbnx deploy -c <config>' instead"
        );
    }
    output::warning("workspace-level deploy not yet implemented (stage 3 WIP)");
    Ok(())
}
