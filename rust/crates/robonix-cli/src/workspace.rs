// SPDX-License-Identifier: MulanPSL-2.0
// Workspace configuration types shared by deploy and build commands.
//
// Defines the Rust structs that map to:
//   - robonix_workspace.yaml  (WorkspaceConfig)
//   - deploy/<target>.yaml    (DeployConfig)

use anyhow::{Context, Result};
use serde::Deserialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

use crate::output;

// ── robonix_workspace.yaml ──────────────────────────────────────────────────

/// Top-level structure of `robonix_workspace.yaml`.
#[derive(Debug, Deserialize, Default, Clone)]
pub struct WorkspaceConfig {
    /// Workspace name.
    #[serde(default)]
    pub workspace: Option<String>,
    /// Global environment variables (injected into all child processes).
    #[serde(default)]
    pub env: HashMap<String, String>,
    /// Packages declared in this workspace (url or path, at least one required).
    #[serde(default)]
    pub packages: Vec<WorkspacePackageEntry>,
}

/// A package entry inside `robonix_workspace.yaml`.
#[derive(Debug, Deserialize, Clone)]
pub struct WorkspacePackageEntry {
    /// Fully-qualified package name (e.g. `com.robonix.pkg.memory`).
    pub name: String,
    /// Git URL to clone from (optional).
    #[serde(default)]
    pub url: Option<String>,
    /// Local path relative to workspace root (optional).
    #[serde(default)]
    pub path: Option<String>,
}

// ── deploy/<target>.yaml ────────────────────────────────────────────────────

/// Top-level structure of a deploy config file (`deploy/<target>.yaml`).
#[derive(Debug, Deserialize)]
pub struct DeployConfig {
    /// Path to upstream workspace config (e.g. `../robonix_workspace.yaml`),
    /// resolved relative to this config file's directory.
    #[serde(default)]
    pub upstream_config: Option<String>,
    /// Target platform / profile name (e.g. `sim`, `jetson`).
    #[serde(default)]
    pub target: Option<String>,
    /// Environment variable overrides (merged with workspace env, local wins).
    #[serde(default)]
    pub env: HashMap<String, String>,
    /// Packages and nodes to run.
    /// Format: `"package_name:node_id"` or `"package_name:all"`.
    #[serde(default)]
    pub packages_run: Vec<PackageRunEntry>,
}

/// A single entry in `packages_run`.
#[derive(Debug, Deserialize, Clone)]
pub struct PackageRunEntry {
    /// Format: `<package_name>:<node_id>` or `<package_name>:all`.
    pub name: String,
}

// ── Parsed / resolved types ─────────────────────────────────────────────────

/// Parsed representation of a `PackageRunEntry`.
#[derive(Debug, Clone)]
pub struct ParsedPackageRun {
    pub package_name: String,
    pub node_selector: NodeSelector,
}

/// Which node(s) to start for a given package.
#[derive(Debug, Clone)]
pub enum NodeSelector {
    /// Start every node declared in the manifest.
    All,
    /// Start a single node by id.
    Single(String),
}

/// The merged view produced after loading deploy config + workspace config.
#[derive(Debug)]
pub struct MergedConfig {
    /// Merged environment variables (workspace defaults + deploy overrides).
    pub env: HashMap<String, String>,
    /// Workspace name from `robonix_workspace.yaml`.
    pub workspace_name: Option<String>,
    /// Package declarations from `robonix_workspace.yaml`.
    pub workspace_packages: Vec<WorkspacePackageEntry>,
    /// Parsed `packages_run` from the deploy config.
    pub packages_run: Vec<ParsedPackageRun>,
    /// Target name from the deploy config.
    pub target: Option<String>,
}

// ── Parsing helpers ─────────────────────────────────────────────────────────

/// Parse a raw `PackageRunEntry` (e.g. `"com.pkg:all"`) into a `ParsedPackageRun`.
pub fn parse_package_run(entry: &PackageRunEntry) -> Result<ParsedPackageRun> {
    // Use rsplitn so that the *last* colon is the delimiter.
    // This handles package names that might theoretically contain colons
    // (unlikely, but defensive).
    let parts: Vec<&str> = entry.name.rsplitn(2, ':').collect();
    if parts.len() != 2 {
        anyhow::bail!(
            "invalid packages_run entry '{}': expected format 'package_name:node_id' or 'package_name:all'",
            entry.name
        );
    }
    let node_part = parts[0];
    let pkg_name = parts[1];
    let node_selector = if node_part == "all" {
        NodeSelector::All
    } else {
        NodeSelector::Single(node_part.to_string())
    };
    Ok(ParsedPackageRun {
        package_name: pkg_name.to_string(),
        node_selector,
    })
}

// ── Workspace discovery ─────────────────────────────────────────────────────

/// Find the workspace root by searching upward for `robonix_workspace.yaml`.
pub fn find_workspace_root(base_dir: &Path) -> Result<PathBuf> {
    let mut dir = base_dir.to_path_buf();
    for _ in 0..10 {
        if dir.join("robonix_workspace.yaml").exists() {
            return Ok(dir);
        }
        if !dir.pop() {
            break;
        }
    }
    anyhow::bail!(
        "robonix_workspace.yaml not found (searched from {} upward)",
        base_dir.display()
    );
}

// ── Package resolution (with git clone support) ─────────────────────────────

/// Ensure all workspace-declared packages exist locally.
/// If a package has a `url` but is not found locally, it will be cloned via git.
/// Returns a map of package_name → resolved local path.
pub fn ensure_packages_exist(
    workspace_root: &Path,
    packages: &[WorkspacePackageEntry],
) -> Result<HashMap<String, PathBuf>> {
    let mut package_paths: HashMap<String, PathBuf> = HashMap::new();

    for pkg in packages {
        let local_path = if let Some(ref path) = pkg.path {
            // Explicit path specified — resolve relative to workspace root.
            let resolved = workspace_root.join(path);
            if !resolved.exists() {
                anyhow::bail!(
                    "package '{}' path '{}' does not exist (resolved to {})",
                    pkg.name,
                    path,
                    resolved.display()
                );
            }
            resolved.canonicalize().unwrap_or(resolved)
        } else {
            // No path — check packages/<short_name>, auto-clone if url is set.
            let pkg_dir_name = pkg.name.rsplit('.').next().unwrap_or(&pkg.name);
            let candidate = workspace_root.join("packages").join(pkg_dir_name);
            if !candidate.exists() {
                if let Some(ref url) = pkg.url {
                    output::step("Cloning", &format!("{} from {}", pkg.name, url));
                    // Ensure packages/ directory exists.
                    let packages_dir = workspace_root.join("packages");
                    std::fs::create_dir_all(&packages_dir).with_context(|| {
                        format!("failed to create {}", packages_dir.display())
                    })?;
                    clone_package_to_workspace(url, &candidate)?;
                    output::check(&format!("{} cloned to {}", pkg.name, candidate.display()));
                } else {
                    anyhow::bail!(
                        "package '{}' not found at {} and no url specified",
                        pkg.name,
                        candidate.display()
                    );
                }
            }
            candidate.canonicalize().unwrap_or(candidate)
        };
        output::check(&format!("{} → {}", pkg.name, local_path.display()));
        package_paths.insert(pkg.name.clone(), local_path);
    }
    Ok(package_paths)
}

/// Normalize a git URL: handle short forms like `user/repo`.
fn normalize_git_url(url: &str) -> String {
    let s = url.trim().trim_end_matches('/');
    if s.starts_with("http://") || s.starts_with("https://") || s.starts_with("git@") {
        return s.to_string();
    }
    // Short form: user/repo → https://github.com/user/repo.git
    if s.contains('/') && !s.contains(' ') {
        return format!("https://github.com/{}.git", s.trim_end_matches(".git"));
    }
    s.to_string()
}

/// Clone a git repository to the target directory.
fn clone_package_to_workspace(url: &str, target: &Path) -> Result<()> {
    let clone_url = normalize_git_url(url);
    git2::build::RepoBuilder::new()
        .clone(&clone_url, target)
        .with_context(|| format!("failed to clone {} to {}", url, target.display()))?;
    Ok(())
}
