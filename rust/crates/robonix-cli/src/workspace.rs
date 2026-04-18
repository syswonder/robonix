// SPDX-License-Identifier: MulanPSL-2.0
// Workspace configuration types shared by deploy and build commands.
//
// Defines the Rust structs that map to:
//   - robonix_workspace.yaml  (WorkspaceConfig)
//   - deploy/<target>.yaml    (DeployConfig)

use anyhow::Result;
use serde::Deserialize;
use std::collections::HashMap;

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
