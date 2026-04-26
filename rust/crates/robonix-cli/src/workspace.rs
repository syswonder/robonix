// SPDX-License-Identifier: MulanPSL-2.0
// Runtime configuration types for robonix-cli
//
// This module defines the runtime configuration loaded from `robonix_manifest.yaml`,
// which is the single entry-point for a Robonix project. It declares system components,
// primitives, services, and skills.

use anyhow::{Context, Result};
use serde::Deserialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// File name of the runtime manifest (project entry-point).
pub const RUNTIME_MANIFEST_FILE: &str = "robonix_manifest.yaml";

// ════════════════════════════════════════════════════════════════
// Runtime config: robonix_manifest.yaml
// ════════════════════════════════════════════════════════════════

/// Top-level structure of `robonix_manifest.yaml` — the single project entry-point.
#[derive(Debug, Deserialize, Default, Clone)]
pub struct RuntimeConfig {
    /// Scene / deployment name (e.g. "fetch-demo").
    #[serde(default)]
    pub name: String,
    /// Global environment variables injected into all child processes.
    #[serde(default)]
    pub env: HashMap<String, String>,
    /// System service declarations (core components: nexus, atlas, executor, pilot, liaison).
    #[serde(default)]
    pub system: SystemConfig,
    /// Hardware-bound driver modules (stored in `primitives/` directory).
    #[serde(default)]
    pub primitives: Vec<RuntimePackageEntry>,
    /// Hardware-independent persistent services (stored in `services/` directory).
    #[serde(default)]
    pub services: Vec<RuntimePackageEntry>,
    /// On-demand skill units (stored in `skills/` directory).
    #[serde(default)]
    pub skills: Vec<RuntimePackageEntry>,
}

/// System core component configuration.
/// Each component is optional; if present (even as `{}`), it will be started.
#[derive(Debug, Deserialize, Default, Clone)]
pub struct SystemConfig {
    #[serde(default)]
    pub nexus: Option<ComponentConfig>,
    #[serde(default)]
    pub atlas: Option<ComponentConfig>,
    #[serde(default)]
    pub executor: Option<ComponentConfig>,
    #[serde(default)]
    pub pilot: Option<PilotConfig>,
    #[serde(default)]
    pub liaison: Option<ComponentConfig>,
}

/// Generic system component configuration (atlas, executor, liaison, nexus).
#[derive(Debug, Deserialize, Default, Clone)]
pub struct ComponentConfig {
    /// Listen endpoint (e.g. "127.0.0.1:50051"). Uses built-in default if absent.
    #[serde(default)]
    pub endpoint: Option<String>,
}

/// Pilot-specific configuration.
#[derive(Debug, Deserialize, Default, Clone)]
pub struct PilotConfig {
    /// Listen endpoint (e.g. "127.0.0.1:50071"). Uses built-in default if absent.
    #[serde(default)]
    pub endpoint: Option<String>,
    #[serde(default)]
    pub vlm_base_url: Option<String>,
    #[serde(default)]
    pub vlm_api_key: Option<String>,
    #[serde(default)]
    pub vlm_api_format: Option<String>,
}

/// Entry in the `primitives`, `services`, or `skills` arrays of `robonix_manifest.yaml`.
#[derive(Debug, Deserialize, Clone)]
pub struct RuntimePackageEntry {
    /// Fully-qualified package name (e.g. "com.realsense.rs_driver").
    pub package: String,
    /// Local path relative to project root (e.g. "./primitives/my_driver"). Optional.
    #[serde(default)]
    pub path: Option<String>,
    /// Git URL for auto-clone when the package is not found locally. Optional.
    #[serde(default)]
    pub url: Option<String>,
    /// Instance name (e.g. "external_cam", "slam").
    pub name: String,
    /// Runtime parameters passed to the package.
    #[serde(default)]
    pub config: HashMap<String, serde_yaml::Value>,
}

// ════════════════════════════════════════════════════════════════
// Loading helpers
// ════════════════════════════════════════════════════════════════

/// Load and parse the runtime config from a `robonix_manifest.yaml` file.
pub fn load_runtime_config(path: &Path) -> Result<RuntimeConfig> {
    let content = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    let cfg: RuntimeConfig = serde_yaml::from_str(&content)
        .with_context(|| format!("failed to parse {}", path.display()))?;
    Ok(cfg)
}

// ════════════════════════════════════════════════════════════════
// Package path resolution (shared by deploy & build)
// ════════════════════════════════════════════════════════════════

/// Resolve the local filesystem path of a package declared in the runtime config.
///
/// Resolution order:
/// 1. If `entry.path` is set and starts with `./` or `../` — resolve relative to `project_root`.
/// 2. Look in `<project_root>/<role_dir>/<short_name>`.
/// 3. If not found and `entry.url` is set — git clone into `<project_root>/<role_dir>/<short_name>`.
/// 4. Otherwise — error.
pub fn resolve_package_path(
    project_root: &Path,
    role_dir: &str,
    entry: &RuntimePackageEntry,
) -> Result<PathBuf> {
    // 1. Explicit local path.
    if let Some(ref path_str) = entry.path
        && (path_str.starts_with("./") || path_str.starts_with("../"))
    {
        let resolved = project_root.join(path_str);
        if resolved.exists() {
            return Ok(resolved.canonicalize().unwrap_or(resolved));
        }
        anyhow::bail!(
            "package '{}' local path '{}' not found (resolved to {})",
            entry.name,
            path_str,
            resolved.display()
        );
    }

    // 2. Convention: <project_root>/<role_dir>/<short_name>
    let short_name = entry.package.rsplit('.').next().unwrap_or(&entry.package);
    let candidate = project_root.join(role_dir).join(short_name);
    if candidate.exists() {
        return Ok(candidate.canonicalize().unwrap_or(candidate));
    }

    // 3. Auto git-clone if url is provided.
    if let Some(ref url) = entry.url {
        crate::output::step("Cloning", &format!("{} from {}", entry.name, url));
        let clone_url = normalize_git_url(url);
        git2::build::RepoBuilder::new()
            .clone(&clone_url, &candidate)
            .with_context(|| format!("failed to clone '{}' to {}", url, candidate.display()))?;
        crate::output::check(&format!("{} cloned to {}", entry.name, candidate.display()));
        return Ok(candidate);
    }

    // 4. Nothing found.
    anyhow::bail!(
        "package '{}' not found at {} and no url specified for auto-clone",
        entry.name,
        candidate.display(),
    )
}

/// Normalize a git URL: bare `user/repo` → `https://github.com/user/repo.git`.
fn normalize_git_url(url: &str) -> String {
    let s = url.trim().trim_end_matches('/');
    if s.starts_with("http://") || s.starts_with("https://") || s.starts_with("git@") {
        return s.to_string();
    }
    if s.contains('/') && !s.contains(' ') {
        return format!("https://github.com/{}.git", s.trim_end_matches(".git"));
    }
    s.to_string()
}

/// Build stamp file name within a project's `_build/` directory.
pub const BUILD_STAMP_DIR: &str = "_build";

/// Walk up from `base_dir` looking for `robonix_manifest.yaml` (the project root).
pub fn find_project_root(base_dir: &Path) -> Result<PathBuf> {
    let mut dir = if base_dir.is_absolute() {
        base_dir.to_path_buf()
    } else {
        base_dir
            .canonicalize()
            .unwrap_or_else(|_| base_dir.to_path_buf())
    };
    for _ in 0..10 {
        if dir.join(RUNTIME_MANIFEST_FILE).exists() {
            return Ok(dir);
        }
        if !dir.pop() {
            break;
        }
    }
    anyhow::bail!(
        "{} not found (searched from {} upward)",
        RUNTIME_MANIFEST_FILE,
        base_dir.display()
    )
}

// ════════════════════════════════════════════════════════════════
// Topological sort (shared by deploy & build)
// ════════════════════════════════════════════════════════════════

/// Topological sort over items identified by name with dependency lists.
///
/// Input: a slice of `(name, &[dependency_name])` pairs.
/// Output: a `Vec<usize>` of indices into the input slice, in dependency order
/// (items with no dependencies first).
///
/// Dependencies that reference names **not present** in the input slice are
/// silently ignored (they may be satisfied by a different layer, e.g. a
/// primitive depended on by a service).
///
/// Returns an error if a cycle is detected among the items in the input.
///
/// # Example
/// ```ignore
/// let items = vec![
///     ("B", vec!["A".to_string()]),
///     ("A", vec![]),
///     ("C", vec!["B".to_string()]),
/// ];
/// let refs: Vec<(&str, &[String])> = items.iter().map(|(n, d)| (*n, d.as_slice())).collect();
/// let order = topo_sort(&refs)?;
/// // order == [1, 0, 2]  →  A, B, C
/// ```
pub fn topo_sort(items: &[(&str, &[String])]) -> Result<Vec<usize>> {
    let n = items.len();
    // Map name → index for items in this set.
    let idx_of: HashMap<&str, usize> = items
        .iter()
        .enumerate()
        .map(|(i, (name, _))| (*name, i))
        .collect();

    // Build adjacency list + in-degree (only for edges within this set).
    let mut in_degree = vec![0u32; n];
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];

    for (i, (_name, deps)) in items.iter().enumerate() {
        for dep in *deps {
            if let Some(&j) = idx_of.get(dep.as_str()) {
                // j → i  (j must come before i)
                adj[j].push(i);
                in_degree[i] += 1;
            }
            // deps referencing names outside this set are ignored
        }
    }

    // Kahn's algorithm.
    let mut queue: std::collections::VecDeque<usize> = in_degree
        .iter()
        .enumerate()
        .filter(|&(_, &d)| d == 0)
        .map(|(i, _)| i)
        .collect();

    let mut order = Vec::with_capacity(n);
    while let Some(u) = queue.pop_front() {
        order.push(u);
        for &v in &adj[u] {
            in_degree[v] -= 1;
            if in_degree[v] == 0 {
                queue.push_back(v);
            }
        }
    }

    if order.len() != n {
        // Find cycle members for a useful error message.
        let cycle_members: Vec<&str> = (0..n)
            .filter(|i| in_degree[*i] > 0)
            .map(|i| items[i].0)
            .collect();
        anyhow::bail!(
            "circular dependency detected among: {}",
            cycle_members.join(", ")
        );
    }

    Ok(order)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topo_sort_linear() {
        // A → B → C
        let deps_c = vec!["B".to_string()];
        let deps_b = vec!["A".to_string()];
        let items: Vec<(&str, &[String])> = vec![
            ("C", deps_c.as_slice()),
            ("A", &[]),
            ("B", deps_b.as_slice()),
        ];
        let order = topo_sort(&items).unwrap();
        // A(1) must come before B(2), B(2) must come before C(0).
        let pos: HashMap<usize, usize> = order
            .iter()
            .enumerate()
            .map(|(pos, &idx)| (idx, pos))
            .collect();
        assert!(pos[&1] < pos[&2]); // A before B
        assert!(pos[&2] < pos[&0]); // B before C
    }

    #[test]
    fn topo_sort_no_deps() {
        let items: Vec<(&str, &[String])> = vec![("X", &[]), ("Y", &[]), ("Z", &[])];
        let order = topo_sort(&items).unwrap();
        assert_eq!(order.len(), 3);
    }

    #[test]
    fn topo_sort_external_dep_ignored() {
        // B depends on "external" which is not in the set → ignored.
        let deps_b = vec!["external".to_string()];
        let items: Vec<(&str, &[String])> = vec![("A", &[]), ("B", deps_b.as_slice())];
        let order = topo_sort(&items).unwrap();
        assert_eq!(order.len(), 2);
    }

    #[test]
    fn topo_sort_cycle_detected() {
        let deps_a = vec!["B".to_string()];
        let deps_b = vec!["A".to_string()];
        let items: Vec<(&str, &[String])> =
            vec![("A", deps_a.as_slice()), ("B", deps_b.as_slice())];
        let err = topo_sort(&items).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("circular dependency"), "got: {}", msg);
    }

    #[test]
    fn topo_sort_diamond() {
        //   A
        //  / \
        // B   C
        //  \ /
        //   D
        let deps_b = vec!["A".to_string()];
        let deps_c = vec!["A".to_string()];
        let deps_d = vec!["B".to_string(), "C".to_string()];
        let items: Vec<(&str, &[String])> = vec![
            ("D", deps_d.as_slice()),
            ("B", deps_b.as_slice()),
            ("A", &[]),
            ("C", deps_c.as_slice()),
        ];
        let order = topo_sort(&items).unwrap();
        let pos: HashMap<usize, usize> = order
            .iter()
            .enumerate()
            .map(|(pos, &idx)| (idx, pos))
            .collect();
        assert!(pos[&2] < pos[&1]); // A before B
        assert!(pos[&2] < pos[&3]); // A before C
        assert!(pos[&1] < pos[&0]); // B before D
        assert!(pos[&3] < pos[&0]); // C before D
    }
}
