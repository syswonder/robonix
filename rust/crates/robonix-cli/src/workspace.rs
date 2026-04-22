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
#[derive(Debug, Deserialize, Default, Clone)]
pub struct SystemConfig {
    #[serde(default)]
    pub nexus: Option<serde_yaml::Value>,
    #[serde(default)]
    pub atlas: Option<serde_yaml::Value>,
    #[serde(default)]
    pub executor: Option<serde_yaml::Value>,
    #[serde(default)]
    pub pilot: Option<PilotConfig>,
    #[serde(default)]
    pub liaison: Option<serde_yaml::Value>,
}

/// Pilot-specific configuration.
#[derive(Debug, Deserialize, Clone)]
pub struct PilotConfig {
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
    /// Local path (relative to project root) or git URL.
    pub path: String,
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
