// SPDX-License-Identifier: MulanPSL-2.0
// Manifest parsing and validation for robonix-cli

use anyhow::{Context, Result};
use serde::Deserialize;
use serde_json::json;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Package manifest file name.
pub const PACKAGE_MANIFEST_FILE: &str = "package_manifest.yaml";

#[derive(Debug, Clone)]
pub struct DetectedManifest {
    pub path: PathBuf,
    pub manifest: Manifest,
}

#[derive(Debug, Clone)]
pub struct PackageSummary {
    pub name: String,
    pub version: String,
    pub provided_interfaces: Vec<String>,
    pub consumed_interfaces: Vec<String>,
    pub nodes: Vec<String>,
    pub capabilities: Vec<String>,
    pub depends: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Manifest {
    #[serde(rename = "manifestVersion")]
    pub manifest_version: u32,
    pub package: Package,
    #[serde(default)]
    pub nodes: Vec<Node>,
    pub interfaces: Option<Interfaces>,
    #[serde(rename = "launchProfiles")]
    pub launch_profiles: Option<HashMap<String, LaunchProfile>>,
    /// Build command string. Supports any shell command, e.g.:
    ///   - `bash scripts/build.sh`
    ///   - `make -j$(nproc)`
    ///   - `cargo build --release`
    ///   - `./build.sh`
    pub build: String,
    /// Capabilities exported by this package (like EXPORT_SYMBOL in Linux kernel modules).
    #[serde(default)]
    pub capabilities: Vec<Capability>,
    /// Package-level dependencies (other packages this package depends on).
    #[serde(default, alias = "depend")]
    pub depends: Vec<DependEntry>,
    /// Start command string. Supports any shell command, e.g.:
    ///   - `bash run.sh`
    ///   - `python -m my_service`
    ///   - `./my_binary --port 8080`
    pub start: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Package {
    /// Package identifier (optional in new format; use `name` as primary key).
    #[serde(default)]
    pub id: Option<String>,
    pub name: String,
    pub version: String,
    pub vendor: String,
    pub description: String,
    pub license: String,
}

/// Capability declaration.
#[derive(Debug, Clone, Deserialize)]
pub struct Capability {
    pub name: String,
    /// Path to a custom capability definition TOML (relative to package root).
    #[serde(default)]
    pub definition: Option<String>,
}

/// Dependency declaration.
#[derive(Debug, Clone, Deserialize)]
pub struct DependEntry {
    pub name: String,
}

impl Package {
    /// Return the effective identifier: `id` if present, otherwise `name`.
    pub fn effective_id(&self) -> &str {
        self.id.as_deref().unwrap_or(&self.name)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct Node {
    pub id: String,
    #[serde(rename = "type")]
    pub node_type: Option<String>,
    /// Shell command for `rbnx start -n <id>` (run from package root via `bash -c`, after env exports).
    #[serde(default)]
    pub start: String,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct Interfaces {
    #[serde(default)]
    pub provides: Vec<InterfaceRef>,
    #[serde(default)]
    pub consumes: Vec<InterfaceRef>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct InterfaceRef {
    pub id: String,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct LaunchProfile {
    #[serde(default, alias = "components")]
    pub nodes: HashMap<String, LaunchNode>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct LaunchNode {
    #[serde(default)]
    pub env: HashMap<String, String>,
}

pub fn detect_manifest_path(package_root: &Path) -> Result<PathBuf> {
    let path = package_root.join(PACKAGE_MANIFEST_FILE);
    if path.exists() {
        return Ok(path);
    }
    anyhow::bail!(
        "Package does not have {}",
        PACKAGE_MANIFEST_FILE,
    )
}

pub fn detect_and_load(package_root: &Path) -> Result<DetectedManifest> {
    let path = detect_manifest_path(package_root)?;
    let manifest = load_from_path(&path)?;
    Ok(DetectedManifest { path, manifest })
}

pub fn load_from_path(manifest_path: &Path) -> Result<Manifest> {
    let content = std::fs::read_to_string(manifest_path)
        .with_context(|| format!("Failed to read manifest: {}", manifest_path.display()))?;
    let manifest: Manifest = serde_yaml::from_str(&content)
        .with_context(|| format!("Failed to parse manifest: {}", manifest_path.display()))?;
    Ok(manifest)
}

impl Manifest {
    pub fn validate_and_summarize(&self) -> Result<PackageSummary> {
        if self.manifest_version == 0 {
            anyhow::bail!("Invalid 'manifestVersion': must be >= 1");
        }
        // package.id is optional in the new format; warn if missing for awareness.
        if self.package.id.as_ref().map_or(true, |id| id.trim().is_empty()) {
            log::debug!(
                "package.id is empty or absent for '{}'; using package.name as identifier",
                self.package.name
            );
        }
        if self.package.name.trim().is_empty() {
            anyhow::bail!("Missing 'package.name' in manifest");
        }
        if self.package.version.trim().is_empty() {
            anyhow::bail!("Missing 'package.version' in manifest");
        }
        if self.package.vendor.trim().is_empty() {
            anyhow::bail!("Missing 'package.vendor' in manifest");
        }
        if self.package.description.trim().is_empty() {
            anyhow::bail!("Missing 'package.description' in manifest");
        }
        if self.package.license.trim().is_empty() {
            anyhow::bail!("Missing 'package.license' in manifest");
        }

        validate_node_specs(self)?;

        if self.build.trim().is_empty() {
            anyhow::bail!("'build' is required in package_manifest.yaml");
        }
        if self.start.trim().is_empty() {
            anyhow::bail!("'start' is required in package_manifest.yaml");
        }

        let interfaces = self.interfaces.clone().unwrap_or_default();

        Ok(PackageSummary {
            name: self.package.name.clone(),
            version: self.package.version.clone(),
            provided_interfaces: interfaces.provides.into_iter().map(|i| i.id).collect(),
            consumed_interfaces: interfaces.consumes.into_iter().map(|i| i.id).collect(),
            nodes: self.nodes.iter().map(|n| n.id.clone()).collect(),
            capabilities: self.capabilities.iter().map(|c| c.name.clone()).collect(),
            depends: self.depends.iter().map(|d| d.name.clone()).collect(),
        })
    }
}

fn validate_node_specs(manifest: &Manifest) -> Result<()> {
    for n in &manifest.nodes {
        if n.start.trim().is_empty() {
            anyhow::bail!("Node '{}': missing or empty `start`", n.id);
        }
    }
    Ok(())
}

// ── Skills scanner (skills/ directory) ───────────────────────

#[derive(Debug, Clone)]
pub struct SkillMeta {
    pub name: String,
    pub description: String,
    pub path: PathBuf,
    /// Flags as JSON (`disable_model_invocation`, `user_invocable`, …).
    pub metadata_json: String,
}

/// Scan `<package_root>/skills/` for subdirectories containing `SKILL.md`.
/// Each SKILL.md must have YAML frontmatter with `name` and `description`.
pub fn scan_skills(package_root: &Path) -> Vec<SkillMeta> {
    let skills_dir = package_root.join("skills");
    if !skills_dir.is_dir() {
        return Vec::new();
    }
    let mut out = Vec::new();
    let entries = match std::fs::read_dir(&skills_dir) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };
    for entry in entries.flatten() {
        if !entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
            continue;
        }
        let skill_md = entry.path().join("SKILL.md");
        if !skill_md.is_file() {
            continue;
        }
        match parse_skill_frontmatter(&skill_md) {
            Ok(meta) => out.push(meta),
            Err(e) => {
                log::warn!("skipping {}: {}", skill_md.display(), e);
            }
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

/// Parse YAML frontmatter (between `---` delimiters) from a SKILL.md file.
fn parse_skill_frontmatter(path: &Path) -> Result<SkillMeta> {
    let content = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;

    let trimmed = content.trim_start();
    if !trimmed.starts_with("---") {
        anyhow::bail!("SKILL.md must start with YAML frontmatter (---)");
    }
    let after_first = &trimmed[3..];
    let end = after_first
        .find("\n---")
        .ok_or_else(|| anyhow::anyhow!("SKILL.md: missing closing --- for frontmatter"))?;
    let yaml_block = &after_first[..end];

    #[derive(Deserialize)]
    struct FrontMatter {
        name: String,
        description: String,
        #[serde(default, rename = "disable-model-invocation")]
        disable_model_invocation: Option<bool>,
        #[serde(default, rename = "user-invocable")]
        user_invocable: Option<bool>,
    }
    let fm: FrontMatter = serde_yaml::from_str(yaml_block)
        .with_context(|| format!("invalid YAML frontmatter in {}", path.display()))?;

    if fm.name.trim().is_empty() {
        anyhow::bail!("SKILL.md frontmatter: `name` must not be empty");
    }

    let metadata_json = json!({
        "disable_model_invocation": fm.disable_model_invocation.unwrap_or(false),
        "user_invocable": fm.user_invocable.unwrap_or(true),
    })
    .to_string();

    let abs_path = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
    Ok(SkillMeta {
        name: fm.name,
        description: fm.description,
        path: abs_path,
        metadata_json,
    })
}
