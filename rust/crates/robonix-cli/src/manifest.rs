// SPDX-License-Identifier: MulanPSL-2.0
// Manifest parsing and validation for robonix-cli.
//
// New (dev-packaging) spec: `package_manifest.yaml` with a single top-level
// `build` + `start` shell string, a list of `capabilities`, optional
// `depends`. One package = one `start` body — no `nodes: [...]` list, no
// `-n` flag.
//
// Legacy (pre-dev-packaging) spec still accepted for backward compatibility:
//   - filename `robonix_manifest.yaml`
//   - `package.id` (used as identity if `name` is missing)
//   - `build: { script: <path> }` instead of top-level `build: <string>`
//   - `nodes: [{id, type, start}, ...]` instead of top-level `start: <string>`
//     → node `start` blocks are concatenated and run as a single background
//       process group (best-effort; use the new spec for deterministic order).
//
// `node` / `runtime` terminology is gone from the spec — "node" is deprecated
// in favour of "capability"; "runtime" is "system".

use anyhow::{Context, Result};
use serde::Deserialize;
use serde_json::json;
use std::path::{Path, PathBuf};

/// Preferred per-package manifest filename. Legacy `robonix_manifest.yaml`
/// is also accepted by [`detect_manifest_path`].
pub const MANIFEST_FILE: &str = "package_manifest.yaml";
pub const LEGACY_MANIFEST_FILE: &str = "robonix_manifest.yaml";

#[derive(Debug, Clone)]
pub struct DetectedManifest {
    pub path: PathBuf,
    pub manifest: Manifest,
}

#[derive(Debug, Clone)]
pub struct PackageSummary {
    pub name: String,
    pub version: String,
    pub capabilities: Vec<String>,
    pub depends: Vec<String>,
}

#[derive(Debug, Clone, Default)]
pub struct Manifest {
    pub manifest_version: u32,
    pub package: Package,
    pub build: String,
    pub start: String,
    pub capabilities: Vec<CapabilityRef>,
    pub depends: Vec<DependsRef>,
    /// True iff the manifest was parsed from legacy fields (id/nodes/build.script).
    /// `rbnx` prints a deprecation warning in this case.
    pub is_legacy: bool,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct Package {
    /// New spec: `package.name`. Legacy spec allowed a separate `package.id`
    /// — if present and `name` is missing, we fall back to `id`.
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub id: Option<String>,
    #[serde(default)]
    pub version: String,
    #[serde(default)]
    pub vendor: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub license: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct CapabilityRef {
    /// Contract id (matches one of the TOMLs under `capabilities/`).
    pub name: String,
    /// Optional path to a package-local TOML that overrides / defines
    /// this capability (for experimental caps not yet in the official
    /// contracts directory). Relative to the package root.
    #[serde(default, alias = "definition")]
    pub path: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DependsRef {
    pub name: String,
}

// ── raw shape accepting both old and new ────────────────────────────────

#[derive(Debug, Clone, Deserialize, Default)]
struct RawManifest {
    #[serde(rename = "manifestVersion", default)]
    manifest_version: u32,
    #[serde(default)]
    package: Package,
    /// Accept either a plain shell string (new) or `{ script: <path> }` (legacy).
    #[serde(default)]
    build: Option<BuildField>,
    /// New: top-level shell string.
    #[serde(default)]
    start: Option<String>,
    /// Legacy: list of nodes, each with its own `start` block.
    #[serde(default)]
    nodes: Vec<LegacyNode>,
    #[serde(default)]
    capabilities: Vec<CapabilityRef>,
    #[serde(default)]
    depends: Vec<DependsRef>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
enum BuildField {
    Shell(String),
    Script { script: String },
}

#[derive(Debug, Clone, Deserialize)]
struct LegacyNode {
    #[serde(default)]
    #[allow(dead_code)]
    id: String,
    #[serde(default, rename = "type")]
    #[allow(dead_code)]
    node_type: Option<String>,
    #[serde(default)]
    start: String,
}

pub fn detect_manifest_path(package_root: &Path) -> Result<PathBuf> {
    let new_path = package_root.join(MANIFEST_FILE);
    if new_path.exists() {
        return Ok(new_path);
    }
    let legacy = package_root.join(LEGACY_MANIFEST_FILE);
    if legacy.exists() {
        return Ok(legacy);
    }
    anyhow::bail!("Package does not have {MANIFEST_FILE} (or legacy {LEGACY_MANIFEST_FILE})")
}

pub fn detect_and_load(package_root: &Path) -> Result<DetectedManifest> {
    let path = detect_manifest_path(package_root)?;
    let manifest = load_from_path(&path)?;
    Ok(DetectedManifest { path, manifest })
}

pub fn load_from_path(manifest_path: &Path) -> Result<Manifest> {
    let content = std::fs::read_to_string(manifest_path)
        .with_context(|| format!("Failed to read manifest: {}", manifest_path.display()))?;
    let raw: RawManifest = serde_yaml::from_str(&content)
        .with_context(|| format!("Failed to parse manifest: {}", manifest_path.display()))?;
    Ok(normalize(raw, manifest_path))
}

fn normalize(raw: RawManifest, manifest_path: &Path) -> Manifest {
    let mut is_legacy = false;
    let filename_is_legacy = manifest_path
        .file_name()
        .and_then(|n| n.to_str())
        .map(|s| s == LEGACY_MANIFEST_FILE)
        .unwrap_or(false);

    // package.name fallback to package.id (legacy used id as canonical name).
    let mut package = raw.package;
    if package.name.trim().is_empty() {
        if let Some(id) = &package.id {
            if !id.trim().is_empty() {
                package.name = id.clone();
                is_legacy = true;
            }
        }
    }

    // build: string (new) or { script } (legacy).
    let build = match raw.build {
        Some(BuildField::Shell(s)) => s,
        Some(BuildField::Script { script }) => {
            is_legacy = true;
            format!("bash {script}")
        }
        None => String::new(),
    };

    // start: top-level string (new) or concatenate nodes (legacy).
    let start = match (raw.start, raw.nodes.is_empty()) {
        (Some(s), _) => s,
        (None, false) => {
            is_legacy = true;
            // Concatenate legacy node start blocks. Each block gets wrapped
            // in a subshell and backgrounded; a `wait` at the end keeps
            // `rbnx start` alive until all nodes exit. This is a best-effort
            // port — for deterministic deploys, migrate to the new spec.
            let parts: Vec<String> = raw
                .nodes
                .iter()
                .filter(|n| !n.start.trim().is_empty())
                .map(|n| format!("( {} ) &", n.start.trim()))
                .collect();
            if parts.is_empty() {
                String::new()
            } else {
                format!("{}\nwait", parts.join("\n"))
            }
        }
        (None, true) => String::new(),
    };

    if filename_is_legacy {
        is_legacy = true;
    }

    Manifest {
        manifest_version: raw.manifest_version,
        package,
        build,
        start,
        capabilities: raw.capabilities,
        depends: raw.depends,
        is_legacy,
    }
}

impl Manifest {
    pub fn validate_and_summarize(&self) -> Result<PackageSummary> {
        if self.manifest_version == 0 {
            anyhow::bail!("Invalid 'manifestVersion': must be >= 1");
        }
        let p = &self.package;
        for (name, val) in [
            ("package.name", &p.name),
            ("package.version", &p.version),
            ("package.vendor", &p.vendor),
            ("package.description", &p.description),
            ("package.license", &p.license),
        ] {
            if val.trim().is_empty() {
                anyhow::bail!("Missing '{name}' in manifest");
            }
        }
        // `build` remains optional — packages that ship pre-built binaries
        // can omit it. `start` is required for `rbnx start` to do anything.
        if self.start.trim().is_empty() {
            anyhow::bail!(
                "manifest.start is required (shell string, run at package root). \
                 If migrating from the legacy spec with `nodes: [...]`, either \
                 concatenate their start blocks into one top-level `start:` or \
                 split into multiple packages."
            );
        }

        if self.is_legacy {
            log::warn!(
                "package '{}' uses legacy manifest fields (package.id / nodes[] / \
                 build.script / robonix_manifest.yaml). These still work but are \
                 deprecated — migrate to the new spec (package_manifest.yaml with \
                 top-level build/start strings + capabilities).",
                self.package.name
            );
        }

        Ok(PackageSummary {
            name: p.name.clone(),
            version: p.version.clone(),
            capabilities: self.capabilities.iter().map(|c| c.name.clone()).collect(),
            depends: self.depends.iter().map(|d| d.name.clone()).collect(),
        })
    }
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
