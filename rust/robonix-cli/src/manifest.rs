// SPDX-License-Identifier: MulanPSL-2.0
// Manifest parsing and validation for robonix-cli

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use serde_yaml::Value;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

pub const VNEXT_MANIFEST_FILE: &str = "robonix_manifest.yaml";
pub const LEGACY_MANIFEST_FILE: &str = "rbnx_manifest.yaml";

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum ManifestKind {
    #[default]
    LegacyRbnx,
    VNext,
}

impl ManifestKind {
    pub fn label(&self) -> &'static str {
        match self {
            ManifestKind::LegacyRbnx => "legacy-rbnx",
            ManifestKind::VNext => "vnext",
        }
    }
}

#[derive(Debug, Clone)]
pub struct DetectedManifest {
    pub path: PathBuf,
    pub manifest: PackageManifest,
}

#[derive(Debug, Clone)]
pub enum PackageManifest {
    Legacy(LegacyManifest),
    VNext(VNextManifest),
}

#[derive(Debug, Clone)]
pub enum BuildStrategy {
    LegacyScript { script: Option<String> },
    VNextCommand { command: String, workspace_root: Option<String> },
    None,
}

#[derive(Debug, Clone)]
pub struct PackageSummary {
    pub name: String,
    pub version: String,
    pub manifest_kind: ManifestKind,
    pub primitives: Vec<String>,
    pub services: Vec<String>,
    pub skills: Vec<String>,
    pub provided_interfaces: Vec<String>,
    pub consumed_interfaces: Vec<String>,
    pub nodes: Vec<String>,
    pub build_strategy: BuildStrategy,
}

#[derive(Debug, Clone)]
pub struct InterfaceCatalogCheck {
    pub catalog_root: Option<PathBuf>,
    pub checked_interfaces: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LegacyManifest {
    pub package: LegacyPackage,
    #[serde(default)]
    pub primitives: Vec<LegacyCapability>,
    #[serde(default)]
    pub services: Vec<LegacyCapability>,
    #[serde(default)]
    pub skills: Vec<LegacyCapability>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LegacyPackage {
    pub name: String,
    pub version: String,
    pub build_script: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LegacyCapability {
    pub name: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VNextManifest {
    #[serde(rename = "manifestVersion")]
    pub manifest_version: u32,
    pub package: VNextPackage,
    pub build: Option<VNextBuild>,
    #[serde(default)]
    pub nodes: Vec<VNextNode>,
    pub interfaces: Option<VNextInterfaces>,
    pub config: Option<VNextConfig>,
    #[serde(rename = "launchProfiles")]
    pub launch_profiles: Option<HashMap<String, VNextLaunchProfile>>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VNextPackage {
    pub id: String,
    pub name: String,
    pub version: String,
    pub vendor: String,
    pub description: String,
    pub license: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VNextBuild {
    pub command: Option<String>,
    #[serde(rename = "workspaceRoot")]
    pub workspace_root: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VNextNode {
    pub id: String,
    #[serde(rename = "type")]
    pub node_type: Option<String>,
    pub entry: Option<String>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct VNextInterfaces {
    #[serde(default)]
    pub provides: Vec<VNextInterfaceRef>,
    #[serde(default)]
    pub consumes: Vec<VNextInterfaceRef>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VNextInterfaceRef {
    pub id: String,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct VNextConfig {
    #[serde(default)]
    pub schema: HashMap<String, VNextConfigField>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct VNextConfigField {
    #[serde(rename = "type")]
    pub field_type: Option<String>,
    pub default: Option<Value>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct VNextLaunchProfile {
    #[serde(default, alias = "components")]
    pub nodes: HashMap<String, VNextLaunchNode>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct VNextLaunchNode {
    #[serde(default)]
    pub env: HashMap<String, String>,
}

pub fn detect_manifest_path(package_root: &Path) -> Result<PathBuf> {
    let vnext_path = package_root.join(VNEXT_MANIFEST_FILE);
    if vnext_path.exists() {
        return Ok(vnext_path);
    }

    let legacy_path = package_root.join(LEGACY_MANIFEST_FILE);
    if legacy_path.exists() {
        return Ok(legacy_path);
    }

    anyhow::bail!(
        "Package does not have {} or {}",
        VNEXT_MANIFEST_FILE,
        LEGACY_MANIFEST_FILE
    );
}

pub fn detect_and_load(package_root: &Path) -> Result<DetectedManifest> {
    let path = detect_manifest_path(package_root)?;
    let manifest = load_from_path(&path)?;
    Ok(DetectedManifest { path, manifest })
}

pub fn load_from_path(manifest_path: &Path) -> Result<PackageManifest> {
    let content = std::fs::read_to_string(manifest_path)
        .with_context(|| format!("Failed to read manifest: {}", manifest_path.display()))?;

    match manifest_path.file_name().and_then(|name| name.to_str()) {
        Some(VNEXT_MANIFEST_FILE) => {
            let manifest: VNextManifest = serde_yaml::from_str(&content).with_context(|| {
                format!("Failed to parse manifest: {}", manifest_path.display())
            })?;
            Ok(PackageManifest::VNext(manifest))
        }
        Some(LEGACY_MANIFEST_FILE) => {
            let manifest: LegacyManifest = serde_yaml::from_str(&content).with_context(|| {
                format!("Failed to parse manifest: {}", manifest_path.display())
            })?;
            Ok(PackageManifest::Legacy(manifest))
        }
        _ => anyhow::bail!("Unsupported manifest filename: {}", manifest_path.display()),
    }
}

impl PackageManifest {
    pub fn validate_and_summarize(&self) -> Result<PackageSummary> {
        match self {
            PackageManifest::Legacy(manifest) => validate_legacy(manifest),
            PackageManifest::VNext(manifest) => validate_vnext(manifest),
        }
    }
}

pub fn validate_interface_references(
    summary: &PackageSummary,
    package_root: &Path,
) -> Result<InterfaceCatalogCheck> {
    let interface_ids: Vec<String> = summary
        .provided_interfaces
        .iter()
        .chain(summary.consumed_interfaces.iter())
        .cloned()
        .collect();

    if interface_ids.is_empty() {
        return Ok(InterfaceCatalogCheck {
            catalog_root: find_interface_catalog_root(package_root),
            checked_interfaces: Vec::new(),
        });
    }

    let Some(catalog_root) = find_interface_catalog_root(package_root) else {
        return Ok(InterfaceCatalogCheck {
            catalog_root: None,
            checked_interfaces: interface_ids,
        });
    };

    for interface_id in &interface_ids {
        let ridl_path = interface_id_to_ridl_path(&catalog_root, interface_id);
        if !ridl_path.exists() {
            anyhow::bail!(
                "Interface '{}' was not found under interface catalog root {}",
                interface_id,
                catalog_root.display()
            );
        }
    }

    Ok(InterfaceCatalogCheck {
        catalog_root: Some(catalog_root),
        checked_interfaces: interface_ids,
    })
}

fn validate_legacy(manifest: &LegacyManifest) -> Result<PackageSummary> {
    if manifest.package.name.trim().is_empty() {
        anyhow::bail!("Missing 'package.name' in manifest");
    }
    if manifest.package.version.trim().is_empty() {
        anyhow::bail!("Missing 'package.version' in manifest");
    }

    Ok(PackageSummary {
        name: manifest.package.name.clone(),
        version: manifest.package.version.clone(),
        manifest_kind: ManifestKind::LegacyRbnx,
        primitives: manifest.primitives.iter().map(|item| item.name.clone()).collect(),
        services: manifest.services.iter().map(|item| item.name.clone()).collect(),
        skills: manifest.skills.iter().map(|item| item.name.clone()).collect(),
        provided_interfaces: Vec::new(),
        consumed_interfaces: Vec::new(),
        nodes: Vec::new(),
        build_strategy: BuildStrategy::LegacyScript {
            script: manifest.package.build_script.clone(),
        },
    })
}

fn validate_vnext(manifest: &VNextManifest) -> Result<PackageSummary> {
    if manifest.manifest_version == 0 {
        anyhow::bail!("Invalid 'manifestVersion': must be >= 1");
    }
    if manifest.package.id.trim().is_empty() {
        anyhow::bail!("Missing 'package.id' in manifest");
    }
    if manifest.package.name.trim().is_empty() {
        anyhow::bail!("Missing 'package.name' in manifest");
    }
    if manifest.package.version.trim().is_empty() {
        anyhow::bail!("Missing 'package.version' in manifest");
    }
    if manifest.package.vendor.trim().is_empty() {
        anyhow::bail!("Missing 'package.vendor' in manifest");
    }
    if manifest.package.description.trim().is_empty() {
        anyhow::bail!("Missing 'package.description' in manifest");
    }
    if manifest.package.license.trim().is_empty() {
        anyhow::bail!("Missing 'package.license' in manifest");
    }

    let interfaces = manifest.interfaces.clone().unwrap_or_default();
    if manifest.nodes.is_empty() && interfaces.provides.is_empty() && interfaces.consumes.is_empty() {
        anyhow::bail!("Manifest must declare at least one node or interface");
    }

    let build_strategy = if let Some(build) = &manifest.build {
        if let Some(command) = &build.command {
            BuildStrategy::VNextCommand {
                command: command.clone(),
                workspace_root: build.workspace_root.clone(),
            }
        } else {
            BuildStrategy::None
        }
    } else {
        BuildStrategy::None
    };

    Ok(PackageSummary {
        name: manifest.package.name.clone(),
        version: manifest.package.version.clone(),
        manifest_kind: ManifestKind::VNext,
        primitives: Vec::new(),
        services: Vec::new(),
        skills: Vec::new(),
        provided_interfaces: interfaces.provides.into_iter().map(|item| item.id).collect(),
        consumed_interfaces: interfaces.consumes.into_iter().map(|item| item.id).collect(),
        nodes: manifest.nodes.iter().map(|item| item.id.clone()).collect(),
        build_strategy,
    })
}

fn find_interface_catalog_root(package_root: &Path) -> Option<PathBuf> {
    for ancestor in package_root.ancestors() {
        let candidate = ancestor.join("robonix-interfaces").join("ridl");
        if candidate.exists() {
            return Some(candidate);
        }
    }
    None
}

fn interface_id_to_ridl_path(catalog_root: &Path, interface_id: &str) -> PathBuf {
    let rel = interface_id
        .trim()
        .trim_start_matches("robonix/")
        .trim_start_matches('/');
    catalog_root.join(format!("{}.ridl", rel))
}
