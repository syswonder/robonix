// SPDX-License-Identifier: MulanPSL-2.0
// Manifest parsing and validation for robonix-cli

use anyhow::{Context, Result};
use serde::Deserialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

pub const MANIFEST_FILE: &str = "robonix_manifest.yaml";

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
}

#[derive(Debug, Clone)]
pub struct InterfaceCatalogCheck {
    pub catalog_root: Option<PathBuf>,
    pub checked_interfaces: Vec<String>,
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
}

#[derive(Debug, Clone, Deserialize)]
pub struct Package {
    pub id: String,
    pub name: String,
    pub version: String,
    pub vendor: String,
    pub description: String,
    pub license: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Node {
    pub id: String,
    #[serde(rename = "type")]
    pub node_type: Option<String>,
    pub entry: Option<String>,
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
    let path = package_root.join(MANIFEST_FILE);
    if path.exists() {
        Ok(path)
    } else {
        anyhow::bail!("Package does not have {}", MANIFEST_FILE)
    }
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
        if self.package.id.trim().is_empty() {
            anyhow::bail!("Missing 'package.id' in manifest");
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

        let interfaces = self.interfaces.clone().unwrap_or_default();
        if self.nodes.is_empty() && interfaces.provides.is_empty() && interfaces.consumes.is_empty()
        {
            anyhow::bail!("Manifest must declare at least one node or interface");
        }

        Ok(PackageSummary {
            name: self.package.name.clone(),
            version: self.package.version.clone(),
            provided_interfaces: interfaces.provides.into_iter().map(|i| i.id).collect(),
            consumed_interfaces: interfaces.consumes.into_iter().map(|i| i.id).collect(),
            nodes: self.nodes.iter().map(|n| n.id.clone()).collect(),
        })
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
