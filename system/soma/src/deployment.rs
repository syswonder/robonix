// SPDX-License-Identifier: MulanPSL-2.0

use crate::config::SomaConfig;
use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone)]
pub struct DeploymentStore {
    deployments: Vec<DeploymentRecord>,
}

#[derive(Debug, Clone)]
pub struct DeploymentRecord {
    pub deployment_path: PathBuf,
    pub manifest_path: PathBuf,
    pub packages: Vec<PackageLaunchTarget>,
    pub skipped: Vec<SkippedPackage>,
}

#[derive(Debug, Clone)]
pub struct PackageLaunchTarget {
    pub kind: PackageKind,
    pub name: String,
    pub package_dir: PathBuf,
    pub package_manifest_path: PathBuf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PackageKind {
    Primitive,
    Skill,
}

#[derive(Debug, Clone)]
pub struct SkippedPackage {
    pub kind: String,
    pub name: String,
    pub reason: String,
}

#[derive(Debug, Deserialize)]
struct DeployManifest {
    #[serde(default)]
    primitive: Vec<ManifestEntry>,
    #[serde(default)]
    service: Vec<ManifestEntry>,
    #[serde(default)]
    skill: Vec<ManifestEntry>,
}

#[derive(Debug, Deserialize)]
struct ManifestEntry {
    name: String,
    #[serde(default)]
    path: Option<PathBuf>,
    #[serde(default)]
    url: Option<String>,
}

impl DeploymentStore {
    /// Load all deployment manifests from config and collect primitive/skill package targets.
    pub fn load(config: &SomaConfig) -> Result<Self> {
        let mut deployments = Vec::new();
        for deployment in &config.deployments {
            deployments.push(
                DeploymentRecord::load(&deployment.path)
                    .with_context(|| format!("load deployment '{}'", deployment.path.display()))?,
            );
        }
        Ok(Self { deployments })
    }

    pub fn records(&self) -> &[DeploymentRecord] {
        &self.deployments
    }
}

impl DeploymentRecord {
    /// Read one robonix_manifest.yaml and resolve local primitive/skill package paths.
    pub fn load(deployment_path: &Path) -> Result<Self> {
        let manifest_path = deployment_path.join("robonix_manifest.yaml");
        let raw = std::fs::read_to_string(&manifest_path)
            .with_context(|| format!("read '{}'", manifest_path.display()))?;
        let manifest: DeployManifest = serde_yaml::from_str(&raw)
            .with_context(|| format!("parse '{}'", manifest_path.display()))?;
        let mut skipped = Vec::new();
        let mut packages = Vec::new();
        collect_kind(
            deployment_path,
            PackageKind::Primitive,
            "primitive",
            &manifest.primitive,
            &mut packages,
            &mut skipped,
        );
        collect_kind(
            deployment_path,
            PackageKind::Skill,
            "skill",
            &manifest.skill,
            &mut packages,
            &mut skipped,
        );
        for entry in manifest.service {
            skipped.push(SkippedPackage {
                kind: "service".into(),
                name: entry.name,
                reason: "Soma v2 only starts primitive and skill packages".into(),
            });
        }
        Ok(Self {
            deployment_path: deployment_path.to_path_buf(),
            manifest_path,
            packages,
            skipped,
        })
    }
}

impl std::fmt::Display for PackageKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PackageKind::Primitive => write!(f, "primitive"),
            PackageKind::Skill => write!(f, "skill"),
        }
    }
}

fn collect_kind(
    deployment_path: &Path,
    kind: PackageKind,
    kind_label: &str,
    entries: &[ManifestEntry],
    packages: &mut Vec<PackageLaunchTarget>,
    skipped: &mut Vec<SkippedPackage>,
) {
    for entry in entries {
        let Some(path) = &entry.path else {
            let reason = if entry.url.is_some() {
                "remote url-only package is not started by Soma v2"
            } else {
                "missing local path"
            };
            skipped.push(SkippedPackage {
                kind: kind_label.into(),
                name: entry.name.clone(),
                reason: reason.into(),
            });
            continue;
        };
        let package_dir = if path.is_absolute() {
            path.clone()
        } else {
            deployment_path.join(path)
        };
        let package_manifest_path = package_dir.join("package_manifest.yaml");
        packages.push(PackageLaunchTarget {
            kind,
            name: entry.name.clone(),
            package_dir,
            package_manifest_path,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_test_ci_manifest_targets() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("examples/test_ci");
        let record = DeploymentRecord::load(&root).expect("load deployment");
        assert_eq!(record.packages.len(), 2);
        assert!(record.packages.iter().any(|p| p.name == "test_primitive"));
        assert!(record.packages.iter().any(|p| p.name == "test_skills"));
        assert!(record.skipped.iter().any(|p| p.kind == "service"));
    }
}
