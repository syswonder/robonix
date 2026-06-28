// SPDX-License-Identifier: MulanPSL-2.0
//
// Soma now owns primitive + skill bring-up at runtime (rbnx boot
// stops at the builtin system processes). This module parses the
// deploy manifest's `primitive:` and `skill:` lists, resolves each
// entry to an on-disk package directory, and surfaces the per-entry
// `config:` block so the launcher can deliver it via
// Driver(CMD_INIT, config_json). Service entries stay in rbnx boot's
// orbit (they're optional and not always primitive-shaped) — they
// pass through as `Skipped` here.
//
// Path resolution must match `rbnx`'s deploy.rs verbatim (`path:`
// resolves under the deployment dir, `url:` resolves under
// `<deployment>/rbnx-boot/cache/<name>`), otherwise soma boots
// against a different on-disk layout than `rbnx build` populated and
// every url-remote package looks "missing". We don't fetch or build
// inline — rbnx build owns that — but we do surface a clear error
// when the cache dir or build sentinel is missing so the operator
// knows to run rbnx build first.

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
    pub primitives: Vec<PackageLaunchTarget>,
    pub skills: Vec<PackageLaunchTarget>,
    pub skipped: Vec<SkippedPackage>,
}

#[derive(Debug, Clone)]
pub struct PackageLaunchTarget {
    pub kind: PackageKind,
    /// `name:` field from the deploy manifest. This is also the
    /// expected `Capability(id=...)` the package registers with atlas
    /// after spawn. wait_for_registration_core verifies it.
    pub name: String,
    /// Final on-disk package dir (canonicalized lazily by the
    /// launcher right before spawn so we don't fail boot just because
    /// `rbnx build` hasn't created the cache dir yet).
    pub package_dir: PathBuf,
    /// `<package_dir>/<manifest_override or package_manifest.yaml>`,
    /// kept around mostly so the startup report can name it.
    pub package_manifest_path: PathBuf,
    /// Optional per-deployment manifest selector (`manifest:` in the
    /// deploy entry). Forwarded to `rbnx start --manifest <m>` so the
    /// right start path runs for THIS deployment target.
    pub manifest_override: Option<String>,
    /// Opaque config block from the manifest, raw serde value. Pushed
    /// to the package via Driver(CMD_INIT, config_json) — never written
    /// as a config file, never exported as an env var. Matches the
    /// v0.1 layering invariant in rbnx deploy.rs.
    pub config: serde_yaml::Value,
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
    /// Git URL for remote packages. Mutually exclusive with `path:`.
    /// `rbnx build` is responsible for cloning into
    /// `<deployment>/rbnx-boot/cache/<name>`; soma only reads from
    /// that cache.
    #[serde(default)]
    url: Option<String>,
    /// Optional package_manifest filename override. See
    /// `PackageLaunchTarget::manifest_override`.
    #[serde(default)]
    manifest: Option<String>,
    /// Opaque config block; serialized to JSON at Driver(CMD_INIT)
    /// time. Default = null, matches rbnx's behaviour.
    #[serde(default)]
    config: serde_yaml::Value,
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
    /// Read one robonix_manifest.yaml and resolve primitive/skill package paths
    /// (both `path:` and `url:` cache locations).
    pub fn load(deployment_path: &Path) -> Result<Self> {
        let manifest_path = deployment_path.join("robonix_manifest.yaml");
        let raw = std::fs::read_to_string(&manifest_path)
            .with_context(|| format!("read '{}'", manifest_path.display()))?;
        let manifest: DeployManifest = serde_yaml::from_str(&raw)
            .with_context(|| format!("parse '{}'", manifest_path.display()))?;
        let cache_root = deployment_path.join("rbnx-boot").join("cache");
        let mut skipped = Vec::new();
        let mut primitives = Vec::new();
        let mut skills = Vec::new();
        for entry in &manifest.primitive {
            match resolve_entry(deployment_path, &cache_root, entry, PackageKind::Primitive) {
                Ok(target) => primitives.push(target),
                Err(reason) => skipped.push(SkippedPackage {
                    kind: "primitive".into(),
                    name: entry.name.clone(),
                    reason,
                }),
            }
        }
        for entry in &manifest.skill {
            match resolve_entry(deployment_path, &cache_root, entry, PackageKind::Skill) {
                Ok(target) => skills.push(target),
                Err(reason) => skipped.push(SkippedPackage {
                    kind: "skill".into(),
                    name: entry.name.clone(),
                    reason,
                }),
            }
        }
        for entry in manifest.service {
            // service entries are still spawned by rbnx boot (they're
            // not always lifecycle-shaped). Soma doesn't touch them;
            // surface in the startup report so the operator sees we
            // saw and intentionally skipped them.
            skipped.push(SkippedPackage {
                kind: "service".into(),
                name: entry.name,
                reason: "service packages are owned by rbnx, not soma".into(),
            });
        }
        Ok(Self {
            deployment_path: deployment_path.to_path_buf(),
            manifest_path,
            primitives,
            skills,
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

/// Resolve a single deploy entry to its on-disk package directory.
///
/// Mirrors `rbnx::cmd::deploy::resolve_entry_path` exactly: `path:`
/// resolves relative to the deployment dir, `url:` resolves to
/// `<cache_root>/<name>` (where name falls back to the repo basename
/// when `entry.name` is empty). Anything else returns an Err with a
/// human-readable reason that ends up in the startup report.
fn resolve_entry(
    deployment_path: &Path,
    cache_root: &Path,
    entry: &ManifestEntry,
    kind: PackageKind,
) -> std::result::Result<PackageLaunchTarget, String> {
    let package_dir = match (&entry.path, &entry.url) {
        (Some(p), None) => {
            if p.is_absolute() {
                p.clone()
            } else {
                deployment_path.join(p)
            }
        }
        (None, Some(url)) => {
            let cache_name = if entry.name.is_empty() {
                url.trim_end_matches(".git")
                    .rsplit('/')
                    .next()
                    .unwrap_or("pkg")
                    .to_string()
            } else {
                entry.name.clone()
            };
            cache_root.join(cache_name)
        }
        (Some(_), Some(_)) => {
            return Err("package entry has both `path` and `url`; pick one".into());
        }
        (None, None) => {
            return Err("package entry has neither `path` nor `url`".into());
        }
    };
    let manifest_file = entry.manifest.as_deref().unwrap_or("package_manifest.yaml");
    let package_manifest_path = package_dir.join(manifest_file);
    Ok(PackageLaunchTarget {
        kind,
        name: entry.name.clone(),
        package_dir,
        package_manifest_path,
        manifest_override: entry.manifest.clone(),
        config: entry.config.clone(),
    })
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
        assert!(record.primitives.iter().any(|p| p.name == "test_primitive"));
        assert!(record.skills.iter().any(|p| p.name == "test_skills"));
        assert!(record.skipped.iter().any(|p| p.kind == "service"));
    }

    #[test]
    fn url_entry_resolves_under_rbnx_boot_cache() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let deployment = tmp.path().join("deploy");
        std::fs::create_dir_all(&deployment).expect("create deploy dir");
        let manifest = "primitive:\n  - name: remote_pkg\n    url: https://example.test/foo.git\n";
        std::fs::write(deployment.join("robonix_manifest.yaml"), manifest).expect("write");
        let record = DeploymentRecord::load(&deployment).expect("load");
        assert_eq!(record.primitives.len(), 1);
        let target = &record.primitives[0];
        assert!(
            target.package_dir.ends_with("rbnx-boot/cache/remote_pkg"),
            "{}",
            target.package_dir.display()
        );
    }
}
