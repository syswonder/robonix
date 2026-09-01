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
//
// One `Deployment` per soma process. The v1 shape carried a
// `Vec<DeploymentRecord>` behind a store; the reviewer flagged that as
// speculative generality — rbnx spawns exactly one soma per boot
// against exactly one deployment dir, so we now load it directly.

use anyhow::{Context, Result};
use robonix_cli::manifest::{deploy_repo_dir_name, prepare_deployment_manifest};
use serde::Deserialize;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone)]
pub struct Deployment {
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
    /// to the package via its required Driver(CMD_INIT, config_json) — never
    /// written as a config file and never exported as an env var.
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

impl Deployment {
    /// Read one robonix_manifest.yaml and resolve primitive/skill package paths
    /// (both `path:` and `url:` cache locations).
    pub fn load(deployment_path: &Path) -> Result<Self> {
        Self::load_manifest(&deployment_path.join("robonix_manifest.yaml"))
    }

    /// Load the exact manifest selected by `rbnx boot -f`, while keeping its
    /// parent as the deployment root for local paths and rbnx-boot/cache.
    pub fn load_manifest(manifest_path: &Path) -> Result<Self> {
        let deployment_path = manifest_path
            .parent()
            .context("deployment manifest has no parent directory")?;
        let raw = std::fs::read_to_string(manifest_path)
            .with_context(|| format!("read '{}'", manifest_path.display()))?;
        let raw: serde_yaml::Value = serde_yaml::from_str(&raw)
            .with_context(|| format!("parse '{}'", manifest_path.display()))?;
        let prepared = prepare_deployment_manifest(raw, None)
            .with_context(|| format!("prepare '{}'", manifest_path.display()))?;
        robonix_cli::manifest::validate_deployment_instance_names(&prepared)
            .with_context(|| format!("validate identities in '{}'", manifest_path.display()))?;
        let manifest: DeployManifest = serde_yaml::from_value(prepared)
            .with_context(|| format!("decode '{}'", manifest_path.display()))?;
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
            manifest_path: manifest_path.to_path_buf(),
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
/// Uses the shared deployment manifest rules: `path:`
/// resolves relative to the deployment dir, `url:` resolves to
/// `<cache_root>/<deploy_repo_dir_name(url)>`. Anything else returns an Err
/// with a human-readable reason that ends up in the startup report.
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
        (None, Some(url)) => cache_root.join(deploy_repo_dir_name(url)),
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
        let deployment = Deployment::load(&root).expect("load deployment");
        assert!(
            deployment
                .primitives
                .iter()
                .any(|p| p.name == "test_primitive")
        );
        assert!(deployment.skills.iter().any(|p| p.name == "test_skills"));
        assert!(deployment.skipped.iter().any(|p| p.kind == "service"));
    }

    #[test]
    fn url_entry_resolves_under_rbnx_boot_cache() {
        // Regression: soma MUST cache-key by repo name (last path
        // segment of the url minus `.git`), NOT by `entry.name`. If
        // this ever flips back, `rbnx build` and soma disagree on
        // where the package lives and every url-remote skill/primitive
        // whose provider-id differs from its repo name silently fails
        // at stage 2 with `MissingManifest`. The concrete case that
        // motivated this test: `name: explore` +
        // `url: .../explore_rbnx` — clone lands in
        // `.../cache/explore_rbnx/`, soma was looking at
        // `.../cache/explore/`.
        let tmp = tempfile::tempdir().expect("tempdir");
        let deployment_dir = tmp.path().join("deploy");
        std::fs::create_dir_all(&deployment_dir).expect("create deploy dir");
        let manifest = "primitive:\n  - name: remote_pkg\n    url: https://example.test/foo.git\n";
        std::fs::write(deployment_dir.join("robonix_manifest.yaml"), manifest).expect("write");
        let deployment = Deployment::load(&deployment_dir).expect("load");
        assert_eq!(deployment.primitives.len(), 1);
        let target = &deployment.primitives[0];
        assert!(
            target.package_dir.ends_with("rbnx-boot/cache/foo"),
            "expected cache dir keyed by repo name `foo` (from url), got {}",
            target.package_dir.display()
        );
    }

    #[test]
    fn url_entry_ignores_entry_name_when_it_differs_from_repo() {
        // Direct guard for the explore_rbnx regression. `entry.name`
        // and repo basename intentionally differ here; the resolved
        // package_dir must follow the repo, not the name.
        let tmp = tempfile::tempdir().expect("tempdir");
        let deployment_dir = tmp.path().join("deploy");
        std::fs::create_dir_all(&deployment_dir).expect("create deploy dir");
        let manifest =
            "skill:\n  - name: explore\n    url: https://github.com/enkerewpo/explore_rbnx\n";
        std::fs::write(deployment_dir.join("robonix_manifest.yaml"), manifest).expect("write");
        let deployment = Deployment::load(&deployment_dir).expect("load");
        assert_eq!(deployment.skills.len(), 1);
        let target = &deployment.skills[0];
        assert_eq!(
            target.name, "explore",
            "provider-id should stay as declared"
        );
        assert!(
            target.package_dir.ends_with("rbnx-boot/cache/explore_rbnx"),
            "cache dir must key on repo (`explore_rbnx`), not name (`explore`); got {}",
            target.package_dir.display()
        );
        assert!(
            target
                .package_manifest_path
                .ends_with("rbnx-boot/cache/explore_rbnx/package_manifest.yaml"),
            "got {}",
            target.package_manifest_path.display()
        );
    }

    #[test]
    fn local_path_uses_shared_deployment_env_expansion() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let deployment_dir = tmp.path().join("deploy");
        std::fs::create_dir_all(&deployment_dir).expect("create deploy dir");
        std::fs::write(
            deployment_dir.join("robonix_manifest.yaml"),
            "primitive:\n  - name: local_audio\n    path: ${HOME}/shared-audio\n",
        )
        .expect("write manifest");

        let deployment = Deployment::load(&deployment_dir).expect("load deployment");
        let expected = PathBuf::from(std::env::var("HOME").expect("HOME")).join("shared-audio");
        assert_eq!(deployment.primitives[0].package_dir, expected);
    }

    #[test]
    fn load_manifest_uses_the_selected_profile_not_the_default() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let deployment_dir = tmp.path().join("deploy");
        std::fs::create_dir_all(&deployment_dir).expect("create deploy dir");
        std::fs::write(
            deployment_dir.join("robonix_manifest.yaml"),
            "primitive:\n  - name: default_base\n    path: packages/default_base\n",
        )
        .expect("write default manifest");
        let selected = deployment_dir.join("robonix_manifest.arm.yaml");
        std::fs::write(
            &selected,
            "primitive:\n  - name: arm_base\n    path: packages/arm_base\n",
        )
        .expect("write selected manifest");

        let deployment = Deployment::load_manifest(&selected).expect("load selected profile");
        assert_eq!(deployment.manifest_path, selected);
        assert_eq!(deployment.primitives.len(), 1);
        assert_eq!(deployment.primitives[0].name, "arm_base");
        assert_eq!(
            deployment.primitives[0].package_dir,
            deployment_dir.join("packages/arm_base")
        );
    }

    #[test]
    fn repo_dir_name_comes_from_shared_manifest_logic() {
        assert_eq!(deploy_repo_dir_name("https://github.com/foo/bar"), "bar");
        assert_eq!(
            deploy_repo_dir_name("https://github.com/foo/bar.git"),
            "bar"
        );
        assert_eq!(deploy_repo_dir_name("https://github.com/foo/bar/"), "bar");
        assert_eq!(
            deploy_repo_dir_name("git@github.com:foo/bar_baz.git"),
            "bar_baz"
        );
        assert_eq!(
            deploy_repo_dir_name("https://github.com/enkerewpo/explore_rbnx"),
            "explore_rbnx"
        );
    }
}
