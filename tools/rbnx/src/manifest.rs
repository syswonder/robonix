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
use robonix_scribe::warn;
use serde::Deserialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Preferred per-package manifest filename. Legacy `robonix_manifest.yaml`
/// is also accepted by [`detect_manifest_path`].
pub const MANIFEST_FILE: &str = "package_manifest.yaml";
pub const LEGACY_MANIFEST_FILE: &str = "robonix_manifest.yaml";

/// Cache directory name for a URL-backed deploy package.
///
/// The name comes from the repository, not the configured provider id: one
/// repository can supply more than one configured instance.
pub fn deploy_repo_dir_name(url: &str) -> String {
    url.trim_end_matches('/')
        .trim_end_matches(".git")
        .rsplit('/')
        .next()
        .filter(|segment| !segment.is_empty())
        .unwrap_or("pkg")
        .to_string()
}

/// Apply deployment `env:` and expand `$VAR` / `${VAR}` in every scalar.
///
/// `rbnx` and Soma call this same entry point before parsing a deployment so
/// they cannot disagree about paths, selected manifests, or package config.
pub fn prepare_deployment_manifest(
    mut root: serde_yaml::Value,
    robonix_source_path: Option<&Path>,
) -> Result<serde_yaml::Value> {
    if std::env::var_os("ROBONIX_SOURCE_PATH").is_none()
        && let Some(path) = robonix_source_path
    {
        // This runs before package child processes start.
        unsafe { std::env::set_var("ROBONIX_SOURCE_PATH", path) };
    }
    let env: HashMap<String, String> = root
        .get("env")
        .cloned()
        .map(serde_yaml::from_value)
        .transpose()
        .context("parse top-level env")?
        .unwrap_or_default();
    let expanded: Vec<(&String, String)> = env
        .iter()
        .map(|(key, value)| (key, expand_deployment_env(value)))
        .collect();
    for (key, value) in expanded {
        unsafe { std::env::set_var(key, value) };
    }
    expand_deployment_yaml(&mut root);
    Ok(root)
}

pub fn expand_deployment_env(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'$' && i + 1 < bytes.len() {
            if bytes[i + 1] == b'{' {
                if let Some(end) = s[i + 2..].find('}') {
                    out.push_str(&std::env::var(&s[i + 2..i + 2 + end]).unwrap_or_default());
                    i = i + 2 + end + 1;
                    continue;
                }
            } else if bytes[i + 1].is_ascii_alphabetic() || bytes[i + 1] == b'_' {
                let start = i + 1;
                let mut end = start;
                while end < bytes.len()
                    && (bytes[end].is_ascii_alphanumeric() || bytes[end] == b'_')
                {
                    end += 1;
                }
                out.push_str(&std::env::var(&s[start..end]).unwrap_or_default());
                i = end;
                continue;
            }
        }
        let ch = s[i..].chars().next().expect("non-empty by loop guard");
        out.push(ch);
        i += ch.len_utf8();
    }
    out
}

/// Split the optional package-manifest selector from a non-builtin `system:`
/// package's runtime configuration.
///
/// System packages historically used their mapping directly as Driver INIT
/// config. Keep that shape backward compatible while reserving `manifest` as
/// the deployment-owned build/start/stop selector. The selector is removed
/// before the remaining opaque mapping is sent to the provider.
pub fn split_system_package_config(
    value: &serde_yaml::Value,
) -> Result<(Option<String>, serde_yaml::Value)> {
    let mut config = value.clone();
    let Some(mapping) = config.as_mapping_mut() else {
        return Ok((None, config));
    };
    let manifest_key = serde_yaml::Value::String("manifest".to_string());
    let Some(raw_manifest) = mapping.remove(&manifest_key) else {
        return Ok((None, config));
    };
    let manifest = raw_manifest
        .as_str()
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .ok_or_else(|| anyhow::anyhow!("system package `manifest` must be a non-empty string"))?;
    Ok((Some(manifest.to_string()), config))
}

fn expand_deployment_yaml(value: &mut serde_yaml::Value) {
    match value {
        serde_yaml::Value::String(s) => *s = expand_deployment_env(s),
        serde_yaml::Value::Sequence(sequence) => {
            for item in sequence {
                expand_deployment_yaml(item);
            }
        }
        serde_yaml::Value::Mapping(mapping) => {
            for (_, item) in mapping.iter_mut() {
                expand_deployment_yaml(item);
            }
        }
        _ => {}
    }
}

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
    pub stop: String,
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
    /// Legacy publisher label retained for manifest compatibility. It is not
    /// used as package identity or Catalog metadata; new manifests should use
    /// `name`, `tags`, and `maintainers` instead.
    #[serde(default)]
    pub vendor: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub tags: Vec<String>,
    #[serde(default)]
    pub maintainers: Vec<String>,
    #[serde(default)]
    pub license: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct CapabilityRef {
    /// Contract id (matches one of the TOMLs under `capabilities/`).
    pub name: String,
    /// Optional path to a package-local TOML that overrides / defines
    /// this capability (for experimental providers not yet in the official
    /// capabilities directory). Relative to the package root.
    #[serde(default, alias = "definition")]
    pub path: Option<String>,
}

/// One entry under a package's `depends:` list. Models a *source / lib*
/// dependency (think Linux kernel module SOFT_DEPS) — i.e. another
/// package whose codegen output / Python package this package needs at
/// build or import time. NOT a boot-order dependency.
///
/// `name` is required (the depended-on package's `package.name`).
/// Exactly one of `path` / `url` should be set:
///   - `path`: filesystem path relative to this package's manifest dir
///   - `url`:  git URL (cloned to `<pkg>/rbnx-build/deps/<name>/` on first build)
///     Neither set means "expect it to already be installed / on PYTHONPATH".
#[derive(Debug, Clone, Deserialize)]
pub struct DependsRef {
    pub name: String,
    #[serde(default)]
    pub path: Option<String>,
    #[serde(default)]
    pub url: Option<String>,
    #[serde(default)]
    pub branch: Option<String>,
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
    /// Optional package-owned cleanup command, symmetric with `start`.
    #[serde(default)]
    stop: Option<String>,
    /// Legacy: list of nodes, each with its own `start` block.
    #[serde(default)]
    nodes: Vec<LegacyNode>,
    #[serde(default)]
    capabilities: Vec<CapabilityRef>,
    #[serde(default)]
    depends: Vec<DependsRef>,
    /// Legacy `provider_id:` field. Removed from spec — accepted by serde but
    /// ignored. rbnx-boot now discovers the provider_id by polling atlas for any provider
    /// that registered after `start.sh` spawned. Future warning: emit deprecation.
    #[serde(default)]
    #[allow(dead_code)]
    provider_id: Option<String>,
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

/// Locate a package's manifest file inside `package_root`.
///
/// `override_name` lets a deploy entry select a non-default manifest file
/// (the `manifest:` field on a deploy `PackageEntry`) so one package can
/// ship per-deployment-target variants — e.g. `package_manifest.yaml`
/// (x86 + docker), `package_manifest.jetson-native.yaml`, and
/// `package_manifest.jetson-docker.yaml`, each with its own build/start —
/// without changing the manifest schema itself. When set, the named file
/// MUST exist (a typo'd target should fail loud, not silently fall back to
/// the default manifest and build the wrong thing). When `None`, the
/// default `package_manifest.yaml` (then legacy) is used as before.
pub fn detect_manifest_path(package_root: &Path, override_name: Option<&str>) -> Result<PathBuf> {
    if let Some(name) = override_name {
        let p = package_root.join(name);
        if p.is_file() {
            return Ok(p);
        }
        anyhow::bail!(
            "manifest override `{name}` not found in {} — the deploy entry's \
             `manifest:` field names a file the package does not ship",
            package_root.display()
        );
    }
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

pub fn detect_and_load(
    package_root: &Path,
    override_name: Option<&str>,
) -> Result<DetectedManifest> {
    let path = detect_manifest_path(package_root, override_name)?;
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
    if package.name.trim().is_empty()
        && let Some(id) = &package.id
        && !id.trim().is_empty()
    {
        package.name = id.clone();
        is_legacy = true;
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

    let stop = raw.stop.unwrap_or_default();

    if filename_is_legacy {
        is_legacy = true;
    }

    Manifest {
        manifest_version: raw.manifest_version,
        package,
        build,
        start,
        stop,
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
            warn!(
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn package_vendor_is_backward_compatible() {
        let raw: RawManifest = serde_yaml::from_str(
            r#"
manifestVersion: 1
package:
  name: com.robonix.system.scene
  version: 0.1.0
  vendor: robonix
  description: Legacy package metadata example.
  license: MulanPSL-2.0
start: bash scripts/start.sh
capabilities: []
"#,
        )
        .expect("legacy package.vendor must parse");

        let manifest = normalize(raw, Path::new("package_manifest.yaml"));

        assert_eq!(manifest.package.vendor, "robonix");
        manifest
            .validate_and_summarize()
            .expect("package.vendor must not invalidate an otherwise valid manifest");
    }

    #[test]
    fn system_package_manifest_is_separate_from_runtime_config() {
        let value: serde_yaml::Value = serde_yaml::from_str(
            r#"
manifest: package_manifest.jetson-native.yaml
camera_provider_id: front_camera
web_port: 50107
"#,
        )
        .unwrap();

        let (manifest, config) = split_system_package_config(&value).unwrap();

        assert_eq!(
            manifest.as_deref(),
            Some("package_manifest.jetson-native.yaml")
        );
        assert_eq!(config["camera_provider_id"], "front_camera");
        assert_eq!(config["web_port"], 50107);
        assert!(config.get("manifest").is_none());
    }

    #[test]
    fn legacy_system_package_config_is_unchanged() {
        let value: serde_yaml::Value = serde_yaml::from_str(
            r#"
camera_provider_id: front_camera
web_port: 50107
"#,
        )
        .unwrap();

        let (manifest, config) = split_system_package_config(&value).unwrap();

        assert!(manifest.is_none());
        assert_eq!(config, value);
    }

    #[test]
    fn system_package_manifest_rejects_non_string_values() {
        let value: serde_yaml::Value = serde_yaml::from_str("manifest: 42\n").unwrap();
        let error = split_system_package_config(&value).unwrap_err();
        assert!(error.to_string().contains("must be a non-empty string"));
    }
}
