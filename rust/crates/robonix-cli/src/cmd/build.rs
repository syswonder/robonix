// SPDX-License-Identifier: MulanPSL-2.0
// Build command: run the package's build.script
//
// Two modes:
//   1) Single-package mode: `rbnx build -p <path>` or `rbnx build -g <name>`
//   2) Config-file mode:    `rbnx build --config <config.yaml>`
//      Reads config.yaml (+ upstream robonix_workspace.yaml), discovers all
//      packages, topo-sorts by depends_on, validates contracts, and builds each.

use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use serde::Deserialize;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use super::deploy; // Reuse topo_sort from deploy module.

const RBNX_BUILD_DIR: &str = "rbnx-build";
const RBNX_BUILT_STAMP: &str = "rbnx-build/.rbnx-built";

// ── Config-file schema (subset of deploy config) ────────────────────────────

#[derive(Debug, Deserialize)]
struct BuildConfig {
    #[serde(default)]
    upstream_config: Option<String>,
    #[serde(default)]
    packages: Vec<BuildPackageEntry>,
}

#[derive(Debug, Deserialize, Default)]
struct UpstreamWorkspace {
    #[serde(default)]
    workspace: Option<String>,
    /// System services define contract_ids that downstream packages may consume.
    #[serde(default)]
    system: Vec<SystemEntry>,
}

#[derive(Debug, Deserialize, Clone)]
struct SystemEntry {
    #[serde(default)]
    name: String,
    #[serde(default)]
    contract_id: String,
}

#[derive(Debug, Deserialize, Clone)]
struct BuildPackageEntry {
    name: String,
    path: String,
    #[serde(default)]
    depends_on: Vec<String>,
}

// ── Existing helpers ────────────────────────────────────────────────────────

pub fn build_stamp_path(package_root: &Path) -> PathBuf {
    package_root.join(RBNX_BUILT_STAMP)
}

fn run_build_script(package_root: &Path, script_rel: &str, clean: bool) -> Result<()> {
    let script_path = package_root.join(script_rel);
    let mut cmd = Command::new("bash");
    cmd.arg(&script_path);
    cmd.current_dir(package_root);
    cmd.env("RBNX_PACKAGE_ROOT", package_root.as_os_str());
    if clean {
        cmd.env("RBNX_BUILD_CLEAN", "1");
    }
    let status = cmd.status().with_context(|| {
        format!(
            "Failed to run build script {} in {}",
            script_path.display(),
            package_root.display()
        )
    })?;
    if !status.success() {
        anyhow::bail!(
            "Build script {} failed with exit code {:?}",
            script_path.display(),
            status.code()
        );
    }
    Ok(())
}

fn build_local(package_root: &Path, manifest: &manifest::Manifest, clean: bool) -> Result<()> {
    let _summary = manifest.validate_and_summarize()?;
    let script_rel = manifest.build.script.trim();
    let script_path = package_root.join(script_rel);
    if !script_path.is_file() {
        anyhow::bail!(
            "build.script not found: {} (manifest.build.script = {:?})",
            script_path.display(),
            script_rel
        );
    }
    output::action(
        "Building",
        &format!("{} via {}", manifest.package.name, script_rel),
    );
    run_build_script(package_root, script_rel, clean)?;
    fs::create_dir_all(package_root.join(RBNX_BUILD_DIR))?;
    fs::write(build_stamp_path(package_root), "").with_context(|| {
        format!(
            "Failed to write {}",
            build_stamp_path(package_root).display()
        )
    })?;
    output::success(&format!(
        "Package '{}' build finished",
        manifest.package.name
    ));
    Ok(())
}

pub fn build_local_package(path: &Path, clean: bool) -> Result<()> {
    let package_root = path
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize package path {}", path.display()))?;
    let detected = manifest::detect_and_load(&package_root)?;
    build_local(&package_root, &detected.manifest, clean)
}

pub async fn execute_local(path: PathBuf, clean: bool) -> Result<()> {
    let package_root = path
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize package path {}", path.display()))?;
    output::action(
        "Building",
        &format!("local package at {}", package_root.display()),
    );
    build_local_package(&package_root, clean)?;
    Ok(())
}

// ── Config-file mode ────────────────────────────────────────────────────────

/// Build all packages defined in a config.yaml file:
///   1. Parse config.yaml, optionally load upstream robonix_workspace.yaml
///   2. Resolve each package path, load its robonix_manifest.yaml
///   3. Validate contracts against upstream (consumed interfaces vs upstream contract_ids)
///   4. Topological sort by depends_on
///   5. Build each package in order
pub async fn execute_from_config(config_path: PathBuf, clean: bool) -> Result<()> {
    let config_path = config_path
        .canonicalize()
        .unwrap_or_else(|_| config_path.clone());

    output::action("Build", &format!("from config {}", config_path.display()));

    let content = fs::read_to_string(&config_path)
        .with_context(|| format!("failed to read {}", config_path.display()))?;
    let cfg: BuildConfig = serde_yaml::from_str(&content)
        .with_context(|| format!("failed to parse {}", config_path.display()))?;

    let base_dir = config_path
        .parent()
        .map(|p| {
            if p.as_os_str().is_empty() {
                PathBuf::from(".")
            } else {
                p.to_path_buf()
            }
        })
        .unwrap_or_else(|| PathBuf::from("."));
    let base_dir = base_dir
        .canonicalize()
        .unwrap_or_else(|_| base_dir.to_path_buf());

    // ── Load upstream config for contract validation ────────────────────────
    let upstream = if let Some(ref upstream_path) = cfg.upstream_config {
        let resolved = base_dir.join(upstream_path);
        if resolved.exists() {
            let upstream_content = fs::read_to_string(&resolved)
                .with_context(|| format!("failed to read upstream config {}", resolved.display()))?;
            let up: UpstreamWorkspace = serde_yaml::from_str(&upstream_content).with_context(|| {
                format!("failed to parse upstream config {}", resolved.display())
            })?;
            output::step(
                "Loaded",
                &format!(
                    "upstream config: {} (workspace: {})",
                    resolved.display(),
                    up.workspace.as_deref().unwrap_or("unnamed")
                ),
            );
            up
        } else {
            output::warning(&format!(
                "upstream_config '{}' not found at {}, skipping",
                upstream_path,
                resolved.display()
            ));
            UpstreamWorkspace::default()
        }
    } else {
        UpstreamWorkspace::default()
    };

    // Build set of available upstream contract_ids for validation.
    let upstream_contracts: Vec<&str> = upstream
        .system
        .iter()
        .filter(|s| !s.contract_id.is_empty())
        .map(|s| s.contract_id.as_str())
        .collect();
    if !upstream_contracts.is_empty() {
        output::step("Upstream", &format!("{} contract(s) available", upstream_contracts.len()));
        for cid in &upstream_contracts {
            output::sub_step(&format!("contract: {}", cid));
        }
    }

    if cfg.packages.is_empty() {
        output::warning("no packages defined in config — nothing to build");
        return Ok(());
    }

    // ── Resolve packages and load manifests ─────────────────────────────────
    output::step("Resolving", &format!("{} package(s)", cfg.packages.len()));

    struct ResolvedPackage {
        name: String,
        root: PathBuf,
        manifest: manifest::Manifest,
        depends_on: Vec<String>,
    }

    let mut resolved: Vec<ResolvedPackage> = Vec::new();

    for entry in &cfg.packages {
        let pkg_path = base_dir.join(&entry.path);
        if !pkg_path.exists() {
            output::cross(&format!(
                "package '{}': path {} does not exist",
                entry.name,
                pkg_path.display()
            ));
            anyhow::bail!(
                "package '{}' path {} not found",
                entry.name,
                pkg_path.display()
            );
        }

        let pkg_root = pkg_path
            .canonicalize()
            .unwrap_or_else(|_| pkg_path.clone());

        // Load robonix_manifest.yaml.
        let manifest_path = pkg_root.join(manifest::MANIFEST_FILE);
        if !manifest_path.exists() {
            output::cross(&format!(
                "package '{}': {} not found at {}",
                entry.name,
                manifest::MANIFEST_FILE,
                manifest_path.display()
            ));
            anyhow::bail!(
                "package '{}' is missing {}",
                entry.name,
                manifest::MANIFEST_FILE
            );
        }

        let pkg_manifest = manifest::load_from_path(&manifest_path).with_context(|| {
            format!(
                "failed to load {} for package '{}'",
                manifest::MANIFEST_FILE,
                entry.name
            )
        })?;

        output::check(&format!(
            "{} — {} v{}",
            entry.name, pkg_manifest.package.name, pkg_manifest.package.version
        ));

        resolved.push(ResolvedPackage {
            name: entry.name.clone(),
            root: pkg_root,
            manifest: pkg_manifest,
            depends_on: entry.depends_on.clone(),
        });
    }

    // ── Validate contracts against upstream ─────────────────────────────────
    output::step("Validating", "contracts against upstream");

    let mut contract_warnings = 0usize;
    for pkg in &resolved {
        if let Some(ref interfaces) = pkg.manifest.interfaces {
            for consumed in &interfaces.consumes {
                // Exact match only — avoid prefix matching which could produce
                // false positives (e.g. "robonix/sys/model" matching
                // "robonix/sys/model_evil").
                let matched = upstream_contracts
                    .iter()
                    .any(|cid| *cid == consumed.id);
                if matched {
                    output::check(&format!(
                        "{}: consumed interface '{}' ← upstream OK",
                        pkg.name, consumed.id
                    ));
                } else {
                    output::warning(&format!(
                        "{}: consumed interface '{}' not found in upstream contracts",
                        pkg.name, consumed.id
                    ));
                    contract_warnings += 1;
                }
            }
        }
    }
    if contract_warnings == 0 {
        output::check("all contracts validated");
    } else {
        output::warning(&format!(
            "{} contract warning(s) — some consumed interfaces may not be satisfied at runtime",
            contract_warnings
        ));
    }

    // ── Topological sort (reuse shared implementation) ──────────────────────
    let items: Vec<(&str, &[String])> = resolved
        .iter()
        .map(|p| (p.name.as_str(), p.depends_on.as_slice()))
        .collect();
    let build_order = deploy::topo_sort(&items)?;
    let order_names: Vec<&str> = build_order.iter().map(|&i| resolved[i].name.as_str()).collect();
    output::step("Build order", &order_names.join(" → "));

    // ── Build each package ──────────────────────────────────────────────────
    let mut succeeded = 0usize;
    let mut failed = 0usize;

    for (step_num, &idx) in build_order.iter().enumerate() {
        let pkg = &resolved[idx];
        output::action(
            &format!("[{}/{}] Building", step_num + 1, build_order.len()),
            &format!("{} ({})", pkg.name, pkg.root.display()),
        );
        match build_local(&pkg.root, &pkg.manifest, clean) {
            Ok(()) => succeeded += 1,
            Err(e) => {
                output::error(&format!("package '{}' build failed: {}", pkg.name, e));
                failed += 1;
                // Stop on first failure (downstream packages likely depend on this).
                anyhow::bail!(
                    "stopping build — package '{}' failed ({} succeeded, {} failed)",
                    pkg.name,
                    succeeded,
                    failed
                );
            }
        }
    }

    output::success(&format!(
        "Build complete — {}/{} package(s) built successfully",
        succeeded,
        resolved.len()
    ));

    Ok(())
}
