// SPDX-License-Identifier: MulanPSL-2.0
// Build command: run the package's build.script
//
// Modes:
//   1) Single-package:   `rbnx build -p <path>` or `rbnx build -g <name>`
//   2) Config-file:      `rbnx build -c <config.yaml>`
//   3) Target:           `rbnx build <target>`  (finds deploy/<target>.yaml)
//   4) Workspace:        `rbnx build`           (reads robonix_workspace.yaml)

use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::workspace::{
    DeployConfig, WorkspaceConfig, ensure_packages_exist, find_workspace_root,
};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use super::deploy; // Reuse topo_sort from deploy module.

const RBNX_BUILD_DIR: &str = "rbnx-build";
const RBNX_BUILT_STAMP: &str = "rbnx-build/.rbnx-built";

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

fn build_local(
    package_root: &Path,
    manifest: &manifest::Manifest,
    clean: bool,
    workspace_build_dir: Option<&Path>,
) -> Result<()> {
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

    // Write workspace-level build stamp if workspace_build_dir provided.
    if let Some(build_dir) = workspace_build_dir {
        if fs::create_dir_all(build_dir).is_ok() {
            let stamp = build_dir.join(format!("{}.built", manifest.package.name));
            let _ = fs::write(&stamp, chrono::Utc::now().to_rfc3339());
        }
    }

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
    build_local(&package_root, &detected.manifest, clean, None)
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

// ── Shared: resolve + topo sort + build ─────────────────────────────────────

struct ResolvedPackage {
    name: String,
    root: PathBuf,
    manifest: manifest::Manifest,
}

/// Resolve workspace packages, topo-sort by manifest.depend, build each.
fn resolve_and_build(
    package_paths: &std::collections::HashMap<String, PathBuf>,
    packages: &[robonix_cli::workspace::WorkspacePackageEntry],
    clean: bool,
    workspace_build_dir: Option<&Path>,
) -> Result<()> {
    let mut resolved: Vec<ResolvedPackage> = Vec::new();

    for pkg_entry in packages {
        let pkg_path = package_paths.get(&pkg_entry.name).unwrap();
        let manifest_path = pkg_path.join(manifest::MANIFEST_FILE);
        if !manifest_path.exists() {
            anyhow::bail!(
                "package '{}' is missing {} at {}",
                pkg_entry.name,
                manifest::MANIFEST_FILE,
                manifest_path.display()
            );
        }
        let pkg_manifest = manifest::load_from_path(&manifest_path)?;
        output::check(&format!(
            "{} — {} v{}",
            pkg_entry.name, pkg_manifest.package.name, pkg_manifest.package.version
        ));
        resolved.push(ResolvedPackage {
            name: pkg_entry.name.clone(),
            root: pkg_path.clone(),
            manifest: pkg_manifest,
        });
    }

    if resolved.is_empty() {
        output::warning("no packages to build");
        return Ok(());
    }

    // Topological sort using manifest.depend.
    let items: Vec<(&str, &[String])> = resolved
        .iter()
        .map(|p| (p.name.as_str(), p.manifest.depend.as_slice()))
        .collect();
    let build_order = deploy::topo_sort(&items)?;
    let order_names: Vec<&str> = build_order
        .iter()
        .map(|&i| resolved[i].name.as_str())
        .collect();
    output::step("Build order", &order_names.join(" → "));

    // Build each package.
    let mut succeeded = 0usize;
    for (step_num, &idx) in build_order.iter().enumerate() {
        let pkg = &resolved[idx];
        output::action(
            &format!("[{}/{}] Building", step_num + 1, build_order.len()),
            &format!("{} ({})", pkg.name, pkg.root.display()),
        );
        match build_local(&pkg.root, &pkg.manifest, clean, workspace_build_dir) {
            Ok(()) => succeeded += 1,
            Err(e) => {
                output::error(&format!("package '{}' build failed: {}", pkg.name, e));
                anyhow::bail!(
                    "stopping build — package '{}' failed ({} succeeded)",
                    pkg.name,
                    succeeded,
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

// ── Config-file mode (deploy/<target>.yaml) ─────────────────────────────────

/// Build packages referenced by a deploy config file.
pub async fn execute_from_config(config_path: PathBuf, clean: bool) -> Result<()> {
    let config_path = config_path
        .canonicalize()
        .unwrap_or_else(|_| config_path.clone());

    output::action("Build", &format!("from config {}", config_path.display()));

    let content = fs::read_to_string(&config_path)
        .with_context(|| format!("failed to read {}", config_path.display()))?;
    let cfg: DeployConfig = serde_yaml::from_str(&content)
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

    // Load upstream workspace config.
    let ws = if let Some(ref upstream_path) = cfg.upstream_config {
        let resolved = base_dir.join(upstream_path);
        if resolved.exists() {
            let upstream_content = fs::read_to_string(&resolved)
                .with_context(|| format!("failed to read upstream config {}", resolved.display()))?;
            let ws: WorkspaceConfig = serde_yaml::from_str(&upstream_content).with_context(|| {
                format!("failed to parse upstream config {}", resolved.display())
            })?;
            output::step(
                "Loaded",
                &format!(
                    "upstream config: {} (workspace: {})",
                    resolved.display(),
                    ws.workspace.as_deref().unwrap_or("unnamed")
                ),
            );
            ws
        } else {
            output::warning(&format!(
                "upstream_config '{}' not found at {}, skipping",
                upstream_path,
                resolved.display()
            ));
            WorkspaceConfig::default()
        }
    } else {
        WorkspaceConfig::default()
    };

    if ws.packages.is_empty() {
        output::warning("no packages declared in workspace — nothing to build");
        return Ok(());
    }

    let workspace_root = find_workspace_root(&base_dir)?;

    // Ensure all packages exist (with git clone support).
    let package_paths = ensure_packages_exist(&workspace_root, &ws.packages)?;

    // Workspace-level build directory.
    let build_dir = workspace_root.join("build");

    resolve_and_build(&package_paths, &ws.packages, clean, Some(&build_dir))
}

// ── Workspace mode (no config file) ─────────────────────────────────────────

/// Build all packages from robonix_workspace.yaml in the current directory.
pub async fn execute_from_workspace(clean: bool) -> Result<()> {
    let ws_yaml = PathBuf::from("robonix_workspace.yaml");
    if !ws_yaml.exists() {
        anyhow::bail!(
            "no robonix_workspace.yaml found in current directory; \
             use 'rbnx build <target>' or 'rbnx build -c <config>' instead"
        );
    }

    let workspace_root = std::env::current_dir()?;
    let content = fs::read_to_string(&ws_yaml)?;
    let ws: WorkspaceConfig = serde_yaml::from_str(&content)
        .with_context(|| "failed to parse robonix_workspace.yaml")?;

    output::action(
        "Build",
        &format!(
            "from workspace {} ({} package(s))",
            ws.workspace.as_deref().unwrap_or("unnamed"),
            ws.packages.len(),
        ),
    );

    if ws.packages.is_empty() {
        output::warning("no packages declared in workspace — nothing to build");
        return Ok(());
    }

    // Ensure all packages exist (with git clone support).
    let package_paths = ensure_packages_exist(&workspace_root, &ws.packages)?;

    // Workspace-level build directory.
    let build_dir = workspace_root.join("build");

    resolve_and_build(&package_paths, &ws.packages, clean, Some(&build_dir))
}
