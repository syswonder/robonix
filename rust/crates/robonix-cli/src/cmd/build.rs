// SPDX-License-Identifier: MulanPSL-2.0
// Build command: run the package's build command

use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use robonix_cli::workspace;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const RBNX_BUILD_DIR: &str = "rbnx-build";
const RBNX_BUILT_STAMP: &str = "rbnx-build/.rbnx-built";

pub fn build_stamp_path(package_root: &Path) -> PathBuf {
    package_root.join(RBNX_BUILT_STAMP)
}

fn build_local(package_root: &Path, manifest: &manifest::Manifest, clean: bool) -> Result<()> {
    let _summary = manifest.validate_and_summarize()?;
    let build_cmd = manifest.build.trim();

    if build_cmd.is_empty() {
        anyhow::bail!("Package '{}' has no build command", manifest.package.name);
    }

    output::action(
        "Building",
        &format!("{} via `{}`", manifest.package.name, build_cmd),
    );

    // Run the build command via `bash -c` so it supports any form:
    //   bash scripts/build.sh, make -j$(nproc), cargo build, ./my_binary, etc.
    let mut cmd = Command::new("bash");
    cmd.args(["-c", build_cmd]);
    cmd.current_dir(package_root);
    cmd.env("RBNX_PACKAGE_ROOT", package_root.as_os_str());
    if clean {
        cmd.env("RBNX_BUILD_CLEAN", "1");
    }
    let status = cmd.status().with_context(|| {
        format!(
            "Failed to run build command '{}' in {}",
            build_cmd,
            package_root.display()
        )
    })?;
    if !status.success() {
        anyhow::bail!(
            "Build command '{}' failed with exit code {:?}",
            build_cmd,
            status.code()
        );
    }

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

/// Build all packages declared in a robonix_manifest.yaml, in dependency order.
pub async fn execute_all(config_path: PathBuf, clean: bool) -> Result<()> {
    let config_path = if config_path.is_absolute() {
        config_path
    } else {
        std::env::current_dir()?.join(&config_path)
    };
    if !config_path.exists() {
        anyhow::bail!("{} not found", config_path.display());
    }

    let config = workspace::load_runtime_config(&config_path)?;
    let project_root = config_path.parent().unwrap_or(Path::new(".")).to_path_buf();

    output::action(
        "Build",
        &format!(
            "all packages from '{}' ({})",
            config.name,
            config_path.display()
        ),
    );

    // Collect all packages across layers.
    let all_entries: Vec<(&str, &workspace::RuntimePackageEntry)> = config
        .primitives
        .iter()
        .map(|e| ("primitives", e))
        .chain(config.services.iter().map(|e| ("services", e)))
        .chain(config.skills.iter().map(|e| ("skills", e)))
        .collect();

    if all_entries.is_empty() {
        output::warning("no packages declared in manifest");
        return Ok(());
    }

    // Resolve paths and load manifests.
    struct Resolved<'a> {
        pkg_path: PathBuf,
        entry: &'a workspace::RuntimePackageEntry,
        manifest: manifest::Manifest,
        depends: Vec<String>,
    }
    let mut resolved: Vec<Resolved> = Vec::with_capacity(all_entries.len());
    for (role_dir, entry) in &all_entries {
        let pkg_path = workspace::resolve_package_path(&project_root, role_dir, entry)?;
        let detected = manifest::detect_and_load(&pkg_path)?;
        let depends: Vec<String> = detected
            .manifest
            .depends
            .iter()
            .map(|d| d.name.clone())
            .collect();
        resolved.push(Resolved {
            pkg_path,
            entry,
            manifest: detected.manifest,
            depends,
        });
    }

    // Topological sort using package name as key.
    let topo_input: Vec<(&str, &[String])> = resolved
        .iter()
        .map(|r| (r.entry.package.as_str(), r.depends.as_slice()))
        .collect();
    let order = workspace::topo_sort(&topo_input)?;

    let order_names: Vec<&str> = order
        .iter()
        .map(|&i| resolved[i].entry.name.as_str())
        .collect();
    output::step("Build order", &order_names.join(" → "));

    // Build each package in order.
    let total = order.len();
    let mut succeeded = 0usize;
    for (step_num, &idx) in order.iter().enumerate() {
        let r = &resolved[idx];
        output::action(
            &format!("[{}/{}]", step_num + 1, total),
            &format!("Building {}", r.entry.name),
        );
        build_local(&r.pkg_path, &r.manifest, clean)?;
        succeeded += 1;
    }

    output::success(&format!(
        "Build complete — {}/{} package(s) built",
        succeeded, total
    ));

    // Write project-level build stamp so `rbnx deploy` can check.
    let stamp_dir = project_root.join(workspace::BUILD_STAMP_DIR);
    fs::create_dir_all(&stamp_dir)?;
    let stamp_file = stamp_dir.join(".built");
    fs::write(&stamp_file, format!("{} packages built\n", succeeded))?;
    output::sub_step(&format!("build stamp written to {}", stamp_file.display()));

    Ok(())
}
