// SPDX-License-Identifier: MulanPSL-2.0
// Build command: run the package's build.script

use anyhow::{Context, Result};
use robonix_cli::manifest;
use robonix_cli::output;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const RBNX_BUILD_DIR: &str = "rbnx-build";
const RBNX_BUILT_STAMP: &str = "rbnx-build/.rbnx-built";

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
