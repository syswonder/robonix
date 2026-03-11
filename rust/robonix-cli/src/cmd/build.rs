// SPDX-License-Identifier: MulanPSL-2.0
// Build Command Module
//
// Build command implementation for robonix-cli

use crate::manifest::{self, BuildStrategy, PackageManifest};
use crate::{Config, PackageDatabase, output};
use anyhow::{Context, Result};
use colored::*;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const RBNX_BUILD_DIR: &str = "rbnx-build";

#[derive(Debug, Clone)]
pub struct LocalBuildLayout {
    pub package_name: String,
    pub package_root: PathBuf,
    pub install_setup: PathBuf,
}

/// If the package has already been built (rbnx-build/ws/install/setup.bash exists), return its layout so start can skip rebuild.
pub fn get_existing_build_layout(package_root: &Path) -> Result<Option<LocalBuildLayout>> {
    let detected = manifest::detect_and_load(package_root)?;
    let package_name = match &detected.manifest {
        PackageManifest::VNext(m) => m.package.name.clone(),
        PackageManifest::Legacy(m) => m.package.name.clone(),
    };
    let install_setup = package_root
        .join(RBNX_BUILD_DIR)
        .join("ws")
        .join("install")
        .join("setup.bash");
    if install_setup.exists() {
        Ok(Some(LocalBuildLayout {
            package_name,
            package_root: package_root.to_path_buf(),
            install_setup,
        }))
    } else {
        Ok(None)
    }
}

/// Build a single package (shared logic)
fn build_package(pkg_info: &crate::database::PackageInfo) -> Result<()> {
    println!(
        "{} {}",
        format!("[{}]", "Building").green().bold(),
        pkg_info.name.bright_white().bold()
    );

    // Load manifest to determine build strategy.
    let manifest_path = &pkg_info.manifest_path;
    let manifest = crate::manifest::load_from_path(manifest_path)
        .with_context(|| format!("Failed to load manifest: {}", manifest_path.display()))?;
    let summary = manifest.validate_and_summarize()?;

    let build_result = match summary.build_strategy {
        BuildStrategy::LegacyScript { script } => {
            let build_script_path = if let Some(script) = script {
                let script_path = pkg_info.path.join(script);
                if !script_path.exists() {
                    anyhow::bail!(
                        "Build script not found: {} (specified in manifest)",
                        script_path.display()
                    );
                }
                script_path
            } else {
                let default_script = pkg_info.path.join("rbnx").join("build.sh");
                if !default_script.exists() {
                    output::info(&format!(
                        "No build script found for {}, skipping build",
                        pkg_info.name
                    ));
                    return Ok(());
                }
                default_script
            };

            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                let mut perms = std::fs::metadata(&build_script_path)?.permissions();
                perms.set_mode(0o755);
                std::fs::set_permissions(&build_script_path, perms)?;
            }

            output::sub_step(&format!(
                "Running legacy build script: {}",
                build_script_path.display()
            ));

            Command::new(&build_script_path)
                .current_dir(&pkg_info.path)
                .status()
                .with_context(|| {
                    format!(
                        "Failed to execute build script: {}",
                        build_script_path.display()
                    )
                })?
        }
        BuildStrategy::VNextCommand {
            command,
            workspace_root,
        } => {
            let work_dir = workspace_root
                .as_deref()
                .map(|rel| pkg_info.path.join(rel))
                .unwrap_or_else(|| pkg_info.path.clone());
            output::sub_step(&format!("Running manifest build command: {}", command));

            #[cfg(unix)]
            let status = Command::new("bash")
                .arg("-lc")
                .arg(&command)
                .current_dir(&work_dir)
                .status()
                .with_context(|| format!("Failed to execute build command in {}", work_dir.display()))?;

            #[cfg(not(unix))]
            let status = Command::new("sh")
                .arg("-lc")
                .arg(&command)
                .current_dir(&work_dir)
                .status()
                .with_context(|| format!("Failed to execute build command in {}", work_dir.display()))?;

            status
        }
        BuildStrategy::None => {
            output::info(&format!(
                "No build metadata found for {}, skipping build",
                pkg_info.name
            ));
            return Ok(());
        }
    };

    if !build_result.success() {
        anyhow::bail!(
            "Build script failed for {} with exit code: {:?}",
            pkg_info.name,
            build_result.code()
        );
    }

    output::success(&format!("Package '{}' built successfully", pkg_info.name));
    Ok(())
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("robonix-cli should live under repo root")
        .to_path_buf()
}

fn robonix_interfaces_root_from_catalog(catalog_root: &Path) -> Result<PathBuf> {
    catalog_root
        .parent()
        .map(Path::to_path_buf)
        .context("Interface catalog root should be robonix-interfaces/ridl")
}

fn copy_package_tree(src: &Path, dst: &Path) -> Result<()> {
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let file_name = entry.file_name();
        let src_path = entry.path();
        let dst_path = dst.join(&file_name);

        if file_name == RBNX_BUILD_DIR {
            continue;
        }

        let file_type = entry.file_type()?;
        if file_type.is_dir() {
            copy_package_tree(&src_path, &dst_path)?;
        } else if file_type.is_file() {
            fs::copy(&src_path, &dst_path).with_context(|| {
                format!(
                    "Failed to copy file from {} to {}",
                    src_path.display(),
                    dst_path.display()
                )
            })?;
        }
    }
    Ok(())
}

#[cfg(unix)]
fn link_current_interfaces(interfaces_root: &Path, build_root: &Path) -> Result<()> {
    use std::os::unix::fs as unix_fs;

    let link_path = build_root.join("interfaces");
    if link_path.exists() {
        if link_path.is_dir() {
            fs::remove_dir_all(&link_path)?;
        } else {
            fs::remove_file(&link_path)?;
        }
    }
    unix_fs::symlink(interfaces_root, &link_path).with_context(|| {
        format!(
            "Failed to create symlink {} -> {}",
            link_path.display(),
            interfaces_root.display()
        )
    })?;
    Ok(())
}

#[cfg(not(unix))]
fn link_current_interfaces(interfaces_root: &Path, build_root: &Path) -> Result<()> {
    let dst = build_root.join("interfaces");
    if dst.exists() {
        fs::remove_dir_all(&dst)?;
    }
    copy_package_tree(interfaces_root, &dst)
}

fn run_shell(command: &str, current_dir: &Path) -> Result<()> {
    #[cfg(unix)]
    let status = Command::new("bash")
        .arg("-lc")
        .arg(command)
        .current_dir(current_dir)
        .status()
        .with_context(|| format!("Failed to execute shell command in {}", current_dir.display()))?;

    #[cfg(not(unix))]
    let status = Command::new("sh")
        .arg("-lc")
        .arg(command)
        .current_dir(current_dir)
        .status()
        .with_context(|| format!("Failed to execute shell command in {}", current_dir.display()))?;

    if !status.success() {
        anyhow::bail!(
            "Command failed in {} with exit code {:?}: {}",
            current_dir.display(),
            status.code(),
            command
        );
    }
    Ok(())
}

fn shell_quote(path: &Path) -> String {
    format!("\"{}\"", path.display())
}

fn build_local_vnext(package_root: &Path, manifest: &manifest::VNextManifest) -> Result<LocalBuildLayout> {
    let summary = PackageManifest::VNext(manifest.clone()).validate_and_summarize()?;
    let package_name = summary.name.clone();
    let interface_check = manifest::validate_interface_references(&summary, package_root)?;
    let catalog_root = interface_check.catalog_root.with_context(|| {
        format!(
            "Could not locate robonix-interfaces/ridl near {}",
            package_root.display()
        )
    })?;
    let interfaces_root = robonix_interfaces_root_from_catalog(&catalog_root)?;
    let repo_root = repo_root();
    let ridlc_manifest = repo_root.join("ridlc").join("Cargo.toml");

    let build_root = package_root.join(RBNX_BUILD_DIR);
    let workspace_root = build_root.join("ws");
    if build_root.exists() {
        fs::remove_dir_all(&build_root)
            .with_context(|| format!("Failed to clean {}", build_root.display()))?;
    }
    fs::create_dir_all(build_root.join("logs"))?;
    fs::create_dir_all(workspace_root.join("src").join("package"))?;

    copy_package_tree(
        package_root,
        &workspace_root.join("src").join("package").join(&package_name),
    )?;
    link_current_interfaces(&interfaces_root, &build_root)?;

    output::sub_step(&format!("Package root: {}", package_root.display()));
    output::sub_step(&format!("Build root: {}", build_root.display()));
    output::sub_step(&format!("Workspace: {}", workspace_root.display()));
    output::sub_step(&format!("Interface catalog: {}", catalog_root.display()));

    let runtime_interfaces = interfaces_root.join("lib").join("robonix_runtime_interfaces");
    let rcl_interfaces = interfaces_root.join("lib").join("rcl_interfaces");
    let common_interfaces = interfaces_root.join("lib").join("common_interfaces");

    output::step("Generating", "RIDL workspace into rbnx-build/ws");
    let ridlc_command = format!(
        "cargo run --manifest-path {} -- --lang python --layout workspace -I {} -I {} -I {} -o {} -i {}",
        shell_quote(&ridlc_manifest),
        shell_quote(&runtime_interfaces),
        shell_quote(&rcl_interfaces),
        shell_quote(&common_interfaces),
        shell_quote(&workspace_root),
        shell_quote(&catalog_root),
    );
    run_shell(&ridlc_command, &repo_root)?;

    output::step("Building", "colcon workspace under rbnx-build/ws");
    let distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "humble".to_string());
    let ros_setup = format!("/opt/ros/{distro}/setup.bash");
    // Avoid setuptools/packaging canonicalize_version mismatch; do not set PYTHONNOUSERSITE so ~/.local is visible
    let colcon_command = format!(
        "pip3 install --user --upgrade 'packaging>=22.0' 'setuptools>=72' 2>/dev/null || true; set +u; source {ros_setup}; set -u; colcon --log-base {log_base} build --base-paths {generated} {vendor} {app} {package} --build-base {build_base} --install-base {install_base} --packages-up-to robonix_interfaces_app {pkg}",
        ros_setup = shell_quote(Path::new(&ros_setup)),
        generated = shell_quote(&workspace_root.join("src").join("generated")),
        vendor = shell_quote(&workspace_root.join("src").join("vendor")),
        app = shell_quote(&workspace_root.join("src").join("app")),
        package = shell_quote(&workspace_root.join("src").join("package")),
        build_base = shell_quote(&workspace_root.join("build")),
        install_base = shell_quote(&workspace_root.join("install")),
        log_base = shell_quote(&workspace_root.join("log")),
        pkg = package_name,
    );
    run_shell(&colcon_command, package_root)?;

    output::check(&format!("Generated workspace: {}", workspace_root.display()));
    output::check(&format!(
        "Install setup: {}",
        workspace_root.join("install").join("setup.bash").display()
    ));
    output::success(&format!(
        "Local vNext package '{}' built into {}",
        summary.name,
        build_root.display()
    ));
    Ok(LocalBuildLayout {
        package_name: summary.name,
        package_root: package_root.to_path_buf(),
        install_setup: workspace_root.join("install").join("setup.bash"),
    })
}

pub fn build_local_package(path: &Path) -> Result<LocalBuildLayout> {
    let package_root = path
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize package path {}", path.display()))?;

    let detected = manifest::detect_and_load(&package_root)?;
    match detected.manifest {
        PackageManifest::VNext(vnext) => build_local_vnext(&package_root, &vnext),
        PackageManifest::Legacy(_) => anyhow::bail!(
            "build-local currently supports vNext packages only; use 'rbnx package build' for installed legacy packages"
        ),
    }
}

pub async fn execute_local(path: PathBuf) -> Result<()> {
    let package_root = path
        .canonicalize()
        .with_context(|| format!("Failed to canonicalize package path {}", path.display()))?;
    output::action("Building", &format!("local package at {}", package_root.display()));

    build_local_package(&package_root)?;
    Ok(())
}

/// Build packages from package command (no recipe required)
pub async fn execute_package(config: Config, target: String) -> Result<()> {
    let db = PackageDatabase::load(&config.package_storage_path)?;

    let packages_to_build = if target == "all" {
        // Build all installed packages
        db.list_packages().iter().map(|p| p.name.clone()).collect()
    } else {
        // Build specific package
        vec![target]
    };

    let mut built = 0;
    let mut skipped = 0;
    let mut errors = 0;

    for package_name in packages_to_build {
        let pkg_info = db
            .find_by_name(&package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        match build_package(&pkg_info) {
            Ok(_) => built += 1,
            Err(e) => {
                if e.to_string().contains("No build script found") {
                    skipped += 1;
                } else {
                    output::error(&format!("Failed to build {}: {}", package_name, e));
                    errors += 1;
                }
            }
        }
    }

    output::summary(&format!(
        "Summary: {} built, {} skipped, {} errors",
        built, skipped, errors
    ));

    Ok(())
}

