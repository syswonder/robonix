use super::recipe_utils;
use crate::{output, Config, PackageDatabase};
use anyhow::{Context, Result};
use colored::*;
use serde_yaml::Value;
use std::process::Command;

/// Build a single package (shared logic)
fn build_package(pkg_info: &crate::database::PackageInfo) -> Result<()> {
    println!(
        "{} {}",
        format!("[{}]", "Building").green().bold(),
        pkg_info.name.bright_white().bold()
    );

    // Load manifest to check for build_script
    let manifest_path = &pkg_info.manifest_path;
    let manifest_content = std::fs::read_to_string(manifest_path)
        .with_context(|| format!("Failed to read manifest: {}", manifest_path.display()))?;
    let manifest: Value = serde_yaml::from_str(&manifest_content)
        .with_context(|| format!("Failed to parse manifest: {}", manifest_path.display()))?;

    // Get build_script from manifest (optional)
    let build_script = manifest["package"]["build_script"].as_str();

    let build_script_path = if let Some(script) = build_script {
        // Use build_script from manifest (relative to package root)
        let script_path = pkg_info.path.join(script);
        if !script_path.exists() {
            anyhow::bail!(
                "Build script not found: {} (specified in manifest)",
                script_path.display()
            );
        }
        script_path
    } else {
        // Default: look for rbnx/build.sh
        let default_script = pkg_info.path.join("rbnx").join("build.sh");
        if !default_script.exists() {
            output::info(&format!(
                "No build script found for {}, skipping build",
                pkg_info.name
            ));
            return Ok(()); // Skip this package, but don't fail
        }
        default_script
    };

    // Make script executable
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut perms = std::fs::metadata(&build_script_path)?.permissions();
        perms.set_mode(0o755);
        std::fs::set_permissions(&build_script_path, perms)?;
    }

    // Execute build script
    output::sub_step(&format!(
        "Running build script: {}",
        build_script_path.display()
    ));

    let build_result = Command::new(&build_script_path)
        .current_dir(&pkg_info.path)
        .status()
        .with_context(|| {
            format!(
                "Failed to execute build script: {}",
                build_script_path.display()
            )
        })?;

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

/// Build packages from deploy command (requires active recipe)
pub async fn execute(config: Config, target: String) -> Result<()> {
    let db = PackageDatabase::load(&config.package_storage_path)?;

    // Get packages from active recipe
    let all_packages = recipe_utils::get_recipe_packages(&config)?;

    let packages_to_build = if target == "all" {
        all_packages
    } else {
        // Check if target matches any package name
        if all_packages.contains(&target) {
            vec![target]
        } else {
            anyhow::bail!("Package '{}' not found in active recipe", target);
        }
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
