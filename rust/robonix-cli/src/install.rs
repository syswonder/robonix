// SPDX-License-Identifier: MulanPSL-2.0
// Install Module
//
// Package installation functionality for robonix-cli

use crate::config::Config;
use crate::database::{PackageDatabase, PackageInfo, PackageSource};
use crate::manifest::{self};
use crate::output;
use anyhow::{Context, Result};
use std::io::{self, Write};
use std::path::Path;
use std::time::SystemTime;

pub struct PackageInstaller {
    config: Config,
}

impl PackageInstaller {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    pub fn install_from_github(&self, repo: &str, branch: Option<&str>) -> Result<String> {
        let repo_url = repo.to_string(); // Save original repo URL
        let repo_name = repo
            .split('/')
            .last()
            .context("Invalid GitHub repository format")?
            .trim_end_matches(".git");

        // Clone repository to temporary location first
        let temp_path = self
            .config
            .package_storage_path
            .join(format!("{}_temp", repo_name));
        if temp_path.exists() {
            std::fs::remove_dir_all(&temp_path)?;
        }

        let mut repo_builder = git2::build::RepoBuilder::new();
        if let Some(branch) = branch {
            repo_builder.branch(branch);
        }

        let git_repo = repo_builder
            .clone(repo, &temp_path)
            .context("Failed to clone repository")?;

        let head = git_repo.head().context("Failed to get HEAD")?;
        let commit = head.target().context("Failed to get commit OID")?;

        let commit_str = commit.to_string();

        // Verify package has manifest
        let detected_manifest = manifest::detect_and_load(&temp_path)?;

        // Validate manifest and get package info
        output::step("Validating", "package manifest");
        let manifest_summary = detected_manifest
            .manifest
            .validate_and_summarize()
            .with_context(|| "Invalid robonix package manifest")?;
        let package_name = manifest_summary.name.clone();
        let version = manifest_summary.version.clone();
        let target_path = self.config.package_storage_path.join(&package_name);

        // Display validation info
        output::sub_step(&format!("Package name: {}", package_name));
        output::sub_step(&format!("Version: {}", version));
        output::sub_step(&format!(
            "Manifest kind: {}",
            manifest_summary.manifest_kind.label()
        ));

        // Check if package already exists by name
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        if let Some(existing_pkg) = db.get_package(&package_name) {
            let old_version = &existing_pkg.version;

            if !Self::prompt_overwrite(old_version, &version, &package_name)? {
                std::fs::remove_dir_all(&temp_path)?;
                output::info("Installation cancelled.");
                std::process::exit(0);
            }

            // Remove old package directory if it exists
            if existing_pkg.path.exists() {
                std::fs::remove_dir_all(&existing_pkg.path)?;
            }
        }

        // Move temp directory to final location
        if target_path.exists() {
            std::fs::remove_dir_all(&target_path)?;
        }
        std::fs::rename(&temp_path, &target_path)?;

        // Create package info and add to database
        let package_info = Self::create_package_info(
            &target_path,
            &target_path.join(
                detected_manifest
                    .path
                    .file_name()
                    .and_then(|name| name.to_str())
                    .unwrap_or(manifest::LEGACY_MANIFEST_FILE),
            ),
            &manifest_summary,
            PackageSource::GitHub {
                repo: repo_url,
                branch: branch.map(|s| s.to_string()),
                commit: commit_str,
            },
        )?;

        let mut db = PackageDatabase::load(&self.config.package_storage_path)?;
        db.add_package(package_info);
        db.save(&self.config.package_storage_path)?;

        Ok(package_name)
    }

    pub fn install_from_path(&self, source_path: &Path) -> Result<String> {
        let source_path = source_path
            .canonicalize()
            .with_context(|| format!("Failed to canonicalize path: {}", source_path.display()))?;

        // Verify package has manifest
        let detected_manifest = manifest::detect_and_load(&source_path)?;

        // Validate manifest and get package info
        output::step("Validating", "package manifest");
        let manifest_summary = detected_manifest
            .manifest
            .validate_and_summarize()
            .with_context(|| "Invalid robonix package manifest")?;
        let package_name = manifest_summary.name.clone();
        let version = manifest_summary.version.clone();

        // Display validation info
        output::sub_step(&format!("Package name: {}", package_name));
        output::sub_step(&format!("Version: {}", version));
        output::sub_step(&format!(
            "Manifest kind: {}",
            manifest_summary.manifest_kind.label()
        ));

        // Check if package already exists by name
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        if let Some(existing_pkg) = db.get_package(&package_name) {
            let old_version = &existing_pkg.version;

            if !Self::prompt_overwrite(old_version, &version, &package_name)? {
                output::info("Installation cancelled.");
                std::process::exit(0);
            }

            // Remove old package directory if it exists
            if existing_pkg.path.exists() {
                std::fs::remove_dir_all(&existing_pkg.path)?;
            }
        }

        let target_path = self.config.package_storage_path.join(&package_name);

        // Copy package
        copy_dir_all(&source_path, &target_path).with_context(|| {
            format!(
                "Failed to copy package from {} to {}",
                source_path.display(),
                target_path.display()
            )
        })?;

        let manifest_path = target_path.join(
            detected_manifest
                .path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or(manifest::LEGACY_MANIFEST_FILE),
        );
        let package_info = Self::create_package_info(
            &target_path,
            &manifest_path,
            &manifest_summary,
            PackageSource::Local { path: source_path },
        )?;

        let mut db = PackageDatabase::load(&self.config.package_storage_path)?;
        db.add_package(package_info);
        db.save(&self.config.package_storage_path)?;

        Ok(package_name)
    }

    pub fn parse_manifest_name(manifest_path: &Path) -> Result<String> {
        let manifest = manifest::load_from_path(manifest_path)?;
        Ok(manifest.validate_and_summarize()?.name)
    }

    /// Validate that the manifest is a valid robonix package
    /// Returns (name, version) if valid
    pub fn validate_manifest(manifest_path: &Path) -> Result<(String, String)> {
        let manifest = manifest::load_from_path(manifest_path)?;
        let summary = manifest.validate_and_summarize()?;
        Ok((summary.name, summary.version))
    }

    /// Prompt user for confirmation to overwrite existing package
    fn prompt_overwrite(old_version: &str, new_version: &str, package_name: &str) -> Result<bool> {
        output::warning(&format!("Package '{}' is already installed", package_name));
        output::sub_step(&format!("Old version: {}", old_version));
        output::sub_step(&format!("New version: {}", new_version));
        print!("Overwrite? [y/N]: ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;

        let answer = input.trim().to_lowercase();
        Ok(answer == "y" || answer == "yes")
    }

    pub fn create_package_info(
        path: &Path,
        manifest_path: &Path,
        summary: &crate::manifest::PackageSummary,
        source: PackageSource,
    ) -> Result<PackageInfo> {
        let installed_at = chrono::DateTime::<chrono::Utc>::from(SystemTime::now()).to_rfc3339();

        Ok(PackageInfo {
            name: summary.name.clone(),
            version: summary.version.clone(),
            path: path.to_path_buf(),
            manifest_path: manifest_path.to_path_buf(),
            manifest_kind: summary.manifest_kind.clone(),
            primitives: summary.primitives.clone(),
            services: summary.services.clone(),
            skills: summary.skills.clone(),
            provided_interfaces: summary.provided_interfaces.clone(),
            consumed_interfaces: summary.consumed_interfaces.clone(),
            nodes: summary.nodes.clone(),
            installed_at,
            source,
        })
    }
}

fn copy_dir_all(src: &Path, dst: &Path) -> Result<()> {
    std::fs::create_dir_all(dst)?;
    for entry in std::fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());

        if ty.is_dir() {
            copy_dir_all(&src_path, &dst_path)?;
        } else {
            std::fs::copy(&src_path, &dst_path)?;
        }
    }
    Ok(())
}
