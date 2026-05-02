// SPDX-License-Identifier: MulanPSL-2.0
// Package installation (GitHub, local path) into ~/.robonix/packages

use crate::config::Config;
use crate::database::{PackageDatabase, PackageInfo, PackageSource};
use crate::manifest;
use crate::output;
use anyhow::{Context, Result};
use std::io::{self, Write};
use std::path::Path;
use std::time::SystemTime;

pub struct PackageInstaller {
    config: Config,
}

fn normalize_github_url(repo: &str) -> String {
    let s = repo.trim().trim_end_matches('/');
    if s.starts_with("http://") || s.starts_with("https://") || s.starts_with("git@") {
        return s.to_string();
    }
    if s.contains('/') && !s.contains(' ') {
        return format!("https://github.com/{}.git", s.trim_end_matches(".git"));
    }
    s.to_string()
}

impl PackageInstaller {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    pub fn install_from_github(&self, repo: &str, branch: Option<&str>) -> Result<String> {
        let clone_url = normalize_github_url(repo);
        let repo_url = repo.to_string();
        let repo_name = repo
            .split('/')
            .next_back()
            .context("Invalid GitHub repository format")?
            .trim_end_matches(".git");

        let temp_path = self
            .config
            .package_storage_path
            .join(format!("{}_temp", repo_name));
        if temp_path.exists() {
            std::fs::remove_dir_all(&temp_path)?;
        }

        let mut repo_builder = git2::build::RepoBuilder::new();
        if let Some(b) = branch {
            repo_builder.branch(b);
        }

        let git_repo = repo_builder
            .clone(&clone_url, &temp_path)
            .context("Failed to clone repository")?;

        let head = git_repo.head().context("Failed to get HEAD")?;
        let commit = head.target().context("Failed to get commit OID")?;
        let commit_str = commit.to_string();

        let detected = manifest::detect_and_load(&temp_path)?;
        output::step("Validating", "package manifest");
        let summary = detected
            .manifest
            .validate_and_summarize()
            .with_context(|| "Invalid robonix package manifest")?;
        let package_name = summary.name.clone();
        let version = summary.version.clone();
        let target_path = self.config.package_storage_path.join(&package_name);

        output::sub_step(&format!("Package: {} {}", package_name, version));

        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        if let Some(existing) = db.get_package(&package_name) {
            if !Self::prompt_overwrite(&existing.version, &version, &package_name)? {
                std::fs::remove_dir_all(&temp_path)?;
                output::info("Installation cancelled.");
                std::process::exit(0);
            }
            if existing.path.exists() {
                std::fs::remove_dir_all(&existing.path)?;
            }
        }

        if target_path.exists() {
            std::fs::remove_dir_all(&target_path)?;
        }
        std::fs::rename(&temp_path, &target_path)?;

        let manifest_path = target_path.join(
            detected
                .path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or(manifest::MANIFEST_FILE),
        );
        let package_info = Self::create_package_info(
            &target_path,
            &manifest_path,
            &summary,
            PackageSource::GitHub {
                repo: repo_url,
                branch: branch.map(String::from),
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
            .with_context(|| format!("Failed to canonicalize: {}", source_path.display()))?;

        let detected = manifest::detect_and_load(&source_path)?;
        output::step("Validating", "package manifest");
        let summary = detected
            .manifest
            .validate_and_summarize()
            .with_context(|| "Invalid robonix package manifest")?;
        let package_name = summary.name.clone();
        let version = summary.version.clone();

        output::sub_step(&format!("Package: {} {}", package_name, version));

        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        if let Some(existing) = db.get_package(&package_name) {
            if !Self::prompt_overwrite(&existing.version, &version, &package_name)? {
                output::info("Installation cancelled.");
                std::process::exit(0);
            }
            if existing.path.exists() {
                std::fs::remove_dir_all(&existing.path)?;
            }
        }

        let target_path = self.config.package_storage_path.join(&package_name);
        copy_dir_all(&source_path, &target_path).with_context(|| {
            format!(
                "Failed to copy from {} to {}",
                source_path.display(),
                target_path.display()
            )
        })?;

        let manifest_path = target_path.join(
            detected
                .path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or(manifest::MANIFEST_FILE),
        );
        let package_info = Self::create_package_info(
            &target_path,
            &manifest_path,
            &summary,
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

    fn prompt_overwrite(old_version: &str, new_version: &str, package_name: &str) -> Result<bool> {
        output::warning(&format!("Package '{}' is already installed", package_name));
        output::sub_step(&format!("Old: {}  New: {}", old_version, new_version));
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
            capabilities: summary.capabilities.clone(),
            depends: summary.depends.clone(),
            installed_at,
            source,
        })
    }
}

fn copy_dir_all(src: &Path, dst: &Path) -> Result<()> {
    std::fs::create_dir_all(dst)?;
    for entry in std::fs::read_dir(src)? {
        let entry = entry?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_dir_all(&src_path, &dst_path)?;
        } else {
            std::fs::copy(&src_path, &dst_path)?;
        }
    }
    Ok(())
}
