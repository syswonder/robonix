use crate::config::Config;
use crate::database::{PackageDatabase, PackageInfo, PackageSource};
use anyhow::{Context, Result};
use serde_yaml::Value;
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

        let target_path = self.config.package_storage_path.join(repo_name);

        if target_path.exists() {
            anyhow::bail!("Package already exists at: {}", target_path.display());
        }

        // Clone repository
        let mut repo_builder = git2::build::RepoBuilder::new();
        if let Some(branch) = branch {
            repo_builder.branch(branch);
        }

        let git_repo = repo_builder
            .clone(repo, &target_path)
            .context("Failed to clone repository")?;

        let head = git_repo.head().context("Failed to get HEAD")?;
        let commit = head.target().context("Failed to get commit OID")?;

        let commit_str = commit.to_string();

        // Verify package has manifest
        let manifest_path = target_path.join("rbnx_manifest.yaml");
        if !manifest_path.exists() {
            std::fs::remove_dir_all(&target_path)?;
            anyhow::bail!("Package does not have rbnx_manifest.yaml");
        }

        // Parse manifest and add to database
        let package_name = Self::parse_manifest_name(&manifest_path)?;
        let package_info = Self::create_package_info(
            &package_name,
            &target_path,
            &manifest_path,
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
        let manifest_path = source_path.join("rbnx_manifest.yaml");
        if !manifest_path.exists() {
            anyhow::bail!("Package does not have rbnx_manifest.yaml");
        }

        let package_name = Self::parse_manifest_name(&manifest_path)?;
        let target_path = self.config.package_storage_path.join(&package_name);

        if target_path.exists() {
            anyhow::bail!("Package already exists at: {}", target_path.display());
        }

        // Copy package
        copy_dir_all(&source_path, &target_path).with_context(|| {
            format!(
                "Failed to copy package from {} to {}",
                source_path.display(),
                target_path.display()
            )
        })?;

        let manifest_path = target_path.join("rbnx_manifest.yaml");
        let package_info = Self::create_package_info(
            &package_name,
            &target_path,
            &manifest_path,
            PackageSource::Local { path: source_path },
        )?;

        let mut db = PackageDatabase::load(&self.config.package_storage_path)?;
        db.add_package(package_info);
        db.save(&self.config.package_storage_path)?;

        Ok(package_name)
    }

    pub fn parse_manifest_name(manifest_path: &Path) -> Result<String> {
        let content = std::fs::read_to_string(manifest_path)
            .with_context(|| format!("Failed to read manifest: {}", manifest_path.display()))?;

        let manifest: Value = serde_yaml::from_str(&content)
            .with_context(|| format!("Failed to parse manifest: {}", manifest_path.display()))?;

        let name = manifest["package"]["name"]
            .as_str()
            .context("Package name not found in manifest")?;

        Ok(name.to_string())
    }

    pub fn create_package_info(
        name: &str,
        path: &Path,
        manifest_path: &Path,
        source: PackageSource,
    ) -> Result<PackageInfo> {
        let content = std::fs::read_to_string(manifest_path)?;
        let manifest: Value = serde_yaml::from_str(&content)?;

        let version = manifest["package"]["version"]
            .as_str()
            .unwrap_or("0.0.0")
            .to_string();

        let mut capabilities = Vec::new();
        if let Some(caps) = manifest["capabilities"].as_sequence() {
            for cap in caps {
                if let Some(name) = cap["name"].as_str() {
                    capabilities.push(name.to_string());
                }
            }
        }

        let mut skills = Vec::new();
        if let Some(skls) = manifest["skills"].as_sequence() {
            for skl in skls {
                if let Some(name) = skl["name"].as_str() {
                    skills.push(name.to_string());
                }
            }
        }

        let installed_at = chrono::DateTime::<chrono::Utc>::from(SystemTime::now()).to_rfc3339();

        Ok(PackageInfo {
            name: name.to_string(),
            version,
            path: path.to_path_buf(),
            manifest_path: manifest_path.to_path_buf(),
            capabilities,
            skills,
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
