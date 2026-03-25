// SPDX-License-Identifier: MulanPSL-2.0
// Package database for system-installed packages (~/.robonix/packages)

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageInfo {
    pub name: String,
    pub version: String,
    pub path: PathBuf,
    pub manifest_path: PathBuf,
    #[serde(default)]
    pub primitives: Vec<String>,
    #[serde(default)]
    pub services: Vec<String>,
    #[serde(default)]
    pub skills: Vec<String>,
    #[serde(default)]
    pub provided_interfaces: Vec<String>,
    #[serde(default)]
    pub consumed_interfaces: Vec<String>,
    #[serde(default, alias = "components")]
    pub nodes: Vec<String>,
    pub installed_at: String,
    pub source: PackageSource,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PackageSource {
    Local {
        path: PathBuf,
    },
    GitHub {
        repo: String,
        branch: Option<String>,
        commit: String,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageDatabase {
    packages: HashMap<String, PackageInfo>,
}

impl PackageDatabase {
    pub fn db_path(storage_path: &Path) -> PathBuf {
        storage_path.join("db.json")
    }

    pub fn load(storage_path: &Path) -> Result<Self> {
        let db_path = Self::db_path(storage_path);
        if !db_path.exists() {
            return Ok(Self {
                packages: HashMap::new(),
            });
        }
        let content = std::fs::read_to_string(&db_path)
            .with_context(|| format!("Failed to read database: {}", db_path.display()))?;
        let db: PackageDatabase = serde_json::from_str(&content)
            .with_context(|| format!("Failed to parse database: {}", db_path.display()))?;
        Ok(db)
    }

    pub fn save(&self, storage_path: &Path) -> Result<()> {
        let db_path = Self::db_path(storage_path);
        let content = serde_json::to_string_pretty(self).context("Failed to serialize database")?;
        std::fs::write(&db_path, content)
            .with_context(|| format!("Failed to write database: {}", db_path.display()))?;
        Ok(())
    }

    pub fn add_package(&mut self, info: PackageInfo) {
        self.packages.insert(info.name.clone(), info);
    }

    pub fn remove_package(&mut self, name: &str) -> Option<PackageInfo> {
        self.packages.remove(name)
    }

    pub fn get_package(&self, name: &str) -> Option<&PackageInfo> {
        self.packages.get(name)
    }

    pub fn list_packages(&self) -> Vec<&PackageInfo> {
        let mut packages: Vec<&PackageInfo> = self.packages.values().collect();
        packages.sort_by(|a, b| a.name.cmp(&b.name));
        packages
    }

    pub fn find_by_name(&self, name: &str) -> Option<&PackageInfo> {
        self.packages.get(name)
    }

    pub fn sync(storage_path: &Path) -> Result<()> {
        use crate::install::PackageInstaller;

        let mut db = Self::load(storage_path)?;
        let mut found_packages = HashSet::new();

        if storage_path.exists() {
            for entry in std::fs::read_dir(storage_path)
                .with_context(|| format!("Failed to read storage: {}", storage_path.display()))?
            {
                let entry = entry?;
                let path = entry.path();
                if path.file_name().and_then(|n| n.to_str()) == Some("db.json") {
                    continue;
                }
                if !path.is_dir() {
                    continue;
                }

                let manifest_path = match crate::manifest::detect_manifest_path(&path) {
                    Ok(p) => p,
                    Err(_) => continue,
                };
                if !manifest_path.exists() {
                    continue;
                }

                let package_name = match PackageInstaller::parse_manifest_name(&manifest_path) {
                    Ok(n) => n,
                    Err(e) => {
                        log::warn!(
                            "Failed to parse manifest at {}: {}",
                            manifest_path.display(),
                            e
                        );
                        continue;
                    }
                };
                found_packages.insert(package_name.clone());

                let source = db
                    .get_package(&package_name)
                    .filter(|e| e.path == path)
                    .map(|e| e.source.clone())
                    .unwrap_or(PackageSource::Local { path: path.clone() });

                let summary = match crate::manifest::load_from_path(&manifest_path)
                    .and_then(|m| m.validate_and_summarize())
                {
                    Ok(s) => s,
                    Err(e) => {
                        log::warn!("Failed to parse manifest at {}: {}", path.display(), e);
                        continue;
                    }
                };

                match PackageInstaller::create_package_info(&path, &manifest_path, &summary, source)
                {
                    Ok(info) => db.add_package(info),
                    Err(e) => {
                        log::warn!("Failed to create package info at {}: {}", path.display(), e)
                    }
                }
            }
        }

        for name in db.packages.keys().cloned().collect::<Vec<_>>() {
            if !found_packages.contains(&name) {
                if let Some(removed) = db.remove_package(&name) {
                    log::info!(
                        "Removed '{}' from database (not found: {})",
                        name,
                        removed.path.display()
                    );
                }
            }
        }

        db.save(storage_path)?;
        Ok(())
    }
}
