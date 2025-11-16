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
    pub capabilities: Vec<String>, // List of std_name
    pub skills: Vec<String>,       // List of std_name
    pub installed_at: String,      // ISO 8601 timestamp
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

    pub fn find_by_capability(&self, cap_name: &str) -> Vec<&PackageInfo> {
        let mut packages: Vec<&PackageInfo> = self.packages
            .values()
            .filter(|pkg| pkg.capabilities.iter().any(|c| c == cap_name))
            .collect();
        packages.sort_by(|a, b| a.name.cmp(&b.name));
        packages
    }

    pub fn find_by_skill(&self, skill_name: &str) -> Vec<&PackageInfo> {
        let mut packages: Vec<&PackageInfo> = self.packages
            .values()
            .filter(|pkg| pkg.skills.iter().any(|s| s == skill_name))
            .collect();
        packages.sort_by(|a, b| a.name.cmp(&b.name));
        packages
    }

    pub fn find_by_name(&self, name: &str) -> Option<&PackageInfo> {
        self.packages.get(name)
    }

    /// Sync database with actual packages in storage path
    /// This scans the storage directory and updates the database to match the filesystem
    pub fn sync(storage_path: &Path) -> Result<()> {
        use crate::install::PackageInstaller;

        // Load existing database
        let mut db = Self::load(storage_path)?;

        // Track which packages we found in the filesystem
        let mut found_packages = HashSet::new();

        // Scan storage directory for packages
        if storage_path.exists() {
            for entry in std::fs::read_dir(storage_path).with_context(|| {
                format!(
                    "Failed to read storage directory: {}",
                    storage_path.display()
                )
            })? {
                let entry = entry?;
                let path = entry.path();

                // Skip db.json file
                if path.file_name().and_then(|n| n.to_str()) == Some("db.json") {
                    continue;
                }

                // Only process directories
                if !path.is_dir() {
                    continue;
                }

                // Check if this directory contains a manifest
                let manifest_path = path.join("rbnx_manifest.yaml");
                if !manifest_path.exists() {
                    continue;
                }

                // Parse manifest and create/update package info
                match PackageInstaller::parse_manifest_name(&manifest_path) {
                    Ok(package_name) => {
                        found_packages.insert(package_name.clone());

                        // Determine source (try to preserve existing source if available)
                        let source = if let Some(existing) = db.get_package(&package_name) {
                            // Preserve existing source if path matches
                            if existing.path == path {
                                existing.source.clone()
                            } else {
                                // Path changed, treat as local
                                PackageSource::Local { path: path.clone() }
                            }
                        } else {
                            // New package, treat as local
                            PackageSource::Local { path: path.clone() }
                        };

                        // Create or update package info
                        match PackageInstaller::create_package_info(
                            &package_name,
                            &path,
                            &manifest_path,
                            source,
                        ) {
                            Ok(package_info) => {
                                db.add_package(package_info);
                            }
                            Err(e) => {
                                tracing::warn!(
                                    "Failed to parse manifest for package at {}: {}",
                                    path.display(),
                                    e
                                );
                            }
                        }
                    }
                    Err(e) => {
                        tracing::warn!(
                            "Failed to parse manifest at {}: {}",
                            manifest_path.display(),
                            e
                        );
                    }
                }
            }
        }

        // Remove packages from database that no longer exist in filesystem
        let db_package_names: Vec<String> = db.packages.keys().cloned().collect();
        for package_name in db_package_names {
            if !found_packages.contains(&package_name) {
                if let Some(removed) = db.remove_package(&package_name) {
                    tracing::info!(
                        "Removed package '{}' from database (directory not found: {})",
                        package_name,
                        removed.path.display()
                    );
                }
            }
        }

        // Save updated database
        db.save(storage_path)?;

        Ok(())
    }
}
