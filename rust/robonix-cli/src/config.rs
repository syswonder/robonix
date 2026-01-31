// SPDX-License-Identifier: MulanPSL-2.0
// Configuration Module
//
// Configuration management for robonix-cli

use anyhow::{Context, Result};
use dirs;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub package_storage_path: PathBuf,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robonix_sdk_path: Option<PathBuf>,
    /// Node identifier (e.g. hostname) for capabilities registered by this CLI; shown in core web UI.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node_id: Option<String>,
    /// Core web HTTP base URL (e.g. http://core-host:8080) for pushing capability logs when core has a viewer open.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub core_http_url: Option<String>,
}

impl Config {
    pub fn config_file_path() -> Result<PathBuf> {
        // Use ~/.robonix/config.yaml instead of ~/.config/robonix/config.yaml
        let home_dir = dirs::home_dir().context("Failed to get home directory")?;
        Ok(home_dir.join(".robonix").join("config.yaml"))
    }

    pub fn load() -> Result<Self> {
        let config_path = Self::config_file_path()?;

        if !config_path.exists() {
            // Create default config
            let default = Self::default();
            default.save()?;
            return Ok(default);
        }

        let content = std::fs::read_to_string(&config_path)
            .with_context(|| format!("Failed to read config file: {}", config_path.display()))?;

        let config: Config = serde_yaml::from_str(&content)
            .with_context(|| format!("Failed to parse config file: {}", config_path.display()))?;

        Ok(config)
    }

    pub fn save(&self) -> Result<()> {
        let config_path = Self::config_file_path()?;

        // Create parent directory if it doesn't exist
        if let Some(parent) = config_path.parent() {
            std::fs::create_dir_all(parent).with_context(|| {
                format!("Failed to create config directory: {}", parent.display())
            })?;
        }

        let content = serde_yaml::to_string(self).context("Failed to serialize config")?;

        std::fs::write(&config_path, content)
            .with_context(|| format!("Failed to write config file: {}", config_path.display()))?;

        Ok(())
    }

    pub fn default() -> Self {
        let default_path = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("/tmp"))
            .join(".robonix")
            .join("packages");

        Self {
            package_storage_path: default_path,
            robonix_sdk_path: None,
            node_id: None,
            core_http_url: None,
        }
    }

    /// Resolve node_id: config value or hostname.
    pub fn effective_node_id(&self) -> String {
        self.node_id.clone().unwrap_or_else(|| {
            hostname::get()
                .map(|h| h.to_string_lossy().into_owned())
                .unwrap_or_else(|_| String::new())
        })
    }

    pub fn ensure_storage_dir(&self) -> Result<()> {
        // Check if path exists and is a directory (following symlinks)
        if let Ok(metadata) = std::fs::metadata(&self.package_storage_path) {
            if metadata.is_dir() {
                // Directory already exists (or symlink points to directory), nothing to do
                return Ok(());
            } else {
                anyhow::bail!(
                    "Package storage path exists but is not a directory: {}",
                    self.package_storage_path.display()
                );
            }
        }

        // Path doesn't exist, create it
        std::fs::create_dir_all(&self.package_storage_path).with_context(|| {
            format!(
                "Failed to create package storage directory: {}",
                self.package_storage_path.display()
            )
        })?;
        Ok(())
    }
}
