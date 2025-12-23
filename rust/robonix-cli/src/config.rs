use anyhow::{Context, Result};
use dirs;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub package_storage_path: PathBuf,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robonix_sdk_path: Option<PathBuf>,
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
        }
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
