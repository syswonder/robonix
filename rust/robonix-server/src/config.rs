// SPDX-License-Identifier: MulanPSL-2.0
// Core Configuration Module
//
// Configuration management for robonix-server (distributed system, single instance)

use crate::agent::llm::AgentConfig;
use anyhow::{Context, Result};
use dirs;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeechConfig {
    #[serde(default)]
    pub access_token: String,
    #[serde(default)]
    pub appkey: String,
    #[serde(default = "default_region")]
    pub region: String,
}

fn default_region() -> String {
    "shanghai".to_string()
}

impl Default for SpeechConfig {
    fn default() -> Self {
        Self {
            access_token: String::new(),
            appkey: String::new(),
            region: default_region(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoreConfig {
    #[serde(default)]
    pub agent: AgentConfig,
    #[serde(default)]
    pub speech: SpeechConfig,
}

impl CoreConfig {
    /// Get the core config file path
    /// Uses ~/.robonix/core-config.yaml for core-specific configuration
    pub fn config_file_path() -> Result<PathBuf> {
        let home_dir = dirs::home_dir().context("Failed to get home directory")?;
        Ok(home_dir.join(".robonix").join("core-config.yaml"))
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

        let config: CoreConfig = serde_yaml::from_str(&content)
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
        Self {
            agent: AgentConfig::default(),
            speech: SpeechConfig::default(),
        }
    }
}
