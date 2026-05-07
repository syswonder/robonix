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
    /// Absolute path to the cloned robonix repo root (the directory containing `rust/`).
    /// Set by `rbnx setup` from inside a working copy. Required so out-of-tree packages
    /// (e.g. mapping_rbnx on a robot) can find capabilities/ and rust/crates/robonix-interfaces/lib.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robonix_source_path: Option<PathBuf>,
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

    /// Validates that the config has been upgraded for the new `rbnx setup` flow
    /// (robonix_source_path present + pointing to an existing tree). If not,
    /// prints a migration hint and exits; call this only from subcommands that
    /// actually need source paths. Setup / Config / Path itself are exempt.
    pub fn require_source_path(&self) -> Result<&std::path::Path> {
        match self.robonix_source_path.as_deref() {
            Some(p) if p.exists() => Ok(p),
            Some(p) => {
                eprintln!(
                    "[rbnx] configured robonix_source_path no longer exists: {}",
                    p.display()
                );
                eprintln!(
                    "Re-run `rbnx setup` from the robonix source repo root (containing `rust/`)."
                );
                std::process::exit(2);
            }
            None => {
                eprintln!(
                    "[rbnx] config is missing robonix_source_path (legacy config from before the `rbnx setup` migration)."
                );
                eprintln!(
                    "This is required so packages anywhere on disk can resolve capabilities/IDL paths."
                );
                eprintln!();
                eprintln!("Fix:  cd /path/to/robonix   # the repo root (containing `rust/`)");
                eprintln!("      rbnx setup");
                eprintln!();
                eprintln!(
                    "Config file: {}",
                    Self::config_file_path()
                        .map(|p| p.display().to_string())
                        .unwrap_or_else(|_| "~/.robonix/config.yaml".to_string())
                );
                std::process::exit(2);
            }
        }
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

    #[allow(clippy::should_implement_trait)]
    pub fn default() -> Self {
        let default_path = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("/tmp"))
            .join(".robonix")
            .join("packages");

        Self {
            package_storage_path: default_path,
            robonix_source_path: None,
        }
    }

    /// Resolve a well-known path rooted in the robonix source tree.
    /// Returns an error if `robonix_source_path` is unset (tell user to run `rbnx setup`)
    /// or if the computed path doesn't exist.
    pub fn resolve_source_path(&self, key: SourcePathKey) -> Result<PathBuf> {
        let root = self.robonix_source_path.as_ref().ok_or_else(|| {
            anyhow::anyhow!(
                "robonix_source_path is not set. Run `rbnx setup` from the robonix source root (the directory containing `rust/`)."
            )
        })?;
        let abs = match key {
            SourcePathKey::Root => root.clone(),
            SourcePathKey::RustRoot => root.join("rust"),
            SourcePathKey::Capabilities => root.join("capabilities"),
            // capabilities/lib is the unified IDL root: a relative
            // symlink (or in the future, real files) into the
            // robonix-interfaces tree. Codegen and any downstream IDL
            // search starts from here so msg/srv references in contract
            // TOMLs (e.g. `[io.srv].srv = "demo/srv/Hello"`) have a
            // single, unambiguous base.
            SourcePathKey::InterfacesLib => root.join("capabilities").join("lib"),
            SourcePathKey::RuntimeProto => root.join("rust").join("proto"),
            SourcePathKey::RobonixApi => root.join("pylib").join("robonix-api"),
        };
        if !abs.exists() {
            anyhow::bail!(
                "resolved path does not exist: {} (robonix_source_path={}). The source tree may be incomplete — re-run `rbnx setup` from the correct root.",
                abs.display(),
                root.display()
            );
        }
        Ok(abs)
    }
}

/// Well-known paths a package build.sh might need from the robonix source tree.
#[derive(Debug, Clone, Copy)]
pub enum SourcePathKey {
    /// Repository root (the dir containing `rust/`, `docs/`, etc.).
    Root,
    /// `<root>/rust` (cargo workspace).
    RustRoot,
    /// `<root>/capabilities` (contract TOMLs).
    Capabilities,
    /// `<root>/rust/crates/robonix-interfaces/lib` (ROS IDL source).
    InterfacesLib,
    /// `<root>/rust/proto` (runtime / atlas protos).
    RuntimeProto,
    /// `<root>/pylib/robonix-api` — shared Python helper lib.
    /// Carries `mcp_contract` (codegen IO class → FastMCP tool wrapper).
    /// Add this dir to PYTHONPATH; `from robonix_api import mcp_contract`.
    RobonixApi,
}

impl std::str::FromStr for SourcePathKey {
    type Err = String;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s {
            "root" | "source" => Ok(Self::Root),
            "rust" | "rust-root" => Ok(Self::RustRoot),
            "capabilities" => Ok(Self::Capabilities),
            "interfaces-lib" | "idl" => Ok(Self::InterfacesLib),
            "runtime-proto" => Ok(Self::RuntimeProto),
            "robonix-api" => Ok(Self::RobonixApi),
            other => Err(format!(
                "unknown path key: {other}. Valid: root, rust, capabilities, interfaces-lib, runtime-proto, robonix-api"
            )),
        }
    }
}

impl Config {
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
