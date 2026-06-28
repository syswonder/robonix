// SPDX-License-Identifier: MulanPSL-2.0
//
// Vitals config — same three-source resolution as executor:
//   compiled defaults < YAML at $ROBONIX_CONFIG_PATH < CLI flags / env.

use anyhow::{Context, Result};
use clap::Parser;
use serde::Deserialize;
use std::path::{Path, PathBuf};

pub const DEFAULT_VITALS_PROVIDER_ID: &str = "vitals";
pub const VITALS_NAMESPACE: &str = "robonix/service/vitals";
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50091";
pub const DEFAULT_COLLECT_INTERVAL_MS: u64 = 1000;

#[derive(Debug, Clone)]
pub struct VitalsConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub id: String,
    pub collect_interval_ms: u64,
    #[allow(dead_code)] // Phase 3 will load threshold rules from this path
    pub thresholds_path: PathBuf,
    /// Body type for hardware health, e.g. "arm" / "dog". None = board-only.
    pub body_type: Option<String>,
    /// Body model e.g. "piper" / "koch" / "go2".
    pub body_model: Option<String>,
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-vitals",
    about = "Robonix Vitals — health monitoring: power state, component health, threshold alerts"
)]
pub struct Args {
    /// Atlas control-plane endpoint.
    #[arg(long, env = "ROBONIX_ATLAS_ENDPOINT")]
    pub atlas: Option<String>,

    /// Address the Vitals gRPC services bind to.
    #[arg(long, env = "ROBONIX_VITALS_LISTEN")]
    pub listen: Option<String>,

    /// Override vitals's provider id (singleton; rarely needed).
    #[arg(long, env = "ROBONIX_VITALS_PROVIDER_ID")]
    pub id: Option<String>,

    /// Sensor polling interval in milliseconds.
    #[arg(long, env = "ROBONIX_VITALS_COLLECT_INTERVAL_MS")]
    pub collect_interval_ms: Option<u64>,

    /// Path to the YAML threshold table (e.g. thresholds/jetson_agx_orin.yaml).
    #[arg(long, env = "ROBONIX_VITALS_THRESHOLDS_PATH")]
    pub thresholds_path: Option<PathBuf>,

    /// Optional YAML config file (rbnx writes this; CLI/env still override).
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// Log filter (env_logger syntax; e.g. `info`, `robonix_vitals=debug`).
    /// Default: `robonix_vitals=info`. Falls back to `RUST_LOG` if unset.
    #[arg(long)]
    pub log: Option<String>,

    /// Body type for hardware health (e.g. "arm", "dog"). Omit for board-only.
    #[arg(long, env = "ROBONIX_VITALS_BODY_TYPE")]
    pub body_type: Option<String>,

    /// Body model (e.g. "piper", "koch", "go2").
    #[arg(long, env = "ROBONIX_VITALS_BODY_MODEL")]
    pub body_model: Option<String>,
}

#[derive(Default, Deserialize)]
struct FileConfig {
    #[serde(default)]
    atlas_endpoint: Option<String>,
    #[serde(default)]
    listen: Option<String>,
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    collect_interval_ms: Option<u64>,
    #[serde(default)]
    thresholds_path: Option<PathBuf>,
    #[serde(default)]
    body_type: Option<String>,
    #[serde(default)]
    body_model: Option<String>,
}

impl VitalsConfig {
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };

        // Default thresholds path: <crate>/thresholds/jetson_agx_orin.yaml
        let default_thresholds = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("thresholds")
            .join("jetson_agx_orin.yaml");

        Ok(Self {
            atlas_endpoint: args
                .atlas
                .or(file_cfg.atlas_endpoint)
                .unwrap_or_else(|| DEFAULT_ATLAS_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(file_cfg.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            id: args
                .id
                .or(file_cfg.id)
                .unwrap_or_else(|| DEFAULT_VITALS_PROVIDER_ID.to_string()),
            collect_interval_ms: args
                .collect_interval_ms
                .or(file_cfg.collect_interval_ms)
                .unwrap_or(DEFAULT_COLLECT_INTERVAL_MS),
            thresholds_path: args
                .thresholds_path
                .or(file_cfg.thresholds_path)
                .unwrap_or(default_thresholds),
            body_type: args.body_type.or(file_cfg.body_type),
            body_model: args.body_model.or(file_cfg.body_model),
        })
    }
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read vitals config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse vitals config '{}'", path.display()))
}
