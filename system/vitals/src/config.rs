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
pub const DEFAULT_MOCK_SOMA_LISTEN: &str = "127.0.0.1:50092";
pub const DEFAULT_COLLECT_INTERVAL_MS: u64 = 1000;
pub const DEFAULT_MOCK_SOMA_INTERVAL_MS: u64 = 10_000;

#[derive(Debug, Clone)]
pub struct VitalsConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub id: String,
    pub collect_interval_ms: u64,
    pub thresholds_path: PathBuf,
    /// Body threshold file (joint temperatures, fault codes per model).
    pub body_thresholds_path: PathBuf,
    pub soma_endpoint: Option<String>,
    pub mock_soma: bool,
    pub mock_soma_id: String,
    pub mock_soma_listen: String,
    pub mock_soma_scenario: String,
    pub mock_soma_interval_ms: u64,
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

    /// Path to the board threshold YAML (e.g. thresholds/jetson_agx_orin.yaml).
    #[arg(long, env = "ROBONIX_VITALS_THRESHOLDS_PATH")]
    pub thresholds_path: Option<PathBuf>,

    /// Path to the body threshold YAML (joint temps, fault codes per model).
    #[arg(long, env = "ROBONIX_VITALS_BODY_THRESHOLDS_PATH")]
    pub body_thresholds_path: Option<PathBuf>,

    /// Optional Soma gRPC endpoint. When set, Vitals consumes SomaHealthSnapshot.
    #[arg(long, env = "ROBONIX_SOMA_ENDPOINT")]
    pub soma_endpoint: Option<String>,

    /// Run this binary as a mock Soma server instead of Vitals.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA", default_value_t = false)]
    pub mock_soma: bool,

    /// Provider id used when --mock-soma registers with Atlas.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_ID")]
    pub mock_soma_id: Option<String>,

    /// Address the mock Soma gRPC services bind to.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_LISTEN")]
    pub mock_soma_listen: Option<String>,

    /// Mock Soma scenario: normal, ramp, fault, toggle, or mixed.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_SCENARIO")]
    pub mock_soma_scenario: Option<String>,

    /// Mock Soma stream update interval in milliseconds.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_INTERVAL_MS")]
    pub mock_soma_interval_ms: Option<u64>,

    /// Optional YAML config file (rbnx writes this; CLI/env still override).
    #[arg(long, env = "ROBONIX_CONFIG_PATH")]
    pub config: Option<PathBuf>,

    /// Log filter (env_logger syntax; e.g. `info`, `robonix_vitals=debug`).
    /// Default: `robonix_vitals=info`. Falls back to `RUST_LOG` if unset.
    #[arg(long)]
    pub log: Option<String>,
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
    body_thresholds_path: Option<PathBuf>,
    #[serde(default)]
    soma_endpoint: Option<String>,
    #[serde(default)]
    mock_soma_id: Option<String>,
    #[serde(default)]
    mock_soma_listen: Option<String>,
    #[serde(default)]
    mock_soma_scenario: Option<String>,
    #[serde(default)]
    mock_soma_interval_ms: Option<u64>,
}

impl VitalsConfig {
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };

        // Default threshold paths: <crate>/thresholds/
        let thresholds_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("thresholds");
        let default_thresholds = thresholds_dir.join("jetson_agx_orin.yaml");
        let default_body_thresholds = thresholds_dir.join("body.yaml");

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
            body_thresholds_path: args
                .body_thresholds_path
                .or(file_cfg.body_thresholds_path)
                .unwrap_or(default_body_thresholds),
            soma_endpoint: args.soma_endpoint.or(file_cfg.soma_endpoint),
            mock_soma: args.mock_soma,
            mock_soma_id: args
                .mock_soma_id
                .or(file_cfg.mock_soma_id)
                .unwrap_or_else(|| "mock-soma".to_string()),
            mock_soma_listen: args
                .mock_soma_listen
                .or(file_cfg.mock_soma_listen)
                .unwrap_or_else(|| DEFAULT_MOCK_SOMA_LISTEN.to_string()),
            mock_soma_scenario: args
                .mock_soma_scenario
                .or(file_cfg.mock_soma_scenario)
                .unwrap_or_else(|| "normal".to_string()),
            mock_soma_interval_ms: args
                .mock_soma_interval_ms
                .or(file_cfg.mock_soma_interval_ms)
                .unwrap_or(DEFAULT_MOCK_SOMA_INTERVAL_MS),
        })
    }
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read vitals config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse vitals config '{}'", path.display()))
}
