// SPDX-License-Identifier: MulanPSL-2.0
//
// Vitals config — same three-source resolution as executor:
//   compiled defaults < YAML at $ROBONIX_CONFIG_PATH < CLI flags / env.

use anyhow::{Context, Result};
use clap::Parser;
use serde::Deserialize;
use std::path::{Path, PathBuf};

/// Default Atlas provider id registered by Vitals.
pub const DEFAULT_VITALS_PROVIDER_ID: &str = "vitals";
/// Atlas service namespace for Vitals capability declarations.
pub const VITALS_NAMESPACE: &str = "robonix/service/vitals";
/// Default Atlas control-plane endpoint.
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
/// Default Vitals gRPC listen address.
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50091";
/// Default mock Soma gRPC listen address.
pub const DEFAULT_MOCK_SOMA_LISTEN: &str = "127.0.0.1:50092";
/// Default mock Soma stream update interval in milliseconds.
pub const DEFAULT_MOCK_SOMA_INTERVAL_MS: u64 = 10_000;

/// Resolved Vitals configuration: compiled defaults < YAML < CLI/env.
#[derive(Debug, Clone)]
pub struct VitalsConfig {
    pub atlas_endpoint: String,
    pub listen: String,
    pub id: String,
    pub thresholds_path: PathBuf,
    pub soma_endpoint: Option<String>,
    pub mock_soma: bool,
    pub mock_soma_id: String,
    pub mock_soma_listen: String,
    pub mock_soma_scenario: String,
    pub mock_soma_interval_ms: u64,
    /// CAN port for real Piper hardware bridge (e.g. "can0"). Empty = synthetic data.
    pub mock_soma_piper_can: Option<String>,
    /// Python binary for the Piper bridge subprocess.
    pub mock_soma_piper_python: String,
    /// Path to piper_bridge.py.
    pub mock_soma_piper_script: PathBuf,
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

    /// Path to the board threshold YAML (e.g. thresholds/example_thresholds.yaml).
    #[arg(long, env = "ROBONIX_VITALS_THRESHOLDS_PATH")]
    pub thresholds_path: Option<PathBuf>,

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

    /// CAN port for real Piper hardware (e.g. "can0"). Empty = fully synthetic mock data.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_PIPER_CAN")]
    pub mock_soma_piper_can: Option<String>,

    /// Python binary for the Piper bridge subprocess.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_PIPER_PYTHON")]
    pub mock_soma_piper_python: Option<String>,

    /// Path to piper_bridge.py script.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_PIPER_SCRIPT")]
    pub mock_soma_piper_script: Option<PathBuf>,

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
    thresholds_path: Option<PathBuf>,
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
    #[serde(default)]
    mock_soma_piper_can: Option<String>,
    #[serde(default)]
    mock_soma_piper_python: Option<String>,
    #[serde(default)]
    mock_soma_piper_script: Option<PathBuf>,
}

impl VitalsConfig {
    /// Resolve configuration from defaults, optional YAML file, and CLI args.
    /// Priority: CLI/env > YAML file > compiled defaults.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };

        // Default threshold paths: <crate>/thresholds/
        let thresholds_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("thresholds");
        let default_thresholds = thresholds_dir.join("example_thresholds.yaml");
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
            thresholds_path: args
                .thresholds_path
                .or(file_cfg.thresholds_path)
                .unwrap_or(default_thresholds),
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
            mock_soma_piper_can: args.mock_soma_piper_can.or(file_cfg.mock_soma_piper_can),
            mock_soma_piper_python: args
                .mock_soma_piper_python
                .or(file_cfg.mock_soma_piper_python)
                .unwrap_or_else(|| "python3".to_string()),
            mock_soma_piper_script: args
                .mock_soma_piper_script
                .or(file_cfg.mock_soma_piper_script)
                .unwrap_or_else(|| {
                    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts/piper_bridge.py")
                }),
        })
    }
}

fn load_yaml(path: &Path) -> Result<FileConfig> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("read vitals config '{}'", path.display()))?;
    serde_yaml::from_str(&raw).with_context(|| format!("parse vitals config '{}'", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn args_defaults() {
        let args = Args::try_parse_from(["robonix-vitals"]).expect("parse empty args");
        assert!(!args.mock_soma);
        assert!(args.atlas.is_none());
        assert!(args.listen.is_none());
        assert!(args.soma_endpoint.is_none());
    }

    #[test]
    fn args_mock_soma_flags() {
        let args = Args::try_parse_from([
            "robonix-vitals",
            "--mock-soma",
            "--mock-soma-scenario",
            "ramp",
            "--mock-soma-interval-ms",
            "5000",
        ])
        .expect("parse mock soma args");
        assert!(args.mock_soma);
        assert_eq!(args.mock_soma_scenario.unwrap(), "ramp");
        assert_eq!(args.mock_soma_interval_ms.unwrap(), 5000);
    }

    #[test]
    fn args_piper_bridge_flags() {
        let args = Args::try_parse_from([
            "robonix-vitals",
            "--mock-soma",
            "--mock-soma-piper-can",
            "can0",
            "--mock-soma-piper-python",
            "/usr/bin/python3",
        ])
        .expect("parse piper bridge args");
        assert_eq!(args.mock_soma_piper_can.unwrap(), "can0");
        assert_eq!(args.mock_soma_piper_python.unwrap(), "/usr/bin/python3");
    }

    #[test]
    fn resolve_defaults_without_yaml() {
        let args = Args::try_parse_from(["robonix-vitals"]).unwrap();
        let cfg = VitalsConfig::resolve(args).expect("resolve defaults");
        assert_eq!(cfg.id, DEFAULT_VITALS_PROVIDER_ID);
        assert_eq!(cfg.atlas_endpoint, DEFAULT_ATLAS_ENDPOINT);
        assert_eq!(cfg.listen, DEFAULT_LISTEN);
        assert_eq!(cfg.mock_soma_scenario, "normal");
        assert_eq!(cfg.mock_soma_interval_ms, DEFAULT_MOCK_SOMA_INTERVAL_MS);
        assert!(cfg.soma_endpoint.is_none());
        assert!(!cfg.mock_soma);
    }

    #[test]
    fn resolve_cli_overrides_yaml_defaults() {
        let args = Args::try_parse_from([
            "robonix-vitals",
            "--id",
            "custom-vitals",
            "--listen",
            "0.0.0.0:9999",
            "--soma-endpoint",
            "10.0.0.1:50092",
        ])
        .unwrap();
        let cfg = VitalsConfig::resolve(args).expect("resolve with overrides");
        assert_eq!(cfg.id, "custom-vitals");
        assert_eq!(cfg.listen, "0.0.0.0:9999");
        assert_eq!(cfg.soma_endpoint.unwrap(), "10.0.0.1:50092");
    }
}
