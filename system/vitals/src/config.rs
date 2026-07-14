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
pub const VITALS_NAMESPACE: &str = "robonix/system/vitals";
/// Default Atlas control-plane endpoint.
pub const DEFAULT_ATLAS_ENDPOINT: &str = "127.0.0.1:50051";
/// Default Vitals gRPC listen address.
pub const DEFAULT_LISTEN: &str = "127.0.0.1:50093";
/// Default mock Soma gRPC listen address.
pub const DEFAULT_MOCK_SOMA_LISTEN: &str = "127.0.0.1:50092";
/// Default mock Soma stream update interval in milliseconds.
pub const DEFAULT_MOCK_SOMA_INTERVAL_MS: u64 = 10_000;
/// Default Python binary for hardware bridge subprocesses.
pub const DEFAULT_BRIDGE_PYTHON: &str = "python3";
/// Default module health report TTL in milliseconds.
pub const DEFAULT_MODULE_HEALTH_TTL_MS: u32 = 5_000;
/// Executor module health capability.
pub const EXECUTOR_GET_HEALTH_CONTRACT: &str = "robonix/system/executor/get_health";
/// Pilot module health capability.
pub const PILOT_GET_HEALTH_CONTRACT: &str = "robonix/system/pilot/get_health";

/// Mock Soma arm data source: fully synthetic, or real hardware via bridge.
#[derive(Debug, Clone)]
pub enum MockArmConfig {
    /// Fully synthetic arm data — no bridge subprocess.
    Synthetic,
    /// Real Piper arm via piper_sdk on a CAN bus.
    Piper {
        can_port: String,
        python_bin: String,
        script: PathBuf,
    },
    /// Real Koch arm via dynamixel_sdk on a serial port.
    Koch {
        serial_port: String,
        python_bin: String,
        script: PathBuf,
    },
}

impl MockArmConfig {
    /// Short label for logging / display.
    pub fn label(&self) -> &'static str {
        match self {
            Self::Synthetic => "synthetic",
            Self::Piper { .. } => "piper",
            Self::Koch { .. } => "koch",
        }
    }
}

/// Deployment policy for a module that Vitals expects to supervise.
#[derive(Debug, Clone, Copy, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ExpectedModulePolicy {
    Required,
    #[default]
    Optional,
    Disabled,
}

/// One module that Vitals should include in the module-health view.
#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
pub struct ExpectedModuleConfig {
    pub module_id: String,
    #[serde(default)]
    pub provider_id: Option<String>,
    #[serde(default)]
    pub capability: Option<String>,
    #[serde(default)]
    pub policy: ExpectedModulePolicy,
    #[serde(default = "default_module_health_ttl_ms")]
    pub ttl_ms: u32,
}

impl ExpectedModuleConfig {
    pub fn module_key(&self) -> String {
        self.provider_id
            .as_deref()
            .map(str::trim)
            .filter(|provider_id| !provider_id.is_empty())
            .unwrap_or_else(|| self.module_id.trim())
            .to_string()
    }

    pub fn provider_id_or_empty(&self) -> &str {
        self.provider_id
            .as_deref()
            .map(str::trim)
            .unwrap_or_default()
    }
}

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
    /// Mock arm data source (default: Synthetic).
    pub mock_soma_arm: MockArmConfig,
    /// Modules Vitals should supervise in the module-health path.
    pub expected_modules: Vec<ExpectedModuleConfig>,
}

#[derive(Parser, Debug)]
#[command(
    name = "robonix-vitals",
    about = "Robonix Vitals — health monitoring: power state, component health, threshold alerts"
)]
pub struct Args {
    /// Full system manifest block forwarded by rbnx. Typed flags below remain
    /// authoritative; accepting this keeps the standard builtin interface.
    #[arg(long, hide = true)]
    pub config_json: Option<String>,

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

    /// Mock arm data source: synthetic (default), piper, or koch.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_ARM")]
    pub mock_soma_arm: Option<String>,

    /// CAN port for real Piper hardware (e.g. "can0"). Only used with --mock-soma-arm=piper.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_PIPER_CAN")]
    pub mock_soma_piper_can: Option<String>,

    /// Serial port for real Koch hardware (e.g. "/dev/ttyUSB0"). Only used with --mock-soma-arm=koch.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_KOCH_PORT")]
    pub mock_soma_koch_port: Option<String>,

    /// Python binary for hardware bridge subprocesses.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_BRIDGE_PYTHON")]
    pub mock_soma_bridge_python: Option<String>,

    /// Path to piper_bridge.py script.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_PIPER_SCRIPT")]
    pub mock_soma_piper_script: Option<PathBuf>,

    /// Path to koch_bridge.py script.
    #[arg(long, env = "ROBONIX_VITALS_MOCK_SOMA_KOCH_SCRIPT")]
    pub mock_soma_koch_script: Option<PathBuf>,

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
    mock_soma_arm: Option<String>,
    #[serde(default)]
    mock_soma_piper_can: Option<String>,
    #[serde(default)]
    mock_soma_koch_port: Option<String>,
    #[serde(default)]
    mock_soma_bridge_python: Option<String>,
    #[serde(default)]
    mock_soma_piper_script: Option<PathBuf>,
    #[serde(default)]
    mock_soma_koch_script: Option<PathBuf>,
    #[serde(default)]
    expected_modules: Option<Vec<ExpectedModuleConfig>>,
}

impl VitalsConfig {
    /// Resolve configuration from defaults, optional YAML file, and CLI args.
    /// Priority: CLI/env > YAML file > compiled defaults.
    pub fn resolve(args: Args) -> Result<Self> {
        let file_cfg: FileConfig = match &args.config {
            Some(path) => load_yaml(path)?,
            None => FileConfig::default(),
        };

        let thresholds_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("thresholds");
        let default_thresholds = thresholds_dir.join("example_thresholds.yaml");

        let arm_kind = args
            .mock_soma_arm
            .or(file_cfg.mock_soma_arm)
            .unwrap_or_else(|| "synthetic".to_string());

        let bridge_python = args
            .mock_soma_bridge_python
            .or(file_cfg.mock_soma_bridge_python)
            .unwrap_or_else(|| DEFAULT_BRIDGE_PYTHON.to_string());

        let mock_soma_arm = match arm_kind.trim().to_ascii_lowercase().as_str() {
            "piper" => {
                let can = args
                    .mock_soma_piper_can
                    .or(file_cfg.mock_soma_piper_can)
                    .unwrap_or_else(|| "can0".to_string());
                let script = args
                    .mock_soma_piper_script
                    .or(file_cfg.mock_soma_piper_script)
                    .unwrap_or_else(|| {
                        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts/piper_bridge.py")
                    });
                MockArmConfig::Piper {
                    can_port: can,
                    python_bin: bridge_python,
                    script,
                }
            }
            "koch" => {
                let port = args
                    .mock_soma_koch_port
                    .or(file_cfg.mock_soma_koch_port)
                    .unwrap_or_else(|| "/dev/ttyUSB0".to_string());
                let script = args
                    .mock_soma_koch_script
                    .or(file_cfg.mock_soma_koch_script)
                    .unwrap_or_else(|| {
                        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts/koch_bridge.py")
                    });
                MockArmConfig::Koch {
                    serial_port: port,
                    python_bin: bridge_python,
                    script,
                }
            }
            other => {
                if other != "synthetic" {
                    log::warn!(
                        "[vitals] unrecognized --mock-soma-arm '{}', using synthetic",
                        other
                    );
                }
                MockArmConfig::Synthetic
            }
        };

        let id = args
            .id
            .or(file_cfg.id)
            .unwrap_or_else(|| DEFAULT_VITALS_PROVIDER_ID.to_string());
        let expected_modules = file_cfg
            .expected_modules
            .unwrap_or_else(|| default_expected_modules(&id));
        validate_expected_modules(&expected_modules)?;

        Ok(Self {
            atlas_endpoint: args
                .atlas
                .or(file_cfg.atlas_endpoint)
                .unwrap_or_else(|| DEFAULT_ATLAS_ENDPOINT.to_string()),
            listen: args
                .listen
                .or(file_cfg.listen)
                .unwrap_or_else(|| DEFAULT_LISTEN.to_string()),
            id,
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
            mock_soma_arm,
            expected_modules,
        })
    }
}

pub fn default_expected_modules(vitals_provider_id: &str) -> Vec<ExpectedModuleConfig> {
    vec![
        ExpectedModuleConfig {
            module_id: "vitals".to_string(),
            provider_id: Some(vitals_provider_id.to_string()),
            capability: None,
            policy: ExpectedModulePolicy::Required,
            ttl_ms: 0,
        },
        ExpectedModuleConfig {
            module_id: "executor".to_string(),
            provider_id: Some("executor".to_string()),
            capability: Some(EXECUTOR_GET_HEALTH_CONTRACT.to_string()),
            policy: ExpectedModulePolicy::Required,
            ttl_ms: DEFAULT_MODULE_HEALTH_TTL_MS,
        },
        ExpectedModuleConfig {
            module_id: "pilot".to_string(),
            provider_id: Some("pilot".to_string()),
            capability: Some(PILOT_GET_HEALTH_CONTRACT.to_string()),
            policy: ExpectedModulePolicy::Required,
            ttl_ms: DEFAULT_MODULE_HEALTH_TTL_MS,
        },
    ]
}

fn default_module_health_ttl_ms() -> u32 {
    DEFAULT_MODULE_HEALTH_TTL_MS
}

fn validate_expected_modules(modules: &[ExpectedModuleConfig]) -> Result<()> {
    for module in modules {
        if module.module_id.trim().is_empty() {
            anyhow::bail!("expected_modules contains an empty module_id");
        }
    }
    Ok(())
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
        assert!(args.config_json.is_none());
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
    fn args_piper_arm_flags() {
        let args = Args::try_parse_from([
            "robonix-vitals",
            "--mock-soma",
            "--mock-soma-arm",
            "piper",
            "--mock-soma-piper-can",
            "can0",
            "--mock-soma-bridge-python",
            "/usr/bin/python3",
        ])
        .expect("parse piper arm args");
        assert_eq!(args.mock_soma_arm.unwrap(), "piper");
        assert_eq!(args.mock_soma_piper_can.unwrap(), "can0");
        assert_eq!(args.mock_soma_bridge_python.unwrap(), "/usr/bin/python3");
    }

    #[test]
    fn args_koch_arm_flags() {
        let args = Args::try_parse_from([
            "robonix-vitals",
            "--mock-soma",
            "--mock-soma-arm",
            "koch",
            "--mock-soma-koch-port",
            "/dev/ttyUSB0",
        ])
        .expect("parse koch arm args");
        assert_eq!(args.mock_soma_arm.unwrap(), "koch");
        assert_eq!(args.mock_soma_koch_port.unwrap(), "/dev/ttyUSB0");
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
        assert!(matches!(cfg.mock_soma_arm, MockArmConfig::Synthetic));
        assert_eq!(cfg.expected_modules.len(), 3);
        assert_eq!(cfg.expected_modules[0].module_id, "vitals");
        assert_eq!(
            cfg.expected_modules[0].policy,
            ExpectedModulePolicy::Required
        );
        assert_eq!(cfg.expected_modules[1].module_id, "executor");
        assert_eq!(
            cfg.expected_modules[1].policy,
            ExpectedModulePolicy::Required
        );
        assert_eq!(cfg.expected_modules[2].module_id, "pilot");
        assert_eq!(
            cfg.expected_modules[2].policy,
            ExpectedModulePolicy::Required
        );
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

    #[test]
    fn resolve_piper_arm_config() {
        let args = Args::try_parse_from([
            "robonix-vitals",
            "--mock-soma-arm",
            "piper",
            "--mock-soma-piper-can",
            "can1",
        ])
        .unwrap();
        let cfg = VitalsConfig::resolve(args).expect("resolve piper arm");
        match &cfg.mock_soma_arm {
            MockArmConfig::Piper { can_port, .. } => assert_eq!(can_port, "can1"),
            other => panic!("expected Piper, got {:?}", other.label()),
        }
    }

    #[test]
    fn resolve_koch_arm_config() {
        let args = Args::try_parse_from([
            "robonix-vitals",
            "--mock-soma-arm",
            "koch",
            "--mock-soma-koch-port",
            "/dev/ttyUSB1",
        ])
        .unwrap();
        let cfg = VitalsConfig::resolve(args).expect("resolve koch arm");
        match &cfg.mock_soma_arm {
            MockArmConfig::Koch { serial_port, .. } => assert_eq!(serial_port, "/dev/ttyUSB1"),
            other => panic!("expected Koch, got {:?}", other.label()),
        }
    }

    #[test]
    fn file_config_parses_expected_modules() {
        let file_cfg: FileConfig = serde_yaml::from_str(
            r#"
expected_modules:
  - module_id: executor
    provider_id: executor-main
    capability: robonix/system/executor/get_health
    policy: required
    ttl_ms: 3000
  - module_id: speech
    policy: disabled
"#,
        )
        .expect("parse expected modules");

        let modules = file_cfg.expected_modules.expect("expected modules");
        assert_eq!(modules.len(), 2);
        assert_eq!(modules[0].module_key(), "executor-main");
        assert_eq!(modules[0].policy, ExpectedModulePolicy::Required);
        assert_eq!(modules[0].ttl_ms, 3000);
        assert_eq!(modules[1].module_key(), "speech");
        assert_eq!(modules[1].policy, ExpectedModulePolicy::Disabled);
        assert_eq!(modules[1].ttl_ms, DEFAULT_MODULE_HEALTH_TTL_MS);
    }
}
