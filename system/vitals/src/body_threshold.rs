// SPDX-License-Identifier: MulanPSL-2.0
//
// body_threshold — per-model joint temperature thresholds and fault-code
// decoding for body health monitoring.
//
// Thresholds are loaded from a mandatory YAML file (thresholds/body.yaml)
// at startup.  Startup fails if the file is missing or invalid.

use anyhow::Context;
use std::path::Path;
use std::sync::OnceLock;

// ROS IDL health constants (mirrored — same as normalize.rs).
pub const HEALTH_OK: u32 = 0;
pub const HEALTH_WARN: u32 = 1;
pub const HEALTH_ERROR: u32 = 2;

// ── YAML config types ────────────────────────────────────────────────────────

#[derive(serde::Deserialize, Clone, Debug)]
struct FaultEntry {
    bit: u8,
    label: String,
}

#[derive(serde::Deserialize, Clone, Debug)]
struct ModelThresholds {
    #[serde(default)]
    temp_warn_c: Option<f32>,
    #[serde(default)]
    temp_error_c: Option<f32>,
    #[serde(default)]
    faults: Vec<FaultEntry>,
}

#[derive(serde::Deserialize, Clone, Debug)]
struct BodyYaml {
    default: ModelThresholds,
    #[serde(default)]
    models: std::collections::HashMap<String, ModelThresholds>,
}

// ── Runtime threshold type ───────────────────────────────────────────────────

pub struct JointThresholds {
    pub temp_warn_c: f32,
    pub temp_error_c: f32,
    pub fault_labels: Vec<(u8, String)>,
}

// ── Global config ────────────────────────────────────────────────────────────

static CONFIG: OnceLock<BodyYaml> = OnceLock::new();

/// Load body thresholds from a YAML file.  Call once at startup.
/// Returns an error if the file is missing or invalid — no silent fallback.
pub fn load_config(path: &Path) -> anyhow::Result<()> {
    let yaml_str = std::fs::read_to_string(path)
        .with_context(|| format!("read body threshold file '{}'", path.display()))?;
    let cfg: BodyYaml = serde_yaml::from_str(&yaml_str)
        .with_context(|| format!("parse body threshold file '{}'", path.display()))?;
    log::info!(
        "loaded body thresholds from {} ({} models)",
        path.display(),
        cfg.models.len()
    );
    CONFIG.set(cfg).map_err(|_| anyhow::anyhow!("body thresholds already loaded")).ok();
    Ok(())
}

// ── Threshold lookup ─────────────────────────────────────────────────────────

/// Return the threshold table for a given body model.
/// Must be called after load_config() — panics if thresholds are not loaded.
pub fn thresholds_for(model: &str) -> JointThresholds {
    let cfg = CONFIG
        .get()
        .expect("body thresholds not loaded — call load_config() at startup");
    let m = cfg.models.get(model).unwrap_or(&cfg.default);
    JointThresholds {
        temp_warn_c: m
            .temp_warn_c
            .or(cfg.default.temp_warn_c)
            .unwrap_or(60.0),
        temp_error_c: m
            .temp_error_c
            .or(cfg.default.temp_error_c)
            .unwrap_or(75.0),
        fault_labels: m.faults.iter().map(|f| (f.bit, f.label.clone())).collect(),
    }
}

// ── Fault decoding ───────────────────────────────────────────────────────────

/// Decode a joint error_code bitmask into human-readable fault labels.
pub fn decode_faults(model: &str, error_code: u32) -> Vec<String> {
    if error_code == 0 {
        return Vec::new();
    }
    let th = thresholds_for(model);
    let mut labels: Vec<String> = Vec::new();
    for (bit, label) in &th.fault_labels {
        if error_code & (1u32 << bit) != 0 {
            labels.push(label.clone());
        }
    }
    if labels.is_empty() {
        labels.push(format!("0x{:02X}", error_code));
    }
    labels
}

// ── Temperature evaluation ───────────────────────────────────────────────────

/// Evaluate a joint temperature against model thresholds.
/// Returns (health, detail_string).  Returns (OK, "") for unavailable sensors
/// (temp < 0) so that missing sensors don't trigger false positives.
pub fn evaluate_temp(temp_c: f32, model: &str) -> (u32, String) {
    if temp_c < 0.0 {
        return (HEALTH_OK, String::new());
    }
    let th = thresholds_for(model);
    if temp_c >= th.temp_error_c {
        (
            HEALTH_ERROR,
            format!(
                "temp {:.0}°C exceeds ERROR threshold {:.0}°C",
                temp_c, th.temp_error_c
            ),
        )
    } else if temp_c >= th.temp_warn_c {
        (
            HEALTH_WARN,
            format!(
                "temp {:.0}°C exceeds WARN threshold {:.0}°C",
                temp_c, th.temp_warn_c
            ),
        )
    } else {
        (HEALTH_OK, String::new())
    }
}

// ── Per-joint temperature health tracking ────────────────────────────────────

/// Compact representation of a joint's last-known threshold health so that
/// service.rs can detect OK→WARN→ERROR transitions.
#[derive(Clone, Debug)]
pub struct JointTempHealth {
    pub health: u32,
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// Initialize the global CONFIG with test data matching body.yaml.
    /// Uses get_or_init so it's safe to call in every test — only the first
    /// call actually runs the initializer; subsequent calls are no-ops.
    fn init_test_config() {
        let _ = CONFIG.get_or_init(|| {
            let mut models: HashMap<String, ModelThresholds> = HashMap::new();
            models.insert(
                "piper".into(),
                ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
                    faults: vec![
                        FaultEntry { bit: 0, label: "欠压".into() },
                        FaultEntry { bit: 1, label: "电机过热".into() },
                        FaultEntry { bit: 2, label: "过流".into() },
                        FaultEntry { bit: 3, label: "驱动器过热".into() },
                        FaultEntry { bit: 4, label: "碰撞".into() },
                        FaultEntry { bit: 5, label: "驱动器故障".into() },
                        FaultEntry { bit: 7, label: "堵转".into() },
                    ],
                },
            );
            models.insert(
                "koch".into(),
                ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
                    faults: vec![
                        FaultEntry { bit: 0, label: "电压异常".into() },
                        FaultEntry { bit: 2, label: "过热".into() },
                        FaultEntry { bit: 3, label: "编码器故障".into() },
                        FaultEntry { bit: 5, label: "过载".into() },
                    ],
                },
            );
            models.insert(
                "go2".into(),
                ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
                    faults: vec![],
                },
            );
            BodyYaml {
                default: ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
                    faults: vec![],
                },
                models,
            }
        });
    }

    // ── thresholds_for ───────────────────────────────────────────────────

    #[test]
    fn test_thresholds_piper() {
        init_test_config();
        let t = thresholds_for("piper");
        assert_eq!(t.temp_warn_c, 60.0);
        assert_eq!(t.temp_error_c, 75.0);
        assert_eq!(t.fault_labels.len(), 7);
        assert_eq!(t.fault_labels[0], (0, "欠压".into()));
        assert_eq!(t.fault_labels[1], (1, "电机过热".into()));
        assert_eq!(t.fault_labels[6], (7, "堵转".into()));
    }

    #[test]
    fn test_thresholds_koch() {
        init_test_config();
        let t = thresholds_for("koch");
        assert_eq!(t.temp_warn_c, 60.0);
        assert_eq!(t.temp_error_c, 75.0);
        assert_eq!(t.fault_labels.len(), 4);
    }

    #[test]
    fn test_thresholds_go2() {
        init_test_config();
        let t = thresholds_for("go2");
        assert_eq!(t.temp_warn_c, 60.0);
        assert_eq!(t.temp_error_c, 75.0);
        assert!(t.fault_labels.is_empty());
    }

    #[test]
    fn test_thresholds_unknown_fallback() {
        init_test_config();
        let t = thresholds_for("some_future_robot");
        assert_eq!(t.temp_warn_c, 60.0);
        assert_eq!(t.temp_error_c, 75.0);
        assert!(t.fault_labels.is_empty());
    }

    // ── YAML loading ─────────────────────────────────────────────────────

    #[test]
    fn test_load_yaml_config() {
        let yaml = r#"
default:
  temp_warn_c: 55.0
  temp_error_c: 70.0
models:
  testbot:
    temp_warn_c: 50.0
    temp_error_c: 65.0
    faults:
      - bit: 0
        label: "故障A"
      - bit: 1
        label: "故障B"
"#;
        let cfg: BodyYaml = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(cfg.default.temp_warn_c, Some(55.0));
        assert_eq!(cfg.default.temp_error_c, Some(70.0));
        let tb = &cfg.models["testbot"];
        assert_eq!(tb.temp_warn_c, Some(50.0));
        assert_eq!(tb.faults.len(), 2);
        assert_eq!(tb.faults[0].label, "故障A");
    }

    #[test]
    fn test_yaml_parse_minimal() {
        // Parse-only test — does NOT touch the global OnceLock.
        let yaml = r#"
default:
  temp_warn_c: 55.0
  temp_error_c: 70.0
models: {}
"#;
        let cfg: BodyYaml = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(cfg.default.temp_warn_c, Some(55.0));
        assert_eq!(cfg.default.temp_error_c, Some(70.0));
        assert!(cfg.models.is_empty());
    }

    // ── decode_faults ───────────────────────────────────────────────────

    #[test]
    fn test_decode_no_fault() {
        init_test_config();
        let faults = decode_faults("piper", 0);
        assert!(faults.is_empty());
    }

    #[test]
    fn test_decode_single_fault() {
        init_test_config();
        let faults = decode_faults("piper", 1 << 2);
        assert_eq!(faults, vec!["过流"]);
    }

    #[test]
    fn test_decode_multiple_faults() {
        init_test_config();
        let code = (1u32 << 0) | (1u32 << 2) | (1u32 << 7);
        let faults = decode_faults("piper", code);
        assert_eq!(faults, vec!["欠压", "过流", "堵转"]);
    }

    #[test]
    fn test_decode_unknown_bit_falls_back_to_hex() {
        init_test_config();
        let faults = decode_faults("piper", 1 << 6);
        assert_eq!(faults, vec!["0x40"]);
    }

    #[test]
    fn test_decode_mixed_known_and_unknown() {
        init_test_config();
        let code = (1u32 << 1) | (1u32 << 6);
        let faults = decode_faults("piper", code);
        assert_eq!(faults, vec!["电机过热"]);
    }

    #[test]
    fn test_decode_no_labels_for_model() {
        init_test_config();
        let faults = decode_faults("go2", 1 << 2);
        assert_eq!(faults, vec!["0x04"]);
    }

    // ── evaluate_temp ───────────────────────────────────────────────────

    #[test]
    fn test_evaluate_temp_ok() {
        init_test_config();
        let (health, detail) = evaluate_temp(35.0, "piper");
        assert_eq!(health, HEALTH_OK);
        assert!(detail.is_empty());
    }

    #[test]
    fn test_evaluate_temp_warn() {
        init_test_config();
        let (health, detail) = evaluate_temp(65.0, "piper");
        assert_eq!(health, HEALTH_WARN);
        assert!(detail.contains("65°C"));
        assert!(detail.contains("60°C"));
    }

    #[test]
    fn test_evaluate_temp_error() {
        init_test_config();
        let (health, detail) = evaluate_temp(80.0, "piper");
        assert_eq!(health, HEALTH_ERROR);
        assert!(detail.contains("80°C"));
        assert!(detail.contains("75°C"));
    }

    #[test]
    fn test_evaluate_temp_boundary_warn() {
        init_test_config();
        let (health, _) = evaluate_temp(60.0, "piper");
        assert_eq!(health, HEALTH_WARN);
    }

    #[test]
    fn test_evaluate_temp_boundary_error() {
        init_test_config();
        let (health, _) = evaluate_temp(75.0, "piper");
        assert_eq!(health, HEALTH_ERROR);
    }

    #[test]
    fn test_evaluate_temp_unavailable_sensor() {
        // No init_test_config() needed — returns early on temp < 0.
        let (health, detail) = evaluate_temp(-1.0, "piper");
        assert_eq!(health, HEALTH_OK);
        assert!(detail.is_empty());
    }

    #[test]
    fn test_evaluate_temp_different_models_same_thresholds() {
        init_test_config();
        for model in &["piper", "koch", "go2"] {
            let (h_ok, _) = evaluate_temp(50.0, model);
            assert_eq!(h_ok, HEALTH_OK, "model={model}");
            let (h_warn, _) = evaluate_temp(70.0, model);
            assert_eq!(h_warn, HEALTH_WARN, "model={model}");
            let (h_err, _) = evaluate_temp(80.0, model);
            assert_eq!(h_err, HEALTH_ERROR, "model={model}");
        }
    }

    // ── JointTempHealth ─────────────────────────────────────────────────

    #[test]
    fn test_joint_temp_health_clone() {
        let h = JointTempHealth { health: HEALTH_WARN };
        let h2 = h.clone();
        assert_eq!(h2.health, HEALTH_WARN);
    }
}
