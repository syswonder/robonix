// SPDX-License-Identifier: MulanPSL-2.0
//
// body_threshold — per-model joint temperature thresholds for body health
// monitoring.
//
// Thresholds are loaded from a mandatory YAML file (thresholds/body.yaml)
// at startup.  Startup fails if the file is missing or invalid.
//
// Fault code decoding is handled by the Python collector scripts — the
// collector writes decoded fault labels into BodyHealth.message.  Vitals
// only does temperature threshold evaluation here.

use anyhow::Context;
use std::path::Path;
use std::sync::OnceLock;

// ROS IDL health constants (mirrored — same as normalize.rs).
pub const HEALTH_OK: u32 = 0;
pub const HEALTH_WARN: u32 = 1;
pub const HEALTH_ERROR: u32 = 2;

// ── YAML config types ────────────────────────────────────────────────────────

#[derive(serde::Deserialize, Clone, Debug)]
struct ModelThresholds {
    #[serde(default)]
    temp_warn_c: Option<f32>,
    #[serde(default)]
    temp_error_c: Option<f32>,
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
    CONFIG
        .set(cfg)
        .map_err(|_| anyhow::anyhow!("body thresholds already loaded"))
        .ok();
    Ok(())
}

// ── Threshold lookup ─────────────────────────────────────────────────────────

/// Return the temperature thresholds for a given body model.
/// Must be called after load_config() — panics if thresholds are not loaded.
pub fn thresholds_for(model: &str) -> JointThresholds {
    let cfg = CONFIG
        .get()
        .expect("body thresholds not loaded — call load_config() at startup");
    let m = cfg.models.get(model).unwrap_or(&cfg.default);
    JointThresholds {
        temp_warn_c: m.temp_warn_c.or(cfg.default.temp_warn_c).unwrap_or(60.0),
        temp_error_c: m.temp_error_c.or(cfg.default.temp_error_c).unwrap_or(75.0),
    }
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
    fn init_test_config() {
        let _ = CONFIG.get_or_init(|| {
            let mut models: HashMap<String, ModelThresholds> = HashMap::new();
            models.insert(
                "piper".into(),
                ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
                },
            );
            models.insert(
                "koch".into(),
                ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
                },
            );
            models.insert(
                "go2".into(),
                ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
                },
            );
            BodyYaml {
                default: ModelThresholds {
                    temp_warn_c: Some(60.0),
                    temp_error_c: Some(75.0),
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
    }

    #[test]
    fn test_thresholds_koch() {
        init_test_config();
        let t = thresholds_for("koch");
        assert_eq!(t.temp_warn_c, 60.0);
        assert_eq!(t.temp_error_c, 75.0);
    }

    #[test]
    fn test_thresholds_go2() {
        init_test_config();
        let t = thresholds_for("go2");
        assert_eq!(t.temp_warn_c, 60.0);
        assert_eq!(t.temp_error_c, 75.0);
    }

    #[test]
    fn test_thresholds_unknown_fallback() {
        init_test_config();
        let t = thresholds_for("some_future_robot");
        assert_eq!(t.temp_warn_c, 60.0);
        assert_eq!(t.temp_error_c, 75.0);
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
"#;
        let cfg: BodyYaml = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(cfg.default.temp_warn_c, Some(55.0));
        assert_eq!(cfg.default.temp_error_c, Some(70.0));
        let tb = &cfg.models["testbot"];
        assert_eq!(tb.temp_warn_c, Some(50.0));
        assert_eq!(tb.temp_error_c, Some(65.0));
    }

    #[test]
    fn test_yaml_parse_minimal() {
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
        let h = JointTempHealth {
            health: HEALTH_WARN,
        };
        let h2 = h.clone();
        assert_eq!(h2.health, HEALTH_WARN);
    }
}
