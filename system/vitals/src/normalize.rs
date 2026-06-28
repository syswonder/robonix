// SPDX-License-Identifier: MulanPSL-2.0
//
// normalize — threshold-check raw sensor values against a YAML threshold table,
// producing ComponentHealth entries with OK / WARN / ERROR status.
//
// Phase 3 will integrate with the collector for full normalization pipeline.

use crate::pb::vitals::ComponentHealth;

// ROS IDL constants (uint8 OK=0 WARN=1 ERROR=2) — mirrored here because the
// codegen does not preserve them as Rust associated constants.
pub const HEALTH_OK: u32 = 0;
pub const HEALTH_WARN: u32 = 1;
pub const HEALTH_ERROR: u32 = 2;

/// Threshold rule for one component.
#[derive(Debug, Clone)]
pub struct ThresholdRule {
    pub name: String,
    #[allow(dead_code)] // used by collector for source matching
    pub source: String,
    pub warn_above_c: Option<f32>,
    pub error_above_c: Option<f32>,
    pub warn_below_percent: Option<f32>,
    pub error_below_percent: Option<f32>,
    pub warn_below_voltage: Option<f32>,
    pub error_below_voltage: Option<f32>,
}

/// A raw sensor reading from the collector.
#[derive(Debug, Clone)]
pub struct RawReading {
    #[allow(dead_code)] // used for debugging/tracing in Phase 4
    pub name: String,
    pub temp_c: Option<f32>,
    pub voltage: Option<f32>,
    #[allow(dead_code)] // captured for future per-component current monitoring
    pub current_a: Option<f32>,
    pub battery_percent: Option<f32>,
}

/// Evaluate a raw reading against a threshold rule, returning a ComponentHealth.
/// Returns the health regardless of whether an alert is triggered — OK entries
/// with value info are also useful for downstream consumers.
pub fn evaluate(reading: &RawReading, rule: &ThresholdRule) -> ComponentHealth {
    let mut health = ComponentHealth {
        name: rule.name.clone(),
        health: HEALTH_OK,
        detail: String::new(),
        value: -1.0,
        threshold: -1.0,
    };

    // Temperature thresholds (check ERROR first, then WARN).
    if let (Some(temp), Some(error_c)) = (reading.temp_c, rule.error_above_c) {
        health.value = temp;
        if temp >= error_c {
            health.health = HEALTH_ERROR;
            health.threshold = error_c;
            health.detail = format!(
                "{} temp {:.1}°C exceeds ERROR threshold {:.1}°C",
                rule.name, temp, error_c
            );
        }
    }
    if health.health == HEALTH_OK
        && let (Some(temp), Some(warn_c)) = (reading.temp_c, rule.warn_above_c)
    {
        health.value = temp;
        if temp >= warn_c {
            health.health = HEALTH_WARN;
            health.threshold = warn_c;
            health.detail = format!(
                "{} temp {:.1}°C exceeds WARN threshold {:.1}°C",
                rule.name, temp, warn_c
            );
        }
    }

    // Battery voltage thresholds (low = bad).
    if health.health == HEALTH_OK
        && let (Some(voltage), Some(error_v)) = (reading.voltage, rule.error_below_voltage)
    {
        health.value = voltage;
        if voltage <= error_v {
            health.health = HEALTH_ERROR;
            health.threshold = error_v;
            health.detail = format!(
                "{} voltage {:.2}V below ERROR threshold {:.2}V",
                rule.name, voltage, error_v
            );
        }
    }
    if health.health == HEALTH_OK
        && let (Some(voltage), Some(warn_v)) = (reading.voltage, rule.warn_below_voltage)
    {
        health.value = voltage;
        if voltage <= warn_v {
            health.health = HEALTH_WARN;
            health.threshold = warn_v;
            health.detail = format!(
                "{} voltage {:.2}V below WARN threshold {:.2}V",
                rule.name, voltage, warn_v
            );
        }
    }

    // Battery percentage thresholds (low = bad).
    if health.health == HEALTH_OK
        && let (Some(pct), Some(error_pct)) = (reading.battery_percent, rule.error_below_percent)
    {
        health.value = pct;
        if pct <= error_pct {
            health.health = HEALTH_ERROR;
            health.threshold = error_pct;
            health.detail = format!(
                "{} battery {:.0}% below ERROR threshold {:.0}%",
                rule.name, pct, error_pct
            );
        }
    }
    if health.health == HEALTH_OK
        && let (Some(pct), Some(warn_pct)) = (reading.battery_percent, rule.warn_below_percent)
    {
        health.value = pct;
        if pct <= warn_pct {
            health.health = HEALTH_WARN;
            health.threshold = warn_pct;
            health.detail = format!(
                "{} battery {:.0}% below WARN threshold {:.0}%",
                rule.name, pct, warn_pct
            );
        }
    }

    health
}

/// Load threshold rules from a YAML string (contents of jetson_agx_orin.yaml etc.).
pub fn load_thresholds(yaml_str: &str) -> anyhow::Result<Vec<ThresholdRule>> {
    #[derive(serde::Deserialize)]
    struct ComponentYaml {
        name: String,
        #[serde(default)]
        source: String,
        #[serde(default)]
        warn_above_c: Option<f32>,
        #[serde(default)]
        error_above_c: Option<f32>,
        #[serde(default)]
        warn_below_percent: Option<f32>,
        #[serde(default)]
        error_below_percent: Option<f32>,
        #[serde(default)]
        warn_below_voltage: Option<f32>,
        #[serde(default)]
        error_below_voltage: Option<f32>,
    }

    #[derive(serde::Deserialize)]
    struct ThresholdDoc {
        #[serde(default)]
        #[allow(dead_code)] // metadata field for future multi-robot support
        robot_model: String,
        components: Vec<ComponentYaml>,
    }

    let doc: ThresholdDoc = serde_yaml::from_str(yaml_str)?;
    let rules: Vec<ThresholdRule> = doc
        .components
        .into_iter()
        .map(|c| ThresholdRule {
            name: c.name,
            source: c.source,
            warn_above_c: c.warn_above_c,
            error_above_c: c.error_above_c,
            warn_below_percent: c.warn_below_percent,
            error_below_percent: c.error_below_percent,
            warn_below_voltage: c.warn_below_voltage,
            error_below_voltage: c.error_below_voltage,
        })
        .collect();
    Ok(rules)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_temp_ok_and_warn() {
        let rule = ThresholdRule {
            name: "cpu".into(),
            source: "thermal/cpu-thermal".into(),
            warn_above_c: Some(80.0),
            error_above_c: Some(90.0),
            warn_below_percent: None,
            error_below_percent: None,
            warn_below_voltage: None,
            error_below_voltage: None,
        };

        // OK range
        let r = RawReading {
            name: "cpu".into(),
            temp_c: Some(50.0),
            voltage: None,
            current_a: None,
            battery_percent: None,
        };
        let h = evaluate(&r, &rule);
        assert_eq!(h.health, HEALTH_OK);

        // WARN range
        let r = RawReading {
            name: "cpu".into(),
            temp_c: Some(85.0),
            voltage: None,
            current_a: None,
            battery_percent: None,
        };
        let h = evaluate(&r, &rule);
        assert_eq!(h.health, HEALTH_WARN);
        assert_eq!(h.value, 85.0);
        assert_eq!(h.threshold, 80.0);

        // ERROR range
        let r = RawReading {
            name: "cpu".into(),
            temp_c: Some(92.0),
            voltage: None,
            current_a: None,
            battery_percent: None,
        };
        let h = evaluate(&r, &rule);
        assert_eq!(h.health, HEALTH_ERROR);
        assert_eq!(h.value, 92.0);
        assert_eq!(h.threshold, 90.0);
    }

    #[test]
    fn test_battery_voltage() {
        let rule = ThresholdRule {
            name: "battery".into(),
            source: "battery".into(),
            warn_above_c: None,
            error_above_c: None,
            warn_below_percent: None,
            error_below_percent: None,
            warn_below_voltage: Some(11.0),
            error_below_voltage: Some(10.0),
        };

        let r = RawReading {
            name: "battery".into(),
            temp_c: None,
            voltage: Some(10.5),
            current_a: None,
            battery_percent: None,
        };
        let h = evaluate(&r, &rule);
        assert_eq!(h.health, HEALTH_WARN);

        let r = RawReading {
            name: "battery".into(),
            temp_c: None,
            voltage: Some(9.5),
            current_a: None,
            battery_percent: None,
        };
        let h = evaluate(&r, &rule);
        assert_eq!(h.health, HEALTH_ERROR);
    }

    #[test]
    fn test_load_thresholds() {
        let yaml = r#"
robot_model: "jetson_agx_orin"
components:
  - name: "cpu"
    source: "thermal/cpu-thermal"
    warn_above_c: 80.0
    error_above_c: 90.0
  - name: "battery"
    source: "battery"
    warn_below_percent: 20.0
    error_below_percent: 5.0
"#;
        let rules = load_thresholds(yaml).unwrap();
        assert_eq!(rules.len(), 2);
        assert_eq!(rules[0].name, "cpu");
        assert_eq!(rules[0].warn_above_c, Some(80.0));
        assert_eq!(rules[1].name, "battery");
        assert_eq!(rules[1].warn_below_percent, Some(20.0));
    }
}
