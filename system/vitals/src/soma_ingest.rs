// SPDX-License-Identifier: MulanPSL-2.0
//
// Soma ingestion converts SomaHealthSnapshot facts into the existing Vitals
// output surface. Vitals keeps ownership of threshold judgement here.

use crate::pb::contracts::robonix_system_soma_health_client::RobonixSystemSomaHealthClient;
use crate::pb::soma::{ComponentStatus, Scalar, SomaHealthSnapshot, StreamHealthRequest};
use crate::pb::vitals::{BodyComponent, BodyHealth, ComponentHealth, PowerState, VitalsSnapshot};
use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::collections::HashMap;
use tonic::transport::{Channel, Endpoint};

pub const HEALTH_OK: u32 = 0;
pub const HEALTH_WARN: u32 = 1;
pub const HEALTH_ERROR: u32 = 2;
pub const HEALTH_STALE: u32 = 3;

const QUALITY_VALID: u32 = 0;
const QUALITY_STALE: u32 = 1;
const QUALITY_INVALID: u32 = 3;

const KIND_BODY: u32 = 1;
const KIND_ARM: u32 = 2;
const KIND_LEG: u32 = 3;
const KIND_JOINT: u32 = 4;
const KIND_WHEEL: u32 = 5;
const KIND_GRIPPER: u32 = 6;
const KIND_BATTERY: u32 = 7;
const KIND_COMPUTER: u32 = 8;
const KIND_SENSOR: u32 = 9;
const KIND_CONTROLLER: u32 = 10;
const KIND_END_EFFECTOR: u32 = 11;

const SAFETY_ESTOP: u32 = 4;
const FAULT_WARN: u32 = 1;
const FAULT_ERROR: u32 = 2;
const FAULT_CRITICAL: u32 = 3;

#[derive(Debug, Clone, Default)]
pub struct SomaThresholdRule {
    #[allow(dead_code)] // Rule ids are kept for logging/debug output as the pipeline grows.
    pub id: String,
    pub selector: SomaThresholdSelector,
    pub warn_above: Option<f64>,
    pub error_above: Option<f64>,
    pub warn_below: Option<f64>,
    pub error_below: Option<f64>,
    #[allow(dead_code)] // Units document threshold intent; comparisons use normalized Soma units.
    pub unit: String,
}

#[derive(Debug, Clone, Default)]
pub struct SomaThresholdSelector {
    pub kind: Option<u32>,
    pub component_id: Option<String>,
    pub component_id_glob: Option<String>,
    pub signal: String,
}

#[derive(Debug, Clone, Copy)]
struct Threshold {
    warn_above: Option<f64>,
    error_above: Option<f64>,
    warn_below: Option<f64>,
    error_below: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
struct ThresholdBounds {
    warn_above: Option<f64>,
    error_above: Option<f64>,
    warn_below: Option<f64>,
    error_below: Option<f64>,
}

/// Open the Soma health stream, either from an explicit endpoint or via Atlas.
/// Returns Ok(None) when Atlas discovery finds no provider and no explicit
/// endpoint was supplied, allowing Vitals to fall back to Python collectors.
pub async fn open_soma_stream(
    atlas: &mut AtlasClient,
    consumer_id: &str,
    endpoint: Option<&str>,
) -> Result<Option<tonic::codec::Streaming<SomaHealthSnapshot>>> {
    let channel = if let Some(endpoint) = endpoint {
        ChannelSource::Direct(connect_direct(endpoint).await?)
    } else {
        match atlas_client::connect_to_capability(atlas, consumer_id, "robonix/system/soma/health")
            .await
        {
            Ok((_channel_id, provider_id, channel)) => {
                log::info!("[vitals] connected to Soma provider '{provider_id}' through Atlas");
                ChannelSource::Discovered(channel)
            }
            Err(e) => {
                log::info!("[vitals] Soma health stream not available: {e:#}");
                return Ok(None);
            }
        }
    };

    let mut client = RobonixSystemSomaHealthClient::new(channel.into_channel());
    let stream = client
        .stream_health(StreamHealthRequest {})
        .await
        .context("open Soma StreamHealth")?
        .into_inner();
    Ok(Some(stream))
}

enum ChannelSource {
    Direct(Channel),
    Discovered(Channel),
}

impl ChannelSource {
    fn into_channel(self) -> Channel {
        match self {
            Self::Direct(c) | Self::Discovered(c) => c,
        }
    }
}

/// Load the selector-style Soma threshold YAML. If the file is an older
/// Vitals threshold file or contains no rules, default demo-safe rules are
/// returned so Soma mock scenarios work out of the box.
pub fn load_soma_thresholds(yaml_str: &str) -> Result<Vec<SomaThresholdRule>> {
    #[derive(serde::Deserialize)]
    struct Doc {
        #[serde(default)]
        rules: Vec<RuleYaml>,
    }

    #[derive(serde::Deserialize)]
    struct RuleYaml {
        id: String,
        selector: SelectorYaml,
        #[serde(default)]
        warn_above: Option<f64>,
        #[serde(default)]
        error_above: Option<f64>,
        #[serde(default)]
        warn_below: Option<f64>,
        #[serde(default)]
        error_below: Option<f64>,
        #[serde(default)]
        unit: String,
    }

    #[derive(serde::Deserialize)]
    struct SelectorYaml {
        #[serde(default)]
        kind: Option<String>,
        #[serde(default)]
        component_id: Option<String>,
        #[serde(default)]
        component_id_glob: Option<String>,
        signal: String,
    }

    let doc: Doc = serde_yaml::from_str(yaml_str)?;
    if doc.rules.is_empty() {
        return Ok(default_thresholds());
    }
    let mut out = Vec::with_capacity(doc.rules.len());
    for rule in doc.rules {
        out.push(SomaThresholdRule {
            id: rule.id,
            selector: SomaThresholdSelector {
                kind: rule.selector.kind.as_deref().and_then(kind_from_name),
                component_id: rule.selector.component_id,
                component_id_glob: rule.selector.component_id_glob,
                signal: rule.selector.signal,
            },
            warn_above: rule.warn_above,
            error_above: rule.error_above,
            warn_below: rule.warn_below,
            error_below: rule.error_below,
            unit: rule.unit,
        });
    }
    Ok(out)
}

pub fn default_thresholds() -> Vec<SomaThresholdRule> {
    vec![
        rule_kind(
            "joint_motor_temp",
            KIND_JOINT,
            "motor_temp",
            ThresholdBounds::above(60.0, 75.0),
            "degC",
        ),
        rule_kind(
            "joint_driver_temp",
            KIND_JOINT,
            "driver_temp",
            ThresholdBounds::above(70.0, 85.0),
            "degC",
        ),
        rule_kind(
            "wheel_driver_temp",
            KIND_WHEEL,
            "driver_temp",
            ThresholdBounds::above(75.0, 90.0),
            "degC",
        ),
        rule_kind(
            "sensor_temperature",
            KIND_SENSOR,
            "temperature",
            ThresholdBounds::above(80.0, 90.0),
            "degC",
        ),
        rule_kind(
            "battery_soc",
            KIND_BATTERY,
            "soc_percent",
            ThresholdBounds::below(20.0, 8.0),
            "percent",
        ),
        rule_kind(
            "battery_voltage",
            KIND_BATTERY,
            "voltage",
            ThresholdBounds::below(22.0, 19.0),
            "V",
        ),
    ]
}

/// Convert one Soma snapshot into the current VitalsSnapshot contract. The
/// output keeps Vitals' existing fields stable while deriving health from
/// Soma facts, active faults, and configured thresholds.
pub fn snapshot_to_vitals(
    snapshot: &SomaHealthSnapshot,
    rules: &[SomaThresholdRule],
    ts_ns: i64,
) -> VitalsSnapshot {
    let component_kind: HashMap<&str, u32> = snapshot
        .components
        .iter()
        .map(|c| (c.id.as_str(), c.kind))
        .collect();
    let mut components = Vec::new();

    for actuator in &snapshot.actuators {
        let kind = component_kind
            .get(actuator.component_id.as_str())
            .copied()
            .unwrap_or(KIND_JOINT);
        push_scalar_health(
            &mut components,
            rules,
            &actuator.component_id,
            kind,
            "motor_temp",
            actuator.motor_temp.as_ref(),
        );
        push_scalar_health(
            &mut components,
            rules,
            &actuator.component_id,
            kind,
            "driver_temp",
            actuator.driver_temp.as_ref(),
        );
        if !actuator.communication_ok {
            components.push(ComponentHealth {
                name: format!("{}/communication", actuator.component_id),
                health: HEALTH_ERROR,
                detail: format!("{} communication is not OK", actuator.component_id),
                value: 0.0,
                threshold: 1.0,
            });
        }
        if actuator.vendor_error_code != 0 {
            components.push(ComponentHealth {
                name: format!("{}/vendor_error", actuator.component_id),
                health: HEALTH_ERROR,
                detail: format!(
                    "{} vendor_error_code=0x{:X}",
                    actuator.component_id, actuator.vendor_error_code
                ),
                value: actuator.vendor_error_code as f32,
                threshold: 0.0,
            });
        }
        if !actuator.torque_enabled {
            components.push(ComponentHealth {
                name: format!("{}/torque_enabled", actuator.component_id),
                health: HEALTH_WARN,
                detail: format!("{} torque is disabled", actuator.component_id),
                value: 0.0,
                threshold: 1.0,
            });
        }
    }

    for power in &snapshot.power_sources {
        let kind = component_kind
            .get(power.component_id.as_str())
            .copied()
            .unwrap_or(KIND_BATTERY);
        push_scalar_health(
            &mut components,
            rules,
            &power.component_id,
            kind,
            "soc_percent",
            power.soc_percent.as_ref(),
        );
        push_scalar_health(
            &mut components,
            rules,
            &power.component_id,
            kind,
            "voltage",
            power.voltage.as_ref(),
        );
        push_scalar_health(
            &mut components,
            rules,
            &power.component_id,
            kind,
            "temperature",
            power.temperature.as_ref(),
        );
    }

    for metric in &snapshot.metrics {
        let kind = component_kind
            .get(metric.component_id.as_str())
            .copied()
            .unwrap_or(KIND_SENSOR);
        push_scalar_health(
            &mut components,
            rules,
            &metric.component_id,
            kind,
            &metric.name,
            metric.value.as_ref(),
        );
    }

    for fault in &snapshot.faults {
        if !fault.active {
            continue;
        }
        let health = match fault.severity {
            FAULT_CRITICAL | FAULT_ERROR => HEALTH_ERROR,
            FAULT_WARN => HEALTH_WARN,
            _ => HEALTH_OK,
        };
        components.push(ComponentHealth {
            name: format!("{}/fault/{}", fault.component_id, fault.fault_id),
            health,
            detail: if fault.message.is_empty() {
                format!("{} active fault {}", fault.component_id, fault.fault_id)
            } else {
                fault.message.clone()
            },
            value: fault.vendor_code as f32,
            threshold: 0.0,
        });
    }

    VitalsSnapshot {
        ts_ns,
        power: Some(power_state(snapshot)),
        components,
        bodies: vec![body_health(snapshot)],
    }
}

fn connect_endpoint(raw: &str) -> Result<Endpoint> {
    Endpoint::new(normalize_grpc_endpoint(raw))
        .with_context(|| format!("invalid Soma endpoint '{raw}'"))
}

async fn connect_direct(endpoint: &str) -> Result<Channel> {
    connect_endpoint(endpoint)?
        .connect()
        .await
        .with_context(|| format!("connect to Soma at '{endpoint}'"))
}

fn normalize_grpc_endpoint(raw: &str) -> String {
    let trimmed = raw.trim();
    if trimmed.starts_with("http://") || trimmed.starts_with("https://") {
        trimmed.to_string()
    } else {
        format!("http://{trimmed}")
    }
}

fn rule_kind(
    id: &str,
    kind: u32,
    signal: &str,
    bounds: ThresholdBounds,
    unit: &str,
) -> SomaThresholdRule {
    SomaThresholdRule {
        id: id.to_string(),
        selector: SomaThresholdSelector {
            kind: Some(kind),
            component_id: None,
            component_id_glob: None,
            signal: signal.to_string(),
        },
        warn_above: bounds.warn_above,
        error_above: bounds.error_above,
        warn_below: bounds.warn_below,
        error_below: bounds.error_below,
        unit: unit.to_string(),
    }
}

impl ThresholdBounds {
    fn above(warn: f64, error: f64) -> Self {
        Self {
            warn_above: Some(warn),
            error_above: Some(error),
            warn_below: None,
            error_below: None,
        }
    }

    fn below(warn: f64, error: f64) -> Self {
        Self {
            warn_above: None,
            error_above: None,
            warn_below: Some(warn),
            error_below: Some(error),
        }
    }
}

fn push_scalar_health(
    out: &mut Vec<ComponentHealth>,
    rules: &[SomaThresholdRule],
    component_id: &str,
    kind: u32,
    signal: &str,
    scalar: Option<&Scalar>,
) {
    let Some(scalar) = scalar else {
        return;
    };
    let name = format!("{component_id}/{signal}");
    if scalar.quality == QUALITY_STALE {
        out.push(ComponentHealth {
            name,
            health: HEALTH_STALE,
            detail: format!("{component_id} {signal} is stale"),
            value: scalar.value as f32,
            threshold: -1.0,
        });
        return;
    }
    if scalar.quality == QUALITY_INVALID {
        out.push(ComponentHealth {
            name,
            health: HEALTH_ERROR,
            detail: format!("{component_id} {signal} is invalid"),
            value: scalar.value as f32,
            threshold: -1.0,
        });
        return;
    }
    if scalar.quality != QUALITY_VALID {
        return;
    }

    let Some(threshold) = select_threshold(rules, component_id, kind, signal) else {
        return;
    };
    let (health, threshold_value, detail) =
        evaluate_scalar(component_id, signal, scalar.value, threshold);
    out.push(ComponentHealth {
        name,
        health,
        detail,
        value: scalar.value as f32,
        threshold: threshold_value as f32,
    });
}

fn select_threshold(
    rules: &[SomaThresholdRule],
    component_id: &str,
    kind: u32,
    signal: &str,
) -> Option<Threshold> {
    let effective_rules = if rules.is_empty() {
        default_thresholds()
    } else {
        rules.to_vec()
    };
    let mut selected: Option<(u8, usize, &SomaThresholdRule)> = None;
    for (idx, rule) in effective_rules.iter().enumerate() {
        let Some(priority) = match_rule(rule, component_id, kind, signal) else {
            continue;
        };
        let replace = selected
            .map(|(old_priority, old_idx, _)| {
                priority > old_priority || (priority == old_priority && idx > old_idx)
            })
            .unwrap_or(true);
        if replace {
            selected = Some((priority, idx, rule));
        }
    }
    selected.map(|(_, _, rule)| Threshold {
        warn_above: rule.warn_above,
        error_above: rule.error_above,
        warn_below: rule.warn_below,
        error_below: rule.error_below,
    })
}

fn match_rule(rule: &SomaThresholdRule, component_id: &str, kind: u32, signal: &str) -> Option<u8> {
    if rule.selector.signal != signal {
        return None;
    }
    if let Some(exact) = &rule.selector.component_id {
        return (exact == component_id).then_some(3);
    }
    if let Some(glob) = &rule.selector.component_id_glob {
        return glob_matches(glob, component_id).then_some(2);
    }
    if let Some(rule_kind) = rule.selector.kind {
        return (rule_kind == kind).then_some(1);
    }
    None
}

fn evaluate_scalar(
    component_id: &str,
    signal: &str,
    value: f64,
    threshold: Threshold,
) -> (u32, f64, String) {
    if let Some(error) = threshold.error_above
        && value >= error
    {
        return (
            HEALTH_ERROR,
            error,
            format!("{component_id} {signal} {value:.1} exceeds ERROR threshold {error:.1}"),
        );
    }
    if let Some(warn) = threshold.warn_above
        && value >= warn
    {
        return (
            HEALTH_WARN,
            warn,
            format!("{component_id} {signal} {value:.1} exceeds WARN threshold {warn:.1}"),
        );
    }
    if let Some(error) = threshold.error_below
        && value <= error
    {
        return (
            HEALTH_ERROR,
            error,
            format!("{component_id} {signal} {value:.1} below ERROR threshold {error:.1}"),
        );
    }
    if let Some(warn) = threshold.warn_below
        && value <= warn
    {
        return (
            HEALTH_WARN,
            warn,
            format!("{component_id} {signal} {value:.1} below WARN threshold {warn:.1}"),
        );
    }
    (HEALTH_OK, -1.0, String::new())
}

fn power_state(snapshot: &SomaHealthSnapshot) -> PowerState {
    let Some(power) = snapshot.power_sources.first() else {
        return PowerState {
            battery_percent: -1.0,
            voltage: -1.0,
            charging: false,
            remaining_s: -1,
        };
    };
    PowerState {
        battery_percent: scalar_value(power.soc_percent.as_ref()).unwrap_or(-1.0) as f32,
        voltage: scalar_value(power.voltage.as_ref()).unwrap_or(-1.0) as f32,
        charging: scalar_value(power.current.as_ref()).unwrap_or(0.0) > 0.0,
        remaining_s: scalar_value(power.remaining_s.as_ref()).unwrap_or(-1.0) as i64,
    }
}

fn body_health(snapshot: &SomaHealthSnapshot) -> BodyHealth {
    let (body_type, model) = body_identity(snapshot);
    let mut state = 0;
    if snapshot
        .safety
        .as_ref()
        .map(|s| s.aggregate_state == SAFETY_ESTOP)
        .unwrap_or(false)
    {
        state = 2;
    } else if snapshot
        .faults
        .iter()
        .any(|f| f.active && f.severity >= FAULT_ERROR)
        || snapshot.actuators.iter().any(|a| !a.communication_ok)
    {
        state = 1;
    }

    BodyHealth {
        body_type,
        model,
        state,
        message: body_message(snapshot),
        components: snapshot
            .actuators
            .iter()
            .map(|a| BodyComponent {
                name: if a.joint_name.is_empty() {
                    a.component_id.clone()
                } else {
                    a.joint_name.clone()
                },
                kind: "joint".to_string(),
                temperature: scalar_value(a.motor_temp.as_ref()).unwrap_or(-1.0) as f32,
                error_code: a.vendor_error_code,
                enabled: a.torque_enabled,
            })
            .collect(),
    }
}

fn body_identity(snapshot: &SomaHealthSnapshot) -> (String, String) {
    if let Some(component) = primary_actuator_owner(snapshot) {
        return (
            body_type_for_kind(component.kind).to_string(),
            component_display_model(component, &snapshot.body_id),
        );
    }

    if let Some(component) = snapshot
        .components
        .iter()
        .find(|c| matches!(c.kind, KIND_ARM | KIND_LEG | KIND_BODY))
    {
        return (
            body_type_for_kind(component.kind).to_string(),
            component_display_model(component, &snapshot.body_id),
        );
    }

    ("body".to_string(), first_non_empty(&[&snapshot.body_id]))
}

fn primary_actuator_owner(snapshot: &SomaHealthSnapshot) -> Option<&ComponentStatus> {
    for actuator in &snapshot.actuators {
        let Some(actuator_component) = component_by_id(snapshot, &actuator.component_id) else {
            continue;
        };
        if actuator_component.parent_id.is_empty() {
            continue;
        }
        let Some(parent) = component_by_id(snapshot, &actuator_component.parent_id) else {
            continue;
        };
        if matches!(parent.kind, KIND_ARM | KIND_LEG | KIND_BODY) {
            return Some(parent);
        }
    }
    None
}

fn component_by_id<'a>(
    snapshot: &'a SomaHealthSnapshot,
    component_id: &str,
) -> Option<&'a ComponentStatus> {
    snapshot.components.iter().find(|c| c.id == component_id)
}

fn body_type_for_kind(kind: u32) -> &'static str {
    match kind {
        KIND_ARM => "arm",
        KIND_LEG => "leg",
        KIND_WHEEL => "wheel",
        KIND_GRIPPER => "gripper",
        KIND_BODY => "body",
        _ => "body",
    }
}

fn component_display_model(component: &ComponentStatus, body_id: &str) -> String {
    first_non_empty(&[&component.model, &component.name, &component.id, body_id])
}

fn first_non_empty(values: &[&str]) -> String {
    values
        .iter()
        .copied()
        .find(|value| !value.trim().is_empty())
        .unwrap_or("unknown")
        .to_string()
}

fn body_message(snapshot: &SomaHealthSnapshot) -> String {
    let active_faults: Vec<&str> = snapshot
        .faults
        .iter()
        .filter(|f| f.active)
        .map(|f| f.fault_id.as_str())
        .collect();
    if !active_faults.is_empty() {
        return format!("active faults: {}", active_faults.join(", "));
    }
    if snapshot.actuators.iter().any(|a| !a.communication_ok) {
        return "actuator communication fault".to_string();
    }
    String::new()
}

fn scalar_value(scalar: Option<&Scalar>) -> Option<f64> {
    scalar.and_then(|s| (s.quality == QUALITY_VALID).then_some(s.value))
}

fn kind_from_name(raw: &str) -> Option<u32> {
    match raw.trim().to_ascii_uppercase().as_str() {
        "BODY" => Some(KIND_BODY),
        "ARM" => Some(KIND_ARM),
        "LEG" => Some(KIND_LEG),
        "JOINT" => Some(KIND_JOINT),
        "WHEEL" => Some(KIND_WHEEL),
        "GRIPPER" => Some(KIND_GRIPPER),
        "BATTERY" => Some(KIND_BATTERY),
        "COMPUTER" => Some(KIND_COMPUTER),
        "SENSOR" => Some(KIND_SENSOR),
        "CONTROLLER" => Some(KIND_CONTROLLER),
        "END_EFFECTOR" => Some(KIND_END_EFFECTOR),
        _ => None,
    }
}

fn glob_matches(pattern: &str, value: &str) -> bool {
    let pattern_parts: Vec<&str> = pattern.split('/').collect();
    let value_parts: Vec<&str> = value.split('/').collect();
    if pattern_parts.len() != value_parts.len() {
        return false;
    }
    pattern_parts
        .iter()
        .zip(value_parts.iter())
        .all(|(pattern, value)| segment_matches(pattern, value))
}

fn segment_matches(pattern: &str, value: &str) -> bool {
    if pattern == "*" {
        return true;
    }
    let Some(star_idx) = pattern.find('*') else {
        return pattern == value;
    };
    let (prefix, suffix_with_star) = pattern.split_at(star_idx);
    let suffix = &suffix_with_star[1..];
    value.starts_with(prefix) && value.ends_with(suffix)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock_soma::{MockScenario, generate_snapshot};

    #[test]
    fn ramp_snapshot_crosses_joint_error_threshold() {
        let snapshot = generate_snapshot(MockScenario::Ramp, 24);
        let vitals = snapshot_to_vitals(&snapshot, &default_thresholds(), 123);
        let joint = vitals
            .components
            .iter()
            .find(|c| c.name == "body/arm_right/joint_1/motor_temp")
            .expect("joint_1 motor temp health");
        assert_eq!(joint.health, HEALTH_ERROR);
    }

    #[test]
    fn body_identity_uses_soma_component_model() {
        let snapshot = generate_snapshot(MockScenario::Normal, 1);
        let vitals = snapshot_to_vitals(&snapshot, &default_thresholds(), 123);
        let body = vitals.bodies.first().expect("body health");
        assert_eq!(body.body_type, "arm");
        assert_eq!(body.model, "piper");
    }

    #[test]
    fn selector_yaml_overrides_kind_rule() {
        let rules = load_soma_thresholds(
            r#"
rules:
  - id: loose
    selector:
      kind: "JOINT"
      signal: "motor_temp"
    warn_above: 90.0
    error_above: 95.0
  - id: exact
    selector:
      component_id: "body/arm_right/joint_1"
      signal: "motor_temp"
    warn_above: 36.0
    error_above: 50.0
"#,
        )
        .unwrap();
        let snapshot = generate_snapshot(MockScenario::Normal, 1);
        let vitals = snapshot_to_vitals(&snapshot, &rules, 123);
        let joint = vitals
            .components
            .iter()
            .find(|c| c.name == "body/arm_right/joint_1/motor_temp")
            .expect("joint_1 motor temp health");
        assert_eq!(joint.health, HEALTH_WARN);
    }
}
