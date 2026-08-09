// SPDX-License-Identifier: MulanPSL-2.0
//
// Soma ingestion converts SomaHealthSnapshot facts into the existing Vitals
// output surface. Vitals keeps ownership of threshold judgement here.

use crate::pb::contracts::robonix_system_soma_health_client::RobonixSystemSomaHealthClient;
use crate::pb::soma::{
    ActuatorState, ComponentStatus, PowerSourceState, Scalar, SomaHealthSnapshot,
    StreamHealthRequest,
};
use crate::pb::vitals::{BodyComponent, BodyHealth, ComponentHealth, PowerState, VitalsSnapshot};
use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::collections::HashMap;
use tonic::transport::{Channel, Endpoint};

/// Component health is nominal.
pub const HEALTH_OK: u32 = 0;
/// Component health is degraded but functional.
pub const HEALTH_WARN: u32 = 1;
/// Component health requires attention.
pub const HEALTH_ERROR: u32 = 2;
/// Component data is stale (sensor / stream timed out).
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
const SAFETY_FAULT: u32 = 5;
const FAULT_WARN: u32 = 1;
const FAULT_ERROR: u32 = 2;
const FAULT_CRITICAL: u32 = 3;

/// One threshold evaluation rule keyed by a component selector + signal name.
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

/// Identifies which components a threshold rule applies to. Priority:
/// exact component_id (3) > component_id_glob (2) > kind (1).
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

/// Open the Soma health stream, either from an explicit endpoint or via Atlas
/// discovery.  Returns `Ok(None)` when Atlas discovery finds no provider and
/// no explicit endpoint was supplied, signalling that no Soma is available.
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
        let kind = match rule.selector.kind.as_deref() {
            Some(raw) => {
                let k = kind_from_name(raw);
                if k.is_none() {
                    log::error!(
                        "[vitals] threshold rule '{}': unrecognized kind '{}' — rule will not fire via kind selector",
                        rule.id,
                        raw
                    );
                }
                k
            }
            None => None,
        };
        out.push(SomaThresholdRule {
            id: rule.id,
            selector: SomaThresholdSelector {
                kind,
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

/// Return the built-in Soma threshold rules used when no YAML file is found.
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
///
/// TODO(gap): TTL-based STALE detection is not yet implemented.
///   Design doc §4.1 requires: when `soma_ts_ns + ttl_ms < now`, affected
///   components should be marked HEALTH_STALE.  The snapshot carries `ttl_ms`
///   but `snapshot_to_vitals` does not check it — a stale stream (e.g. Soma
///   process hung) will keep reporting the last-known values indefinitely.
pub fn snapshot_to_vitals(
    snapshot: &SomaHealthSnapshot,
    rules: &[SomaThresholdRule],
    ts_ns: u64,
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

    // NOTE(gap): Metrics are routed through the same threshold-evaluation
    //   pipeline as typed actuator/power signals.  The design doc positions
    //   Metric as extension data that "does not participate in core threshold
    //   judgment," but in practice a Metric will be evaluated if a threshold
    //   rule matches its (component_id/kind, name).  This is harmless when no
    //   rule matches (the metric is silently skipped), but it means adding a
    //   broad kind-based rule (e.g. SENSOR + "temperature") will also pull in
    //   Metrics like fan_rpm or packet_loss if they happen to share the signal
    //   name.  If this becomes noisy, consider a separate MetricThresholdRule
    //   table or an explicit `evaluate_metrics: bool` flag per rule.
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
            other => {
                log::warn!(
                    "[vitals] unknown fault severity {} for fault '{}' on {} — treating as ERROR",
                    other,
                    fault.fault_id,
                    fault.component_id
                );
                HEALTH_ERROR
            }
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
        bodies: body_healths(snapshot),
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
    if value.is_nan() {
        return (
            HEALTH_ERROR,
            -1.0,
            format!("{component_id} {signal} value is NaN"),
        );
    }
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

fn body_healths(snapshot: &SomaHealthSnapshot) -> Vec<BodyHealth> {
    let Some(root) = root_component(snapshot) else {
        return Vec::new();
    };
    let mut bodies: Vec<BodyHealth> = snapshot
        .components
        .iter()
        .filter(|component| component.parent_id == root.id)
        .map(|component| body_health_for_component(snapshot, component))
        .collect();
    if bodies.is_empty() {
        bodies.push(body_health_for_component(snapshot, root));
    }
    bodies
}

fn body_health_for_component(snapshot: &SomaHealthSnapshot, root: &ComponentStatus) -> BodyHealth {
    // NOTE(gap): Only SafetyState.aggregate_state is checked here.
    //   SafetyEndpointState[] (individual hardware/software/remote e-stops)
    //   is present in the snapshot but not consumed per-endpoint.  The design
    //   doc does not mandate per-endpoint health decisions, but an operator
    //   debugging an e-stop trigger currently has to inspect raw snapshot data
    //   rather than seeing which endpoint fired in the Vitals output.
    let mut state = 0;
    if snapshot
        .safety
        .as_ref()
        .map(|s| s.aggregate_state == SAFETY_ESTOP)
        .unwrap_or(false)
    {
        state = 2;
    } else if snapshot
        .safety
        .as_ref()
        .map(|s| s.aggregate_state == SAFETY_FAULT)
        .unwrap_or(false)
        || snapshot.faults.iter().any(|f| {
            f.active && f.severity >= FAULT_ERROR && component_contains(root, &f.component_id)
        })
        || snapshot
            .actuators
            .iter()
            .any(|a| !a.communication_ok && component_contains(root, &a.component_id))
    {
        state = 1;
    }

    BodyHealth {
        body_type: component_type_name(root),
        model: component_display_model(root, &snapshot.body_id),
        state,
        message: body_message(snapshot, root),
        components: body_components(snapshot, root),
    }
}

fn root_component(snapshot: &SomaHealthSnapshot) -> Option<&ComponentStatus> {
    snapshot
        .components
        .iter()
        .find(|c| c.parent_id.is_empty())
        .or_else(|| snapshot.components.iter().find(|c| c.kind == KIND_BODY))
}

fn actuator_by_component_id<'a>(
    snapshot: &'a SomaHealthSnapshot,
    component_id: &str,
) -> Option<&'a ActuatorState> {
    snapshot
        .actuators
        .iter()
        .find(|a| a.component_id == component_id)
}

fn power_by_component_id<'a>(
    snapshot: &'a SomaHealthSnapshot,
    component_id: &str,
) -> Option<&'a PowerSourceState> {
    snapshot
        .power_sources
        .iter()
        .find(|p| p.component_id == component_id)
}

fn body_components(snapshot: &SomaHealthSnapshot, root: &ComponentStatus) -> Vec<BodyComponent> {
    snapshot
        .components
        .iter()
        .filter(|component| component.id != root.id && component_contains(root, &component.id))
        .map(|component| {
            let actuator = actuator_by_component_id(snapshot, &component.id);
            let power = power_by_component_id(snapshot, &component.id);
            BodyComponent {
                name: first_non_empty(&[&component.name, &component.id]),
                kind: kind_label(component.kind).to_string(),
                temperature: component_temperature(actuator, power),
                error_code: component_error_code(snapshot, component, actuator, power),
                enabled: component_enabled(component, actuator),
                id: component.id.clone(),
                parent_id: component.parent_id.clone(),
                model: component.model.clone(),
            }
        })
        .collect()
}

/// Prefer typed device status and fall back to an exact active component fault.
fn component_error_code(
    snapshot: &SomaHealthSnapshot,
    component: &ComponentStatus,
    actuator: Option<&ActuatorState>,
    power: Option<&PowerSourceState>,
) -> u32 {
    actuator
        .map(|state| state.vendor_error_code)
        .filter(|code| *code != 0)
        .or_else(|| power.map(|state| state.vendor_status_code))
        .filter(|code| *code != 0)
        .or_else(|| {
            snapshot
                .faults
                .iter()
                .find(|fault| fault.active && fault.component_id == component.id)
                .map(|fault| fault.vendor_code)
        })
        .unwrap_or(0)
}

fn component_contains(root: &ComponentStatus, component_id: &str) -> bool {
    component_id == root.id || component_id.starts_with(&format!("{}/", root.id))
}

fn component_temperature(
    actuator: Option<&ActuatorState>,
    power: Option<&PowerSourceState>,
) -> f32 {
    if let Some(actuator) = actuator
        && let Some(temp) = scalar_value(actuator.motor_temp.as_ref())
    {
        return temp as f32;
    }
    if let Some(power) = power
        && let Some(temp) = scalar_value(power.temperature.as_ref())
    {
        return temp as f32;
    }
    -1.0
}

fn component_enabled(component: &ComponentStatus, actuator: Option<&ActuatorState>) -> bool {
    actuator
        .map(|a| a.torque_enabled)
        .unwrap_or(component.present && component.online)
}

fn component_type_name(component: &ComponentStatus) -> String {
    let path_name = component.id.rsplit('/').next().unwrap_or("");
    first_non_empty(&[path_name, &component.name, &component.model])
}

fn kind_label(kind: u32) -> &'static str {
    match kind {
        KIND_BODY => "body",
        KIND_ARM => "arm",
        KIND_LEG => "leg",
        KIND_JOINT => "joint",
        KIND_WHEEL => "wheel",
        KIND_GRIPPER => "gripper",
        KIND_BATTERY => "battery",
        KIND_COMPUTER => "computer",
        KIND_SENSOR => "sensor",
        KIND_CONTROLLER => "controller",
        KIND_END_EFFECTOR => "end_effector",
        _ => "unknown",
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

fn body_message(snapshot: &SomaHealthSnapshot, root: &ComponentStatus) -> String {
    let active_faults: Vec<&str> = snapshot
        .faults
        .iter()
        .filter(|f| f.active && component_contains(root, &f.component_id))
        .map(|f| f.fault_id.as_str())
        .collect();
    if !active_faults.is_empty() {
        return format!("active faults: {}", active_faults.join(", "));
    }
    if snapshot
        .actuators
        .iter()
        .any(|a| !a.communication_ok && component_contains(root, &a.component_id))
    {
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
    use crate::pb::soma::FaultState;

    #[test]
    fn ramp_snapshot_crosses_joint_error_threshold() {
        let snapshot = generate_snapshot(MockScenario::Ramp, 24, None);
        let vitals = snapshot_to_vitals(&snapshot, &default_thresholds(), 123);
        let joint = vitals
            .components
            .iter()
            .find(|c| c.name == "body/arm/joint_1/motor_temp")
            .expect("joint_1 motor temp health");
        assert_eq!(joint.health, HEALTH_ERROR);
    }

    #[test]
    fn body_health_groups_root_children() {
        let snapshot = generate_snapshot(MockScenario::Normal, 1, None);
        let vitals = snapshot_to_vitals(&snapshot, &default_thresholds(), 123);
        assert_eq!(vitals.bodies.len(), 3);

        let computer = vitals
            .bodies
            .iter()
            .find(|body| body.body_type == "computer_jetson")
            .expect("computer_jetson health");
        assert_eq!(computer.model, "jetson_agx_orin");
        assert!(
            computer
                .components
                .iter()
                .any(|c| c.id == "body/computer_jetson/cpu" && c.kind == "sensor")
        );

        let arm = vitals
            .bodies
            .iter()
            .find(|body| body.body_type == "arm")
            .expect("arm health");
        assert_eq!(arm.model, "mock_arm");
        let joint = arm
            .components
            .iter()
            .find(|c| c.id == "body/arm/joint_1")
            .expect("joint_1 component");
        assert_eq!(joint.parent_id, "body/arm");
        assert_eq!(joint.model, "mock_motor");

        let battery = vitals
            .bodies
            .iter()
            .find(|body| body.body_type == "battery_main")
            .expect("battery_main health");
        assert_eq!(battery.model, "mock_bms");
        assert!(battery.components.is_empty());
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
      component_id: "body/arm/joint_1"
      signal: "motor_temp"
    warn_above: 36.0
    error_above: 50.0
"#,
        )
        .unwrap();
        let snapshot = generate_snapshot(MockScenario::Normal, 1, None);
        let vitals = snapshot_to_vitals(&snapshot, &rules, 123);
        let joint = vitals
            .components
            .iter()
            .find(|c| c.name == "body/arm/joint_1/motor_temp")
            .expect("joint_1 motor temp health");
        assert_eq!(joint.health, HEALTH_WARN);
    }

    #[test]
    fn unknown_fault_severity_maps_to_error() {
        let snapshot = SomaHealthSnapshot {
            faults: vec![FaultState {
                component_id: "body/arm/joint_1".to_string(),
                fault_id: "future_critical".to_string(),
                severity: 99, // unknown severity from a newer Soma version
                active: true,
                clearable: false,
                onset_ts_ns: 0,
                vendor_code: 0,
                vendor_code_text: String::new(),
                message: String::new(),
                attributes: vec![],
                vendor_raw_json: String::new(),
            }],
            ..generate_snapshot(MockScenario::Normal, 1, None)
        };
        let vitals = snapshot_to_vitals(&snapshot, &default_thresholds(), 123);
        let fault = vitals
            .components
            .iter()
            .find(|c| c.name == "body/arm/joint_1/fault/future_critical")
            .expect("fault component");
        assert_eq!(
            fault.health, HEALTH_ERROR,
            "unknown fault severity must be treated as ERROR, not OK"
        );
    }

    /// A non-actuator component retains its vendor code in the body projection.
    #[test]
    fn component_fault_supplies_body_error_code() {
        let mut snapshot = generate_snapshot(MockScenario::Normal, 1, None);
        snapshot.components.push(ComponentStatus {
            id: "body/arm/gripper".to_string(),
            parent_id: "body/arm".to_string(),
            kind: KIND_GRIPPER,
            name: "gripper".to_string(),
            frame_id: "gripper_link".to_string(),
            model: "parallel_gripper".to_string(),
            serial: String::new(),
            health: HEALTH_ERROR,
            operational_state: 8,
            present: true,
            online: false,
            detail: "error_code=0x17".to_string(),
        });
        snapshot.faults.push(FaultState {
            component_id: "body/arm/gripper".to_string(),
            fault_id: "device_fault".to_string(),
            severity: FAULT_ERROR,
            active: true,
            clearable: true,
            onset_ts_ns: 0,
            vendor_code: 23,
            vendor_code_text: "0x17".to_string(),
            message: "gripper error_code=0x17".to_string(),
            attributes: vec![],
            vendor_raw_json: String::new(),
        });

        let vitals = snapshot_to_vitals(&snapshot, &default_thresholds(), 123);
        let gripper = vitals
            .bodies
            .iter()
            .find(|body| body.body_type == "arm")
            .and_then(|body| {
                body.components
                    .iter()
                    .find(|component| component.id == "body/arm/gripper")
            })
            .expect("gripper body component");
        assert_eq!(gripper.error_code, 23);
        assert!(!gripper.enabled);
    }

    #[test]
    fn kind_from_name_unknown_returns_none() {
        assert_eq!(kind_from_name("UNICORN"), None);
        assert_eq!(kind_from_name(""), None);
    }

    #[test]
    fn kind_from_name_known_returns_value() {
        assert_eq!(kind_from_name("JOINT"), Some(KIND_JOINT));
        assert_eq!(kind_from_name("  joint  "), Some(KIND_JOINT));
        assert_eq!(kind_from_name("BATTERY"), Some(KIND_BATTERY));
    }

    #[test]
    fn glob_matches_exact_and_wildcard() {
        assert!(glob_matches("body/arm/*", "body/arm/joint_1"));
        assert!(!glob_matches("body/leg/*", "body/arm/joint_1"));
        assert!(glob_matches("body/*/joint_1", "body/arm/joint_1"));
        assert!(!glob_matches("body/*/joint_1", "body/arm/joint_2"));
    }
}
