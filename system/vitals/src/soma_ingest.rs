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
use std::time::{Duration, Instant};
use tonic::transport::{Channel, Endpoint};

/// Component health is nominal.
pub const HEALTH_OK: u32 = 0;
/// Component health is degraded but functional.
pub const HEALTH_WARN: u32 = 1;
/// Component health requires attention.
pub const HEALTH_ERROR: u32 = 2;
/// Component data is stale (sensor / stream timed out).
pub const HEALTH_STALE: u32 = 3;
/// No health sample has been reported for the component.
pub const HEALTH_UNKNOWN: u32 = 4;

/// Keep the last known component states and expire them when Soma stops
/// refreshing the health stream or omits a previously reported component.
#[derive(Debug, Default)]
pub struct SomaFreshnessTracker {
    ttl: Duration,
    last_signal_at: HashMap<String, Instant>,
    signal_status: HashMap<String, ComponentHealth>,
}

impl SomaFreshnessTracker {
    /// Merge one Soma frame, retaining monitored components that disappear from
    /// a later fallback frame so their last known state can age into STALE.
    pub fn update(
        &mut self,
        soma: &SomaHealthSnapshot,
        mut snapshot: VitalsSnapshot,
        now: Instant,
    ) -> VitalsSnapshot {
        self.ttl = Duration::from_millis(u64::from(soma.ttl_ms.max(1)));
        for status in &snapshot.components {
            if status.health == HEALTH_UNKNOWN {
                self.signal_status
                    .entry(status.name.clone())
                    .or_insert_with(|| status.clone());
                self.last_signal_at
                    .entry(status.name.clone())
                    .or_insert(now);
            } else {
                self.last_signal_at.insert(status.name.clone(), now);
                self.signal_status
                    .insert(status.name.clone(), status.clone());
            }
        }
        let cleared: Vec<_> = self
            .signal_status
            .keys()
            .filter(|name| fault_signal_cleared(soma, name))
            .cloned()
            .collect();
        for name in cleared {
            self.signal_status.remove(&name);
            self.last_signal_at.remove(&name);
        }
        self.apply(&mut snapshot, now);
        snapshot
    }

    /// Age cached hardware states even when Soma has stopped sending frames.
    pub fn expire(&mut self, snapshot: &mut VitalsSnapshot, now: Instant) -> bool {
        self.apply(snapshot, now)
    }

    /// Refresh retained component signals and mark expired observations stale.
    fn apply(&mut self, snapshot: &mut VitalsSnapshot, now: Instant) -> bool {
        let mut changed = false;
        for (signal_name, status) in &mut self.signal_status {
            let signal_expired = self
                .last_signal_at
                .get(signal_name)
                .is_some_and(|last| now.duration_since(*last) >= self.ttl);
            if signal_expired && status.health != HEALTH_STALE {
                status.health = HEALTH_STALE;
                status.detail = format!(
                    "{signal_name} health report timed out after {} ms",
                    self.ttl.as_millis()
                );
                changed = true;
            }
        }
        let ids: std::collections::HashSet<_> = self.signal_status.keys().collect();
        snapshot
            .components
            .retain(|component| !ids.contains(&component.name));
        snapshot
            .components
            .extend(self.signal_status.values().cloned());
        snapshot
            .components
            .sort_by(|left, right| left.name.cmp(&right.name));
        changed
    }
}

/// Retire a fault signal only when its source explicitly confirms recovery.
fn fault_signal_cleared(snapshot: &SomaHealthSnapshot, name: &str) -> bool {
    let Some((component_id, fault_id)) = name.split_once("/fault/") else {
        return false;
    };
    if let Some(fault) = snapshot
        .faults
        .iter()
        .find(|fault| fault.component_id == component_id && fault.fault_id == fault_id)
    {
        return !fault.active;
    }
    false
}
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

    for component in &snapshot.components {
        components.push(ComponentHealth {
            name: component.id.clone(),
            health: match component.health {
                0 => HEALTH_OK,
                1 => HEALTH_WARN,
                2 => HEALTH_ERROR,
                3 => HEALTH_STALE,
                _ => HEALTH_UNKNOWN,
            },
            detail: component.detail.clone(),
            value: if component.present && component.online {
                1.0
            } else {
                0.0
            },
            threshold: 1.0,
        });
    }

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
        if let Some(value) = actuator_control_value(snapshot, actuator, "communication_ok") {
            let communication_ok = value >= 0.5;
            components.push(ComponentHealth {
                name: format!("{}/communication", actuator.component_id),
                health: if communication_ok {
                    HEALTH_OK
                } else {
                    HEALTH_ERROR
                },
                detail: if communication_ok {
                    String::new()
                } else {
                    format!("{} communication is not OK", actuator.component_id)
                },
                value: if communication_ok { 1.0 } else { 0.0 },
                threshold: 1.0,
            });
        }
        if let Some(value) = actuator_control_value(snapshot, actuator, "vendor_error_code") {
            let code = value as u32;
            components.push(ComponentHealth {
                name: format!("{}/vendor_error", actuator.component_id),
                health: if code == 0 { HEALTH_OK } else { HEALTH_ERROR },
                detail: if code == 0 {
                    String::new()
                } else {
                    format!("{} vendor_error_code=0x{:X}", actuator.component_id, code)
                },
                value: code as f32,
                threshold: 0.0,
            });
        }
        if let Some(value) = actuator_control_value(snapshot, actuator, "torque_enabled") {
            let torque_enabled = value >= 0.5;
            components.push(ComponentHealth {
                name: format!("{}/torque_enabled", actuator.component_id),
                health: if torque_enabled {
                    HEALTH_OK
                } else {
                    HEALTH_WARN
                },
                detail: if torque_enabled {
                    String::new()
                } else {
                    format!("{} torque is disabled", actuator.component_id)
                },
                value: if torque_enabled { 1.0 } else { 0.0 },
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

/// Aggregate observed body faults without turning missing actuator data into a fault.
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
        || snapshot.actuators.iter().any(|a| {
            actuator_control_value(snapshot, a, "communication_ok").is_some_and(|value| value < 0.5)
                && component_contains(root, &a.component_id)
        })
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

/// Exclude placeholder actuator flags when component availability is unobserved.
fn actuator_status_observed(snapshot: &SomaHealthSnapshot, actuator: &ActuatorState) -> bool {
    !snapshot.components.iter().any(|component| {
        component.id == actuator.component_id
            && matches!(component.health, HEALTH_UNKNOWN | HEALTH_STALE)
    })
}

/// Prefer reported control values; legacy full-state producers use typed flags.
fn actuator_control_value(
    snapshot: &SomaHealthSnapshot,
    actuator: &ActuatorState,
    name: &str,
) -> Option<f64> {
    if let Some(metric) = snapshot
        .metrics
        .iter()
        .find(|metric| metric.component_id == actuator.component_id && metric.name == name)
    {
        return scalar_value(metric.value.as_ref());
    }
    if name == "vendor_error_code" && actuator.vendor_error_code != 0 {
        return Some(f64::from(actuator.vendor_error_code));
    }
    if !actuator_status_observed(snapshot, actuator) {
        return None;
    }
    match name {
        "communication_ok" => Some(if actuator.communication_ok { 1.0 } else { 0.0 }),
        "torque_enabled" => Some(if actuator.torque_enabled { 1.0 } else { 0.0 }),
        "vendor_error_code" => Some(f64::from(actuator.vendor_error_code)),
        _ => None,
    }
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

/// Summarize explicit faults and observed communication failures within this body.
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
    if snapshot.actuators.iter().any(|a| {
        actuator_control_value(snapshot, a, "communication_ok").is_some_and(|value| value < 0.5)
            && component_contains(root, &a.component_id)
    }) {
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
    use crate::pb::soma::{ComponentStatus, FaultState};

    #[test]
    /// Missing temperature samples expire independently of fresh availability.
    fn missing_temperature_signal_is_retained_then_expires_to_stale() {
        let now = Instant::now();
        let mut source = generate_snapshot(MockScenario::Ramp, 24, None);
        source.ttl_ms = 100;
        let mut tracker = SomaFreshnessTracker::default();
        tracker.update(
            &source,
            snapshot_to_vitals(&source, &default_thresholds(), 1),
            now,
        );
        for actuator in &mut source.actuators {
            actuator.motor_temp = None;
        }
        let mut latest = tracker.update(
            &source,
            snapshot_to_vitals(&source, &default_thresholds(), 2),
            now + Duration::from_millis(50),
        );
        let name = "body/arm/joint_1/motor_temp";
        assert_eq!(
            latest
                .components
                .iter()
                .find(|signal| signal.name == name)
                .unwrap()
                .health,
            HEALTH_ERROR
        );
        assert!(tracker.expire(&mut latest, now + Duration::from_millis(101)));
        assert_eq!(
            latest
                .components
                .iter()
                .find(|signal| signal.name == name)
                .unwrap()
                .health,
            HEALTH_STALE
        );
        assert_ne!(
            latest
                .components
                .iter()
                .find(|signal| signal.name == "body/arm/joint_1")
                .unwrap()
                .health,
            HEALTH_STALE
        );
    }

    #[test]
    /// Mock fault recovery clears retained faults instead of leaving stale alarms.
    fn mock_fault_profile_recovers_and_remains_clear_after_ttl() {
        let now = Instant::now();
        let mut fault = generate_snapshot(MockScenario::Fault, 4, None);
        fault.ttl_ms = 100;
        let mut tracker = SomaFreshnessTracker::default();
        let first = tracker.update(&fault, snapshot_to_vitals(&fault, &[], 1), now);
        let name = "body/arm/joint_3/fault/overcurrent";
        assert!(
            first
                .components
                .iter()
                .any(|signal| signal.name == name && signal.health == HEALTH_ERROR)
        );
        let mut recovery = generate_snapshot(MockScenario::Fault, 8, None);
        recovery.ttl_ms = 100;
        let clear = tracker.update(
            &recovery,
            snapshot_to_vitals(&recovery, &[], 2),
            now + Duration::from_millis(50),
        );
        assert!(clear.components.iter().all(|signal| signal.name != name));
        let mut latest = tracker.update(
            &recovery,
            snapshot_to_vitals(&recovery, &[], 3),
            now + Duration::from_millis(140),
        );
        tracker.expire(&mut latest, now + Duration::from_millis(151));
        assert!(latest.components.iter().all(|signal| signal.name != name));
    }

    #[test]
    /// Explicit normal controls clear retained communication and torque alarms.
    fn observed_actuator_recovery_clears_control_signals() {
        let now = Instant::now();
        let mut source = generate_snapshot(MockScenario::Normal, 1, None);
        let actuator = source
            .actuators
            .iter_mut()
            .find(|actuator| actuator.component_id == "body/arm/joint_1")
            .unwrap();
        actuator.communication_ok = false;
        actuator.torque_enabled = false;
        actuator.vendor_error_code = 9;
        let mut tracker = SomaFreshnessTracker::default();
        tracker.update(&source, snapshot_to_vitals(&source, &[], 1), now);
        let actuator = source
            .actuators
            .iter_mut()
            .find(|actuator| actuator.component_id == "body/arm/joint_1")
            .unwrap();
        actuator.communication_ok = true;
        actuator.torque_enabled = true;
        actuator.vendor_error_code = 0;
        let recovered = tracker.update(
            &source,
            snapshot_to_vitals(&source, &[], 2),
            now + Duration::from_millis(1),
        );
        for name in ["communication", "torque_enabled", "vendor_error"] {
            let key = format!("body/arm/joint_1/{name}");
            assert_eq!(
                recovered
                    .components
                    .iter()
                    .find(|signal| signal.name == key)
                    .unwrap()
                    .health,
                HEALTH_OK
            );
        }
    }

    #[test]
    /// An explicit inactive record retires its retained fault signal.
    fn inactive_fault_record_clears_retained_alarm() {
        let now = Instant::now();
        let mut source = SomaHealthSnapshot {
            ttl_ms: 1000,
            components: vec![ComponentStatus {
                id: "body/joint".into(),
                health: HEALTH_ERROR,
                ..Default::default()
            }],
            faults: vec![FaultState {
                component_id: "body/joint".into(),
                fault_id: "overcurrent".into(),
                severity: FAULT_ERROR,
                active: true,
                ..Default::default()
            }],
            ..Default::default()
        };
        let mut tracker = SomaFreshnessTracker::default();
        tracker.update(&source, snapshot_to_vitals(&source, &[], 1), now);
        source.components[0].health = HEALTH_OK;
        source.faults[0].active = false;
        let recovered = tracker.update(
            &source,
            snapshot_to_vitals(&source, &[], 2),
            now + Duration::from_millis(1),
        );
        assert!(
            recovered
                .components
                .iter()
                .all(|signal| !signal.name.contains("/fault/"))
        );
    }

    #[test]
    /// Availability does not certify controls omitted from a sparse frame.
    fn sparse_control_reports_retain_alarms_until_explicit_recovery() {
        let now = Instant::now();
        let id = "body/arm/joint_1";
        let mut source = generate_snapshot(MockScenario::Normal, 1, None);
        source.ttl_ms = 100;
        source.metrics.extend(
            [
                ("communication_ok", 0.0),
                ("torque_enabled", 0.0),
                ("vendor_error_code", 7.0),
            ]
            .into_iter()
            .map(|(name, value)| crate::pb::soma::Metric {
                component_id: id.into(),
                name: name.into(),
                value: Some(Scalar {
                    value,
                    ..Default::default()
                }),
                ..Default::default()
            }),
        );
        source.faults.push(FaultState {
            component_id: id.into(),
            fault_id: "device_fault".into(),
            severity: FAULT_ERROR,
            active: true,
            ..Default::default()
        });
        let mut tracker = SomaFreshnessTracker::default();
        tracker.update(&source, snapshot_to_vitals(&source, &[], 1), now);
        for metric in &mut source.metrics {
            if metric.component_id == id
                && matches!(
                    metric.name.as_str(),
                    "communication_ok" | "torque_enabled" | "vendor_error_code"
                )
            {
                metric.value = None;
            }
        }
        source.faults.clear();
        let mut partial = tracker.update(
            &source,
            snapshot_to_vitals(&source, &[], 2),
            now + Duration::from_millis(50),
        );
        for suffix in ["communication", "vendor_error", "fault/device_fault"] {
            let name = format!("{id}/{suffix}");
            assert_eq!(
                partial
                    .components
                    .iter()
                    .find(|signal| signal.name == name)
                    .unwrap()
                    .health,
                HEALTH_ERROR
            );
        }
        tracker.expire(&mut partial, now + Duration::from_millis(101));
        assert_eq!(
            partial
                .components
                .iter()
                .find(|signal| signal.name == format!("{id}/vendor_error"))
                .unwrap()
                .health,
            HEALTH_STALE
        );
        for metric in &mut source.metrics {
            if metric.component_id == id
                && matches!(
                    metric.name.as_str(),
                    "communication_ok" | "torque_enabled" | "vendor_error_code"
                )
            {
                metric.value = Some(Scalar {
                    value: if metric.name == "vendor_error_code" {
                        0.0
                    } else {
                        1.0
                    },
                    ..Default::default()
                });
            }
        }
        source.faults.push(FaultState {
            component_id: id.into(),
            fault_id: "device_fault".into(),
            active: false,
            ..Default::default()
        });
        let recovered = tracker.update(
            &source,
            snapshot_to_vitals(&source, &[], 3),
            now + Duration::from_millis(120),
        );
        assert!(
            recovered
                .components
                .iter()
                .all(|signal| !signal.name.ends_with("/fault/device_fault"))
        );
        assert_eq!(
            recovered
                .components
                .iter()
                .find(|signal| signal.name == format!("{id}/vendor_error"))
                .unwrap()
                .health,
            HEALTH_OK
        );
    }

    #[test]
    /// Placeholder actuator flags do not bypass the UNKNOWN grace period.
    fn unknown_actuator_does_not_generate_body_or_control_faults() {
        let mut source = SomaHealthSnapshot {
            components: vec![
                ComponentStatus {
                    id: "body".into(),
                    kind: KIND_BODY,
                    present: true,
                    online: true,
                    ..Default::default()
                },
                ComponentStatus {
                    id: "body/base".into(),
                    parent_id: "body".into(),
                    kind: KIND_BODY,
                    present: true,
                    online: true,
                    ..Default::default()
                },
                ComponentStatus {
                    id: "body/base/left_wheel".into(),
                    parent_id: "body/base".into(),
                    kind: KIND_WHEEL,
                    health: HEALTH_UNKNOWN,
                    present: true,
                    ..Default::default()
                },
            ],
            actuators: vec![ActuatorState {
                component_id: "body/base/left_wheel".into(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let unknown = snapshot_to_vitals(&source, &[], 1);
        assert_eq!(unknown.bodies[0].state, 0);
        assert!(unknown.bodies[0].message.is_empty());
        assert!(
            unknown
                .components
                .iter()
                .all(|signal| !signal.name.ends_with("/communication")
                    && !signal.name.ends_with("/torque_enabled"))
        );
        source.components[2].health = HEALTH_ERROR;
        let offline = snapshot_to_vitals(&source, &[], 2);
        assert_eq!(offline.bodies[0].state, 1);
        assert!(offline.bodies[0].message.contains("communication"));
        assert!(
            offline
                .components
                .iter()
                .any(|signal| signal.name.ends_with("/communication")
                    && signal.health == HEALTH_ERROR)
        );
    }

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
    /// Preserve Soma's component status in the normalized Vitals signal.
    fn component_status_is_exposed_as_vitals_health() {
        let soma = SomaHealthSnapshot {
            body_id: "robot".into(),
            components: vec![ComponentStatus {
                id: "body/head_camera".into(),
                health: HEALTH_ERROR,
                online: false,
                present: true,
                detail: "driver offline".into(),
                ..Default::default()
            }],
            ..Default::default()
        };

        let vitals = snapshot_to_vitals(&soma, &[], 123);
        let camera = vitals
            .components
            .iter()
            .find(|component| component.name == "body/head_camera")
            .expect("camera component health");

        assert_eq!(camera.health, HEALTH_ERROR);
        assert_eq!(camera.detail, "driver offline");
    }

    #[test]
    /// An omitted component becomes stale after its last observation expires.
    fn missing_component_report_ages_to_stale() {
        let now = Instant::now();
        let frame = |health, online, detail: &str| SomaHealthSnapshot {
            body_id: "robot".into(),
            ttl_ms: 100,
            components: vec![ComponentStatus {
                id: "body/head_camera".into(),
                health,
                online,
                present: true,
                detail: detail.into(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let mut tracker = SomaFreshnessTracker::default();
        let healthy = frame(HEALTH_OK, true, "");
        tracker.update(&healthy, snapshot_to_vitals(&healthy, &[], 1), now);

        let no_sample = frame(HEALTH_UNKNOWN, false, "no health reading");
        let mut vitals = tracker.update(
            &no_sample,
            snapshot_to_vitals(&no_sample, &[], 2),
            now + Duration::from_millis(50),
        );
        assert_eq!(
            vitals
                .components
                .iter()
                .find(|component| component.name == "body/head_camera")
                .expect("camera component")
                .health,
            HEALTH_OK
        );

        assert!(tracker.expire(&mut vitals, now + Duration::from_millis(101)));
        let camera = vitals
            .components
            .iter()
            .find(|component| component.name == "body/head_camera")
            .expect("timed-out camera component");
        assert_eq!(camera.health, HEALTH_STALE);
        assert!(camera.detail.contains("timed out"));
    }

    #[test]
    /// A device with no initial report eventually becomes stale, not healthy.
    fn initially_unknown_component_ages_to_stale() {
        let now = Instant::now();
        let soma = SomaHealthSnapshot {
            body_id: "robot".into(),
            ttl_ms: 100,
            components: vec![ComponentStatus {
                id: "body/head_camera".into(),
                health: HEALTH_UNKNOWN,
                online: false,
                present: true,
                detail: "no health reading".into(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let mut tracker = SomaFreshnessTracker::default();
        let mut vitals = tracker.update(&soma, snapshot_to_vitals(&soma, &[], 1), now);
        assert_eq!(vitals.components[0].health, HEALTH_UNKNOWN);

        assert!(tracker.expire(&mut vitals, now + Duration::from_millis(101)));
        assert_eq!(vitals.components[0].health, HEALTH_STALE);
    }

    #[test]
    /// Repeated fallback frames must not keep a prior observation fresh.
    fn previously_reported_component_times_out_when_later_frames_omit_health() {
        let now = Instant::now();
        let make_snapshot = |health, online, detail: &str| SomaHealthSnapshot {
            body_id: "robot".into(),
            ttl_ms: 1_000,
            components: vec![ComponentStatus {
                id: "body/head_camera".into(),
                health,
                online,
                present: true,
                detail: detail.into(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let mut tracker = SomaFreshnessTracker::default();
        let first = make_snapshot(HEALTH_OK, true, "");
        tracker.update(&first, snapshot_to_vitals(&first, &[], 1), now);
        let missing = make_snapshot(HEALTH_UNKNOWN, false, "no reading");
        let mut vitals = tracker.update(
            &missing,
            snapshot_to_vitals(&missing, &[], 2),
            now + Duration::from_millis(500),
        );
        assert_eq!(
            vitals
                .components
                .iter()
                .find(|component| component.name == "body/head_camera")
                .expect("camera component")
                .health,
            HEALTH_OK
        );

        assert!(tracker.expire(&mut vitals, now + Duration::from_millis(1_001),));
        let camera = vitals
            .components
            .iter()
            .find(|component| component.name == "body/head_camera")
            .expect("timed-out camera component");
        assert_eq!(camera.health, HEALTH_STALE);
        assert!(camera.detail.contains("timed out"));
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
