// SPDX-License-Identifier: MulanPSL-2.0
//
// Soma health data collector — discovers health primitives via Atlas,
// consumes their gRPC HealthState streams, aggregates data into
// SomaHealthSnapshot, and broadcasts to subscribers (Vitals).

use crate::pb::soma::{
    ActuatorState, ComponentStatus, FaultState, Metric, PowerSourceState, SafetyEndpointState,
    SafetyState, Scalar, SomaHealthSnapshot,
};
use crate::service::SomaService;
use crate::store::{SomaBody, SomaComponent};
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_scribe::info;
use std::collections::HashMap;
use std::sync::Arc;
use tokio_stream::StreamExt;
use tonic::transport::Channel;

const SCHEMA_VERSION: u32 = 1;
const QUALITY_VALID: u32 = 0;
const HEALTH_OK: u32 = 0;
const HEALTH_ERROR: u32 = 2;
const HEALTH_UNKNOWN: u32 = 4;
const KIND_UNKNOWN: u32 = 0;
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
const OP_UNKNOWN: u32 = 0;
const OP_ACTIVE: u32 = 4;
const OP_FAULT: u32 = 8;
const SAFETY_NORMAL: u32 = 1;
const SAFETY_FAULT: u32 = 5;
const SAFETY_ESTOP: u32 = 4;
const ESTOP_TYPE_HARDWARE: u32 = 1;
const FAULT_ERROR: u32 = 2;

/// Start health data collection: discover primitives via Atlas,
/// consume their gRPC HealthState streams, aggregate into
/// SomaHealthSnapshot, and publish them through Soma's snapshot service.
pub async fn start_health_collector(
    mut atlas: AtlasClient,
    body: Arc<SomaBody>,
    service: Arc<SomaService>,
) -> Result<()> {
    use robonix_atlas::pb as atlas_pb;

    info!("[soma-health] discovering health primitives...");

    // Discover providers implementing robonix/primitive/health/stream.
    let capabilities = atlas
        .flatten_capabilities(
            "robonix/primitive/health/stream",
            "",
            atlas_pb::Transport::Grpc,
        )
        .await
        .context("discover health primitives")?;

    if capabilities.is_empty() {
        info!("[soma-health] no health primitives found; using runtime-state fallback");
        return Ok(());
    }

    // Deduplicate by provider_id.
    let mut seen = std::collections::HashSet::new();
    let providers: Vec<_> = capabilities
        .into_iter()
        .filter(|c| seen.insert(c.provider_id.clone()))
        .collect();

    info!(
        "[soma-health] found {} health primitive(s)",
        providers.len()
    );

    for cap in providers {
        let provider_id = cap.provider_id.clone();
        let body = Arc::clone(&body);
        let service = Arc::clone(&service);
        let mut atlas = atlas.clone();

        tokio::spawn(async move {
            if let Err(e) = consume_primitive_stream(&mut atlas, &provider_id, body, service).await
            {
                robonix_scribe::warn!(
                    "[soma-health] primitive '{}' stream ended: {e:#}",
                    provider_id
                );
            }
        });
    }

    Ok(())
}

/// Connect to one health primitive's StreamHealthState gRPC and forward data.
async fn consume_primitive_stream(
    atlas: &mut AtlasClient,
    provider_id: &str,
    body: Arc<SomaBody>,
    service: Arc<SomaService>,
) -> Result<()> {
    use crate::pb::contracts::robonix_primitive_health_stream_client::RobonixPrimitiveHealthStreamClient;
    use crate::pb::health::StreamHealthStateRequest;
    use robonix_atlas::pb as atlas_pb;

    // Get the endpoint from Atlas.
    let (channel_id, endpoint_str, _params) = atlas
        .connect_capability(
            "soma",
            provider_id,
            "robonix/primitive/health/stream",
            atlas_pb::Transport::Grpc,
        )
        .await
        .with_context(|| format!("connect to health primitive '{provider_id}'"))?;

    let normalized = if endpoint_str.starts_with("http") {
        endpoint_str.clone()
    } else {
        format!("http://{endpoint_str}")
    };

    info!(
        "[soma-health] connected to health primitive '{}' at {}",
        provider_id, normalized
    );

    let result = async {
        let channel = Channel::from_shared(normalized)
            .context("invalid endpoint")?
            .connect()
            .await
            .context("dial health primitive")?;
        let mut client = RobonixPrimitiveHealthStreamClient::new(channel);
        let mut stream = client
            .stream_health_state(StreamHealthStateRequest {})
            .await
            .context("open StreamHealthState")?
            .into_inner();

        while let Some(result) = stream.next().await {
            match result {
                Ok(health_state) => {
                    let snapshot = health_state_to_snapshot(&health_state, &body, 0);
                    service.publish_primitive_snapshot(snapshot).await;
                }
                Err(error) => {
                    robonix_scribe::warn!(
                        "[soma-health] stream error from '{}': {error:#}",
                        provider_id
                    );
                    break;
                }
            }
        }
        Ok(())
    }
    .await;

    let _ = atlas.disconnect_capability(&channel_id).await;
    result
}

/// Project generic HealthState readings onto components declared in Soma YAML.
fn health_state_to_snapshot(
    state: &crate::pb::health::HealthState,
    body: &SomaBody,
    seq: u64,
) -> SomaHealthSnapshot {
    let now_ns = chrono_now_ns();
    let readings: HashMap<&str, &crate::pb::health::SensorReading> = state
        .readings
        .iter()
        .map(|reading| (reading.name.as_str(), reading))
        .collect();
    let safety_state = aggregate_safety_state(&readings);

    let mut components = Vec::with_capacity(body.components.len() + 1);
    components.push(observed_component(
        component(
            "body",
            "",
            KIND_BODY,
            &body.display_name,
            &body.root_link,
            &body.model_name,
        ),
        &readings,
    ));
    components.extend(body.components.iter().map(|body_component| {
        observed_component(
            component(
                &body_component.id,
                &body_component.parent_id,
                component_kind(&body_component.component_type),
                body_component
                    .id
                    .rsplit('/')
                    .next()
                    .unwrap_or(&body_component.id),
                &body_component.frame_id,
                &body_component.component_type,
            ),
            &readings,
        )
    }));

    let actuators: Vec<ActuatorState> = body
        .components
        .iter()
        .filter(|component| {
            matches!(
                component_kind(&component.component_type),
                KIND_JOINT | KIND_WHEEL
            )
        })
        .map(|component| described_actuator_state(component, &readings))
        .collect();
    let power_sources: Vec<PowerSourceState> = body
        .components
        .iter()
        .filter(|component| component_kind(&component.component_type) == KIND_BATTERY)
        .map(|component| described_power_source(component, state, &readings))
        .collect();
    let metrics = described_metrics(body, &readings);
    let faults = described_faults(body, &readings, now_ns);

    if (safety_state != SAFETY_NORMAL || !faults.is_empty())
        && let Some(root) = components.first_mut()
    {
        root.health = HEALTH_ERROR;
        root.operational_state = OP_FAULT;
        root.detail = if safety_state == SAFETY_ESTOP {
            "emergency stop active".to_string()
        } else {
            "health fault active".to_string()
        };
    }

    SomaHealthSnapshot {
        schema_version: SCHEMA_VERSION,
        body_id: body.robot_id.clone(),
        seq,
        source_ts_ns: now_ns,
        soma_ts_ns: now_ns,
        ttl_ms: 1500,
        components,
        actuators,
        power_sources,
        safety: Some(SafetyState {
            motion_allowed: safety_state == SAFETY_NORMAL,
            motor_power_allowed: safety_state == SAFETY_NORMAL,
            aggregate_state: safety_state,
            detail: String::new(),
        }),
        safety_endpoints: vec![SafetyEndpointState {
            name: "hardware_estop".to_string(),
            r#type: ESTOP_TYPE_HARDWARE,
            state: if safety_state == SAFETY_ESTOP { 1 } else { 0 },
            detail: String::new(),
        }],
        faults,
        metrics,
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────

/// Apply availability and fault controls from one HealthState frame.
fn observed_component(
    mut status: ComponentStatus,
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
) -> ComponentStatus {
    let observed = component_observed(readings, &status.id);
    let online_control = control_bool(readings, &status.id, "online")
        .or_else(|| control_bool(readings, &status.id, "communication_ok"));
    let online = online_control.unwrap_or(observed);
    let present = control_bool(readings, &status.id, "present").unwrap_or(true);
    let error_code = control_code(readings, &status.id, "error");
    status.present = present;
    status.online = online;
    status.health = if error_code != 0 || !present || !online {
        HEALTH_ERROR
    } else if observed || online_control.is_some() {
        HEALTH_OK
    } else {
        HEALTH_UNKNOWN
    };
    status.operational_state = match status.health {
        HEALTH_OK => OP_ACTIVE,
        HEALTH_ERROR => OP_FAULT,
        _ => OP_UNKNOWN,
    };
    if error_code != 0 {
        status.detail = format!("error_code=0x{error_code:X}");
    } else if !present || !online {
        status.detail = format!("present={present}; online={online}");
    }
    status
}

/// Convert a Soma component type label into the stable wire enum value.
fn component_kind(component_type: &str) -> u32 {
    match component_type.trim().to_ascii_lowercase().as_str() {
        "body" | "mobile_base" => KIND_BODY,
        "arm" => KIND_ARM,
        "leg" => KIND_LEG,
        "joint" => KIND_JOINT,
        "wheel" => KIND_WHEEL,
        "gripper" => KIND_GRIPPER,
        "battery" => KIND_BATTERY,
        "computer" => KIND_COMPUTER,
        "controller" => KIND_CONTROLLER,
        "end_effector" => KIND_END_EFFECTOR,
        "sensor" | "camera" | "rgb_camera" | "depth_camera" | "rgbd_camera" | "lidar"
        | "lidar_2d" | "lidar_3d" | "imu" | "audio_io" => KIND_SENSOR,
        _ => KIND_UNKNOWN,
    }
}

/// Build actuator telemetry for one described joint or wheel.
fn described_actuator_state(
    component: &SomaComponent,
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
) -> ActuatorState {
    let reading = readings.get(component.id.as_str()).copied();
    let motor_temp = reading.or_else(|| child_reading(readings, &component.id, "motor_temp"));
    let driver_temp = child_reading(readings, &component.id, "driver_temp");
    let error_code = control_code(readings, &component.id, "error");
    ActuatorState {
        component_id: component.id.clone(),
        joint_name: component
            .id
            .rsplit('/')
            .next()
            .unwrap_or(&component.id)
            .to_string(),
        position: None,
        velocity: None,
        effort: None,
        current: reading.and_then(|value| observed_scalar(value.current_a, "A")),
        voltage: reading.and_then(|value| observed_scalar(value.voltage, "V")),
        motor_temp: motor_temp.and_then(|value| observed_scalar(value.temp_c, "degC")),
        driver_temp: driver_temp.and_then(|value| observed_scalar(value.temp_c, "degC")),
        torque_enabled: control_bool(readings, &component.id, "enabled").unwrap_or(true),
        brake_engaged: control_bool(readings, &component.id, "brake_engaged").unwrap_or(false),
        communication_ok: control_bool(readings, &component.id, "communication_ok")
            .unwrap_or(motor_temp.is_some()),
        vendor_mode: 0,
        vendor_error_code: error_code,
        status_flags: error_code,
    }
}

/// Build one battery state, preferring component readings over frame totals.
fn described_power_source(
    component: &SomaComponent,
    state: &crate::pb::health::HealthState,
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
) -> PowerSourceState {
    let reading = readings.get(component.id.as_str()).copied();
    let voltage = reading
        .and_then(|value| observed_scalar(value.voltage, "V"))
        .or_else(|| observed_scalar(state.voltage, "V"));
    let current = reading
        .and_then(|value| observed_scalar(value.current_a, "A"))
        .or_else(|| Some(scalar(if state.charging { 1.0 } else { 0.0 }, "A")));
    PowerSourceState {
        component_id: component.id.clone(),
        soc_percent: reading.and_then(|value| observed_scalar(value.battery_percent, "percent")),
        soh_percent: None,
        voltage,
        current,
        temperature: reading.and_then(|value| observed_scalar(value.temp_c, "degC")),
        remaining_s: (state.remaining_s >= 0).then(|| scalar(state.remaining_s as f64, "s")),
        cycle_count: 0,
        cell_voltages: vec![],
        vendor_status_code: control_code(readings, &component.id, "error"),
    }
}

/// Convert non-actuator component readings into thresholdable Soma metrics.
fn described_metrics(
    body: &SomaBody,
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
) -> Vec<Metric> {
    let mut metrics = Vec::new();
    for component in &body.components {
        if matches!(
            component_kind(&component.component_type),
            KIND_JOINT | KIND_WHEEL | KIND_BATTERY
        ) {
            continue;
        }
        let Some(reading) = readings.get(component.id.as_str()).copied() else {
            continue;
        };
        push_metric(
            &mut metrics,
            &component.id,
            "temperature",
            observed_scalar(reading.temp_c, "degC"),
        );
        push_metric(
            &mut metrics,
            &component.id,
            "voltage",
            observed_scalar(reading.voltage, "V"),
        );
        push_metric(
            &mut metrics,
            &component.id,
            "current",
            observed_scalar(reading.current_a, "A"),
        );
    }
    metrics
}

/// Append an available metric while omitting unavailable scalar fields.
fn push_metric(metrics: &mut Vec<Metric>, component_id: &str, name: &str, value: Option<Scalar>) {
    if let Some(value) = value {
        metrics.push(Metric {
            component_id: component_id.to_string(),
            name: name.to_string(),
            value: Some(value),
            source_key: component_id.to_string(),
        });
    }
}

/// Convert non-zero component error controls into active fault records.
fn described_faults(
    body: &SomaBody,
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
    now_ns: i64,
) -> Vec<FaultState> {
    body.components
        .iter()
        .filter_map(|component| {
            let error_code = control_code(readings, &component.id, "error");
            (error_code != 0).then(|| FaultState {
                component_id: component.id.clone(),
                fault_id: "device_fault".to_string(),
                severity: FAULT_ERROR,
                active: true,
                clearable: true,
                onset_ts_ns: now_ns,
                vendor_code: error_code,
                vendor_code_text: format!("0x{error_code:X}"),
                message: format!("{} error_code=0x{error_code:X}", component.id),
                attributes: vec![],
                vendor_raw_json: String::new(),
            })
        })
        .collect()
}

/// Read the whole-body safety code from the root component.
fn aggregate_safety_state(readings: &HashMap<&str, &crate::pb::health::SensorReading>) -> u32 {
    let raw = readings
        .get("body/state")
        .map(|reading| reading.current_a as u32)
        .unwrap_or(0);
    match raw {
        0 => SAFETY_NORMAL,
        2 => SAFETY_ESTOP,
        _ => SAFETY_FAULT,
    }
}

/// Treat exact and suffixed readings as observations of the same component.
fn component_observed(
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
    component_id: &str,
) -> bool {
    let child_prefix = format!("{component_id}/");
    readings.contains_key(component_id)
        || readings.keys().any(|name| name.starts_with(&child_prefix))
}

/// Look up a control reading encoded as `<component>/<suffix>`.
fn child_reading<'a>(
    readings: &'a HashMap<&str, &'a crate::pb::health::SensorReading>,
    component_id: &str,
    suffix: &str,
) -> Option<&'a crate::pb::health::SensorReading> {
    let key = format!("{component_id}/{suffix}");
    readings.get(key.as_str()).copied()
}

fn control_bool(
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
    component_id: &str,
    suffix: &str,
) -> Option<bool> {
    child_reading(readings, component_id, suffix).map(|reading| reading.current_a >= 0.5)
}

fn control_code(
    readings: &HashMap<&str, &crate::pb::health::SensorReading>,
    component_id: &str,
    suffix: &str,
) -> u32 {
    child_reading(readings, component_id, suffix)
        .map(|reading| reading.current_a.max(0.0) as u32)
        .unwrap_or(0)
}

fn observed_scalar(value: f32, unit: &str) -> Option<Scalar> {
    (value.is_finite() && value >= 0.0).then(|| scalar(value as f64, unit))
}

/// Construct one component record before applying frame-specific health.
fn component(
    id: &str,
    parent_id: &str,
    kind: u32,
    name: &str,
    frame_id: &str,
    model: &str,
) -> ComponentStatus {
    ComponentStatus {
        id: id.to_string(),
        parent_id: parent_id.to_string(),
        kind,
        name: name.to_string(),
        frame_id: frame_id.to_string(),
        model: model.to_string(),
        serial: String::new(),
        health: HEALTH_UNKNOWN,
        operational_state: OP_ACTIVE,
        present: true,
        online: true,
        detail: String::new(),
    }
}

fn scalar(value: f64, unit: &str) -> Scalar {
    Scalar {
        value,
        unit: unit.to_string(),
        quality: QUALITY_VALID,
    }
}

fn chrono_now_ns() -> i64 {
    // Use a simple monotonic approach; avoids pulling in chrono just for this.
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    let start = START.get_or_init(std::time::Instant::now);
    start.elapsed().as_nanos() as i64
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::health::{HealthState, SensorReading};
    use std::path::PathBuf;

    fn webots_body() -> SomaBody {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("examples/webots/soma.yaml");
        SomaBody::load(&path).expect("load Webots body")
    }

    /// Build one test reading with battery data marked unavailable.
    fn reading(name: &str, temp_c: f32, voltage: f32, current_a: f32) -> SensorReading {
        SensorReading {
            name: name.to_string(),
            temp_c,
            voltage,
            current_a,
            battery_percent: -1.0,
        }
    }

    /// Nominal TIAGo readings retain wheel, battery, and sensor semantics.
    #[test]
    fn projects_described_webots_health() {
        let mut battery = reading("body/base/battery", 32.0, 24.8, 0.0);
        battery.battery_percent = 82.0;
        let state = HealthState {
            voltage: 24.8,
            charging: false,
            remaining_s: 10800,
            readings: vec![
                reading("body", 36.0, -1.0, -1.0),
                reading("body/base", 34.0, -1.0, -1.0),
                reading("body/base/left_wheel", 38.0, 24.8, 0.7),
                reading("body/base/left_wheel/driver_temp", 41.0, -1.0, -1.0),
                reading("body/base/left_wheel/communication_ok", -1.0, -1.0, 1.0),
                reading("body/base/right_wheel", 38.5, 24.8, 0.7),
                reading("body/base/right_wheel/driver_temp", 41.5, -1.0, -1.0),
                reading("body/base/right_wheel/communication_ok", -1.0, -1.0, 1.0),
                battery,
                reading("body/head_camera", 42.0, -1.0, -1.0),
                reading("body/hokuyo_lidar", 39.0, -1.0, -1.0),
                reading("body/audio", 35.0, -1.0, -1.0),
                reading("body/state", -1.0, -1.0, 0.0),
            ],
        };

        let snapshot = health_state_to_snapshot(&state, &webots_body(), 7);
        assert_eq!(snapshot.body_id, "tiago_webots");
        assert_eq!(snapshot.seq, 7);
        assert_eq!(snapshot.actuators.len(), 2);
        assert!(
            snapshot
                .actuators
                .iter()
                .all(|actuator| actuator.communication_ok)
        );
        assert!(snapshot.actuators.iter().all(|actuator| {
            actuator
                .motor_temp
                .as_ref()
                .is_some_and(|temperature| temperature.value < 40.0)
        }));
        assert_eq!(snapshot.power_sources.len(), 1);
        assert_eq!(
            snapshot.power_sources[0]
                .soc_percent
                .as_ref()
                .expect("battery SOC")
                .value,
            82.0
        );
        assert!(snapshot.faults.is_empty());
        assert_eq!(
            snapshot.safety.as_ref().expect("safety").aggregate_state,
            SAFETY_NORMAL
        );
        assert!(snapshot.metrics.iter().any(|metric| {
            metric.component_id == "body/head_camera" && metric.name == "temperature"
        }));
        assert!(snapshot.components.iter().any(|component| {
            component.id == "body/base/left_wheel"
                && component.kind == KIND_WHEEL
                && component.health == HEALTH_OK
        }));
    }

    /// Existing suffix-only actuator producers remain valid with a component tree.
    #[test]
    fn accepts_suffix_only_actuator_readings() {
        let state = HealthState {
            voltage: -1.0,
            charging: false,
            remaining_s: -1,
            readings: vec![
                reading("body/base/left_wheel/motor_temp", 37.0, -1.0, -1.0),
                reading("body/base/left_wheel/enabled", -1.0, -1.0, 1.0),
                reading("body/base/left_wheel/error", -1.0, -1.0, 0.0),
            ],
        };

        let snapshot = health_state_to_snapshot(&state, &webots_body(), 1);
        let wheel = snapshot
            .actuators
            .iter()
            .find(|actuator| actuator.component_id == "body/base/left_wheel")
            .expect("left wheel actuator");
        assert!(wheel.communication_ok);
        assert!(wheel.torque_enabled);
        assert_eq!(wheel.motor_temp.as_ref().expect("motor temp").value, 37.0);
        assert!(snapshot.components.iter().any(|component| {
            component.id == "body/base/left_wheel" && component.health == HEALTH_OK
        }));
    }
}
