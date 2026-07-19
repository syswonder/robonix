// SPDX-License-Identifier: MulanPSL-2.0
//
// Soma health data collector — discovers health primitives via Atlas,
// consumes their gRPC HealthState streams, aggregates data into
// SomaHealthSnapshot, and broadcasts to subscribers (Vitals).

use crate::pb::soma::{
    ActuatorState, ComponentStatus, FaultState, SafetyEndpointState, SafetyState, Scalar,
    SomaHealthSnapshot,
};
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_scribe::info;
use tokio::sync::broadcast;
use tokio::time::{Duration, interval};
use tokio_stream::StreamExt;
use tonic::transport::Channel;

const SCHEMA_VERSION: u32 = 1;
const QUALITY_VALID: u32 = 0;
const KIND_BODY: u32 = 1;
const KIND_ARM: u32 = 2;
const KIND_JOINT: u32 = 4;
const OP_ACTIVE: u32 = 4;
const SAFETY_NORMAL: u32 = 1;
const SAFETY_FAULT: u32 = 5;
const SAFETY_ESTOP: u32 = 4;
const ESTOP_RELEASED: u32 = 0;
const ESTOP_TYPE_HARDWARE: u32 = 1;
const FAULT_ERROR: u32 = 2;

/// How often SOMA pushes a new SomaHealthSnapshot (when no new primitive data arrives).
const DEFAULT_PUSH_INTERVAL_MS: u64 = 500;

/// Start health data collection: discover primitives via Atlas,
/// consume their gRPC HealthState streams, aggregate into
/// SomaHealthSnapshot, and broadcast.
pub async fn start_health_collector(
    mut atlas: AtlasClient,
    body_id: String,
    arm_model: String,
    tx: broadcast::Sender<SomaHealthSnapshot>,
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
        info!("[soma-health] no health primitives found — sending empty snapshots");
        // Keep broadcasting empty snapshots so Vitals doesn't stall.
        tokio::spawn(async move {
            let mut tick = interval(Duration::from_millis(DEFAULT_PUSH_INTERVAL_MS));
            let mut seq: u64 = 0;
            loop {
                tick.tick().await;
                seq += 1;
                let snapshot = empty_snapshot(&body_id, &arm_model, seq);
                let _ = tx.send(snapshot);
            }
        });
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
        let tx = tx.clone();
        let body_id = body_id.clone();
        let arm_model = arm_model.clone();
        let mut atlas = atlas.clone();

        tokio::spawn(async move {
            if let Err(e) =
                consume_primitive_stream(&mut atlas, &provider_id, body_id, arm_model, tx).await
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
    body_id: String,
    arm_model: String,
    tx: broadcast::Sender<SomaHealthSnapshot>,
) -> Result<()> {
    use crate::pb::contracts::robonix_primitive_health_stream_client::RobonixPrimitiveHealthStreamClient;
    use crate::pb::health::StreamHealthStateRequest;
    use robonix_atlas::pb as atlas_pb;

    // Get the endpoint from Atlas.
    let (_channel_id, endpoint_str, _params) = atlas
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

    let channel = Channel::from_shared(normalized.clone())
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

    let mut seq: u64 = 0;
    while let Some(result) = stream.next().await {
        match result {
            Ok(health_state) => {
                seq += 1;
                let snapshot = health_state_to_snapshot(&health_state, &body_id, &arm_model, seq);
                let _ = tx.send(snapshot);
            }
            Err(e) => {
                robonix_scribe::warn!("[soma-health] stream error from '{}': {e:#}", provider_id);
                break;
            }
        }
    }

    Ok(())
}

/// Convert a HealthState frame from the primitive into a SomaHealthSnapshot.
fn health_state_to_snapshot(
    state: &crate::pb::health::HealthState,
    body_id: &str,
    arm_model: &str,
    seq: u64,
) -> SomaHealthSnapshot {
    let now_ns = chrono_now_ns();

    // Build component tree: root + arm + joints.
    let mut components = vec![
        component("body", "", KIND_BODY, "Piper Robot", "base_link", "piper"),
        component(
            "body/arm",
            "body",
            KIND_ARM,
            "Piper arm",
            "arm_base_link",
            arm_model,
        ),
    ];
    for joint_idx in 1..=6 {
        components.push(component(
            &format!("body/arm/joint_{joint_idx}"),
            "body/arm",
            KIND_JOINT,
            &format!("joint_{joint_idx}"),
            &format!("joint_{joint_idx}"),
            arm_model,
        ));
    }

    // Parse sensor readings into per-joint data.
    let mut joint_temps = [0.0f64; 6];
    let mut joint_errors = [0u32; 6];
    let mut joint_enabled = [true; 6];
    let mut arm_state: u32 = SAFETY_NORMAL;

    for reading in &state.readings {
        let name = &reading.name;
        // Parse joint index from name like "body/arm/joint_N/..."
        if let Some(joint_idx) = parse_joint_index(name) {
            if name.ends_with("/motor_temp") {
                joint_temps[joint_idx] = reading.temp_c as f64;
            } else if name.ends_with("/error") {
                joint_errors[joint_idx] = reading.current_a as u32;
            } else if name.ends_with("/enabled") {
                joint_enabled[joint_idx] = reading.current_a >= 0.5;
            }
        } else if name == "body/arm/state" {
            let raw = reading.current_a as u32;
            arm_state = match raw {
                0 => SAFETY_NORMAL,
                2 => SAFETY_ESTOP,
                _ => SAFETY_FAULT,
            };
        }
    }

    // Build actuators from parsed joint data.
    let actuators: Vec<ActuatorState> = (0..6)
        .map(|i| {
            let joint_idx = (i + 1) as u32;
            ActuatorState {
                component_id: format!("body/arm/joint_{joint_idx}"),
                joint_name: format!("joint_{joint_idx}"),
                position: Some(scalar(0.0, "rad")),
                velocity: Some(scalar(0.0, "rad/s")),
                effort: Some(scalar(0.0, "Nm")),
                current: Some(scalar(0.0, "A")),
                voltage: Some(scalar(24.0, "V")),
                motor_temp: Some(scalar(joint_temps[i], "degC")),
                driver_temp: Some(scalar(joint_temps[i] + 3.0, "degC")),
                torque_enabled: joint_enabled[i],
                brake_engaged: false,
                communication_ok: joint_temps[i] >= 0.0,
                vendor_mode: 0,
                vendor_error_code: joint_errors[i],
                status_flags: joint_errors[i],
            }
        })
        .collect();

    // Build faults from non-zero error codes.
    let mut faults = Vec::new();
    for (i, err) in joint_errors.iter().enumerate() {
        if *err != 0 {
            faults.push(FaultState {
                component_id: format!("body/arm/joint_{}", i + 1),
                fault_id: "piper_foc_fault".to_string(),
                severity: FAULT_ERROR,
                active: true,
                clearable: true,
                onset_ts_ns: now_ns,
                vendor_code: *err,
                vendor_code_text: format!("0x{err:02X}"),
                message: format!("joint_{} foc_status=0x{err:02X}", i + 1),
                attributes: vec![],
                vendor_raw_json: String::new(),
            });
        }
    }

    let motion_allowed = arm_state == SAFETY_NORMAL;
    let motor_power_allowed = arm_state == SAFETY_NORMAL;

    SomaHealthSnapshot {
        schema_version: SCHEMA_VERSION,
        body_id: body_id.to_string(),
        seq,
        source_ts_ns: now_ns,
        soma_ts_ns: now_ns,
        ttl_ms: 1500,
        components,
        actuators,
        power_sources: vec![],
        safety: Some(SafetyState {
            motion_allowed,
            motor_power_allowed,
            aggregate_state: arm_state,
            detail: String::new(),
        }),
        safety_endpoints: vec![SafetyEndpointState {
            name: "hardware_estop".to_string(),
            r#type: ESTOP_TYPE_HARDWARE,
            state: ESTOP_RELEASED,
            detail: String::new(),
        }],
        faults,
        metrics: vec![],
    }
}

/// Build an empty snapshot (used when no health primitives are available).
fn empty_snapshot(body_id: &str, arm_model: &str, seq: u64) -> SomaHealthSnapshot {
    let now_ns = chrono_now_ns();
    let mut components = vec![
        component("body", "", KIND_BODY, "Piper Robot", "base_link", "piper"),
        component(
            "body/arm",
            "body",
            KIND_ARM,
            "Piper arm",
            "arm_base_link",
            arm_model,
        ),
    ];
    for joint_idx in 1..=6 {
        components.push(component(
            &format!("body/arm/joint_{joint_idx}"),
            "body/arm",
            KIND_JOINT,
            &format!("joint_{joint_idx}"),
            &format!("joint_{joint_idx}"),
            arm_model,
        ));
    }

    SomaHealthSnapshot {
        schema_version: SCHEMA_VERSION,
        body_id: body_id.to_string(),
        seq,
        source_ts_ns: now_ns,
        soma_ts_ns: now_ns,
        ttl_ms: 1500,
        components,
        actuators: vec![],
        power_sources: vec![],
        safety: Some(SafetyState {
            motion_allowed: false,
            motor_power_allowed: false,
            aggregate_state: SAFETY_FAULT,
            detail: "no health primitive connected".to_string(),
        }),
        safety_endpoints: vec![],
        faults: vec![],
        metrics: vec![],
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────

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
        health: 4, // HEALTH_UNKNOWN
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

/// Parse "body/arm/joint_N/..." → Some(N-1) (0-indexed), or None.
fn parse_joint_index(name: &str) -> Option<usize> {
    let prefix = "body/arm/joint_";
    let rest = name.strip_prefix(prefix)?;
    let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
    if digits.is_empty() {
        return None;
    }
    let idx: usize = digits.parse().ok()?;
    if !(1..=6).contains(&idx) {
        return None;
    }
    Some(idx - 1)
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

    #[test]
    fn parse_joint_index_valid() {
        assert_eq!(parse_joint_index("body/arm/joint_1/motor_temp"), Some(0));
        assert_eq!(parse_joint_index("body/arm/joint_6/enabled"), Some(5));
        assert_eq!(parse_joint_index("body/arm/joint_3/error"), Some(2));
    }

    #[test]
    fn parse_joint_index_invalid() {
        assert_eq!(parse_joint_index("body/arm/joint_7/motor_temp"), None);
        assert_eq!(parse_joint_index("body/arm/joint_0/motor_temp"), None);
        assert_eq!(parse_joint_index("body/leg/joint_1/motor_temp"), None);
        assert_eq!(parse_joint_index("random_string"), None);
    }
}
