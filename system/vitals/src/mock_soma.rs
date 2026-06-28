// SPDX-License-Identifier: MulanPSL-2.0
//
// Mock Soma provider for Vitals development. It serves the same health
// contracts a real Soma implementation will expose, with deterministic
// scenarios that exercise Vitals threshold and transition handling.

use crate::pb::contracts::robonix_system_soma_get_health_server::{
    RobonixSystemSomaGetHealth, RobonixSystemSomaGetHealthServer,
};
use crate::pb::contracts::robonix_system_soma_health_server::{
    RobonixSystemSomaHealth, RobonixSystemSomaHealthServer,
};
use crate::pb::soma::{
    ActuatorState, ComponentStatus, FaultState, GetHealthRequest, GetHealthResponse, Metric,
    PowerSourceState, SafetyEndpointState, SafetyState, Scalar, SomaHealthSnapshot,
    StreamHealthRequest,
};
use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::OnceLock;
use std::time::{Duration, Instant};
use tokio::sync::RwLock;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const SCHEMA_VERSION: u32 = 1;
const QUALITY_VALID: u32 = 0;
const HEALTH_OK: u32 = 0;
const HEALTH_UNKNOWN: u32 = 4;
const KIND_BODY: u32 = 1;
const KIND_ARM: u32 = 2;
const KIND_JOINT: u32 = 4;
const KIND_BATTERY: u32 = 7;
const KIND_COMPUTER: u32 = 8;
const KIND_SENSOR: u32 = 9;
const OP_ACTIVE: u32 = 4;
const SAFETY_NORMAL: u32 = 1;
const FAULT_WARN: u32 = 1;
const FAULT_ERROR: u32 = 2;
const ESTOP_RELEASED: u32 = 0;
const ESTOP_TYPE_HARDWARE: u32 = 1;

#[derive(Clone, Copy, Debug)]
pub enum MockScenario {
    Normal,
    Ramp,
    Fault,
    Toggle,
    Mixed,
}

impl MockScenario {
    pub fn parse(raw: &str) -> Self {
        match raw.trim().to_ascii_lowercase().as_str() {
            "ramp" => Self::Ramp,
            "fault" => Self::Fault,
            "toggle" => Self::Toggle,
            "mixed" => Self::Mixed,
            _ => Self::Normal,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::Normal => "normal",
            Self::Ramp => "ramp",
            Self::Fault => "fault",
            Self::Toggle => "toggle",
            Self::Mixed => "mixed",
        }
    }
}

#[derive(Clone)]
struct MockSomaService {
    scenario: MockScenario,
    seq: Arc<RwLock<u64>>,
}

impl MockSomaService {
    fn new(scenario: MockScenario) -> Self {
        Self {
            scenario,
            seq: Arc::new(RwLock::new(0)),
        }
    }

    async fn next_snapshot(&self) -> SomaHealthSnapshot {
        let mut seq = self.seq.write().await;
        *seq += 1;
        generate_snapshot(self.scenario, *seq)
    }
}

pub async fn run_mock_soma(
    atlas_endpoint: &str,
    provider_id: &str,
    listen: &str,
    scenario_raw: &str,
) -> Result<()> {
    let scenario = MockScenario::parse(scenario_raw);
    let listen_addr: SocketAddr = listen
        .parse()
        .with_context(|| format!("invalid mock Soma listen address '{listen}'"))?;
    let advertised = advertised_addr(listen_addr);

    let mut atlas = AtlasClient::connect_with_retry(atlas_endpoint, 10, Duration::from_secs(2))
        .await
        .context("connect to atlas for mock Soma")?;

    atlas
        .register_service(provider_id, "robonix/system/soma", "")
        .await?;
    atlas
        .declare_capability(
            provider_id,
            "robonix/system/soma/get_health",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/service/soma/get_health.v1.toml",
                "robonix.contracts.RobonixSystemSomaGetHealth",
                "/robonix.contracts.RobonixSystemSomaGetHealth/GetHealth",
            ),
        )
        .await?;
    atlas
        .declare_capability(
            provider_id,
            "robonix/system/soma/health",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/service/soma/health.v1.toml",
                "robonix.contracts.RobonixSystemSomaHealth",
                "/robonix.contracts.RobonixSystemSomaHealth/StreamHealth",
            ),
        )
        .await?;
    let _ = atlas
        .set_lifecycle_state(provider_id, atlas_pb::LifecycleState::StateActive, "")
        .await;
    spawn_heartbeat(atlas.clone(), provider_id.to_string());

    let service = MockSomaService::new(scenario);
    log::info!(
        "[mock_soma] scenario={} listening on {}",
        scenario.as_str(),
        listen_addr
    );
    eprintln!(
        "mock Soma ready on {listen_addr} scenario={}",
        scenario.as_str()
    );

    tonic::transport::Server::builder()
        .add_service(RobonixSystemSomaGetHealthServer::new(service.clone()))
        .add_service(RobonixSystemSomaHealthServer::new(service))
        .serve(listen_addr)
        .await
        .context("mock Soma gRPC server failed")?;

    Ok(())
}

fn spawn_heartbeat(mut atlas: AtlasClient, provider_id: String) {
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(20));
        tick.tick().await;
        loop {
            tick.tick().await;
            if let Err(e) = atlas.heartbeat(&provider_id).await {
                log::warn!("[mock_soma] heartbeat failed: {e:#}");
            }
        }
    });
}

fn advertised_addr(listen_addr: SocketAddr) -> String {
    match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaGetHealth for MockSomaService {
    async fn get_health(
        &self,
        _request: Request<GetHealthRequest>,
    ) -> Result<Response<GetHealthResponse>, Status> {
        Ok(Response::new(GetHealthResponse {
            snapshot: Some(self.next_snapshot().await),
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaHealth for MockSomaService {
    type StreamHealthStream = ReceiverStream<Result<SomaHealthSnapshot, Status>>;

    async fn stream_health(
        &self,
        _request: Request<StreamHealthRequest>,
    ) -> Result<Response<Self::StreamHealthStream>, Status> {
        let service = self.clone();
        let (tx, rx) = tokio::sync::mpsc::channel(16);
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(1));
            tick.tick().await;
            loop {
                tick.tick().await;
                let snapshot = service.next_snapshot().await;
                if tx.send(Ok(snapshot)).await.is_err() {
                    break;
                }
            }
        });
        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

pub fn generate_snapshot(scenario: MockScenario, seq: u64) -> SomaHealthSnapshot {
    let now = monotonic_ns();
    let ramp_enabled = matches!(scenario, MockScenario::Ramp | MockScenario::Mixed);
    let fault_enabled = matches!(scenario, MockScenario::Fault | MockScenario::Mixed);
    let toggle_enabled = matches!(scenario, MockScenario::Toggle | MockScenario::Mixed);

    let mut components = vec![
        component(
            "body",
            "",
            KIND_BODY,
            "Mock Ranger + Piper",
            "base_link",
            "ranger_piper_v1",
        ),
        component(
            "body/computer_jetson",
            "body",
            KIND_COMPUTER,
            "Jetson AGX Orin",
            "",
            "jetson_agx_orin",
        ),
        component(
            "body/computer_jetson/cpu",
            "body/computer_jetson",
            KIND_SENSOR,
            "cpu",
            "",
            "thermal_zone",
        ),
        component(
            "body/computer_jetson/gpu",
            "body/computer_jetson",
            KIND_SENSOR,
            "gpu",
            "",
            "thermal_zone",
        ),
        component(
            "body/arm_right",
            "body",
            KIND_ARM,
            "Piper arm",
            "arm_base_link",
            "piper",
        ),
        component(
            "body/battery_main",
            "body",
            KIND_BATTERY,
            "main battery",
            "",
            "mock_bms",
        ),
    ];

    for joint_idx in 1..=6 {
        components.push(component(
            &format!("body/arm_right/joint_{joint_idx}"),
            "body/arm_right",
            KIND_JOINT,
            &format!("joint_{joint_idx}"),
            &format!("joint_{joint_idx}"),
            "piper_motor",
        ));
    }

    let mut actuators = Vec::new();
    for joint_idx in 1..=6 {
        let mut motor_temp = 36.0 + joint_idx as f64;
        if ramp_enabled && joint_idx == 1 {
            let phase = (seq.saturating_sub(1) % 30) as f64;
            motor_temp = 40.0 + phase * 1.6;
        }
        let communication_ok = !(fault_enabled && joint_idx == 3 && seq % 8 >= 4);
        let torque_enabled = !(toggle_enabled && joint_idx == 6 && seq % 8 >= 4);
        let vendor_error_code = if fault_enabled && joint_idx == 3 && seq % 8 >= 4 {
            0x04
        } else {
            0
        };
        actuators.push(actuator(
            joint_idx,
            motor_temp,
            torque_enabled,
            communication_ok,
            vendor_error_code,
        ));
    }

    let mut faults = Vec::new();
    if fault_enabled && seq % 8 >= 4 {
        faults.push(FaultState {
            component_id: "body/arm_right/joint_3".to_string(),
            fault_id: "overcurrent".to_string(),
            severity: FAULT_ERROR,
            active: true,
            clearable: true,
            onset_ts_ns: now,
            vendor_code: 0x04,
            vendor_code_text: "mock_overcurrent".to_string(),
            message: "mock joint_3 overcurrent".to_string(),
            attributes: vec!["foc_status.bit2".to_string()],
            vendor_raw_json: "{\"foc_status\":4}".to_string(),
        });
    }
    if ramp_enabled && seq % 30 >= 22 {
        faults.push(FaultState {
            component_id: "body/arm_right/joint_1".to_string(),
            fault_id: "motor_overheat".to_string(),
            severity: FAULT_WARN,
            active: true,
            clearable: false,
            onset_ts_ns: now,
            vendor_code: 0x02,
            vendor_code_text: "mock_motor_overheat".to_string(),
            message: "mock joint_1 temperature is high".to_string(),
            attributes: vec!["motor_temp".to_string()],
            vendor_raw_json: "{\"motor_temp_high\":true}".to_string(),
        });
    }

    SomaHealthSnapshot {
        schema_version: SCHEMA_VERSION,
        body_id: "mock_ranger_piper_01".to_string(),
        seq,
        source_ts_ns: now,
        soma_ts_ns: now,
        ttl_ms: 1500,
        components,
        actuators,
        power_sources: vec![PowerSourceState {
            component_id: "body/battery_main".to_string(),
            soc_percent: Some(scalar(76.0, "percent")),
            soh_percent: Some(scalar(96.0, "percent")),
            voltage: Some(scalar(24.2, "V")),
            current: Some(scalar(-3.1, "A")),
            temperature: Some(scalar(32.0, "degC")),
            remaining_s: Some(scalar(7200.0, "s")),
            cycle_count: 142,
            cell_voltages: Vec::new(),
            vendor_status_code: 0,
        }],
        safety: Some(SafetyState {
            motion_allowed: true,
            motor_power_allowed: true,
            aggregate_state: SAFETY_NORMAL,
            detail: String::new(),
        }),
        safety_endpoints: vec![SafetyEndpointState {
            name: "hardware_estop".to_string(),
            r#type: ESTOP_TYPE_HARDWARE,
            state: ESTOP_RELEASED,
            detail: String::new(),
        }],
        faults,
        metrics: vec![
            metric(
                "body/computer_jetson/cpu",
                "temperature",
                43.0,
                "degC",
                "thermal_zone0",
            ),
            metric(
                "body/computer_jetson/gpu",
                "temperature",
                45.0,
                "degC",
                "thermal_zone1",
            ),
            metric(
                "body/computer_jetson",
                "fan_rpm",
                1800.0,
                "rpm",
                "fan1_input",
            ),
        ],
    }
}

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

fn actuator(
    joint_idx: u32,
    motor_temp: f64,
    torque_enabled: bool,
    communication_ok: bool,
    vendor_error_code: u32,
) -> ActuatorState {
    ActuatorState {
        component_id: format!("body/arm_right/joint_{joint_idx}"),
        joint_name: format!("joint_{joint_idx}"),
        position: Some(scalar(joint_idx as f64 * 0.05, "rad")),
        velocity: Some(scalar(0.0, "rad/s")),
        effort: Some(scalar(0.4 + joint_idx as f64 * 0.05, "Nm")),
        current: Some(scalar(0.7 + joint_idx as f64 * 0.02, "A")),
        voltage: Some(scalar(24.1, "V")),
        motor_temp: Some(scalar(motor_temp, "degC")),
        driver_temp: Some(scalar(motor_temp + 3.0, "degC")),
        torque_enabled,
        brake_engaged: false,
        communication_ok,
        vendor_mode: 0,
        vendor_error_code,
        status_flags: vendor_error_code,
    }
}

fn metric(component_id: &str, name: &str, value: f64, unit: &str, source_key: &str) -> Metric {
    Metric {
        component_id: component_id.to_string(),
        name: name.to_string(),
        value: Some(scalar(value, unit)),
        source_key: source_key.to_string(),
    }
}

fn scalar(value: f64, unit: &str) -> Scalar {
    Scalar {
        value,
        unit: unit.to_string(),
        quality: QUALITY_VALID,
    }
}

fn monotonic_ns() -> i64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as i64
}

#[allow(dead_code)]
fn _health_ok() -> u32 {
    HEALTH_OK
}
