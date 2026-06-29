// SPDX-License-Identifier: MulanPSL-2.0
//
// Mock Soma provider for Vitals development. It serves the same health
// contracts a real Soma implementation will expose, with deterministic
// scenarios that exercise Vitals threshold and transition handling.
//
// When --mock-soma-piper-can is set, mock Soma spawns a Python subprocess
// (piper_bridge.py) that reads real Piper arm data via piper_sdk and merges
// it into the generated SomaHealthSnapshot, replacing synthetic actuator data
// for matching joints.

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
use crate::subprocess::SubprocessHandle;
use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use serde::Deserialize;
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
// Safety aggregate-state constants per Soma SafetyState IDL.
const SAFETY_NORMAL: u32 = 1;
#[allow(dead_code)]
const SAFETY_REDUCED: u32 = 2;
#[allow(dead_code)]
const SAFETY_PROTECTIVE_STOP: u32 = 3;
const SAFETY_ESTOP: u32 = 4;
const SAFETY_FAULT: u32 = 5;
const FAULT_WARN: u32 = 1;
const FAULT_ERROR: u32 = 2;
const ESTOP_RELEASED: u32 = 0;
const ESTOP_TYPE_HARDWARE: u32 = 1;

/// Determines the synthetic behaviour of the mock Soma server.
#[derive(Clone, Copy, Debug)]
pub enum MockScenario {
    Normal,
    Ramp,
    Fault,
    Toggle,
    Mixed,
}

impl MockScenario {
    /// Parse a scenario from a CLI string. Unrecognized values map to Normal with a warning.
    pub fn parse(raw: &str) -> Self {
        match raw.trim().to_ascii_lowercase().as_str() {
            "ramp" => Self::Ramp,
            "fault" => Self::Fault,
            "toggle" => Self::Toggle,
            "mixed" => Self::Mixed,
            other => {
                log::warn!(
                    "[mock_soma] unrecognized scenario '{}', using Normal",
                    other
                );
                Self::Normal
            }
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

// ── Piper bridge ─────────────────────────────────────────────────────────

/// Real Piper joint data from a Python subprocess (piper_bridge.py).
#[derive(Debug, Clone, Deserialize)]
pub(crate) struct PiperJointData {
    pub name: String,
    #[allow(dead_code)]
    pub kind: String,
    pub temperature: f64,
    pub error_code: u32,
    pub enabled: bool,
}

/// Data returned by PiperCollector.collect() via the bridge subprocess.
#[derive(Debug, Clone, Deserialize)]
pub(crate) struct PiperData {
    #[allow(dead_code)]
    pub body_type: String,
    #[allow(dead_code)]
    pub model: String,
    pub state: u32,
    pub message: String,
    pub components: Vec<PiperJointData>,
}

// ── Koch bridge ─────────────────────────────────────────────────────────

/// Real Koch joint data from a Python subprocess (koch_bridge.py).
#[derive(Debug, Clone, Deserialize)]
pub(crate) struct KochJointData {
    pub name: String,
    #[allow(dead_code)]
    pub kind: String,
    pub temperature: f64,
    pub error_code: u32,
    pub enabled: bool,
}

/// Data returned by KochCollector.collect() via the bridge subprocess.
#[derive(Debug, Clone, Deserialize)]
pub(crate) struct KochData {
    #[allow(dead_code)]
    pub body_type: String,
    #[allow(dead_code)]
    pub model: String,
    pub state: u32,
    pub message: String,
    pub components: Vec<KochJointData>,
}

/// Manages a long-running `koch_bridge.py` subprocess for real Koch arm data.
struct KochBridge {
    sub: Arc<std::sync::Mutex<SubprocessHandle>>,
}

impl KochBridge {
    fn new(script: &str, python_bin: &str, serial_port: &str) -> Result<Self> {
        let sub =
            SubprocessHandle::spawn_with_args(script, python_bin, &[serial_port], "koch-bridge")?;
        Ok(Self {
            sub: Arc::new(std::sync::Mutex::new(sub)),
        })
    }

    /// Collect one reading from the Koch hardware. Returns None on I/O
    /// failure or if the JSON response cannot be parsed.
    async fn collect(&self) -> Option<KochData> {
        let sub = self.sub.clone();
        match tokio::task::spawn_blocking(move || {
            let mut handle = sub.lock().unwrap_or_else(|e| e.into_inner());
            let line = handle.collect_json()?;
            match serde_json::from_str::<KochData>(&line) {
                Ok(data) => Some(data),
                Err(e) => {
                    log::error!("[mock_soma] koch bridge parse error: {e:#}");
                    None
                }
            }
        })
        .await
        {
            Ok(data) => data,
            Err(e) => {
                log::error!("[mock_soma] koch bridge thread panicked: {e:#}");
                None
            }
        }
    }
}

/// Manages a long-running `piper_bridge.py` subprocess for real Piper arm data.
struct PiperBridge {
    sub: Arc<std::sync::Mutex<SubprocessHandle>>,
}

impl PiperBridge {
    fn new(script: &str, python_bin: &str, can_port: &str) -> Result<Self> {
        let sub =
            SubprocessHandle::spawn_with_args(script, python_bin, &[can_port], "piper-bridge")?;
        Ok(Self {
            sub: Arc::new(std::sync::Mutex::new(sub)),
        })
    }

    /// Collect one reading from the Piper hardware. Returns None on I/O
    /// failure or if the JSON response cannot be parsed.
    async fn collect(&self) -> Option<PiperData> {
        let sub = self.sub.clone();
        match tokio::task::spawn_blocking(move || {
            let mut handle = sub.lock().unwrap_or_else(|e| e.into_inner());
            let line = handle.collect_json()?;
            match serde_json::from_str::<PiperData>(&line) {
                Ok(data) => Some(data),
                Err(e) => {
                    log::error!("[mock_soma] piper bridge parse error: {e:#}");
                    None
                }
            }
        })
        .await
        {
            Ok(data) => data,
            Err(e) => {
                log::error!("[mock_soma] piper bridge thread panicked: {e:#}");
                None
            }
        }
    }
}

// ── Mock service ─────────────────────────────────────────────────────────

#[derive(Clone)]
struct MockSomaService {
    scenario: MockScenario,
    interval: Duration,
    seq: Arc<RwLock<u64>>,
    piper_bridge: Option<Arc<PiperBridge>>,
    koch_bridge: Option<Arc<KochBridge>>,
}

impl MockSomaService {
    fn new(
        scenario: MockScenario,
        interval: Duration,
        piper_bridge: Option<Arc<PiperBridge>>,
        koch_bridge: Option<Arc<KochBridge>>,
    ) -> Self {
        Self {
            scenario,
            interval,
            seq: Arc::new(RwLock::new(0)),
            piper_bridge,
            koch_bridge,
        }
    }

    async fn next_snapshot(&self) -> SomaHealthSnapshot {
        let seq = {
            let mut guard = self.seq.write().await;
            *guard += 1;
            *guard
        };
        let piper_data = match &self.piper_bridge {
            Some(bridge) => bridge.collect().await,
            None => None,
        };
        let koch_data = match &self.koch_bridge {
            Some(bridge) => bridge.collect().await,
            None => None,
        };
        generate_snapshot(self.scenario, seq, piper_data.as_ref(), koch_data.as_ref())
    }
}

/// Configuration for the optional Piper hardware bridge.
pub struct PiperBridgeConfig {
    pub can_port: String,
    pub python_bin: String,
    pub script: String,
}

/// Configuration for the optional Koch hardware bridge.
pub struct KochBridgeConfig {
    pub serial_port: String,
    pub python_bin: String,
    pub script: String,
}

/// Start a mock Soma gRPC server that registers with Atlas and serves
/// StreamHealth + GetHealth.  Optionally spawns Piper and/or Koch bridge
/// subprocesses for real hardware data merged into synthetic snapshots.
pub async fn run_mock_soma(
    atlas_endpoint: &str,
    provider_id: &str,
    listen: &str,
    scenario_raw: &str,
    interval_ms: u64,
    piper_config: Option<PiperBridgeConfig>,
    koch_config: Option<KochBridgeConfig>,
) -> Result<()> {
    let scenario = MockScenario::parse(scenario_raw);
    let interval = Duration::from_millis(interval_ms.max(1));
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
                "capabilities/system/soma/get_health.v1.toml",
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
                "capabilities/system/soma/health.v1.toml",
                "robonix.contracts.RobonixSystemSomaHealth",
                "/robonix.contracts.RobonixSystemSomaHealth/StreamHealth",
            ),
        )
        .await?;
    if let Err(e) = atlas
        .set_lifecycle_state(provider_id, atlas_pb::LifecycleState::StateActive, "")
        .await
    {
        log::warn!("[mock_soma] SetLifecycleState(ACTIVE) failed: {e:#}");
    }
    spawn_heartbeat(atlas.clone(), provider_id.to_string());

    // Optionally spawn Piper bridge subprocess for real hardware data.
    let piper_bridge = match piper_config {
        Some(ref cfg) => {
            log::info!(
                "[mock_soma] starting piper bridge: {} {} can={}",
                cfg.python_bin,
                cfg.script,
                cfg.can_port
            );
            match PiperBridge::new(&cfg.script, &cfg.python_bin, &cfg.can_port) {
                Ok(bridge) => {
                    log::info!("[mock_soma] piper bridge connected");
                    Some(Arc::new(bridge))
                }
                Err(e) => {
                    log::warn!(
                        "[mock_soma] piper bridge failed: {e:#}; falling back to synthetic data"
                    );
                    None
                }
            }
        }
        None => None,
    };

    // Optionally spawn Koch bridge subprocess for real hardware data.
    let koch_bridge = match koch_config {
        Some(ref cfg) => {
            log::info!(
                "[mock_soma] starting koch bridge: {} {} port={}",
                cfg.python_bin,
                cfg.script,
                cfg.serial_port
            );
            match KochBridge::new(&cfg.script, &cfg.python_bin, &cfg.serial_port) {
                Ok(bridge) => {
                    log::info!("[mock_soma] koch bridge connected");
                    Some(Arc::new(bridge))
                }
                Err(e) => {
                    log::warn!(
                        "[mock_soma] koch bridge failed: {e:#}; falling back to synthetic data"
                    );
                    None
                }
            }
        }
        None => None,
    };

    log::info!(
        "[mock_soma] scenario={} interval_ms={} listening on {} piper={} koch={}",
        scenario.as_str(),
        interval.as_millis(),
        listen_addr,
        piper_bridge.is_some(),
        koch_bridge.is_some()
    );
    eprintln!(
        "mock Soma ready on {listen_addr} scenario={} interval_ms={} piper={} koch={}",
        scenario.as_str(),
        interval.as_millis(),
        piper_bridge.is_some(),
        koch_bridge.is_some()
    );

    let service = MockSomaService::new(scenario, interval, piper_bridge, koch_bridge);

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
            let mut tick = tokio::time::interval(service.interval);
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

/// Generate one synthetic SomaHealthSnapshot for the given scenario + sequence
/// number.  When `piper_data` / `koch_data` is present, real hardware readings
/// replace synthetic actuator values for matching joint names on the
/// corresponding arm.
pub fn generate_snapshot(
    scenario: MockScenario,
    seq: u64,
    piper_data: Option<&PiperData>,
    koch_data: Option<&KochData>,
) -> SomaHealthSnapshot {
    let now = monotonic_ns();
    let now_i64 = now as i64;
    let ramp_enabled = matches!(scenario, MockScenario::Ramp | MockScenario::Mixed);
    let fault_enabled = matches!(scenario, MockScenario::Fault | MockScenario::Mixed);
    let toggle_enabled = matches!(scenario, MockScenario::Toggle | MockScenario::Mixed);

    let mut components = vec![
        component(
            "body",
            "",
            KIND_BODY,
            "Mock Ranger + Piper + Koch",
            "base_link",
            "ranger_piper_koch_v1",
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
            "body/arm_left",
            "body",
            KIND_ARM,
            "Koch arm",
            "arm_left_base_link",
            "koch",
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

    // Piper arm joints (arm_right).
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

    // Koch arm joints (arm_left).
    for joint_idx in 1..=6 {
        components.push(component(
            &format!("body/arm_left/joint_{joint_idx}"),
            "body/arm_left",
            KIND_JOINT,
            &format!("joint_{joint_idx}"),
            &format!("joint_{joint_idx}"),
            "dynamixel_motor",
        ));
    }

    // ── Piper actuators ────────────────────────────────────────────────
    let piper_actuators: Vec<ActuatorState> = if let Some(pd) = piper_data {
        (1..=6)
            .map(|joint_idx| {
                let component_id = format!("body/arm_right/joint_{joint_idx}");
                let joint_name = format!("joint_{joint_idx}");
                let pj = pd.components.iter().find(|c| c.name == joint_name);

                let motor_temp = pj.map(|c| c.temperature).unwrap_or(-1.0);
                let error_code = pj.map(|c| c.error_code).unwrap_or(0);
                let enabled = pj.map(|c| c.enabled).unwrap_or(false);

                ActuatorState {
                    component_id,
                    joint_name,
                    position: Some(scalar(joint_idx as f64 * 0.05, "rad")),
                    velocity: Some(scalar(0.0, "rad/s")),
                    effort: Some(scalar(0.4 + joint_idx as f64 * 0.05, "Nm")),
                    current: Some(scalar(0.7 + joint_idx as f64 * 0.02, "A")),
                    voltage: Some(scalar(24.1, "V")),
                    motor_temp: Some(scalar(motor_temp, "degC")),
                    driver_temp: Some(scalar(motor_temp + 3.0, "degC")),
                    torque_enabled: enabled,
                    brake_engaged: false,
                    communication_ok: pj.is_some(),
                    vendor_mode: 0,
                    vendor_error_code: error_code,
                    status_flags: error_code,
                }
            })
            .collect()
    } else {
        (1..=6)
            .map(|joint_idx| {
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
                actuator(
                    joint_idx,
                    motor_temp,
                    torque_enabled,
                    communication_ok,
                    vendor_error_code,
                    "body/arm_right",
                )
            })
            .collect()
    };

    // ── Koch actuators ─────────────────────────────────────────────────
    let koch_actuators: Vec<ActuatorState> = if let Some(kd) = koch_data {
        (1..=6)
            .map(|joint_idx| {
                let component_id = format!("body/arm_left/joint_{joint_idx}");
                let joint_name = format!("joint_{joint_idx}");
                let kj = kd.components.iter().find(|c| c.name == joint_name);

                let motor_temp = kj.map(|c| c.temperature).unwrap_or(-1.0);
                let error_code = kj.map(|c| c.error_code).unwrap_or(0);
                let enabled = kj.map(|c| c.enabled).unwrap_or(false);

                ActuatorState {
                    component_id,
                    joint_name,
                    position: Some(scalar(joint_idx as f64 * 0.04, "rad")),
                    velocity: Some(scalar(0.0, "rad/s")),
                    effort: Some(scalar(0.3 + joint_idx as f64 * 0.04, "Nm")),
                    current: Some(scalar(0.5 + joint_idx as f64 * 0.02, "A")),
                    voltage: Some(scalar(12.0, "V")),
                    motor_temp: Some(scalar(motor_temp, "degC")),
                    driver_temp: Some(scalar(motor_temp + 2.0, "degC")),
                    torque_enabled: enabled,
                    brake_engaged: false,
                    communication_ok: kj.is_some(),
                    vendor_mode: 0,
                    vendor_error_code: error_code,
                    status_flags: error_code,
                }
            })
            .collect()
    } else {
        (1..=6)
            .map(|joint_idx| {
                let motor_temp = 32.0 + joint_idx as f64;
                actuator(joint_idx, motor_temp, true, true, 0, "body/arm_left")
            })
            .collect()
    };

    let mut actuators = piper_actuators;
    actuators.extend(koch_actuators);

    // ── Faults ─────────────────────────────────────────────────────────
    let mut faults = Vec::new();
    if fault_enabled && seq % 8 >= 4 {
        faults.push(FaultState {
            component_id: "body/arm_right/joint_3".to_string(),
            fault_id: "overcurrent".to_string(),
            severity: FAULT_ERROR,
            active: true,
            clearable: true,
            onset_ts_ns: now_i64,
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
            onset_ts_ns: now_i64,
            vendor_code: 0x02,
            vendor_code_text: "mock_motor_overheat".to_string(),
            message: "mock joint_1 temperature is high".to_string(),
            attributes: vec!["motor_temp".to_string()],
            vendor_raw_json: "{\"motor_temp_high\":true}".to_string(),
        });
    }

    // Merge faults from real Piper data.
    if let Some(pd) = piper_data {
        for pj in &pd.components {
            if pj.error_code != 0 {
                faults.push(FaultState {
                    component_id: format!("body/arm_right/{}", pj.name),
                    fault_id: "piper_foc_fault".to_string(),
                    severity: FAULT_ERROR,
                    active: true,
                    clearable: true,
                    onset_ts_ns: now_i64,
                    vendor_code: pj.error_code,
                    vendor_code_text: format!("0x{:02X}", pj.error_code),
                    message: format!("{} foc_status=0x{:02X}", pj.name, pj.error_code),
                    attributes: vec![],
                    vendor_raw_json: String::new(),
                });
            }
        }
    }

    // Merge faults from real Koch data.
    if let Some(kd) = koch_data {
        for kj in &kd.components {
            if kj.error_code != 0 {
                faults.push(FaultState {
                    component_id: format!("body/arm_left/{}", kj.name),
                    fault_id: "koch_hw_fault".to_string(),
                    severity: FAULT_ERROR,
                    active: true,
                    clearable: true,
                    onset_ts_ns: now_i64,
                    vendor_code: kj.error_code,
                    vendor_code_text: format!("0x{:02X}", kj.error_code),
                    message: format!("{} hw_error=0x{:02X}", kj.name, kj.error_code),
                    attributes: vec![],
                    vendor_raw_json: String::new(),
                });
            }
        }
    }

    // ── Safety state: take the worst of Piper and Koch ─────────────────
    let mut safety_aggregate = SAFETY_NORMAL;

    if let Some(pd) = piper_data {
        let arm_safety = match pd.state {
            0 => SAFETY_NORMAL,
            2 => SAFETY_ESTOP,
            _ => SAFETY_FAULT,
        };
        safety_aggregate = safety_aggregate.max(arm_safety);
    }

    if let Some(kd) = koch_data {
        let arm_safety = if kd.state == 0 {
            SAFETY_NORMAL
        } else {
            SAFETY_FAULT
        };
        safety_aggregate = safety_aggregate.max(arm_safety);
    }

    let mut safety_detail = String::new();
    if let Some(pd) = piper_data
        && !pd.message.is_empty()
    {
        safety_detail.push_str(&pd.message);
    }
    if let Some(kd) = koch_data
        && !kd.message.is_empty()
    {
        if !safety_detail.is_empty() {
            safety_detail.push_str("; ");
        }
        safety_detail.push_str(&kd.message);
    }

    let motion_allowed = safety_aggregate == SAFETY_NORMAL;
    let motor_power_allowed = safety_aggregate == SAFETY_NORMAL;

    SomaHealthSnapshot {
        schema_version: SCHEMA_VERSION,
        body_id: "mock_ranger_piper_koch_01".to_string(),
        seq,
        source_ts_ns: now_i64,
        soma_ts_ns: now_i64,
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
            motion_allowed,
            motor_power_allowed,
            aggregate_state: safety_aggregate,
            detail: safety_detail,
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
    body_prefix: &str,
) -> ActuatorState {
    ActuatorState {
        component_id: format!("{body_prefix}/joint_{joint_idx}"),
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

fn monotonic_ns() -> u64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

#[allow(dead_code)]
fn _health_ok() -> u32 {
    HEALTH_OK
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenario_parse_known_variants() {
        assert!(matches!(
            MockScenario::parse("normal"),
            MockScenario::Normal
        ));
        assert!(matches!(
            MockScenario::parse("  Ramp  "),
            MockScenario::Ramp
        ));
        assert!(matches!(MockScenario::parse("FAULT"), MockScenario::Fault));
        assert!(matches!(
            MockScenario::parse("Toggle"),
            MockScenario::Toggle
        ));
        assert!(matches!(MockScenario::parse("MIXED"), MockScenario::Mixed));
    }

    #[test]
    fn scenario_parse_unknown_defaults_to_normal() {
        // Unknown input should map to Normal (with a warning log, not tested here).
        assert!(matches!(
            MockScenario::parse("nonexistent"),
            MockScenario::Normal
        ));
        assert!(matches!(MockScenario::parse(""), MockScenario::Normal));
    }

    #[test]
    fn normal_snapshot_has_both_arms() {
        let snapshot = generate_snapshot(MockScenario::Normal, 1, None, None);
        // body + computer_jetson + cpu + gpu + arm_right + arm_left + battery_main = 7
        // + 6 right joints + 6 left joints = 19
        assert_eq!(snapshot.components.len(), 19);
        // 6 Piper actuators + 6 Koch actuators = 12
        assert_eq!(snapshot.actuators.len(), 12);
        assert_eq!(snapshot.faults.len(), 0);
        // Verify both arms are represented.
        assert!(
            snapshot
                .actuators
                .iter()
                .any(|a| a.component_id == "body/arm_right/joint_1")
        );
        assert!(
            snapshot
                .actuators
                .iter()
                .any(|a| a.component_id == "body/arm_left/joint_1")
        );
    }

    #[test]
    fn fault_scenario_produces_fault_at_sequence_4() {
        let snapshot = generate_snapshot(MockScenario::Fault, 4, None, None);
        assert!(!snapshot.faults.is_empty());
        assert!(snapshot.faults.iter().any(|f| f.fault_id == "overcurrent"));
        // joint_3 on arm_right should have communication_ok = false
        let j3 = snapshot
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm_right/joint_3")
            .unwrap();
        assert!(!j3.communication_ok);
    }

    #[test]
    fn toggle_scenario_disables_joint_6_at_sequence_4() {
        let snapshot = generate_snapshot(MockScenario::Toggle, 4, None, None);
        let j6 = snapshot
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm_right/joint_6")
            .unwrap();
        assert!(!j6.torque_enabled);
    }
}
