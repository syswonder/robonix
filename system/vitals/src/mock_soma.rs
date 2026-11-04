// SPDX-License-Identifier: MulanPSL-2.0
//
// Mock Soma provider for Vitals development. It serves the same health
// contracts a real Soma implementation will expose, with deterministic
// scenarios that exercise Vitals threshold and transition handling.
//
// When --mock-soma-arm is set to piper or koch, mock Soma spawns a Python
// subprocess (piper_bridge.py / koch_bridge.py) that reads real arm data
// via the appropriate SDK and merges it into the generated SomaHealthSnapshot,
// replacing synthetic actuator data for matching joints.

use crate::pb::contracts::robonix_lifecycle_driver_server::RobonixLifecycleDriverServer;
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
use crate::{
    CMD_ACTIVATE, CMD_INIT, SHARED_DRIVER_CONTRACT, SystemLifecycleDriver, call_startup_driver,
    connect_startup_driver, startup_driver_endpoint, wait_for_driver_shutdown,
};
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
            "normal" => Self::Normal,
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

// ── Arm bridge ───────────────────────────────────────────────────────────

/// Arm data returned by a hardware bridge subprocess.
pub(crate) enum ArmData {
    Piper(PiperData),
    Koch(KochData),
}

/// A hardware bridge subprocess: Piper (CAN), Koch (Dynamixel), or none.
enum ArmBridgeEnum {
    Piper(PiperBridge),
    Koch(KochBridge),
}

impl ArmBridgeEnum {
    async fn collect(&self) -> Option<ArmData> {
        match self {
            Self::Piper(b) => b.collect().await.map(ArmData::Piper),
            Self::Koch(b) => b.collect().await.map(ArmData::Koch),
        }
    }
}

// ── Mock service ─────────────────────────────────────────────────────────

#[derive(Clone)]
struct MockSomaService {
    scenario: MockScenario,
    interval: Duration,
    seq: Arc<RwLock<u64>>,
    arm_bridge: Option<Arc<ArmBridgeEnum>>,
}

impl MockSomaService {
    fn new(
        scenario: MockScenario,
        interval: Duration,
        arm_bridge: Option<Arc<ArmBridgeEnum>>,
    ) -> Self {
        Self {
            scenario,
            interval,
            seq: Arc::new(RwLock::new(0)),
            arm_bridge,
        }
    }

    async fn next_snapshot(&self) -> SomaHealthSnapshot {
        let seq = {
            let mut guard = self.seq.write().await;
            *guard += 1;
            *guard
        };
        let arm_data = match &self.arm_bridge {
            Some(bridge) => bridge.collect().await,
            None => None,
        };
        generate_snapshot(self.scenario, seq, arm_data.as_ref())
    }
}

use crate::config::MockArmConfig;

/// Start a mock Soma gRPC server that registers with Atlas and serves
/// StreamHealth + GetHealth.  Optionally spawns a hardware bridge subprocess
/// (Piper or Koch) for real arm data merged into synthetic snapshots.
pub async fn run_mock_soma(
    atlas_endpoint: &str,
    provider_id: &str,
    listen: &str,
    scenario_raw: &str,
    interval_ms: u64,
    arm_config: MockArmConfig,
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
            SHARED_DRIVER_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/lifecycle/driver.v1.toml",
                "robonix.contracts.RobonixLifecycleDriver",
                "/robonix.contracts.RobonixLifecycleDriver/Driver",
            ),
        )
        .await
        .context("declare mock Soma shared lifecycle Driver")?;
    let lifecycle = SystemLifecycleDriver::new(atlas.clone(), provider_id.to_string());
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
    // Optionally spawn a hardware bridge subprocess for real arm data.
    let arm_bridge: Option<Arc<ArmBridgeEnum>> = match &arm_config {
        MockArmConfig::Piper {
            can_port,
            python_bin,
            script,
        } => {
            let script_str = script.to_string_lossy();
            log::info!(
                "[mock_soma] starting piper bridge: {} {} can={}",
                python_bin,
                script_str,
                can_port
            );
            match PiperBridge::new(&script_str, python_bin, can_port) {
                Ok(bridge) => {
                    log::info!("[mock_soma] piper bridge connected");
                    Some(Arc::new(ArmBridgeEnum::Piper(bridge)))
                }
                Err(e) => {
                    log::warn!(
                        "[mock_soma] piper bridge failed: {e:#}; falling back to synthetic data"
                    );
                    None
                }
            }
        }
        MockArmConfig::Koch {
            serial_port,
            python_bin,
            script,
        } => {
            let script_str = script.to_string_lossy();
            log::info!(
                "[mock_soma] starting koch bridge: {} {} port={}",
                python_bin,
                script_str,
                serial_port
            );
            match KochBridge::new(&script_str, python_bin, serial_port) {
                Ok(bridge) => {
                    log::info!("[mock_soma] koch bridge connected");
                    Some(Arc::new(ArmBridgeEnum::Koch(bridge)))
                }
                Err(e) => {
                    log::warn!(
                        "[mock_soma] koch bridge failed: {e:#}; falling back to synthetic data"
                    );
                    None
                }
            }
        }
        MockArmConfig::Synthetic => None,
    };

    let bridge_active = arm_bridge.is_some();
    let service = MockSomaService::new(scenario, interval, arm_bridge);
    let server_shutdown = lifecycle.subscribe_shutdown();
    let server_lifecycle = lifecycle.clone();
    let mut server_task = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(server_lifecycle))
            .add_service(RobonixSystemSomaGetHealthServer::new(service.clone()))
            .add_service(RobonixSystemSomaHealthServer::new(service))
            .serve_with_shutdown(listen_addr, wait_for_driver_shutdown(server_shutdown))
            .await
    });
    let startup_endpoint = startup_driver_endpoint(listen_addr);
    let mut startup_driver = tokio::select! {
        client = connect_startup_driver(&startup_endpoint) => client?,
        result = &mut server_task => {
            result.context("join mock Soma gRPC server")?
                .context("mock Soma gRPC server failed before readiness")?;
            anyhow::bail!("mock Soma gRPC server stopped before readiness");
        }
    };
    call_startup_driver(&mut startup_driver, CMD_INIT)
        .await
        .context("initialize mock Soma lifecycle")?;
    call_startup_driver(&mut startup_driver, CMD_ACTIVATE)
        .await
        .context("activate mock Soma lifecycle")?;
    drop(startup_driver);
    spawn_heartbeat(
        atlas.clone(),
        provider_id.to_string(),
        lifecycle.subscribe_shutdown(),
    );

    log::info!(
        "[mock_soma] scenario={} interval_ms={} listening on {} arm={} bridge={}",
        scenario.as_str(),
        interval.as_millis(),
        listen_addr,
        arm_config.label(),
        bridge_active
    );
    eprintln!(
        "mock Soma ready on {listen_addr} scenario={} interval_ms={} arm={} bridge={}",
        scenario.as_str(),
        interval.as_millis(),
        arm_config.label(),
        bridge_active,
    );

    server_task
        .await
        .context("join mock Soma gRPC server")?
        .context("mock Soma gRPC server failed")?;

    Ok(())
}

fn spawn_heartbeat(
    mut atlas: AtlasClient,
    provider_id: String,
    shutdown: tokio::sync::watch::Receiver<bool>,
) {
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_secs(20));
        tick.tick().await;
        let shutdown = wait_for_driver_shutdown(shutdown);
        tokio::pin!(shutdown);
        loop {
            tokio::select! {
                _ = &mut shutdown => break,
                _ = tick.tick() => {
                    if let Err(e) = atlas.heartbeat(&provider_id).await {
                        log::warn!("[mock_soma] heartbeat failed: {e:#}");
                    }
                }
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
/// number.  When `arm_data` is present, real hardware readings replace synthetic
/// actuator values for matching joint names.
pub fn generate_snapshot(
    scenario: MockScenario,
    seq: u64,
    arm_data: Option<&ArmData>,
) -> SomaHealthSnapshot {
    let now = monotonic_ns();
    let now_i64 = now as i64;
    let ramp_enabled = matches!(scenario, MockScenario::Ramp | MockScenario::Mixed);
    let fault_enabled = matches!(scenario, MockScenario::Fault | MockScenario::Mixed);
    let toggle_enabled = matches!(scenario, MockScenario::Toggle | MockScenario::Mixed);

    // Determine arm model name and motor model from arm_data variant.
    let (arm_name, arm_model, motor_model, body_id) = match arm_data {
        Some(ArmData::Piper(_)) => ("Piper arm", "piper", "piper_motor", "mock_ranger_piper_01"),
        Some(ArmData::Koch(_)) => ("Koch arm", "koch", "dynamixel_motor", "mock_ranger_koch_01"),
        None => ("Mock arm", "mock_arm", "mock_motor", "mock_ranger_01"),
    };

    let mut components = vec![
        component(
            "body",
            "",
            KIND_BODY,
            "Mock Ranger",
            "base_link",
            "ranger_mock_v1",
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
            "body/arm",
            "body",
            KIND_ARM,
            arm_name,
            "arm_base_link",
            arm_model,
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
            &format!("body/arm/joint_{joint_idx}"),
            "body/arm",
            KIND_JOINT,
            &format!("joint_{joint_idx}"),
            &format!("joint_{joint_idx}"),
            motor_model,
        ));
    }

    // ── Actuators ──────────────────────────────────────────────────────
    let actuators: Vec<ActuatorState> = match arm_data {
        Some(ArmData::Piper(pd)) => (1..=6)
            .map(|joint_idx| {
                let component_id = format!("body/arm/joint_{joint_idx}");
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
            .collect(),
        Some(ArmData::Koch(kd)) => (1..=6)
            .map(|joint_idx| {
                let component_id = format!("body/arm/joint_{joint_idx}");
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
            .collect(),
        None => (1..=6)
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
                )
            })
            .collect(),
    };

    // ── Faults ─────────────────────────────────────────────────────────
    let mut faults = Vec::new();
    if fault_enabled && seq % 8 >= 4 {
        faults.push(FaultState {
            component_id: "body/arm/joint_3".to_string(),
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
            component_id: "body/arm/joint_1".to_string(),
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

    // Merge faults from real arm data.
    match arm_data {
        Some(ArmData::Piper(pd)) => {
            for pj in &pd.components {
                if pj.error_code != 0 {
                    faults.push(FaultState {
                        component_id: format!("body/arm/{}", pj.name),
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
        Some(ArmData::Koch(kd)) => {
            for kj in &kd.components {
                if kj.error_code != 0 {
                    faults.push(FaultState {
                        component_id: format!("body/arm/{}", kj.name),
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
        None => {}
    }

    // ── Safety state ───────────────────────────────────────────────────
    let (safety_aggregate, safety_detail) = match arm_data {
        Some(ArmData::Piper(pd)) => {
            let s = match pd.state {
                0 => SAFETY_NORMAL,
                2 => SAFETY_ESTOP,
                _ => SAFETY_FAULT,
            };
            (s, pd.message.clone())
        }
        Some(ArmData::Koch(kd)) => {
            let s = if kd.state == 0 {
                SAFETY_NORMAL
            } else {
                SAFETY_FAULT
            };
            (s, kd.message.clone())
        }
        None => (SAFETY_NORMAL, String::new()),
    };

    let motion_allowed = safety_aggregate == SAFETY_NORMAL;
    let motor_power_allowed = safety_aggregate == SAFETY_NORMAL;

    SomaHealthSnapshot {
        schema_version: SCHEMA_VERSION,
        body_id: body_id.to_string(),
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
) -> ActuatorState {
    ActuatorState {
        component_id: format!("body/arm/joint_{joint_idx}"),
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
    use crate::CMD_SHUTDOWN;
    use crate::pb::contracts::robonix_lifecycle_driver_client::RobonixLifecycleDriverClient;
    use crate::pb::lifecycle::DriverRequest;
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};

    fn reserve_address() -> SocketAddr {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("reserve port");
        listener.local_addr().expect("reserved address")
    }

    /// Exercise the actual mock-Soma startup path, then require Driver
    /// SHUTDOWN to publish TERMINATED and finish its tonic server task.
    #[tokio::test]
    async fn mock_soma_driver_shutdown_stops_server() {
        let atlas_addr = reserve_address();
        let registry = Arc::new(AtlasRegistry::default());
        let atlas_server = tokio::spawn(serve_atlas(Arc::clone(&registry), atlas_addr));
        let atlas_endpoint = format!("http://{atlas_addr}");
        let mut atlas =
            AtlasClient::connect_with_retry(&atlas_endpoint, 50, Duration::from_millis(10))
                .await
                .expect("connect test Atlas");
        let driver_addr = reserve_address();
        let provider_id = "mock-soma-driver-shutdown";
        let mock_atlas_endpoint = atlas_endpoint.clone();
        let mut mock_task = tokio::spawn(async move {
            run_mock_soma(
                &mock_atlas_endpoint,
                provider_id,
                &driver_addr.to_string(),
                "normal",
                10,
                MockArmConfig::Synthetic,
            )
            .await
        });

        let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
        loop {
            let active = atlas
                .query(
                    atlas_pb::Kind::Service,
                    provider_id,
                    "",
                    "",
                    atlas_pb::Transport::Unspecified,
                )
                .await
                .ok()
                .and_then(|providers| providers.into_iter().next())
                .is_some_and(|provider| {
                    provider.state == atlas_pb::LifecycleState::StateActive as i32
                });
            if active {
                break;
            }
            assert!(
                tokio::time::Instant::now() < deadline,
                "mock Soma did not become ACTIVE"
            );
            assert!(!mock_task.is_finished(), "mock Soma stopped before ACTIVE");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        let mut driver = RobonixLifecycleDriverClient::connect(format!("http://{driver_addr}"))
            .await
            .expect("connect mock Soma Driver");
        let shutdown = driver
            .driver(DriverRequest {
                command: CMD_SHUTDOWN,
                config_json: String::new(),
            })
            .await
            .expect("mock Soma Driver SHUTDOWN")
            .into_inner();
        assert!(shutdown.ok, "{}", shutdown.error);
        assert_eq!(shutdown.state, "terminated");
        tokio::time::timeout(Duration::from_secs(2), &mut mock_task)
            .await
            .expect("mock Soma server did not stop after SHUTDOWN")
            .expect("join mock Soma task")
            .expect("mock Soma task failed");

        let provider = atlas
            .query(
                atlas_pb::Kind::Service,
                provider_id,
                "",
                "",
                atlas_pb::Transport::Unspecified,
            )
            .await
            .expect("query mock Soma")
            .pop()
            .expect("mock Soma Provider");
        assert_eq!(
            provider.state,
            atlas_pb::LifecycleState::StateTerminated as i32
        );
        atlas_server.abort();
    }

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
    fn normal_snapshot_has_one_arm() {
        let snapshot = generate_snapshot(MockScenario::Normal, 1, None);
        // body + computer_jetson + cpu + gpu + arm + battery_main = 6
        // + 6 joints = 12
        assert_eq!(snapshot.components.len(), 12);
        assert_eq!(snapshot.actuators.len(), 6);
        assert_eq!(snapshot.faults.len(), 0);
        assert!(
            snapshot
                .actuators
                .iter()
                .any(|a| a.component_id == "body/arm/joint_1")
        );
    }

    #[test]
    fn fault_scenario_produces_fault_at_sequence_4() {
        let snapshot = generate_snapshot(MockScenario::Fault, 4, None);
        assert!(!snapshot.faults.is_empty());
        assert!(snapshot.faults.iter().any(|f| f.fault_id == "overcurrent"));
        let j3 = snapshot
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_3")
            .unwrap();
        assert!(!j3.communication_ok);
    }

    #[test]
    fn toggle_scenario_disables_joint_6_at_sequence_4() {
        let snapshot = generate_snapshot(MockScenario::Toggle, 4, None);
        let j6 = snapshot
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_6")
            .unwrap();
        assert!(!j6.torque_enabled);
    }

    /// Run every scenario through a few sequences and verify expected invariants.
    #[test]
    fn all_scenarios_produce_valid_snapshots() {
        for scenario in [
            MockScenario::Normal,
            MockScenario::Ramp,
            MockScenario::Fault,
            MockScenario::Toggle,
            MockScenario::Mixed,
        ] {
            for seq in 1..=12 {
                let s = generate_snapshot(scenario, seq, None);
                // Every snapshot must have 12 components and exactly 6 actuators.
                assert_eq!(
                    s.components.len(),
                    12,
                    "{} seq={}: expected 12 components, got {}",
                    scenario.as_str(),
                    seq,
                    s.components.len()
                );
                assert_eq!(
                    s.actuators.len(),
                    6,
                    "{} seq={}: expected 6 actuators, got {}",
                    scenario.as_str(),
                    seq,
                    s.actuators.len()
                );
                // Arm component must be present.
                assert!(
                    s.components.iter().any(|c| c.id == "body/arm"),
                    "{} seq={}: missing body/arm",
                    scenario.as_str(),
                    seq
                );
                // All 6 joints present.
                for j in 1..=6 {
                    assert!(
                        s.actuators
                            .iter()
                            .any(|a| a.component_id == format!("body/arm/joint_{j}")),
                        "{} seq={}: missing joint_{}",
                        scenario.as_str(),
                        seq,
                        j
                    );
                }
                // All snapshots must have a valid schema_version and body_id.
                assert!(!s.body_id.is_empty());
                assert!(s.schema_version > 0);
                assert!(s.power_sources.len() == 1);
                assert!(s.safety.is_some());
            }
        }
    }

    /// Normal: no faults at any sequence, all joints OK.
    #[test]
    fn normal_scenario_never_produces_faults() {
        for seq in 1..=20 {
            let s = generate_snapshot(MockScenario::Normal, seq, None);
            assert!(s.faults.is_empty(), "Normal seq={}: unexpected faults", seq);
            for a in &s.actuators {
                assert!(a.communication_ok, "Normal seq={}: comm not ok", seq);
                assert!(a.torque_enabled, "Normal seq={}: disabled", seq);
            }
            assert_eq!(s.safety.as_ref().unwrap().aggregate_state, SAFETY_NORMAL);
        }
    }

    /// Ramp: joint_1 temp ramps up, eventually exceeding WARN (60°C) and ERROR (75°C).
    #[test]
    fn ramp_scenario_joint_1_temp_increases() {
        let t1 = generate_snapshot(MockScenario::Ramp, 1, None)
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_1")
            .unwrap()
            .motor_temp
            .as_ref()
            .unwrap()
            .value;
        let t22 = generate_snapshot(MockScenario::Ramp, 22, None)
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_1")
            .unwrap()
            .motor_temp
            .as_ref()
            .unwrap()
            .value;
        let t30 = generate_snapshot(MockScenario::Ramp, 30, None)
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_1")
            .unwrap()
            .motor_temp
            .as_ref()
            .unwrap()
            .value;
        // joint_1 temp should increase from seq 1 to 22.
        assert!(t22 > t1, "ramp temp should rise: t1={t1}, t22={t22}");
        // By seq 30 (end of ramp cycle), temp should be near peak (~88°C).
        assert!(
            t30 > 75.0,
            "ramp seq=30 should exceed ERROR threshold: {t30}"
        );
        // Seq 31 restarts ramp (seq 1 is 40+1.6*0=40, seq 31 wraps same way).
        let t31 = generate_snapshot(MockScenario::Ramp, 31, None)
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_1")
            .unwrap()
            .motor_temp
            .as_ref()
            .unwrap()
            .value;
        assert!(t31 < 42.0, "ramp seq=31 should reset: {t31}");
    }

    /// Fault: joint_3 gets overcurrent fault and communication failure on seq 4..7, 12..15, etc.
    #[test]
    fn fault_scenario_joint_3_toggles_periodically() {
        for seq in 1..=20 {
            let s = generate_snapshot(MockScenario::Fault, seq, None);
            let j3 = s
                .actuators
                .iter()
                .find(|a| a.component_id == "body/arm/joint_3")
                .unwrap();
            let fault_expected = seq % 8 >= 4;
            assert_eq!(
                !j3.communication_ok, fault_expected,
                "Fault seq={}: j3 comm_ok should be {}",
                seq, !fault_expected
            );
            if fault_expected {
                assert!(
                    s.faults.iter().any(|f| f.fault_id == "overcurrent"),
                    "Fault seq={}: missing overcurrent fault",
                    seq
                );
            }
        }
    }

    /// Toggle: joint_6 torque_enabled toggles off on seq 4..7, 12..15, etc.
    #[test]
    fn toggle_scenario_joint_6_toggles_periodically() {
        for seq in 1..=20 {
            let s = generate_snapshot(MockScenario::Toggle, seq, None);
            let j6 = s
                .actuators
                .iter()
                .find(|a| a.component_id == "body/arm/joint_6")
                .unwrap();
            let disabled_expected = seq % 8 >= 4;
            assert_eq!(
                !j6.torque_enabled, disabled_expected,
                "Toggle seq={}: j6 enabled should be {}",
                seq, !disabled_expected
            );
        }
    }

    /// Mixed: ramp + fault + toggle simultaneously.
    #[test]
    fn mixed_scenario_combines_all_behaviors() {
        // Seq 4: fault active + toggle disabled + ramp building up.
        let s4 = generate_snapshot(MockScenario::Mixed, 4, None);
        assert!(s4.faults.iter().any(|f| f.fault_id == "overcurrent"));
        let j3 = s4
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_3")
            .unwrap();
        assert!(!j3.communication_ok);
        let j6 = s4
            .actuators
            .iter()
            .find(|a| a.component_id == "body/arm/joint_6")
            .unwrap();
        assert!(!j6.torque_enabled);

        // Seq 2: no faults, no toggle, ramp at early stage.
        let s2 = generate_snapshot(MockScenario::Mixed, 2, None);
        assert!(s2.faults.is_empty());
    }
}
