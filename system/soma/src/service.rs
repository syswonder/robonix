// SPDX-License-Identifier: MulanPSL-2.0

use crate::pb::contracts::{
    robonix_system_soma_footprint_server::RobonixSystemSomaFootprint,
    robonix_system_soma_get_health_server::RobonixSystemSomaGetHealth,
    robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdf,
    robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYaml,
    robonix_system_soma_health_server::RobonixSystemSomaHealth,
};
use crate::pb::geometry_msgs::Point;
use crate::pb::soma::{
    ActuatorState, ComponentStatus, GetFootprintRequest, GetFootprintResponse, GetHealthRequest,
    GetHealthResponse, GetUrdfRequest, GetUrdfResponse, GetYamlRequest, GetYamlResponse, Metric,
    Scalar, SomaHealthSnapshot, StreamHealthRequest, UrdfAsset,
};
use crate::runtime_state::RuntimeStateStore;
use crate::store::{SomaBody, StoreError};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tokio::sync::{RwLock, broadcast};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

#[derive(Debug)]
pub struct SomaService {
    body: Arc<SomaBody>,
    runtime: RuntimeStateStore,
    snapshot_state: Arc<RwLock<SnapshotState>>,
    snapshot_tx: broadcast::Sender<SomaHealthSnapshot>,
    next_seq: AtomicU64,
}

#[derive(Debug, Default)]
struct SnapshotState {
    latest: Option<SomaHealthSnapshot>,
    primitive_valid_until: Option<Instant>,
}

impl SomaService {
    /// Initialize runtime fallback state and the broadcast channel for health clients.
    pub fn new(body: Arc<SomaBody>) -> Self {
        let runtime = RuntimeStateStore::new(body.grippers.clone());
        let (snapshot_tx, _) = broadcast::channel(16);
        Self {
            body,
            runtime,
            snapshot_state: Arc::default(),
            snapshot_tx,
            next_seq: AtomicU64::new(1),
        }
    }

    pub fn runtime(&self) -> RuntimeStateStore {
        self.runtime.clone()
    }

    /// Publish the ROS-derived fallback unless a health primitive lease is active.
    pub async fn publish_runtime_snapshot(&self) {
        let mut snapshot = self.to_health_snapshot(0).await;
        let mut state = self.snapshot_state.write().await;
        if state
            .primitive_valid_until
            .is_some_and(|deadline| deadline > Instant::now())
        {
            return;
        }
        snapshot.seq = self.next_sequence();
        state.latest = Some(snapshot.clone());
        state.primitive_valid_until = None;
        drop(state);
        let _ = self.snapshot_tx.send(snapshot);
    }

    /// Publish a health-primitive snapshot and suppress fallback until its TTL expires.
    pub async fn publish_primitive_snapshot(&self, mut snapshot: SomaHealthSnapshot) {
        let ttl = Duration::from_millis(u64::from(snapshot.ttl_ms.max(1)));
        snapshot.seq = self.next_sequence();
        snapshot.soma_ts_ns = unix_time_ns();
        let mut state = self.snapshot_state.write().await;
        state.latest = Some(snapshot.clone());
        state.primitive_valid_until = Some(Instant::now() + ttl);
        drop(state);
        let _ = self.snapshot_tx.send(snapshot);
    }

    fn next_sequence(&self) -> u64 {
        self.next_seq.fetch_add(1, Ordering::Relaxed)
    }

    fn map_lookup_error(error: StoreError) -> Status {
        match error {
            StoreError::NotFound(_) => Status::not_found(error.to_string()),
            StoreError::MissingFootprint(_) => Status::failed_precondition(error.to_string()),
        }
    }

    /// Project the latest ROS runtime samples into the Soma health wire model.
    async fn to_health_snapshot(&self, seq: u64) -> SomaHealthSnapshot {
        const HEALTH_OK: u32 = 0;
        const HEALTH_STALE: u32 = 3;
        const KIND_BODY: u32 = 1;
        const KIND_ARM: u32 = 2;
        const KIND_JOINT: u32 = 4;
        const KIND_GRIPPER: u32 = 6;
        const KIND_WHEEL: u32 = 5;
        const OP_IDLE: u32 = 3;
        const OP_ACTIVE: u32 = 4;
        const QUALITY_VALID: u32 = 0;
        const QUALITY_STALE: u32 = 1;
        let runtime = self.runtime.snapshot().await;
        let runtime_detail = runtime.warnings.join("; ");
        let mut components = vec![component(
            "body",
            &runtime_detail,
            KIND_BODY,
            &self.body.robot_id,
            HEALTH_OK,
            OP_ACTIVE,
            "",
        )];
        let mut actuators = Vec::new();
        let mut metrics = Vec::new();
        for arm in runtime.arms {
            let arm_id = format!("body/arm/{}", arm.provider_id);
            let quality = if arm.fresh {
                QUALITY_VALID
            } else {
                QUALITY_STALE
            };
            components.push(component(
                &arm_id,
                "body",
                KIND_ARM,
                &arm.provider_id,
                if arm.fresh { HEALTH_OK } else { HEALTH_STALE },
                OP_ACTIVE,
                &format!("age_sec={:.3}", arm.age_sec),
            ));
            for (index, name) in arm.names.iter().enumerate() {
                let joint_id = format!("{arm_id}/{name}");
                let gripper = arm
                    .grippers
                    .iter()
                    .find(|item| item.config.joint_name == *name);
                components.push(component(
                    &joint_id,
                    &arm_id,
                    if gripper.is_some() {
                        KIND_GRIPPER
                    } else {
                        KIND_JOINT
                    },
                    name,
                    if arm.fresh { HEALTH_OK } else { HEALTH_STALE },
                    if gripper.is_some_and(|item| item.likely_holding) {
                        OP_ACTIVE
                    } else {
                        OP_IDLE
                    },
                    gripper.map(|item| item.state.as_str()).unwrap_or(""),
                ));
                actuators.push(ActuatorState {
                    component_id: joint_id.clone(),
                    joint_name: name.clone(),
                    position: Some(scalar(
                        arm.positions.get(index).copied().unwrap_or_default(),
                        if gripper.is_some() { "m" } else { "rad" },
                        quality,
                    )),
                    velocity: None,
                    effort: None,
                    current: None,
                    voltage: None,
                    motor_temp: None,
                    driver_temp: None,
                    torque_enabled: true,
                    brake_engaged: false,
                    communication_ok: arm.fresh,
                    vendor_mode: 0,
                    vendor_error_code: 0,
                    status_flags: 0,
                });
                if let Some(gripper) = gripper {
                    metrics.push(metric(
                        &joint_id,
                        "likely_holding",
                        if gripper.likely_holding { 1.0 } else { 0.0 },
                        "bool",
                        if gripper.fresh {
                            QUALITY_VALID
                        } else {
                            QUALITY_STALE
                        },
                    ));
                }
            }
        }
        for chassis in runtime.chassis {
            let id = format!("body/chassis/{}", chassis.sample.provider_id);
            let quality = if chassis.fresh {
                QUALITY_VALID
            } else {
                QUALITY_STALE
            };
            components.push(component(
                &id,
                "body",
                KIND_WHEEL,
                &chassis.sample.provider_id,
                if chassis.fresh {
                    HEALTH_OK
                } else {
                    HEALTH_STALE
                },
                if chassis.moving { OP_ACTIVE } else { OP_IDLE },
                &format!("age_sec={:.3}", chassis.age_sec),
            ));
            let linear_speed = chassis
                .sample
                .linear
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            let angular_speed = chassis
                .sample
                .angular
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            metrics.extend([
                metric(&id, "linear_speed", linear_speed, "m/s", quality),
                metric(&id, "angular_speed", angular_speed, "rad/s", quality),
                metric(
                    &id,
                    "moving",
                    if chassis.moving { 1.0 } else { 0.0 },
                    "bool",
                    quality,
                ),
            ]);
        }
        let timestamp_ns = (runtime.observed_at_unix * 1_000_000_000.0) as i64;
        SomaHealthSnapshot {
            schema_version: 1,
            body_id: self.body.robot_id.clone(),
            seq,
            source_ts_ns: timestamp_ns,
            soma_ts_ns: timestamp_ns,
            ttl_ms: 2_000,
            components,
            actuators,
            power_sources: Vec::new(),
            // joint_states and odometry do not prove that motion is safe.
            // A health primitive may populate this once it has real e-stop and
            // protective-stop inputs; until then the safety state is unknown.
            safety: None,
            safety_endpoints: Vec::new(),
            faults: Vec::new(),
            metrics,
        }
    }
}

fn scalar(value: f64, unit: &str, quality: u32) -> Scalar {
    Scalar {
        value,
        unit: unit.into(),
        quality,
    }
}

fn metric(component_id: &str, name: &str, value: f64, unit: &str, quality: u32) -> Metric {
    Metric {
        component_id: component_id.into(),
        name: name.into(),
        value: Some(scalar(value, unit, quality)),
        source_key: "soma_runtime_state".into(),
    }
}

/// Build one runtime-derived component with explicit online and health semantics.
fn component(
    id: &str,
    parent_id: &str,
    kind: u32,
    name: &str,
    health: u32,
    operational_state: u32,
    detail: &str,
) -> ComponentStatus {
    ComponentStatus {
        id: id.into(),
        parent_id: parent_id.into(),
        kind,
        name: name.into(),
        frame_id: String::new(),
        model: String::new(),
        serial: String::new(),
        health,
        operational_state,
        present: true,
        online: health != 3,
        detail: detail.into(),
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaGetHealth for SomaService {
    async fn get_health(
        &self,
        _request: Request<GetHealthRequest>,
    ) -> Result<Response<GetHealthResponse>, Status> {
        Ok(Response::new(GetHealthResponse {
            snapshot: self.snapshot_state.read().await.latest.clone(),
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaHealth for SomaService {
    type StreamHealthStream = ReceiverStream<Result<SomaHealthSnapshot, Status>>;

    /// Send the latest snapshot first, then forward updates until the client disconnects.
    async fn stream_health(
        &self,
        _request: Request<StreamHealthRequest>,
    ) -> Result<Response<Self::StreamHealthStream>, Status> {
        let mut input = self.snapshot_tx.subscribe();
        let state = Arc::clone(&self.snapshot_state);
        let (output, receiver) = tokio::sync::mpsc::channel(16);
        tokio::spawn(async move {
            if let Some(snapshot) = state.read().await.latest.clone()
                && output.send(Ok(snapshot)).await.is_err()
            {
                return;
            }
            loop {
                match input.recv().await {
                    Ok(snapshot) => {
                        if output.send(Ok(snapshot)).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
        });
        Ok(Response::new(ReceiverStream::new(receiver)))
    }
}

/// Return the current Unix timestamp in nanoseconds, saturating at the wire limit.
fn unix_time_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .min(i64::MAX as u128) as i64
}

#[tonic::async_trait]
impl RobonixSystemSomaGetYaml for SomaService {
    /// Resolve the requested robot and return its original Soma YAML document.
    async fn get_yaml(
        &self,
        request: Request<GetYamlRequest>,
    ) -> Result<Response<GetYamlResponse>, Status> {
        let req = request.into_inner();
        let body = self
            .body
            .resolve(&req.robot_id)
            .map_err(Self::map_lookup_error)?;
        Ok(Response::new(GetYamlResponse {
            robot_id: body.robot_id.clone(),
            yaml_text: body.yaml_text.clone(),
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaGetUrdf for SomaService {
    /// Return URDF XML and materialize validated local assets only when requested.
    async fn get_urdf(
        &self,
        request: Request<GetUrdfRequest>,
    ) -> Result<Response<GetUrdfResponse>, Status> {
        let req = request.into_inner();
        let body = self
            .body
            .resolve(&req.robot_id)
            .map_err(Self::map_lookup_error)?;
        let assets = if req.include_assets {
            let body = Arc::clone(&self.body);
            tokio::task::spawn_blocking(move || body.read_urdf_assets())
                .await
                .map_err(|error| Status::internal(format!("join URDF asset reader: {error}")))?
                .map_err(|error| Status::failed_precondition(error.to_string()))?
                .into_iter()
                .map(|asset| UrdfAsset {
                    path: asset.path,
                    data: asset.data,
                })
                .collect()
        } else {
            Vec::new()
        };
        Ok(Response::new(GetUrdfResponse {
            robot_id: body.robot_id.clone(),
            urdf_xml: body.urdf_xml.clone(),
            assets,
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaFootprint for SomaService {
    /// Return the configured footprint while preserving its declared base frame.
    async fn get_footprint(
        &self,
        _request: Request<GetFootprintRequest>,
    ) -> Result<Response<GetFootprintResponse>, Status> {
        let footprint = self.body.footprint().map_err(Self::map_lookup_error)?;
        Ok(Response::new(GetFootprintResponse {
            points: footprint
                .points
                .iter()
                .map(|point| Point {
                    x: point.x,
                    y: point.y,
                    z: 0.0,
                })
                .collect(),
            base_frame: footprint.base_frame.clone(),
            inscribed_radius_m: footprint.inscribed_radius_m,
            circumscribed_radius_m: footprint.circumscribed_radius_m,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::contracts::{
        robonix_system_soma_footprint_client::RobonixSystemSomaFootprintClient,
        robonix_system_soma_footprint_server::RobonixSystemSomaFootprintServer,
        robonix_system_soma_get_urdf_client::RobonixSystemSomaGetUrdfClient,
        robonix_system_soma_get_urdf_server::RobonixSystemSomaGetUrdfServer,
        robonix_system_soma_get_yaml_client::RobonixSystemSomaGetYamlClient,
        robonix_system_soma_get_yaml_server::RobonixSystemSomaGetYamlServer,
    };
    use tokio::net::TcpListener;
    use tokio_stream::wrappers::TcpListenerStream;

    fn fixture_body() -> Arc<SomaBody> {
        let yaml_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("examples/test_ci/soma.yaml");
        Arc::new(SomaBody::load(&yaml_path).expect("load fixture body"))
    }

    #[tokio::test]
    async fn get_yaml_returns_raw_text() {
        let service = SomaService::new(fixture_body());
        let response = service
            .get_yaml(Request::new(GetYamlRequest {
                robot_id: "test_ci_robot".into(),
            }))
            .await
            .expect("get yaml")
            .into_inner();
        assert_eq!(response.robot_id, "test_ci_robot");
        assert!(response.yaml_text.contains("robot:"));
    }

    #[tokio::test]
    async fn get_urdf_returns_xml_text() {
        let service = SomaService::new(fixture_body());
        let response = service
            .get_urdf(Request::new(GetUrdfRequest {
                robot_id: "".into(),
                include_assets: false,
            }))
            .await
            .expect("get urdf")
            .into_inner();
        assert_eq!(response.robot_id, "test_ci_robot");
        assert!(response.urdf_xml.contains("<robot name=\"test_ci_robot\">"));
        assert!(response.assets.is_empty());
    }

    #[tokio::test]
    async fn get_footprint_returns_the_declared_polygon() {
        let service = SomaService::new(fixture_body());
        let response = service
            .get_footprint(Request::new(GetFootprintRequest {}))
            .await
            .expect("get footprint")
            .into_inner();
        assert_eq!(response.base_frame, "base_link");
        assert_eq!(response.points.len(), 4);
        assert_eq!(response.points[0].x, 0.2);
        assert_eq!(response.points[0].y, 0.1);
        assert!((response.inscribed_radius_m - 0.1).abs() < 1e-9);
    }

    #[tokio::test]
    async fn health_snapshot_is_explicit_when_no_samples_exist() {
        let service = SomaService::new(fixture_body());
        service.publish_runtime_snapshot().await;
        let response = service
            .get_health(Request::new(GetHealthRequest {}))
            .await
            .expect("get health")
            .into_inner();
        let snapshot = response.snapshot.expect("snapshot");
        assert_eq!(snapshot.body_id, "test_ci_robot");
        assert_eq!(snapshot.seq, 1);
    }

    /// Primitive data suppresses fallback only for the advertised lease.
    #[tokio::test]
    async fn primitive_snapshot_wins_until_its_ttl_expires() {
        let service = SomaService::new(fixture_body());
        let mut primitive = service.to_health_snapshot(0).await;
        primitive.ttl_ms = 20;
        primitive.components[0].detail = "primitive".into();
        service.publish_primitive_snapshot(primitive).await;
        service.publish_runtime_snapshot().await;

        let active = service
            .get_health(Request::new(GetHealthRequest {}))
            .await
            .expect("get primitive health")
            .into_inner()
            .snapshot
            .expect("primitive snapshot");
        assert_eq!(active.seq, 1);
        assert_eq!(active.components[0].detail, "primitive");

        tokio::time::sleep(Duration::from_millis(25)).await;
        service.publish_runtime_snapshot().await;
        let fallback = service
            .get_health(Request::new(GetHealthRequest {}))
            .await
            .expect("get fallback health")
            .into_inner()
            .snapshot
            .expect("fallback snapshot");
        assert_eq!(fallback.seq, 2);
        assert_ne!(fallback.components[0].detail, "primitive");
    }

    #[tokio::test]
    async fn unknown_robot_maps_to_not_found() {
        let service = SomaService::new(fixture_body());
        let status = service
            .get_yaml(Request::new(GetYamlRequest {
                robot_id: "missing".into(),
            }))
            .await
            .expect_err("missing robot should fail");
        assert_eq!(status.code(), tonic::Code::NotFound);
    }

    #[tokio::test]
    async fn grpc_clients_call_yaml_and_urdf_services() {
        let listener = TcpListener::bind("127.0.0.1:0").await.expect("bind");
        let addr = listener.local_addr().expect("local addr");
        let service = Arc::new(SomaService::new(fixture_body()));
        tokio::spawn(async move {
            tonic::transport::Server::builder()
                .add_service(RobonixSystemSomaGetYamlServer::from_arc(Arc::clone(
                    &service,
                )))
                .add_service(RobonixSystemSomaGetUrdfServer::from_arc(Arc::clone(
                    &service,
                )))
                .add_service(RobonixSystemSomaFootprintServer::from_arc(service))
                .serve_with_incoming(TcpListenerStream::new(listener))
                .await
                .expect("serve");
        });

        let endpoint = format!("http://{addr}");
        let mut yaml_client = RobonixSystemSomaGetYamlClient::connect(endpoint.clone())
            .await
            .expect("connect yaml");
        let mut urdf_client = RobonixSystemSomaGetUrdfClient::connect(endpoint)
            .await
            .expect("connect urdf");
        let mut footprint_client =
            RobonixSystemSomaFootprintClient::connect(format!("http://{addr}"))
                .await
                .expect("connect footprint");

        let yaml = yaml_client
            .get_yaml(GetYamlRequest {
                robot_id: "test_ci_robot".into(),
            })
            .await
            .expect("get yaml")
            .into_inner();
        let urdf = urdf_client
            .get_urdf(GetUrdfRequest {
                robot_id: "test_ci_robot".into(),
                include_assets: false,
            })
            .await
            .expect("get urdf")
            .into_inner();
        let footprint = footprint_client
            .get_footprint(GetFootprintRequest {})
            .await
            .expect("get footprint")
            .into_inner();

        assert!(yaml.yaml_text.contains("Soma v2 test fixture robot"));
        assert!(urdf.urdf_xml.contains("<link name=\"base_link\"/>"));
        assert_eq!(footprint.points.len(), 4);
    }
}
