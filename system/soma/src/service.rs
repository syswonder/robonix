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
    GetFootprintRequest, GetFootprintResponse, GetHealthRequest, GetHealthResponse, GetUrdfRequest,
    GetUrdfResponse, GetYamlRequest, GetYamlResponse, SomaHealthSnapshot, StreamHealthRequest,
};
use crate::store::{SomaBody, StoreError};
use std::sync::Arc;
use tokio::sync::{RwLock, broadcast};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

#[derive(Debug)]
pub struct SomaService {
    body: Arc<SomaBody>,
    latest_snapshot: Arc<RwLock<Option<SomaHealthSnapshot>>>,
    snapshot_tx: broadcast::Sender<SomaHealthSnapshot>,
}

impl SomaService {
    pub fn new(body: Arc<SomaBody>) -> Self {
        let (snapshot_tx, _) = broadcast::channel(8);
        Self {
            body,
            latest_snapshot: Arc::new(RwLock::new(None)),
            snapshot_tx,
        }
    }

    pub fn snapshot_sender(&self) -> broadcast::Sender<SomaHealthSnapshot> {
        self.snapshot_tx.clone()
    }

    pub async fn update_snapshot(&self, snapshot: SomaHealthSnapshot) {
        *self.latest_snapshot.write().await = Some(snapshot);
    }

    fn map_lookup_error(error: StoreError) -> Status {
        match error {
            StoreError::NotFound(_) => Status::not_found(error.to_string()),
            StoreError::MissingFootprint(_) => Status::failed_precondition(error.to_string()),
        }
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaGetYaml for SomaService {
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
    async fn get_urdf(
        &self,
        request: Request<GetUrdfRequest>,
    ) -> Result<Response<GetUrdfResponse>, Status> {
        let req = request.into_inner();
        let body = self
            .body
            .resolve(&req.robot_id)
            .map_err(Self::map_lookup_error)?;
        Ok(Response::new(GetUrdfResponse {
            robot_id: body.robot_id.clone(),
            urdf_xml: body.urdf_xml.clone(),
        }))
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaGetHealth for SomaService {
    async fn get_health(
        &self,
        _request: Request<GetHealthRequest>,
    ) -> Result<Response<GetHealthResponse>, Status> {
        let snapshot = self.latest_snapshot.read().await.clone();
        Ok(Response::new(GetHealthResponse { snapshot }))
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaHealth for SomaService {
    type StreamHealthStream = ReceiverStream<Result<SomaHealthSnapshot, Status>>;

    async fn stream_health(
        &self,
        _request: Request<StreamHealthRequest>,
    ) -> Result<Response<Self::StreamHealthStream>, Status> {
        let mut rx = self.snapshot_tx.subscribe();
        let latest = Arc::clone(&self.latest_snapshot);
        let (tx, out_rx) = tokio::sync::mpsc::channel(16);

        tokio::spawn(async move {
            if let Some(snapshot) = latest.read().await.clone()
                && tx.send(Ok(snapshot)).await.is_err()
            {
                return;
            }
            loop {
                match rx.recv().await {
                    Ok(snapshot) => {
                        if tx.send(Ok(snapshot)).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(count)) => {
                        robonix_scribe::warn!("[soma] health broadcast lagged by {count} frames");
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
        });

        Ok(Response::new(ReceiverStream::new(out_rx)))
    }
}

#[tonic::async_trait]
impl RobonixSystemSomaFootprint for SomaService {
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
            }))
            .await
            .expect("get urdf")
            .into_inner();
        assert_eq!(response.robot_id, "test_ci_robot");
        assert!(response.urdf_xml.contains("<robot name=\"test_ci_robot\">"));
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
