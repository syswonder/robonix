// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

use std::process::{Child, Command};
use std::sync::Arc;
use std::time::Duration;

use robonix_atlas::{
    client::AtlasClient,
    pb as atlas_pb,
    service::{AtlasRegistry, serve_atlas},
};
use tokio::time::timeout;

#[allow(dead_code, clippy::enum_variant_names)]
mod pb {
    include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));
}

use pb::contracts::{
    robonix_lifecycle_driver_client::RobonixLifecycleDriverClient,
    robonix_system_chronos_conversion_models_client::RobonixSystemChronosConversionModelsClient,
    robonix_system_chronos_quality_client::RobonixSystemChronosQualityClient,
};

/// Kill and reap the child if any assertion fails before normal shutdown.
struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[tokio::test]
/// A separate daemon process is discovered through Atlas and serves both real streams.
async fn daemon_process_registers_streams_and_shuts_down() {
    let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let atlas_addr = reserved.local_addr().unwrap();
    drop(reserved);
    let server = tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
    let mut atlas =
        AtlasClient::connect_with_retry(atlas_addr.to_string(), 50, Duration::from_millis(10))
            .await
            .unwrap();
    atlas
        .register_service("chronos-smoke-consumer", "robonix/service/test", "")
        .await
        .unwrap();
    let mut child = ChildGuard(Command::new(env!("CARGO_BIN_EXE_robonix-chronos"))
        .args(["--atlas", &atlas_addr.to_string(), "--listen", "127.0.0.1:0",
            "--provider-id", "chronos-process-test", "--config-json",
            r#"{"core":{"reference":{"kind":"linux_system"},"quality":{"refresh_interval_ns":10000000}}}"#])
        .spawn().unwrap());
    let provider = timeout(Duration::from_secs(10), async {
        loop {
            assert!(
                child.0.try_wait().unwrap().is_none(),
                "daemon exited before readiness"
            );
            let providers = atlas
                .query_services("chronos-process-test", "", "", atlas_pb::Transport::Grpc)
                .await
                .unwrap();
            if let Some(provider) = providers
                .into_iter()
                .find(|p| p.state == atlas_pb::LifecycleState::StateActive as i32)
            {
                break provider;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    assert_eq!(provider.capabilities.len(), 5);
    let (edge, endpoint, params) = atlas
        .connect_capability(
            "chronos-smoke-consumer",
            &provider.id,
            "robonix/system/chronos/quality",
            atlas_pb::Transport::Grpc,
        )
        .await
        .unwrap();
    let Some(atlas_pb::transport_params::Kind::Grpc(params)) = params.kind else {
        panic!("Chronos must advertise gRPC transport parameters");
    };
    assert_eq!(
        params.proto_file,
        "capabilities/system/chronos/quality.v1.toml"
    );
    assert_eq!(
        params.method,
        "/robonix.contracts.RobonixSystemChronosQuality/Quality"
    );
    let address = format!("http://{endpoint}");
    let mut quality_client = RobonixSystemChronosQualityClient::connect(address.clone())
        .await
        .unwrap();
    let mut quality = quality_client.quality(()).await.unwrap().into_inner();
    let first = timeout(Duration::from_secs(2), quality.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert!(first.report_available);
    assert!(!first.reference.unwrap().verified);
    let next = timeout(Duration::from_secs(2), quality.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert!(next.observed_steady_ns >= first.observed_steady_ns);
    drop(quality);
    atlas.disconnect_capability(&edge).await.unwrap();
    let mut model_client = RobonixSystemChronosConversionModelsClient::connect(address.clone())
        .await
        .unwrap();
    let mut models = model_client
        .conversion_models(())
        .await
        .unwrap()
        .into_inner();
    let snapshot = timeout(Duration::from_secs(2), models.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert!(snapshot.published_steady_ns > 0);
    assert!(snapshot.domains.is_empty());
    drop(models);
    let mut driver = RobonixLifecycleDriverClient::connect(address)
        .await
        .unwrap();
    let reply = driver
        .driver(pb::lifecycle::DriverRequest {
            command: 3,
            config_json: String::new(),
        })
        .await
        .unwrap()
        .into_inner();
    assert!(reply.ok, "{}", reply.error);
    timeout(Duration::from_secs(5), async {
        loop {
            if let Some(status) = child.0.try_wait().unwrap() {
                assert!(status.success());
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    server.abort();
}

#[tokio::test]
/// An independent daemon fits real gRPC observations, publishes models, and survives input loss/restarts.
async fn observations_fit_publish_reset_and_recover() {
    use pb::chronos::ObserveDeviceTimestampRequest;
    use pb::contracts::robonix_system_chronos_observe_device_timestamp_client::RobonixSystemChronosObserveDeviceTimestampClient;
    let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let atlas_addr = reserved.local_addr().unwrap();
    drop(reserved);
    let server = tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
    let mut atlas =
        AtlasClient::connect_with_retry(atlas_addr.to_string(), 50, Duration::from_millis(10))
            .await
            .unwrap();
    atlas
        .register_service("observation-consumer", "robonix/service/test", "")
        .await
        .unwrap();
    let config = serde_json::json!({"core": {
        "reference": {"kind": "linux_system"}, "quality": {"refresh_interval_ns": 10000000},
        "domains": {"device": {
            "capabilities": [{"provider_id": "camera", "contract_id": "robonix/primitive/camera/rgb", "transport": "ros2", "timestamp_field": "header.stamp"}],
            "timestamp": {"event": "acquisition", "hardware": false, "time_scale": "device_uptime", "resolution_ns": 1, "uncertainty_ns": 1000000},
            "strategy": {"kind": "affine_model", "observation": {"reference_at": "host_receive_time", "sampling_interval_ns": 1000000, "uncertainty_ns": 20000000},
                "policy": {"minimum_observations": 3, "observation_window": 16, "max_drift_ppm": 100000, "max_residual_ns": 20000000,
                    "tracking_timeout_ns": 200000000, "holdover_duration_ns": 300000000, "max_uncertainty_ns": 100000000}}}}}}).to_string();
    let mut child = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_robonix-chronos"))
            .args([
                "--atlas",
                &atlas_addr.to_string(),
                "--listen",
                "127.0.0.1:0",
                "--provider-id",
                "chronos-observations",
                "--config-json",
                &config,
            ])
            .spawn()
            .unwrap(),
    );
    let (edge, endpoint, params) = timeout(Duration::from_secs(10), async {
        loop {
            assert!(child.0.try_wait().unwrap().is_none());
            let providers = atlas
                .query_services("chronos-observations", "", "", atlas_pb::Transport::Grpc)
                .await
                .unwrap();
            if providers
                .iter()
                .any(|p| p.state == atlas_pb::LifecycleState::StateActive as i32)
            {
                break atlas
                    .connect_capability(
                        "observation-consumer",
                        "chronos-observations",
                        "robonix/system/chronos/observe_device_timestamp",
                        atlas_pb::Transport::Grpc,
                    )
                    .await
                    .unwrap();
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    let Some(atlas_pb::transport_params::Kind::Grpc(params)) = params.kind else {
        panic!("expected gRPC")
    };
    assert_eq!(
        params.proto_file,
        "capabilities/system/chronos/observe_device_timestamp.v1.toml"
    );
    let endpoint = format!("http://{endpoint}");
    let mut input = RobonixSystemChronosObserveDeviceTimestampClient::connect(endpoint.clone())
        .await
        .unwrap();
    let mut quality_client = RobonixSystemChronosQualityClient::connect(endpoint.clone())
        .await
        .unwrap();
    let mut models_client = RobonixSystemChronosConversionModelsClient::connect(endpoint.clone())
        .await
        .unwrap();
    let mut quality = quality_client.quality(()).await.unwrap().into_inner();
    let mut models = models_client
        .conversion_models(())
        .await
        .unwrap()
        .into_inner();
    let origin = std::time::Instant::now();
    let request = |session: &str, generation, sequence| ObserveDeviceTimestampRequest {
        domain_id: "device".into(),
        session_id: session.into(),
        device_generation: generation,
        sequence_id: sequence,
        device_timestamp_ns: origin.elapsed().as_nanos() as i64 + 5000000000,
    };
    for sequence in 1..=12 {
        let reply = submit(&mut input, request("a", 1, sequence))
            .await
            .unwrap()
            .into_inner();
        assert!(reply.accepted, "{}", reply.reason);
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    let fitted = timeout(Duration::from_secs(2), async {
        loop {
            let snapshot = models.message().await.unwrap().unwrap();
            if snapshot.domains[0].has_model && snapshot.domains[0].state == 1 {
                break snapshot;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(fitted.domains[0].generation, 1);
    assert_eq!(fitted.domains[0].model.as_ref().unwrap().generation, 1);
    assert_eq!(
        submit(&mut input, request("a", 1, 12))
            .await
            .unwrap_err()
            .code(),
        tonic::Code::FailedPrecondition
    );
    let mut invalid = request("a", 1, 13);
    invalid.domain_id = "missing".into();
    assert_eq!(
        submit(&mut input, invalid).await.unwrap_err().code(),
        tonic::Code::InvalidArgument
    );
    let stale_at = timeout(Duration::from_secs(2), async {
        let mut holdover = false;
        loop {
            let snapshot = quality.message().await.unwrap().unwrap();
            assert!(!snapshot.reference.unwrap().verified);
            holdover |= snapshot.affine_domains[0].state == 2;
            if snapshot.affine_domains[0].state == 4 {
                assert!(holdover);
                break snapshot.observed_steady_ns;
            }
        }
    })
    .await
    .unwrap();
    for sequence in 13..=24 {
        assert!(
            submit(&mut input, request("a", 1, sequence))
                .await
                .unwrap()
                .into_inner()
                .accepted
        );
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    timeout(Duration::from_secs(2), async {
        loop {
            let snapshot = models.message().await.unwrap().unwrap();
            if snapshot.domains[0].generation == 1
                && snapshot.domains[0].state == 1
                && snapshot.domains[0].has_model
                && snapshot.domains[0]
                    .model
                    .as_ref()
                    .unwrap()
                    .last_observation_steady_ns
                    > stale_at
            {
                break;
            }
        }
    })
    .await
    .unwrap();
    let reset = submit(&mut input, request("a", 2, 1))
        .await
        .unwrap()
        .into_inner();
    assert!(!reset.accepted && reset.reason.contains("identity changed"));
    assert_eq!(
        submit(&mut input, request("a", 1, 14))
            .await
            .unwrap_err()
            .code(),
        tonic::Code::FailedPrecondition
    );
    for sequence in 2..=13 {
        assert!(
            submit(&mut input, request("a", 2, sequence))
                .await
                .unwrap()
                .into_inner()
                .accepted
        );
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    timeout(Duration::from_secs(2), async {
        loop {
            let snapshot = models.message().await.unwrap().unwrap();
            let domain = &snapshot.domains[0];
            if domain.generation == 2
                && domain.state == 1
                && domain.has_model
                && domain.model.as_ref().unwrap().generation == 2
            {
                break;
            }
        }
    })
    .await
    .unwrap();
    assert!(
        !submit(&mut input, request("b", 1, 1))
            .await
            .unwrap()
            .into_inner()
            .accepted
    );
    assert_eq!(
        submit(&mut input, request("a", 3, 20))
            .await
            .unwrap_err()
            .code(),
        tonic::Code::FailedPrecondition
    );
    drop(models);
    drop(quality);
    let mut driver = RobonixLifecycleDriverClient::connect(endpoint)
        .await
        .unwrap();
    for command in [2, 1] {
        assert!(
            driver
                .driver(pb::lifecycle::DriverRequest {
                    command,
                    config_json: "{}".into()
                })
                .await
                .unwrap()
                .into_inner()
                .ok
        );
        if command == 2 {
            assert_eq!(
                submit(&mut input, request("b", 1, 2))
                    .await
                    .unwrap_err()
                    .code(),
                tonic::Code::Unavailable
            );
        }
    }
    assert!(
        submit(&mut input, request("b", 1, 2))
            .await
            .unwrap()
            .into_inner()
            .accepted
    );
    atlas.disconnect_capability(&edge).await.unwrap();
    assert!(
        driver
            .driver(pb::lifecycle::DriverRequest {
                command: 3,
                config_json: String::new()
            })
            .await
            .unwrap()
            .into_inner()
            .ok
    );
    timeout(Duration::from_secs(5), async {
        loop {
            if let Some(status) = child.0.try_wait().unwrap() {
                assert!(status.success());
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    server.abort();
}

/// Busy ingress refuses delayed pairing; generate fresh simulated device time before retrying.
async fn submit(
    client: &mut pb::contracts::robonix_system_chronos_observe_device_timestamp_client::RobonixSystemChronosObserveDeviceTimestampClient<tonic::transport::Channel>,
    mut request: pb::chronos::ObserveDeviceTimestampRequest,
) -> Result<tonic::Response<pb::chronos::ObserveDeviceTimestampResponse>, tonic::Status> {
    let started = std::time::Instant::now();
    let initial_timestamp = request.device_timestamp_ns;
    for attempt in 0..20 {
        request.device_timestamp_ns = initial_timestamp + started.elapsed().as_nanos() as i64;
        match client.observe_device_timestamp(request.clone()).await {
            Err(error) if error.code() == tonic::Code::ResourceExhausted && attempt < 19 => {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            result => return result,
        }
    }
    unreachable!("last attempt returns its result")
}
