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
    assert_eq!(provider.capabilities.len(), 3);
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
