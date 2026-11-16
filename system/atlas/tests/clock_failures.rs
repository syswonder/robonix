// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

use robonix_atlas::{
    client::AtlasClient,
    pb,
    service::{AtlasRegistry, serve_atlas},
};
use std::sync::Arc;
use std::time::Duration;
use tonic::Code;

include!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../chronos/clock/tests/support/mod.rs"
));
use self::mode as clock_mode;

#[test]
fn clock_failures_preserve_state_and_heartbeat_expiry() {
    run_case_with_env(
        "clock_failures_preserve_state_and_heartbeat_expiry",
        &[
            (
                "ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS",
                std::ffi::OsStr::new("1000"),
            ),
            (
                "ROBONIX_ATLAS_EVICTION_INTERVAL_MS",
                std::ffi::OsStr::new("10"),
            ),
            (
                "ROBONIX_ATLAS_GC_AFTER_TERMINATED_MS",
                std::ffi::OsStr::new("10000"),
            ),
        ],
        || {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async {
                    tokio::time::timeout(Duration::from_secs(10), exercise())
                        .await
                        .unwrap();
                });
        },
    );
}

/// Serve real gRPC calls, compare state after failures, and verify time-step immunity.
async fn exercise() {
    let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reserved.local_addr().unwrap();
    drop(reserved);
    let registry = Arc::new(AtlasRegistry::default());
    let server = tokio::spawn(serve_atlas(registry.clone(), address));
    let atlas = AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
        .await
        .unwrap();
    let mut client = atlas.inner();
    let request = pb::RegisterRequest {
        id: "clock-provider".into(),
        namespace: "robonix/service/test".into(),
        capability_md_path: String::new(),
        capability_md: "original".into(),
    };
    client.register_service(request.clone()).await.unwrap();
    client
        .declare_capability(pb::DeclareCapabilityRequest {
            provider_id: request.id.clone(),
            contract_id: "robonix/service/test/stream".into(),
            transport: pb::Transport::Ros2 as i32,
            endpoint: "/clock_test".into(),
            params: Some(pb::TransportParams {
                kind: Some(pb::transport_params::Kind::Ros2(pb::Ros2Params {
                    qos_profile: String::new(),
                })),
            }),
            description: String::new(),
            time_descriptor: None,
        })
        .await
        .unwrap();
    let connection = pb::ConnectCapabilityRequest {
        consumer_id: "clock-consumer".into(),
        provider_id: request.id.clone(),
        contract_id: "robonix/service/test/stream".into(),
        transport: pb::Transport::Ros2 as i32,
    };
    client.connect_capability(connection.clone()).await.unwrap();
    assert!(!robonix_chronos_clock::reference_time().unwrap().verified);

    for mode in [1, 2, 4] {
        let before = registry.inspect_json().await.unwrap();
        clock_mode(mode);
        let mut takeover = request.clone();
        takeover.namespace = "robonix/service/changed".into();
        takeover.capability_md = "replacement".into();
        assert_eq!(
            client.register_service(takeover).await.unwrap_err().code(),
            Code::Unavailable
        );
        let mut new = request.clone();
        new.id = "new-provider".into();
        assert_eq!(
            client.register_service(new).await.unwrap_err().code(),
            Code::Unavailable
        );
        assert_eq!(
            client
                .heartbeat(pb::HeartbeatRequest {
                    id: request.id.clone()
                })
                .await
                .unwrap_err()
                .code(),
            Code::Unavailable
        );
        assert_eq!(
            client
                .connect_capability(connection.clone())
                .await
                .unwrap_err()
                .code(),
            Code::Unavailable
        );
        assert_eq!(registry.inspect_json().await.unwrap(), before);
        assert!(robonix_chronos_clock::steady_time().is_ok());
        clock_mode(0);
        client
            .heartbeat(pb::HeartbeatRequest {
                id: request.id.clone(),
            })
            .await
            .unwrap();
    }

    clock_mode(16);
    assert!(robonix_chronos_clock::steady_time().is_err());
    assert!(robonix_chronos_clock::steady_time().is_ok());
    clock_mode(8);
    tokio::time::sleep(Duration::from_millis(30)).await;
    let state: serde_json::Value =
        serde_json::from_str(&registry.inspect_json().await.unwrap()).unwrap();
    assert!(state["providers"][&request.id]["pushed_state"].is_null());
    clock_mode(32);
    client
        .heartbeat(pb::HeartbeatRequest {
            id: request.id.clone(),
        })
        .await
        .unwrap();
    clock_mode(0);
    tokio::time::sleep(Duration::from_millis(30)).await;
    let state: serde_json::Value =
        serde_json::from_str(&registry.inspect_json().await.unwrap()).unwrap();
    assert!(state["providers"][&request.id]["pushed_state"].is_null());
    // Real expiry still removes channels, despite prior reference failures/steps.
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let state: serde_json::Value =
                serde_json::from_str(&registry.inspect_json().await.unwrap()).unwrap();
            if state["providers"][&request.id]["pushed_state"] == "STATE_TERMINATED" {
                assert!(state["channels"].as_object().unwrap().is_empty());
                break;
            }
            assert!(
                !server.is_finished(),
                "Atlas stopped before heartbeat expiry"
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    server.abort();
    let _ = server.await;
    drop(client);
    drop(atlas);
    tokio::time::timeout(Duration::from_secs(1), async {
        while Arc::strong_count(&registry) != 1 {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .expect("cancelled Atlas must release its registry");
}
