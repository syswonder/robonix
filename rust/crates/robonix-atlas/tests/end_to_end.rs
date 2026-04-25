//! End-to-end system tests against a real Atlas process.
//!
//! Each test spawns Atlas (`AtlasService`) on an ephemeral port, then
//! drives it through the public SDK (`robonix-sdk`) — exactly the path other
//! crates use. This catches regressions in:
//!
//! - proto wire compatibility (proto regen / IDL renames)
//! - service contract IDs (post-rename: `robonix/srv/{pilot,liaison,executor}`)
//! - per-transport endpoint allocation
//! - QueryNodes / NegotiateChannel semantics
use std::net::{SocketAddr, TcpListener};
use std::sync::Arc;
use std::time::Duration;

use robonix_atlas::service::{AtlasRegistry, serve_atlas};
use robonix_sdk::{QueryNodesOpts, RobonixClient};

/// Pick a free TCP port (race-y but fine for tests).
fn pick_port() -> u16 {
    let l = TcpListener::bind("127.0.0.1:0").expect("bind");
    l.local_addr().unwrap().port()
}

/// Spawn an Atlas server on a free port; return its `host:port` endpoint string.
async fn spawn_atlas() -> String {
    let port = pick_port();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let endpoint = format!("127.0.0.1:{port}");
    let endpoint_clone = endpoint.clone();
    tokio::spawn(async move {
        let registry = Arc::new(AtlasRegistry::default());
        let _ = serve_atlas(registry, addr, endpoint_clone).await;
    });
    // give the server a moment to bind
    tokio::time::sleep(Duration::from_millis(200)).await;
    endpoint
}

async fn connect(endpoint: &str) -> RobonixClient {
    RobonixClient::connect(&format!("http://{endpoint}"))
        .await
        .expect("connect to atlas")
}

fn ts(s: &str) -> Vec<String> {
    vec![s.to_string()]
}

#[tokio::test]
async fn pilot_registers_and_is_discoverable() {
    let endpoint = spawn_atlas().await;
    let mut client = connect(&endpoint).await;

    client
        .register_node("com.test.pilot", "robonix/srv/pilot", "service", "humble")
        .await
        .expect("register_node");

    client
        .declare_interface_full(
            "com.test.pilot",
            "pilot",
            ts("grpc"),
            "{}",
            50071,
            "robonix/srv/pilot",
        )
        .await
        .expect("declare_interface");

    let nodes = client
        .query_nodes_opts(QueryNodesOpts {
            namespace: String::new(),
            interface_name: String::new(),
            transport: "grpc".into(),
            distro_prefix: String::new(),
            container_id: String::new(),
            contract_id: "robonix/srv/pilot".into(),
        })
        .await
        .expect("query_nodes");
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0].node_id, "com.test.pilot");
    assert_eq!(nodes[0].interfaces.len(), 1);
    assert_eq!(nodes[0].interfaces[0].contract_id, "robonix/srv/pilot");
    assert!(
        nodes[0].interfaces[0]
            .supported_transports
            .iter()
            .any(|t| t == "grpc")
    );
}

#[tokio::test]
async fn negotiate_channel_grpc_returns_listen_port() {
    let endpoint = spawn_atlas().await;
    let mut client = connect(&endpoint).await;

    client
        .register_node("com.test.exec", "robonix/srv/executor", "service", "humble")
        .await
        .unwrap();
    client
        .declare_interface_full(
            "com.test.exec",
            "executor",
            ts("grpc"),
            "{}",
            50061,
            "robonix/srv/executor",
        )
        .await
        .unwrap();

    let ch = client
        .negotiate_channel("com.test.consumer", "com.test.exec", "executor", "grpc")
        .await
        .expect("negotiate_channel");
    assert_eq!(ch.transport, "grpc");
    // gRPC channel reuses the producer's bound port.
    assert_eq!(ch.endpoint, "localhost:50061");
}

#[tokio::test]
async fn negotiate_channel_ros2_allocates_unique_topic() {
    let endpoint = spawn_atlas().await;
    let mut client = connect(&endpoint).await;

    client
        .register_node(
            "com.test.lidar",
            "robonix/prm/sensor",
            "primitive",
            "humble",
        )
        .await
        .unwrap();
    client
        .declare_interface_full(
            "com.test.lidar",
            "lidar3d",
            ts("ros2"),
            r#"{"ros2_topic":"/scanner/cloud"}"#,
            0,
            "robonix/prm/sensor/lidar3d",
        )
        .await
        .unwrap();

    let ch1 = client
        .negotiate_channel("com.test.c1", "com.test.lidar", "lidar3d", "ros2")
        .await
        .unwrap();
    let ch2 = client
        .negotiate_channel("com.test.c2", "com.test.lidar", "lidar3d", "ros2")
        .await
        .unwrap();
    // Per-channel ROS 2 endpoints — fresh each time.
    assert!(ch1.endpoint.starts_with("/rbnx/ch/n"));
    assert!(ch2.endpoint.starts_with("/rbnx/ch/n"));
    assert_ne!(ch1.endpoint, ch2.endpoint);
    assert_ne!(ch1.channel_id, ch2.channel_id);
}

#[tokio::test]
async fn ros2_declared_topic_surfaced_in_metadata() {
    let endpoint = spawn_atlas().await;
    let mut client = connect(&endpoint).await;

    client
        .register_node("com.test.cam", "robonix/prm/camera", "primitive", "humble")
        .await
        .unwrap();
    client
        .declare_interface_full(
            "com.test.cam",
            "rgb",
            ts("ros2"),
            r#"{"ros2_topic":"/camera/rgb"}"#,
            0,
            "robonix/prm/camera/rgb",
        )
        .await
        .unwrap();

    let nodes = client
        .query_nodes_opts(QueryNodesOpts {
            namespace: String::new(),
            interface_name: String::new(),
            transport: "ros2".into(),
            distro_prefix: String::new(),
            container_id: String::new(),
            contract_id: "robonix/prm/camera/rgb".into(),
        })
        .await
        .unwrap();
    assert_eq!(nodes.len(), 1);
    let meta = &nodes[0].interfaces[0].metadata_json;
    let v: serde_json::Value = serde_json::from_str(meta).unwrap();
    assert_eq!(
        v.get("ros2_topic").and_then(|x| x.as_str()),
        Some("/camera/rgb")
    );
}

#[tokio::test]
async fn heartbeat_returns_server_time() {
    let endpoint = spawn_atlas().await;
    let mut client = connect(&endpoint).await;

    client
        .register_node("com.test.hb", "robonix/srv/liaison", "service", "humble")
        .await
        .unwrap();
    let t1 = client.node_heartbeat("com.test.hb").await.unwrap();
    tokio::time::sleep(Duration::from_millis(20)).await;
    let t2 = client.node_heartbeat("com.test.hb").await.unwrap();
    assert!(
        t2 >= t1,
        "heartbeat timestamp did not advance: t1={t1} t2={t2}"
    );
}

#[tokio::test]
async fn multiple_providers_same_contract_all_returned() {
    let endpoint = spawn_atlas().await;
    let mut client = connect(&endpoint).await;

    // mid360 IMU
    client
        .register_node(
            "com.test.mid360",
            "robonix/prm/sensor",
            "primitive",
            "humble",
        )
        .await
        .unwrap();
    client
        .declare_interface_full(
            "com.test.mid360",
            "imu",
            ts("ros2"),
            r#"{"ros2_topic":"/livox/imu"}"#,
            0,
            "robonix/prm/sensor/imu",
        )
        .await
        .unwrap();

    // realsense IMU
    client
        .register_node(
            "com.test.realsense",
            "robonix/prm/camera",
            "primitive",
            "humble",
        )
        .await
        .unwrap();
    client
        .declare_interface_full(
            "com.test.realsense",
            "imu",
            ts("ros2"),
            r#"{"ros2_topic":"/camera/imu"}"#,
            0,
            "robonix/prm/sensor/imu",
        )
        .await
        .unwrap();

    let nodes = client
        .query_nodes_opts(QueryNodesOpts {
            namespace: String::new(),
            interface_name: String::new(),
            transport: "ros2".into(),
            distro_prefix: String::new(),
            container_id: String::new(),
            contract_id: "robonix/prm/sensor/imu".into(),
        })
        .await
        .unwrap();
    assert_eq!(nodes.len(), 2);
}
