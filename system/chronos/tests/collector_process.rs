// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

use std::path::Path;
use std::process::{Child, Command};
use std::sync::Arc;
use std::time::Duration;

use robonix_atlas::{
    client::AtlasClient,
    contract_registry::ContractRegistry,
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
    robonix_system_chronos_observe_device_timestamp_client::RobonixSystemChronosObserveDeviceTimestampClient,
};

struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Find child processes without reading command arguments or sensitive environment data.
fn children(pid: u32) -> Vec<u32> {
    std::fs::read_dir(format!("/proc/{pid}/task"))
        .unwrap()
        .flatten()
        .flat_map(|entry| {
            std::fs::read_to_string(entry.path().join("children"))
                .unwrap_or_default()
                .split_whitespace()
                .filter_map(|pid| pid.parse().ok())
                .collect::<Vec<u32>>()
        })
        .collect()
}

#[tokio::test]
#[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
/// Real ROS input forms a model through the owned helper; deactivate/restart/shutdown reap it.
async fn ros_collection_is_owned_by_chronos() {
    exercise(Scenario {
        ..Default::default()
    })
    .await;
}

#[tokio::test]
#[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
/// Cached native capture identities survive helper reactivation without terminating Chronos.
async fn native_identity_survives_cached_reactivation() {
    exercise(Scenario {
        native: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
#[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
/// An unexpectedly killed helper causes visible daemon failure and releases its Atlas edge.
async fn helper_failure_is_visible_and_releases_edges() {
    exercise(Scenario {
        fail: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
#[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
/// Subscription startup failure rolls back an already-created Atlas consumer connection.
async fn startup_failure_releases_edges() {
    exercise(Scenario {
        bad_startup: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
#[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
/// Chronos becomes ACTIVE before its device exists, and begins fitting after late activation.
async fn collection_waits_for_late_device() {
    exercise(Scenario {
        late: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
/// Hardware queries use raw u32 counters with no ROS publisher and obey lifecycle cleanup.
async fn hardware_query_collection_is_owned_by_chronos() {
    exercise(Scenario {
        query: true,
        ros_disabled: true,
        ..Default::default()
    })
    .await;
}

#[derive(Default)]
struct Scenario {
    native: bool,
    fail: bool,
    bad_startup: bool,
    late: bool,
    query: bool,
    query_mode: Option<&'static str>,
    ros_disabled: bool,
    total_disabled: bool,
    missing_ros: bool,
    late_ros: bool,
}

#[tokio::test]
/// An old master-only ROS configuration stays ACTIVE without creating subscriptions or helpers.
async fn ros_collection_requires_explicit_opt_in() {
    exercise(Scenario {
        ros_disabled: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
/// The master switch prevents query helpers even when ROS collection is explicitly enabled.
async fn master_switch_disables_all_automatic_collection() {
    exercise(Scenario {
        query: true,
        total_disabled: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
/// A missing ROS device cannot hold up a standalone hardware query or lifecycle cleanup.
async fn hardware_query_runs_while_ros_device_is_absent() {
    exercise(Scenario {
        query: true,
        missing_ros: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
/// Query adapters may wait for an existing gRPC capability without requiring a ROS declaration.
async fn hardware_query_waits_for_grpc_activation() {
    exercise(Scenario {
        query: true,
        ros_disabled: true,
        late: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
/// Query read failure remains fatal and never starts a fallback ROS subscription.
async fn hardware_query_failure_does_not_fall_back_to_ros() {
    exercise(Scenario {
        query: true,
        bad_startup: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
/// Oversized startup and unterminated observation output fail promptly and reap the helper.
async fn oversized_query_output_reaps_the_helper() {
    for mode in ["oversized_startup", "oversized_observation"] {
        exercise(Scenario {
            query: true,
            query_mode: Some(mode),
            bad_startup: true,
            ros_disabled: true,
            ..Default::default()
        })
        .await;
    }
}

#[tokio::test]
/// Fatal helper failure cancels other domains still waiting for discovery.
async fn hardware_helper_failure_cancels_pending_ros_discovery() {
    exercise(Scenario {
        query: true,
        missing_ros: true,
        fail: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
#[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
/// A late ROS source starts alongside the fitted query domain; both helpers are reaped on lifecycle changes.
async fn mixed_domains_collect_and_reap_independently() {
    exercise(Scenario {
        query: true,
        missing_ros: true,
        late_ros: true,
        ..Default::default()
    })
    .await;
}

#[tokio::test]
#[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
/// Failure of one running helper stops and reaps the other helper and releases its ROS edge.
async fn mixed_helper_failure_reaps_all_domains() {
    exercise(Scenario {
        query: true,
        missing_ros: true,
        late_ros: true,
        fail: true,
        ..Default::default()
    })
    .await;
}

/// Exercise real Atlas/daemon/helper processes, source selection, and lifecycle cleanup.
async fn exercise(scenario: Scenario) {
    let Scenario {
        native,
        fail,
        bad_startup,
        late,
        query,
        query_mode,
        ros_disabled,
        total_disabled,
        missing_ros,
        late_ros,
    } = scenario;
    let helper_files = tempfile::tempdir().unwrap();
    let helper_pid_file = helper_files.path().join("helper.pid");
    let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reservation.local_addr().unwrap();
    drop(reservation);
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../capabilities");
    let contracts = ContractRegistry::load_from_capabilities_dir(&root).unwrap();
    let registry = Arc::new(AtlasRegistry::with_contracts(contracts));
    let server = tokio::spawn(serve_atlas(registry.clone(), address));
    let mut atlas =
        AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
            .await
            .unwrap();
    let topic = format!(
        "/chronos_collector_test_{}_{}_{}_{}_{}",
        std::process::id(),
        u8::from(native),
        u8::from(fail),
        u8::from(bad_startup),
        u8::from(late)
    );
    if !late && !query {
        declare_camera(&mut atlas, &topic, native, bad_startup).await;
    }
    let mut publisher = (!query && !ros_disabled).then(|| {
        ChildGuard(
            Command::new("python3")
                .args([
                    "-u",
                    "-c",
                    include_str!("collector_publisher.py"),
                    &topic,
                    if native { "native" } else { "plain" },
                ])
                .spawn()
                .unwrap(),
        )
    });
    let mut config = serde_json::json!({
        "collect_observations": !total_disabled,
        "collect_ros_observations": !ros_disabled,
        "core": {
            "reference": {"kind": "linux_system"},
            "quality": {"refresh_interval_ns": 10000000},
            "domains": {"camera": {
                "capabilities": [{"provider_id": "collector-camera", "contract_id": "robonix/primitive/camera/rgb", "transport": "ros2", "timestamp_field": "header.stamp"}],
                "timestamp": {"event": "acquisition", "hardware": false, "time_scale": "device_uptime", "resolution_ns": 1, "uncertainty_ns": 10000000},
                "strategy": {"kind": "affine_model", "observation": {"reference_at": "host_receive_time", "sampling_interval_ns": 20000000, "uncertainty_ns": 20000000},
                    "policy": {"minimum_observations": 4, "observation_window": 20, "max_drift_ppm": 100000, "max_residual_ns": 100000000, "tracking_timeout_ns": 500000000, "holdover_duration_ns": 500000000, "max_uncertainty_ns": 100000000}}
            }}
        }
    });
    if ros_disabled {
        config
            .as_object_mut()
            .unwrap()
            .remove("collect_ros_observations");
    }
    if native {
        let binding = &mut config["core"]["domains"]["camera"]["capabilities"][0];
        binding["session_id_field"] = "header.frame_id".into();
        binding["device_generation_field"] = "height".into();
        binding["sequence_id_field"] = "width".into();
    }
    if missing_ros {
        let mut missing = config["core"]["domains"]["camera"].clone();
        missing["capabilities"][0]["provider_id"] = if late_ros {
            "collector-camera"
        } else {
            "missing-camera"
        }
        .into();
        config["core"]["domains"]["a_missing_ros"] = missing;
    }
    if query {
        config["core"]["domains"]["camera"]["capabilities"] = if late {
            serde_json::json!([{"provider_id": "collector-camera", "contract_id": "robonix/lifecycle/driver", "transport": "grpc"}])
        } else {
            serde_json::json!([])
        };
        let observation = &mut config["core"]["domains"]["camera"]["strategy"]["observation"];
        observation["reference_at"] = "host_query_midpoint".into();
        observation["query"] = serde_json::json!({"adapter": "query_fixture:create", "tick_ns": 1000, "options": {
            "mode": query_mode.unwrap_or(if bad_startup { "error" } else { "normal" }),
        }});
        if query_mode.is_some() {
            observation["query"]["options"]["pid_file"] =
                helper_pid_file.to_string_lossy().into_owned().into();
        }
        config["core"]["domains"]["camera"]["timestamp"]["wrap_period_ns"] =
            (4_294_967_296_000_u64).into();
    }
    let mut daemon = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_robonix-chronos"))
            .env(
                "PYTHONPATH",
                format!(
                    "{}:{}",
                    Path::new(env!("CARGO_MANIFEST_DIR"))
                        .join("tests")
                        .display(),
                    std::env::var("PYTHONPATH").unwrap_or_default()
                ),
            )
            .args([
                "--atlas",
                &address.to_string(),
                "--listen",
                "127.0.0.1:0",
                "--provider-id",
                "collector-chronos",
                "--config-json",
                &config.to_string(),
            ])
            .spawn()
            .unwrap(),
    );
    if bad_startup {
        timeout(Duration::from_secs(15), async {
            loop {
                if let Some(status) = daemon.0.try_wait().unwrap() {
                    assert!(!status.success());
                    break;
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .unwrap();
        if query_mode.is_some() {
            let pid = std::fs::read_to_string(&helper_pid_file).unwrap();
            assert!(
                !Path::new(&format!("/proc/{}", pid.trim())).exists(),
                "failed helper must be reaped"
            );
        }
        assert_eq!(collector_edges(&registry).await, 0);
        server.abort();
        return;
    }
    timeout(Duration::from_secs(15), async {
        loop {
            assert!(
                daemon.0.try_wait().unwrap().is_none(),
                "Chronos startup failed"
            );
            if let Some(publisher) = publisher.as_mut() {
                assert!(
                    publisher.0.try_wait().unwrap().is_none(),
                    "ROS publisher failed"
                );
            }
            let providers = atlas
                .query_services("collector-chronos", "", "", atlas_pb::Transport::Grpc)
                .await
                .unwrap();
            if providers
                .iter()
                .any(|provider| provider.state == atlas_pb::LifecycleState::StateActive as i32)
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap();
    if late {
        assert!(
            children(daemon.0.id()).is_empty(),
            "no ROS helper before device registration"
        );
        assert_eq!(collector_edges(&registry).await, 0);
        // Read snapshots before the device registers: boot readiness cannot depend on its data.
        let (_, endpoint, _) = atlas
            .connect_capability(
                "test",
                "collector-chronos",
                "robonix/system/chronos/conversion_models",
                atlas_pb::Transport::Grpc,
            )
            .await
            .unwrap();
        let mut client =
            RobonixSystemChronosConversionModelsClient::connect(format!("http://{endpoint}"))
                .await
                .unwrap();
        let snapshot = timeout(Duration::from_secs(2), async {
            client
                .conversion_models(())
                .await
                .unwrap()
                .into_inner()
                .message()
                .await
                .unwrap()
                .unwrap()
        })
        .await
        .unwrap();
        assert!(snapshot.domains.iter().all(|domain| !domain.has_model));
        if query {
            declare_query_driver(&mut atlas, daemon.0.id()).await;
        } else {
            declare_camera(&mut atlas, &topic, native, bad_startup).await;
        }
    }
    let (driver_edge, endpoint, _) = atlas
        .connect_capability(
            "collector-camera",
            "collector-chronos",
            "robonix/lifecycle/driver",
            atlas_pb::Transport::Grpc,
        )
        .await
        .unwrap();
    let mut driver = RobonixLifecycleDriverClient::connect(format!("http://{endpoint}"))
        .await
        .unwrap();
    let mut models =
        RobonixSystemChronosConversionModelsClient::connect(format!("http://{endpoint}"))
            .await
            .unwrap();
    if total_disabled || (ros_disabled && !query) {
        let snapshot = models
            .conversion_models(())
            .await
            .unwrap()
            .into_inner()
            .message()
            .await
            .unwrap()
            .unwrap();
        assert!(snapshot.domains.iter().all(|domain| !domain.has_model));
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!(
            children(daemon.0.id()).is_empty(),
            "disabled collection starts no helper"
        );
        assert_eq!(collector_edges(&registry).await, 0);
        for command in [2, 1, 3] {
            let reply = driver
                .driver(pb::lifecycle::DriverRequest {
                    command,
                    config_json: "{}".into(),
                })
                .await
                .unwrap()
                .into_inner();
            assert!(reply.ok, "{}", reply.error);
        }
        timeout(Duration::from_secs(8), async {
            loop {
                if let Some(status) = daemon.0.try_wait().unwrap() {
                    assert!(status.success());
                    break;
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .unwrap();
        atlas.disconnect_capability(&driver_edge).await.unwrap();
        server.abort();
        return;
    }
    if query {
        let mut ingress =
            RobonixSystemChronosObserveDeviceTimestampClient::connect(format!("http://{endpoint}"))
                .await
                .unwrap();
        let error = ingress
            .observe_device_timestamp(pb::chronos::ObserveDeviceTimestampRequest {
                domain_id: "camera".into(),
                session_id: "invalid-query-origin".into(),
                device_generation: 1,
                sequence_id: 1,
                device_timestamp_ns: 1,
            })
            .await
            .unwrap_err();
        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
    }
    let mut stream = models.conversion_models(()).await.unwrap().into_inner();
    let first = timeout(Duration::from_secs(10), async {
        loop {
            let snapshot = stream.message().await.unwrap().unwrap();
            if let Some(domain) = snapshot.domains.into_iter().find(|domain| domain.has_model) {
                break domain;
            }
        }
    })
    .await
    .unwrap();
    if missing_ros {
        let snapshot = models
            .conversion_models(())
            .await
            .unwrap()
            .into_inner()
            .message()
            .await
            .unwrap()
            .unwrap();
        assert!(
            !snapshot
                .domains
                .iter()
                .find(|domain| domain.domain_id == "a_missing_ros")
                .unwrap()
                .has_model
        );
    }
    if late_ros {
        declare_camera(&mut atlas, &topic, false, false).await;
        publisher = Some(ChildGuard(
            Command::new("python3")
                .args([
                    "-u",
                    "-c",
                    include_str!("collector_publisher.py"),
                    &topic,
                    "plain",
                ])
                .spawn()
                .unwrap(),
        ));
        timeout(Duration::from_secs(10), async {
            loop {
                let snapshot = stream.message().await.unwrap().unwrap();
                if snapshot.domains.iter().all(|domain| domain.has_model) {
                    break;
                }
            }
        })
        .await
        .unwrap();
    }
    let helper_pids = children(daemon.0.id());
    assert_eq!(
        helper_pids.len(),
        if late_ros { 2 } else { 1 },
        "each ready domain owns a helper"
    );
    assert_eq!(
        collector_edges(&registry).await,
        usize::from(!query || late_ros)
    );
    if query {
        let model = first.model.as_ref().unwrap();
        assert_eq!(first.state, 1, "query should produce a Tracking model");
        assert!(model.has_wrap_period);
        assert_eq!(model.wrap_period_ns, 4_294_967_296_000);
        assert_eq!(model.latest_raw_device_ns % 1000, 0);
    }
    if fail {
        // SAFETY: pid belongs to this test's owned child helper; SIGKILL has no pointers.
        assert_eq!(
            unsafe { libc::kill(helper_pids[0] as i32, libc::SIGKILL) },
            0
        );
        timeout(Duration::from_secs(10), async {
            loop {
                if let Some(status) = daemon.0.try_wait().unwrap() {
                    assert!(!status.success());
                    break;
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .unwrap();
        for pid in helper_pids {
            assert!(!Path::new(&format!("/proc/{pid}")).exists());
        }
        assert_eq!(collector_edges(&registry).await, 0);
        server.abort();
        return;
    }
    if native {
        // SAFETY: the test-owned publisher handles SIGUSR1 by pausing captures, keeping DDS alive.
        assert_eq!(
            unsafe { libc::kill(publisher.as_ref().unwrap().0.id() as i32, libc::SIGUSR1) },
            0
        );
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    let deactivated = driver
        .driver(pb::lifecycle::DriverRequest {
            command: 2,
            config_json: "{}".into(),
        })
        .await
        .unwrap()
        .into_inner();
    assert!(deactivated.ok, "{}", deactivated.error);
    assert_eq!(collector_edges(&registry).await, 0);
    for pid in helper_pids {
        assert!(
            !Path::new(&format!("/proc/{pid}")).exists(),
            "deactivated helper must be reaped"
        );
    }
    let activated = driver
        .driver(pb::lifecycle::DriverRequest {
            command: 1,
            config_json: "{}".into(),
        })
        .await
        .unwrap()
        .into_inner();
    assert!(activated.ok, "{}", activated.error);
    let mut stream = models.conversion_models(()).await.unwrap().into_inner();
    let cached = stream.message().await.unwrap().unwrap();
    let baselines: std::collections::BTreeMap<_, _> = cached
        .domains
        .iter()
        .filter_map(|domain| {
            domain
                .model
                .as_ref()
                .map(|model| (domain.domain_id.clone(), model.last_observation_steady_ns))
        })
        .collect();
    if native {
        tokio::time::sleep(Duration::from_millis(300)).await;
        assert!(
            daemon.0.try_wait().unwrap().is_none(),
            "cached native capture must not kill Chronos"
        );
        // SAFETY: the same owned publisher handles SIGUSR2 by resuming captures.
        assert_eq!(
            unsafe { libc::kill(publisher.as_ref().unwrap().0.id() as i32, libc::SIGUSR2) },
            0
        );
    }
    timeout(Duration::from_secs(10), async {
        loop {
            let snapshot = stream.message().await.unwrap().unwrap();
            if snapshot
                .domains
                .iter()
                .filter(|domain| domain.domain_id == "camera" || late_ros)
                .all(|domain| {
                    domain.has_model
                        && (domain.domain_id != "camera"
                            || if native {
                                domain.generation == first.generation
                            } else {
                                domain.generation > first.generation
                            })
                        && domain.model.as_ref().is_some_and(|model| {
                            model.generation == domain.generation
                                && model.last_observation_steady_ns > baselines[&domain.domain_id]
                        })
                })
            {
                break;
            }
        }
    })
    .await
    .unwrap();
    let helper_pids = children(daemon.0.id());
    assert_eq!(helper_pids.len(), if late_ros { 2 } else { 1 });
    let shutdown = driver
        .driver(pb::lifecycle::DriverRequest {
            command: 3,
            config_json: "{}".into(),
        })
        .await
        .unwrap()
        .into_inner();
    assert!(shutdown.ok, "{}", shutdown.error);
    drop(stream);
    timeout(Duration::from_secs(8), async {
        loop {
            if let Some(status) = daemon.0.try_wait().unwrap() {
                assert!(status.success());
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap();
    for pid in helper_pids {
        assert!(
            !Path::new(&format!("/proc/{pid}")).exists(),
            "shutdown helper must be reaped"
        );
    }
    atlas.disconnect_capability(&driver_edge).await.unwrap();
    assert_eq!(collector_edges(&registry).await, 0);
    server.abort();
}

/// Count only Chronos-owned consumer connections, excluding the test's Driver connection.
async fn collector_edges(registry: &AtlasRegistry) -> usize {
    let inspected: serde_json::Value =
        serde_json::from_str(&registry.inspect_json().await.unwrap()).unwrap();
    inspected["channels"]
        .as_object()
        .unwrap()
        .values()
        .filter(|channel| channel["consumer_id"] == "collector-chronos")
        .count()
}

/// Declare a synthetic camera with the shared lifecycle and publish its ACTIVE state.
async fn declare_camera(atlas: &mut AtlasClient, topic: &str, native: bool, bad_startup: bool) {
    atlas
        .register_primitive("collector-camera", "robonix/primitive/camera", "")
        .await
        .unwrap();
    atlas
        .declare_capability(
            "collector-camera",
            "robonix/lifecycle/driver",
            atlas_pb::Transport::Grpc,
            "127.0.0.1:1",
            robonix_atlas::client::grpc_params(
                "capabilities/lifecycle/driver.v1.toml",
                "robonix.contracts.RobonixLifecycleDriver",
                "/robonix.contracts.RobonixLifecycleDriver/Driver",
            ),
        )
        .await
        .unwrap();
    atlas
        .declare_capability(
            "collector-camera",
            "robonix/primitive/camera/rgb",
            atlas_pb::Transport::Ros2,
            topic,
            atlas_pb::TransportParams {
                kind: Some(atlas_pb::transport_params::Kind::Ros2(
                    atlas_pb::Ros2Params {
                        qos_profile: if bad_startup {
                            "unsupported"
                        } else if native {
                            "latched"
                        } else {
                            "reliable"
                        }
                        .into(),
                    },
                )),
            },
        )
        .await
        .unwrap();
    atlas
        .set_lifecycle_state(
            "collector-camera",
            atlas_pb::LifecycleState::StateInactive,
            "",
        )
        .await
        .unwrap();
    atlas
        .set_lifecycle_state(
            "collector-camera",
            atlas_pb::LifecycleState::StateActive,
            "",
        )
        .await
        .unwrap();
}

/// Register a driver with only the existing gRPC lifecycle capability, initially INACTIVE.
async fn declare_query_driver(atlas: &mut AtlasClient, daemon_pid: u32) {
    atlas
        .register_primitive("collector-camera", "robonix/primitive/camera", "")
        .await
        .unwrap();
    atlas
        .declare_capability(
            "collector-camera",
            "robonix/lifecycle/driver",
            atlas_pb::Transport::Grpc,
            "127.0.0.1:1",
            robonix_atlas::client::grpc_params(
                "capabilities/lifecycle/driver.v1.toml",
                "robonix.contracts.RobonixLifecycleDriver",
                "/robonix.contracts.RobonixLifecycleDriver/Driver",
            ),
        )
        .await
        .unwrap();
    atlas
        .set_lifecycle_state(
            "collector-camera",
            atlas_pb::LifecycleState::StateInactive,
            "",
        )
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(750)).await;
    assert!(
        children(daemon_pid).is_empty(),
        "query must wait for ACTIVE driver"
    );
    atlas
        .set_lifecycle_state(
            "collector-camera",
            atlas_pb::LifecycleState::StateActive,
            "",
        )
        .await
        .unwrap();
}
