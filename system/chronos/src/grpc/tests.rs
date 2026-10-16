// SPDX-License-Identifier: MulanPSL-2.0

use std::sync::Arc;

use robonix_atlas::service::{AtlasRegistry, serve_atlas};
use tokio::time::timeout;

use super::pb::contracts::{
    robonix_system_chronos_conversion_models_client::RobonixSystemChronosConversionModelsClient,
    robonix_system_chronos_quality_client::RobonixSystemChronosQualityClient,
};
use super::*;
use crate::snapshot::*;
use crate::{
    ChronosConfig, DomainState, QualityRefreshFailure, QualityRuntimeStatus, VirtualSteadyTime,
};

#[test]
/// Initial failure clears clock payloads while retaining exact failure evidence.
fn initial_failure_encoding_does_not_invent_clock_evidence() {
    let status = QualityRuntimeStatus {
        report: None,
        last_successful_refresh_steady: None,
        refresh_failure: Some(QualityRefreshFailure {
            message: "clock read failed".into(),
            consecutive_failures: 2,
            first_observed_steady: Some(VirtualSteadyTime { timestamp_ns: 42 }),
            last_observed_steady: None,
        }),
    };
    let encoded = encoding::quality(ClockQualitySnapshot::from(&status));
    assert!(!encoded.report_available);
    assert_eq!(encoded.observed_steady_ns, 0);
    assert_eq!(
        encoded.reference.unwrap(),
        pb::chronos::ReferenceQuality::default()
    );
    assert!(encoded.direct_domains.is_empty() && encoded.affine_domains.is_empty());
    let health = encoded.refresh_health.unwrap();
    assert!(!health.has_last_successful_refresh_steady);
    assert_eq!(health.last_successful_refresh_steady_ns, 0);
    assert!(health.has_failure);
    let failure = health.failure.unwrap();
    assert!(failure.has_first_observed_steady);
    assert_eq!(failure.first_observed_steady_ns, 42);
    assert!(!failure.has_last_observed_steady);
    assert_eq!(failure.last_observed_steady_ns, 0);
}

#[test]
/// All absent quality options clear their scalar and nested payloads.
fn absent_quality_payloads_are_cleared() {
    let encoded = encoding::quality(ClockQualitySnapshot {
        report: Some(ClockQualityReportSnapshot {
            observed_steady_time: VirtualSteadyTime { timestamp_ns: 7 },
            reference: ReferenceQualitySnapshot {
                timestamp_ns: 123,
                generation: 4,
                generation_changed: true,
                uncertainty_ns: 9,
                available: true,
                verified: false,
                reason: "unverified".into(),
            },
            direct_domains: vec![],
            affine_domains: vec![AffineDomainQualitySnapshot {
                domain_id: "device".into(),
                generation: 2,
                reference_generation: 4,
                model_version: 0,
                state: DomainState::Initializing,
                accepted_observations: 0,
                rejected_observations: 0,
                last_observation_age_ns: None,
                uncertainty_ns: None,
                reason: "collecting".into(),
            }],
        }),
        refresh_health: RefreshHealthSnapshot {
            last_successful_refresh_steady: None,
            failure: None,
        },
    });
    assert!(encoded.report_available);
    let domain = &encoded.affine_domains[0];
    assert!(!domain.has_last_observation_age && !domain.has_uncertainty);
    assert_eq!(domain.last_observation_age_ns, 0);
    assert_eq!(domain.uncertainty_ns, 0);
    let health = encoded.refresh_health.unwrap();
    assert!(!health.has_failure);
    assert_eq!(
        health.failure.unwrap(),
        pb::chronos::RefreshFailure::default()
    );
}

#[test]
/// Wire encoding keeps old coefficient identity, signed terms, and every wrap input.
fn retained_model_encoding_preserves_identity_and_wrap() {
    let parameters = ClockModelParametersSnapshot {
        generation: 1,
        reference_generation: 2,
        model_version: 3,
        state: DomainState::Unsynced,
        device_anchor_ns: 900,
        reference_anchor_ns: 5_000,
        rate_adjust_ppb: -125_000,
        residual_offset_ns: -10,
        uncertainty_ns: u64::MAX,
        valid_from_device_ns: 900,
        valid_until_device_ns: 1_200,
        sample_count: 8,
        last_observation_steady_ns: 55,
        latest_raw_device_ns: 100,
        wrap_period_ns: Some(1_000),
        wrap_offset_ns: 1_000,
    };
    let snapshot = ClockModelsSnapshot {
        published_steady_time: VirtualSteadyTime { timestamp_ns: 60 },
        reference_generation: 4,
        domains: vec![
            ClockModelSnapshot {
                domain_id: "device".into(),
                generation: 3,
                reference_generation: 4,
                model_version: 0,
                state: DomainState::Reset,
                reason: "reference step".into(),
                model: Some(parameters),
            },
            ClockModelSnapshot {
                domain_id: "unfitted".into(),
                generation: 1,
                reference_generation: 4,
                model_version: 0,
                state: DomainState::Initializing,
                reason: "collecting".into(),
                model: None,
            },
        ],
    };
    let encoded = encoding::models(snapshot);
    let domain = &encoded.domains[0];
    let model = domain.model.as_ref().unwrap();
    assert_eq!((domain.generation, domain.reference_generation), (3, 4));
    assert_eq!(
        (
            model.generation,
            model.reference_generation,
            model.model_version
        ),
        (1, 2, 3)
    );
    assert_eq!(
        (model.rate_adjust_ppb, model.residual_offset_ns),
        (-125_000, -10)
    );
    assert!(model.has_wrap_period);
    assert_eq!(
        (
            model.wrap_period_ns,
            model.latest_raw_device_ns,
            model.wrap_offset_ns
        ),
        (1_000, 100, 1_000)
    );
    assert_eq!(model.uncertainty_ns, u64::MAX);
    assert!(!encoded.domains[1].has_model);
    assert_eq!(
        encoded.domains[1].model.as_ref().unwrap(),
        &pb::chronos::ClockModelParameters::default()
    );
}

#[test]
/// Invalid config is rejected before startup; manifest core overrides YAML defaults.
fn configuration_rejects_unknown_fields_and_invalid_core() {
    assert!(config::parse_manifest(r#"{"unknown":true}"#).is_err());
    let args = config::Args::try_parse_from([
        "robonix-chronos",
        "--config-json",
        r#"{"core":{"reference":{"kind":"linux_system"},"quality":{"refresh_interval_ns":0}}}"#,
    ])
    .unwrap();
    assert!(config::resolve(args).is_err());
    let args = config::Args::try_parse_from([
        "robonix-chronos",
        "--config-json",
        r#"{"provider_id":"other","listen":"127.0.0.1:50199"}"#,
        "--provider-id",
        "chronos-test",
    ])
    .unwrap();
    let cfg = config::resolve(args).unwrap();
    assert_eq!(cfg.provider_id, "chronos-test");
    assert_eq!(cfg.listen.port(), 50199);
}

/// Build a real Linux runtime with a short refresh cadence and one unfitted device.
pub(super) fn test_config() -> ChronosConfig {
    ChronosConfig::from_yaml(
        r#"
chronos:
  core:
    reference:
      kind: linux_system
    quality:
      refresh_interval_ns: 10000000
    domains:
      device:
        capabilities:
          - provider_id: lidar
            contract_id: robonix/primitive/lidar/lidar
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: device_uptime
          resolution_ns: 1
          uncertainty_ns: 1
        strategy:
          kind: affine_model
          observation:
            reference_at: host_receive_time
            sampling_interval_ns: 1
            uncertainty_ns: 1
          policy:
            minimum_observations: 2
            observation_window: 4
            max_drift_ppm: 200
            max_residual_ns: 1000
            tracking_timeout_ns: 1000000000
            holdover_duration_ns: 1000000000
            max_uncertainty_ns: 1000000
"#,
    )
    .unwrap()
}

#[test]
/// Identity fields are all-or-none and field paths cannot contain expressions or whitespace.
fn collector_configuration_is_strict() {
    let mut config = test_config();
    let binding = &mut config
        .chronos
        .core
        .domains
        .get_mut("device")
        .unwrap()
        .capabilities[0];
    binding.session_id_field = Some("capture.session".into());
    assert!(config.chronos.core.validate().is_err());
    let binding = &mut config
        .chronos
        .core
        .domains
        .get_mut("device")
        .unwrap()
        .capabilities[0];
    binding.device_generation_field = Some("capture.generation".into());
    binding.sequence_id_field = Some("capture.sequence".into());
    assert!(config.chronos.core.validate().is_ok());
    for invalid in [
        "",
        "header..stamp",
        "header.stamp()",
        " header.stamp",
        "1stamp",
    ] {
        config
            .chronos
            .core
            .domains
            .get_mut("device")
            .unwrap()
            .capabilities[0]
            .timestamp_field = invalid.into();
        assert!(config.chronos.core.validate().is_err(), "{invalid}");
    }
    assert!(
        config::parse_manifest(r#"{"collect_observations":true}"#)
            .unwrap()
            .collect_observations
            .unwrap()
    );
}

#[test]
/// The shipped first-body example enables collection with a valid existing camera contract.
fn minecraft_collection_example_is_valid() {
    let config =
        ChronosConfig::from_yaml(include_str!("../../examples/minecraft-camera.yaml")).unwrap();
    assert!(config.chronos.collect_observations);
    assert!(config.chronos.collect_ros_observations);
    assert_eq!(
        config.chronos.core.domains["minecraft_realsense"].capabilities[0].provider_id,
        "minecraft_mod"
    );
}

#[test]
/// Manifest switches override YAML independently and survive a replacement core mapping.
fn collection_switches_follow_configuration_precedence() {
    use clap::Parser;
    let file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(file.path(), "chronos:\n  collect_observations: true\n  collect_ros_observations: true\n  core:\n    reference:\n      kind: linux_system\n").unwrap();
    for (manifest, master, ros) in [
        (
            r#"{"core":{"reference":{"kind":"linux_system"}}}"#,
            true,
            true,
        ),
        (r#"{"collect_ros_observations":false}"#, true, false),
        (r#"{"collect_observations":false}"#, false, true),
        (
            r#"{"collect_observations":false,"collect_ros_observations":false,"core":{"reference":{"kind":"linux_system"}}}"#,
            false,
            false,
        ),
    ] {
        let args = config::Args::try_parse_from([
            "robonix-chronos",
            "--config",
            file.path().to_str().unwrap(),
            "--config-json",
            manifest,
        ])
        .unwrap();
        let resolved = config::resolve(args).unwrap();
        assert_eq!(resolved.chronos.chronos.collect_observations, master);
        assert_eq!(resolved.chronos.chronos.collect_ros_observations, ros);
    }
    let args = config::Args::try_parse_from([
        "robonix-chronos",
        "--config-json",
        r#"{"collect_observations":true}"#,
    ])
    .unwrap();
    let resolved = config::resolve(args).unwrap();
    assert!(resolved.chronos.chronos.collect_observations);
    assert!(!resolved.chronos.chronos.collect_ros_observations);
}

#[tokio::test]
/// Real Atlas and tonic clients verify lifecycle, latest snapshots, and slow-reader isolation.
async fn grpc_streams_reconnect_and_lifecycle_preserves_domain_identity() {
    let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let atlas_addr = reserved.local_addr().unwrap();
    drop(reserved);
    let atlas_server = tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
    let mut atlas =
        AtlasClient::connect_with_retry(atlas_addr.to_string(), 50, Duration::from_millis(10))
            .await
            .unwrap();
    let id = "chronos-grpc-test";
    atlas
        .register_service(id, "robonix/system/chronos", "")
        .await
        .unwrap();
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap();
    declare(&mut atlas, id, &address.to_string()).await.unwrap();
    let svc = ChronosService::new(test_config(), atlas.clone(), id.into());
    let shutdown = svc.shutdown_receiver();
    let server_svc = svc.clone();
    let server = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(server_svc.clone()))
            .add_service(RobonixSystemChronosQualityServer::new(server_svc.clone()))
            .add_service(RobonixSystemChronosConversionModelsServer::new(server_svc))
            .serve_with_incoming_shutdown(TcpListenerStream::new(listener), wait_shutdown(shutdown))
            .await
    });
    let endpoint = format!("http://{address}");
    let mut driver = RobonixLifecycleDriverClient::connect(endpoint.clone())
        .await
        .unwrap();
    let mut quality = RobonixSystemChronosQualityClient::connect(endpoint.clone())
        .await
        .unwrap();
    assert_eq!(
        quality.quality(()).await.unwrap_err().code(),
        tonic::Code::Unavailable
    );
    let before_init = driver
        .driver(DriverRequest {
            command: 1,
            config_json: String::new(),
        })
        .await
        .unwrap()
        .into_inner();
    assert!(!before_init.ok);
    for command in [0, 1] {
        assert!(
            driver
                .driver(DriverRequest {
                    command,
                    config_json: "{}".into()
                })
                .await
                .unwrap()
                .into_inner()
                .ok
        );
    }
    let mut slow = quality.quality(()).await.unwrap().into_inner();
    let first = timeout(Duration::from_secs(2), slow.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert!(first.report_available);
    assert!(!first.reference.as_ref().unwrap().verified);
    assert_eq!(
        first.affine_domains[0].state,
        DomainState::Initializing as u32
    );
    let identity = (
        first.affine_domains[0].generation,
        first.affine_domains[0].reference_generation,
    );
    let mut models = RobonixSystemChronosConversionModelsClient::connect(endpoint.clone())
        .await
        .unwrap();
    let mut model_stream = models.conversion_models(()).await.unwrap().into_inner();
    let initial_model = timeout(Duration::from_secs(2), model_stream.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert!(!initial_model.domains[0].has_model);
    // Leave one stream unread while the sole core continues refreshing.
    tokio::time::sleep(Duration::from_millis(60)).await;
    let mut reconnected = quality.quality(()).await.unwrap().into_inner();
    let latest = timeout(Duration::from_secs(2), reconnected.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert!(latest.observed_steady_ns > first.observed_steady_ns);
    drop(model_stream);
    drop(reconnected);
    assert!(
        driver
            .driver(DriverRequest {
                command: 2,
                config_json: String::new()
            })
            .await
            .unwrap()
            .into_inner()
            .ok
    );
    assert_eq!(
        quality.quality(()).await.unwrap_err().code(),
        tonic::Code::Unavailable
    );
    assert!(
        driver
            .driver(DriverRequest {
                command: 1,
                config_json: String::new()
            })
            .await
            .unwrap()
            .into_inner()
            .ok
    );
    // Keep the old reader across a rapid deactivate/reactivate sequence.
    loop {
        match timeout(Duration::from_secs(2), slow.message())
            .await
            .unwrap()
        {
            Err(status) => {
                assert_eq!(status.code(), tonic::Code::Unavailable);
                break;
            }
            Ok(None) => panic!("old stream must terminate with inactive status"),
            Ok(Some(_)) => {}
        }
    }
    let mut active = quality.quality(()).await.unwrap().into_inner();
    let reactivated = active.message().await.unwrap().unwrap();
    assert_eq!(
        (
            reactivated.affine_domains[0].generation,
            reactivated.affine_domains[0].reference_generation
        ),
        identity
    );
    let providers = atlas
        .query_services(id, "", "", atlas_pb::Transport::Grpc)
        .await
        .unwrap();
    assert_eq!(
        providers[0].state,
        atlas_pb::LifecycleState::StateActive as i32
    );
    drop(active);
    assert!(
        driver
            .driver(DriverRequest {
                command: 3,
                config_json: String::new()
            })
            .await
            .unwrap()
            .into_inner()
            .ok
    );
    timeout(Duration::from_secs(2), server)
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    svc.close().await.unwrap();
    atlas.unregister(id).await.unwrap();
    // Local shutdown must complete even if Atlas rejects the lifecycle report.
    atlas
        .register_service("chronos-shutdown-failure", "robonix/system/chronos", "")
        .await
        .unwrap();
    declare(&mut atlas, "chronos-shutdown-failure", &address.to_string())
        .await
        .unwrap();
    let failing = ChronosService::new(
        test_config(),
        atlas.clone(),
        "chronos-shutdown-failure".into(),
    );
    for command in [0, 1] {
        failing
            .transition(DriverRequest {
                command,
                config_json: "{}".into(),
            })
            .await
            .unwrap();
    }
    let shutdown = failing.shutdown_receiver();
    // Lose registration as an Atlas restart would; aborting the listener alone
    // can leave tonic's already accepted HTTP/2 connections serving requests.
    atlas.unregister("chronos-shutdown-failure").await.unwrap();
    atlas_server.abort();
    let _ = atlas_server.await;
    let result = timeout(
        Duration::from_secs(6),
        failing.transition(DriverRequest {
            command: 3,
            config_json: String::new(),
        }),
    )
    .await
    .unwrap();
    assert!(result.is_err());
    assert!(*shutdown.borrow());
    failing.close().await.unwrap();
}
