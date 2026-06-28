// SPDX-License-Identifier: MulanPSL-2.0
//
// robonix-vitals — health monitoring: power state, component health, threshold alerts.
// On startup vitals:
//   1. Connects to atlas, registers as `vitals`.
//   2. Declares two gRPC capabilities:
//      - robonix/service/vitals/get    (rpc: GetVitals → VitalsSnapshot)
//      - robonix/service/vitals/stream (topic_out: StreamVitals → stream VitalsSnapshot)
//   3. Starts the collect loop:
//      - SysfsCollector: board-level (CPU/GPU/NVMe temp, voltage) via sysfs.
//      - BodyCollector (optional): body-level (joint motors) via Python SDK bridge.
//   4. Serves both gRPC services on `listen`.
//
// In v0.1 vitals reads hardware directly. When Soma is ready, both board and
// body collectors become gRPC clients consuming Soma's unified interfaces.
//
// No Driver lifecycle handshake required — vitals is stateless monitoring that
// starts reporting as soon as the gRPC server is up.

mod body;
mod collect;
mod config;
mod normalize;
mod pb;
mod service;

use anyhow::{Context, Result};
use clap::Parser;
use config::{Args, VITALS_NAMESPACE, VitalsConfig};
use log::info;
use pb::contracts::robonix_service_vitals_get_server::RobonixServiceVitalsGetServer;
use pb::contracts::robonix_service_vitals_stream_server::RobonixServiceVitalsStreamServer;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use service::VitalsServiceImpl;
use std::time::Duration;
use std::time::Instant;

#[tokio::main]
async fn main() -> Result<()> {
    let parsed = Args::parse();
    let log_filter = parsed
        .log
        .clone()
        .or_else(|| std::env::var("RUST_LOG").ok())
        .unwrap_or_else(|| "robonix_vitals=info".to_string());
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_filter)).init();

    let cfg = VitalsConfig::resolve(parsed)?;

    info!("connecting to atlas at {}", cfg.atlas_endpoint);
    let mut atlas =
        AtlasClient::connect_with_retry(&cfg.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect to atlas")?;

    atlas
        .register_service(&cfg.id, VITALS_NAMESPACE, "")
        .await?;
    info!("registered as '{}' under '{VITALS_NAMESPACE}'", cfg.id);

    let listen_addr: std::net::SocketAddr = cfg
        .listen
        .parse()
        .with_context(|| format!("invalid vitals listen address '{}'", cfg.listen))?;
    let advertised = match listen_addr.ip() {
        std::net::IpAddr::V4(ip) if ip.is_unspecified() => {
            format!("127.0.0.1:{}", listen_addr.port())
        }
        _ => listen_addr.to_string(),
    };

    // Declare GetVitals RPC.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/service/vitals/get",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/service/vitals/get.v1.toml",
                "robonix.contracts.RobonixServiceVitalsGet",
                "/robonix.contracts.RobonixServiceVitalsGet/GetVitals",
            ),
        )
        .await?;
    info!("declared GetVitals at {advertised}");

    // Declare StreamVitals server_stream.
    // topic_out mode → gRPC server_stream. The method name matches the .srv basename.
    atlas
        .declare_capability(
            &cfg.id,
            "robonix/service/vitals/stream",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/service/vitals/stream.v1.toml",
                "robonix.contracts.RobonixServiceVitalsStream",
                "/robonix.contracts.RobonixServiceVitalsStream/StreamVitals",
            ),
        )
        .await?;
    info!("declared StreamVitals at {advertised}");

    if let Err(e) = atlas
        .set_lifecycle_state(&cfg.id, atlas_pb::LifecycleState::StateActive, "")
        .await
    {
        log::warn!("SetLifecycleState(ACTIVE) failed: {e:#}");
    }

    // Heartbeat every 20s to prevent Atlas eviction (90s timeout).
    {
        let mut hb = atlas.clone();
        let provider_id = cfg.id.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(&provider_id).await {
                    log::warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    // Build the shared service state.
    let svc = VitalsServiceImpl::new();

    // Load threshold rules (optional — falls back to basic collection).
    let rules: Vec<normalize::ThresholdRule> = match std::fs::read_to_string(&cfg.thresholds_path) {
        Ok(yaml_str) => match normalize::load_thresholds(&yaml_str) {
            Ok(r) => {
                info!(
                    "loaded {} threshold rules from {}",
                    r.len(),
                    cfg.thresholds_path.display()
                );
                r
            }
            Err(e) => {
                log::warn!(
                    "failed to parse threshold file '{}': {e:#}",
                    cfg.thresholds_path.display()
                );
                vec![]
            }
        },
        Err(e) => {
            log::warn!(
                "threshold file '{}' not readable: {e:#} — using basic collection",
                cfg.thresholds_path.display()
            );
            vec![]
        }
    };

    // In v0.1 vitals reads sysfs directly. When Soma is ready, this will
    // switch to Soma's unified health contract via Atlas discovery.
    let collector = collect::SysfsCollector::new().context("init sysfs collector")?;
    info!("sysfs collector ready");

    // Optional body collector — only spawned when --body-type is set.
    let mut body_collector: Option<body::BodyCollector> = None;
    if let (Some(body_type), Some(body_model)) = (&cfg.body_type, &cfg.body_model) {
        let script_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("scripts")
            .join(format!("{body_model}_body.py"));
        let python =
            std::env::var("ROBONIX_VITALS_BODY_PYTHON").unwrap_or_else(|_| "python3".to_string());
        match body::BodyCollector::new(
            body_type,
            body_model,
            &script_path.to_string_lossy(),
            &python,
        ) {
            Ok(bc) => {
                body_collector = Some(bc);
                info!("body collector ready ({body_type}/{body_model})");
            }
            Err(e) => log::warn!("[vitals] body collector init failed: {e:#}"),
        }
    }

    // Spawn the collect loop.
    {
        let svc = svc.clone();
        let interval = Duration::from_millis(cfg.collect_interval_ms);
        let script_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts");
        let python =
            std::env::var("ROBONIX_VITALS_BODY_PYTHON").unwrap_or_else(|_| "python3".to_string());
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(interval);
            tick.tick().await;
            loop {
                tick.tick().await;
                let (power, readings) = collector.collect();

                let mut components = Vec::new();
                if !rules.is_empty() {
                    for rule in &rules {
                        let reading = readings.iter().find(|r| r.name == rule.name).cloned();
                        if let Some(ref r) = reading {
                            components.push(normalize::evaluate(r, rule));
                        } else {
                            components.push(crate::pb::vitals::ComponentHealth {
                                name: rule.name.clone(),
                                health: normalize::HEALTH_OK,
                                detail: String::new(),
                                value: -1.0,
                                threshold: -1.0,
                            });
                        }
                    }
                }

                // Collect body health (if a body collector is active).
                let body = match &mut body_collector {
                    Some(bc) => {
                        let bh = bc.collect();
                        // If the subprocess died, try to restart once.
                        if !bc.is_alive() {
                            log::warn!("[vitals] body script died, attempting restart");
                            if let Some(bm) = &cfg.body_model {
                                let name = format!("{bm}_body.py");
                                let path = script_path.join(&name);
                                if let Err(e) = bc.restart(&path.to_string_lossy(), &python) {
                                    log::error!("[vitals] body restart failed: {e:#}");
                                }
                            }
                        }
                        Some(bh)
                    }
                    None => None,
                };

                let snapshot = crate::pb::vitals::VitalsSnapshot {
                    ts_ns: monotonic_ns(),
                    power: Some(power),
                    components,
                    body,
                };

                svc.update_snapshot(snapshot).await;
            }
        });
    }

    info!("Vitals gRPC on {listen_addr}");
    eprintln!("robonix-vitals ready on {listen_addr}");

    // Serve both contracts on the same port.
    tonic::transport::Server::builder()
        .add_service(RobonixServiceVitalsGetServer::new(svc.clone()))
        .add_service(RobonixServiceVitalsStreamServer::new(svc))
        .serve(listen_addr)
        .await
        .context("vitals gRPC server failed")?;

    Ok(())
}

fn monotonic_ns() -> i64 {
    Instant::now().elapsed().as_nanos() as i64
}
