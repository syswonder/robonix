// SPDX-License-Identifier: MulanPSL-2.0
//
// robonix-vitals — health monitoring: power state, component health, threshold alerts.
// On startup vitals:
//   1. Connects to atlas, registers as `vitals`.
//   2. Declares two gRPC capabilities:
//      - robonix/service/vitals/get    (rpc: GetVitals → VitalsSnapshot)
//      - robonix/service/vitals/stream (topic_out: StreamVitals → stream VitalsSnapshot)
//   3. Auto-discovers scripts/:
//      - board.py          → BoardCollector  (always, sysfs)
//      - *_body.py         → BodyCollector   (zero or more, auto-discovered)
//   4. Starts the collect loop and serves both gRPC services on `listen`.
//
// In v0.1 vitals spawns Python subprocesses to read hardware.  When Soma is
// ready, both board and body collectors become gRPC clients consuming Soma's
// unified interfaces — the data structures stay the same.
//
// No Driver lifecycle handshake required — vitals is stateless monitoring that
// starts reporting as soon as the gRPC server is up.

mod board;
mod body;
mod config;
mod normalize;
mod pb;
mod service;
mod subprocess;

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

/// All the state needed by the collect loop to restart dead subprocesses.
struct ScriptPaths {
    board: std::path::PathBuf,
    bodies: Vec<std::path::PathBuf>,
    python_bin: String,
}

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

    // Python binary — shared by all subprocesses.
    let python =
        std::env::var("ROBONIX_VITALS_BODY_PYTHON").unwrap_or_else(|_| "python3".to_string());
    let scripts_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts");

    // ── Auto-discover scripts ────────────────────────────────────────────
    // board.py is always required.  *_body.py scripts are auto-discovered;
    // each script reports its own body_type / model in the JSON response.

    let board_script = scripts_dir.join("board.py");

    // Scan for body scripts (*_body.py, excluding board.py itself).
    let mut body_script_paths: Vec<std::path::PathBuf> = Vec::new();
    if let Ok(entries) = std::fs::read_dir(&scripts_dir) {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name_str = name.to_string_lossy();
            if name_str.ends_with("_body.py") {
                body_script_paths.push(entry.path());
            }
        }
    }

    // ── Board collector (always) ────────────────────────────────────────

    let mut board_collector = board::BoardCollector::new(&board_script.to_string_lossy(), &python)
        .context("init board collector")?;
    info!("board collector ready");

    // ── Body collectors (auto-discovered) ───────────────────────────────

    let mut body_collectors: Vec<body::BodyCollector> = Vec::new();
    for path in &body_script_paths {
        match body::BodyCollector::new(&path.to_string_lossy(), &python) {
            Ok(bc) => {
                // Issue one collect to learn body_type / model.
                info!(
                    "body collector ready ({})",
                    path.file_name().unwrap_or_default().to_string_lossy()
                );
                body_collectors.push(bc);
            }
            Err(e) => {
                log::warn!(
                    "[vitals] body script '{}' init failed: {e:#}",
                    path.display()
                );
            }
        }
    }

    // Paths needed inside the collect loop for subprocess restart.
    let script_paths = ScriptPaths {
        board: board_script,
        bodies: body_script_paths,
        python_bin: python,
    };

    // ── Collect loop ────────────────────────────────────────────────────

    {
        let svc = svc.clone();
        let interval = Duration::from_millis(cfg.collect_interval_ms);
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(interval);
            tick.tick().await;
            loop {
                tick.tick().await;

                // ── Board health ──────────────────────────────────────
                let (power, readings) = board_collector.collect();
                if !board_collector.is_alive() {
                    log::warn!("[vitals] board script died, attempting restart");
                    if let Err(e) = board_collector.restart(
                        &script_paths.board.to_string_lossy(),
                        &script_paths.python_bin,
                    ) {
                        log::error!("[vitals] board restart failed: {e:#}");
                    }
                }

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

                // ── Body health ──────────────────────────────────────
                // In v0.1 there is at most one body; we take the first.
                let body = if body_collectors.is_empty() {
                    None
                } else {
                    let mut body_health = None;
                    for (i, bc) in body_collectors.iter_mut().enumerate() {
                        let bh = bc.collect();
                        if !bc.is_alive() {
                            let path = &script_paths
                                .bodies
                                .get(i)
                                .map(|p| p.to_string_lossy().to_string())
                                .unwrap_or_default();
                            log::warn!("[vitals] body script died, attempting restart");
                            if let Err(e) = bc.restart(path, &script_paths.python_bin) {
                                log::error!("[vitals] body restart failed: {e:#}");
                            }
                        }
                        if body_health.is_none() {
                            body_health = Some(bh);
                        }
                    }
                    body_health
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
