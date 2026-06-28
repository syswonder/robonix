// SPDX-License-Identifier: MulanPSL-2.0
//
// robonix-vitals — health monitoring: power state, component health, threshold alerts.
// On startup vitals:
//   1. Connects to atlas, registers as `vitals`.
//   2. Declares two gRPC capabilities:
//      - robonix/service/vitals/get    (rpc: GetVitals → VitalsSnapshot)
//      - robonix/service/vitals/stream (topic_out: StreamVitals → stream VitalsSnapshot)
//   3. Spawns a single Python subprocess (collect.py) that reads sysfs and
//      optionally body SDK (Piper, etc.) and returns a unified JSON response.
//   4. Starts the collect loop and serves both gRPC services on `listen`.
//
// In v0.1 vitals spawns a Python subprocess to read hardware.  When Soma is
// ready, VitalsCollector becomes a gRPC client consuming Soma's unified
// interface — the data structures stay the same.
//
// No Driver lifecycle handshake required — vitals is stateless monitoring that
// starts reporting as soon as the gRPC server is up.

mod body_threshold;
mod collector;
mod config;
mod mock_soma;
mod normalize;
mod pb;
mod service;
mod soma_ingest;
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
use std::sync::OnceLock;
use std::time::Duration;
use std::time::Instant;

/// Script path + python binary for subprocess restart.
struct ScriptPaths {
    script: std::path::PathBuf,
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
    if cfg.mock_soma {
        return mock_soma::run_mock_soma(
            &cfg.atlas_endpoint,
            &cfg.mock_soma_id,
            &cfg.mock_soma_listen,
            &cfg.mock_soma_scenario,
            cfg.mock_soma_interval_ms,
        )
        .await;
    }

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

    // Body thresholds are used by the shared Vitals service for both Soma and
    // Python collector snapshots.
    body_threshold::load_config(&cfg.body_thresholds_path)?;

    // Build the shared service state.
    let svc = VitalsServiceImpl::new();

    let soma_rules: Vec<soma_ingest::SomaThresholdRule> =
        match std::fs::read_to_string(&cfg.thresholds_path) {
            Ok(yaml_str) => match soma_ingest::load_soma_thresholds(&yaml_str) {
                Ok(r) => {
                    info!(
                        "loaded {} Soma threshold rules from {}",
                        r.len(),
                        cfg.thresholds_path.display()
                    );
                    r
                }
                Err(e) => {
                    log::warn!(
                        "failed to parse Soma threshold file '{}': {e:#}; using defaults",
                        cfg.thresholds_path.display()
                    );
                    soma_ingest::default_thresholds()
                }
            },
            Err(_) => soma_ingest::default_thresholds(),
        };

    if let Some(mut stream) =
        soma_ingest::open_soma_stream(&mut atlas, &cfg.id, cfg.soma_endpoint.as_deref()).await?
    {
        let svc_for_stream = svc.clone();
        let rules_for_stream = soma_rules.clone();
        tokio::spawn(async move {
            loop {
                match stream.message().await {
                    Ok(Some(snapshot)) => {
                        let vitals = soma_ingest::snapshot_to_vitals(
                            &snapshot,
                            &rules_for_stream,
                            monotonic_ns(),
                        );
                        svc_for_stream.update_snapshot(vitals).await;
                    }
                    Ok(None) => {
                        log::warn!("[vitals] Soma StreamHealth ended");
                        break;
                    }
                    Err(e) => {
                        log::warn!("[vitals] Soma StreamHealth error: {e:#}");
                        break;
                    }
                }
            }
        });

        info!("Vitals gRPC on {listen_addr} (Soma input)");
        eprintln!("robonix-vitals ready on {listen_addr} (Soma input)");

        tonic::transport::Server::builder()
            .add_service(RobonixServiceVitalsGetServer::new(svc.clone()))
            .add_service(RobonixServiceVitalsStreamServer::new(svc))
            .serve(listen_addr)
            .await
            .context("vitals gRPC server failed")?;

        return Ok(());
    }

    // Python binary — shared by subprocess.
    let python = std::env::var("ROBONIX_VITALS_PYTHON")
        .or_else(|_| std::env::var("ROBONIX_VITALS_BODY_PYTHON"))
        .unwrap_or_else(|_| "python3".to_string());
    let scripts_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts");

    // ── Unified script ──────────────────────────────────────────────────
    // One script handles board (always) and body (optional, if hardware is
    // connected).  Override via ROBONIX_VITALS_SCRIPT env var.

    let collect_script = std::env::var("ROBONIX_VITALS_SCRIPT")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| scripts_dir.join("collect.py"));

    let mut collector = collector::VitalsCollector::new(&collect_script.to_string_lossy(), &python)
        .context("init vitals collector")?;
    info!("vitals collector ready");

    let script_paths = ScriptPaths {
        script: collect_script,
        python_bin: python,
    };

    // Load board threshold rules for the Python collector path.
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

    // ── Collect loop ────────────────────────────────────────────────────

    {
        let svc = svc.clone();
        let interval = Duration::from_millis(cfg.collect_interval_ms);
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(interval);
            tick.tick().await;
            loop {
                tick.tick().await;

                // ── Unified collect ─────────────────────────────────
                let (power, readings, bodies) = collector.collect();
                if !collector.is_alive() {
                    log::warn!("[vitals] script died, attempting restart");
                    if let Err(e) = collector.restart(
                        &script_paths.script.to_string_lossy(),
                        &script_paths.python_bin,
                    ) {
                        log::error!("[vitals] restart failed: {e:#}");
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

                let snapshot = crate::pb::vitals::VitalsSnapshot {
                    ts_ns: monotonic_ns(),
                    power: Some(power),
                    components,
                    bodies,
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
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as i64
}
