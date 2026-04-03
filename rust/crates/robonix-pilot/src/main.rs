// SPDX-License-Identifier: MulanPSL-2.0
// robonix-pilot — reasoning, planning, and session management

mod pilot_service;
mod pilot_wire;
mod planner;
mod session;
mod session_state;
mod skills;
mod vlm;

pub(crate) mod pilot {
    tonic::include_proto!("robonix.pilot");
}
pub(crate) mod executor {
    tonic::include_proto!("robonix.executor");
}

use anyhow::Result;
use log::info;
use pilot_service::PilotServiceImpl;
use robonix_sdk::RobonixClient;
use std::sync::Arc;
use tokio::sync::Mutex;

const PILOT_NODE_ID: &str = "com.robonix.runtime.pilot";

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("robonix_pilot=info"),
    )
    .init();

    let atlas_endpoint = {
        let raw = resolve_endpoint(
            &["ROBONIX_ATLAS_ENDPOINT", "ROBONIX_ATLAS"],
            "localhost:50051",
        );
        if raw.starts_with("http") {
            raw
        } else {
            format!("http://{}", raw)
        }
    };
    let executor_endpoint = resolve_endpoint(&["ROBONIX_EXECUTOR_ENDPOINT"], "localhost:50061");

    info!("connecting to Atlas at {}", atlas_endpoint);
    let mut sdk =
        RobonixClient::connect_with_retry(&atlas_endpoint, 10, std::time::Duration::from_secs(2))
            .await?;

    sdk.register_node(PILOT_NODE_ID, "robonix/sys/runtime/pilot", "service", "")
        .await?;
    info!("registered as '{}'", PILOT_NODE_ID);

    info!("discovering VLM service...");
    let vlm = vlm::VlmClient::discover(&mut sdk, PILOT_NODE_ID).await?;

    let listen_port: u16 = std::env::var("ROBONIX_PILOT_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50071);

    let listen_addr: std::net::SocketAddr = format!("0.0.0.0:{}", listen_port).parse()?;
    // Use 127.0.0.1 (not `localhost`) so gRPC clients don't resolve ::1 while we listen on IPv4 only.
    let advertised = format!("127.0.0.1:{}", listen_port);

    // Stable contract id for discovery (`rbnx chat` queries this exact path).
    sdk.declare_interface_full(
        PILOT_NODE_ID,
        "pilot",
        vec!["grpc".to_string()],
        serde_json::json!({"endpoint": advertised}).to_string(),
        listen_port as u32,
        "robonix/sys/runtime/pilot",
    )
    .await?;

    info!("PilotService gRPC on :{}", listen_port);
    eprintln!("robonix-pilot ready on :{listen_port} (executor={executor_endpoint})");

    let sdk = Arc::new(Mutex::new(sdk));
    let vlm = Arc::new(Mutex::new(vlm));

    let executor_http = {
        let raw = if executor_endpoint.starts_with("http") {
            executor_endpoint.clone()
        } else {
            format!("http://{}", executor_endpoint)
        };
        raw.replace("localhost", "127.0.0.1")
    };

    let svc = PilotServiceImpl::new(sdk, vlm, executor_http);

    tonic::transport::Server::builder()
        .add_service(pilot::pilot_service_server::PilotServiceServer::new(svc))
        .serve(listen_addr)
        .await?;

    Ok(())
}

fn resolve_endpoint(vars: &[&str], default: &str) -> String {
    for v in vars {
        if let Ok(val) = std::env::var(v) {
            if !val.is_empty() {
                return val;
            }
        }
    }
    default.to_string()
}
