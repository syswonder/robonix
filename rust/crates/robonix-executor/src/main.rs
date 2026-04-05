// SPDX-License-Identifier: MulanPSL-2.0
// robonix-executor — tool-call dispatch runtime
//
// Exposes contract gRPC services from `robonix_contracts.proto` (SysRuntimeExecutor, …).

mod dispatch;
mod exec_wire;
mod routing_kind;
mod service;
mod tools;

use robonix_interfaces::{contracts, executor, pilot};

use anyhow::Result;
use contracts::sys_runtime_executor_list_tools_server::SysRuntimeExecutorListToolsServer;
use contracts::sys_runtime_executor_server::SysRuntimeExecutorServer;
use log::info;
use robonix_sdk::RobonixClient;
use service::ExecutorServiceImpl;
use std::sync::Arc;
use tokio::sync::Mutex;

const EXECUTOR_NODE_ID: &str = "com.robonix.runtime.executor";

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("robonix_executor=info"),
    )
    .init();

    let atlas_endpoint = atlas_endpoint();
    info!("connecting to Atlas at {}", atlas_endpoint);

    let mut sdk =
        RobonixClient::connect_with_retry(&atlas_endpoint, 10, std::time::Duration::from_secs(2))
            .await?;

    sdk.register_node(
        EXECUTOR_NODE_ID,
        "robonix/sys/runtime/executor",
        "service",
        "",
    )
    .await?;
    info!("registered as '{}'", EXECUTOR_NODE_ID);

    let listen_port: u16 = std::env::var("ROBONIX_EXECUTOR_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50061);

    let listen_addr: std::net::SocketAddr = format!("0.0.0.0:{}", listen_port).parse()?;
    let advertised = format!("localhost:{}", listen_port);

    sdk.declare_interface_full(
        EXECUTOR_NODE_ID,
        "executor",
        vec!["grpc".to_string()],
        serde_json::json!({"endpoint": advertised}).to_string(),
        listen_port as u32,
        "robonix/sys/runtime/executor",
    )
    .await?;

    info!("Executor contract gRPC on :{}", listen_port);

    let sdk = Arc::new(Mutex::new(sdk));
    let svc = ExecutorServiceImpl::new(sdk);

    tonic::transport::Server::builder()
        .add_service(SysRuntimeExecutorServer::new(svc.clone()))
        .add_service(SysRuntimeExecutorListToolsServer::new(svc))
        .serve(listen_addr)
        .await?;

    Ok(())
}

fn atlas_endpoint() -> String {
    let raw = resolve_endpoint(
        &["ROBONIX_ATLAS_ENDPOINT", "ROBONIX_ATLAS"],
        "localhost:50051",
    );
    if raw.starts_with("http") {
        raw
    } else {
        format!("http://{}", raw)
    }
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
