// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Server PoC entry point

use log::info;
use robonix_server::generated::ALL_INTERFACES;
use robonix_server::meta_runtime::{MetaRuntimeRegistry, serve_meta_runtime};
use robonix_server::ping_query_runtime::PingQueryRuntime;
use std::sync::Arc;

#[tokio::main]
async fn main() {
    env_logger::init();

    let rmw = std::env::var("RMW_IMPLEMENTATION").unwrap_or_else(|_| "(default)".to_string());
    info!("RMW_IMPLEMENTATION={}", rmw);

    info!("robonix-server PoC starting");
    info!(
        "generated interface catalog loaded: {} interfaces",
        ALL_INTERFACES.len()
    );

    let grpc_addr =
        std::env::var("ROBONIX_META_GRPC_ADDR").unwrap_or_else(|_| "0.0.0.0:50051".to_string());
    let grpc_listen_addr: std::net::SocketAddr = grpc_addr
        .parse()
        .unwrap_or_else(|_| "0.0.0.0:50051".parse().expect("valid default gRPC address"));
    let grpc_advertised_endpoint =
        std::env::var("ROBONIX_META_GRPC_ENDPOINT").unwrap_or_else(|_| grpc_addr.clone());
    let registry = Arc::new(MetaRuntimeRegistry::default());

    let grpc_task = tokio::spawn(serve_meta_runtime(
        registry.clone(),
        grpc_listen_addr,
        grpc_advertised_endpoint.clone(),
    ));

    let _ping_runtime = match PingQueryRuntime::start(&grpc_advertised_endpoint).await {
        Ok(runtime) => runtime,
        Err(e) => {
            eprintln!("failed to start ping query runtime: {e:?}");
            std::process::exit(1);
        }
    };

    info!("meta runtime listening on {}", grpc_addr);
    match grpc_task.await {
        Ok(Ok(())) => {}
        Ok(Err(e)) => {
            eprintln!("robonix meta runtime gRPC server error: {}", e);
            std::process::exit(1);
        }
        Err(e) => {
            eprintln!("robonix meta runtime task join error: {}", e);
            std::process::exit(1);
        }
    }
}
