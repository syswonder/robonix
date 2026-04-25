// SPDX-License-Identifier: MulanPSL-2.0

use log::info;
use robonix_atlas::service::{AtlasRegistry, serve_atlas};
use std::sync::Arc;

#[tokio::main]
async fn main() {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("robonix_atlas=info"),
    )
    .init();

    info!("robonix-atlas starting (control plane)");

    let grpc_addr =
        std::env::var("ROBONIX_META_GRPC_ADDR").unwrap_or_else(|_| "0.0.0.0:50051".to_string());
    let grpc_listen_addr: std::net::SocketAddr = grpc_addr
        .parse()
        .unwrap_or_else(|_| "0.0.0.0:50051".parse().expect("valid default gRPC address"));
    let grpc_advertised_endpoint =
        std::env::var("ROBONIX_META_GRPC_ENDPOINT").unwrap_or_else(|_| grpc_addr.clone());
    let registry = Arc::new(AtlasRegistry::default());

    info!("atlas gRPC on {}", grpc_addr);

    if let Err(e) = serve_atlas(registry, grpc_listen_addr, grpc_advertised_endpoint).await {
        eprintln!("robonix-atlas error: {e:?}");
        std::process::exit(1);
    }
}
