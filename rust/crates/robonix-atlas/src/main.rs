// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>

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

    // ROBONIX_ATLAS_LISTEN — what host:port Atlas binds its gRPC service on.
    // Default 0.0.0.0:50051. Caps register here; consumers query here.
    let grpc_addr =
        std::env::var("ROBONIX_ATLAS_LISTEN").unwrap_or_else(|_| "0.0.0.0:50051".to_string());
    let grpc_listen_addr: std::net::SocketAddr = grpc_addr
        .parse()
        .unwrap_or_else(|_| "0.0.0.0:50051".parse().expect("valid default gRPC address"));
    let registry = Arc::new(AtlasRegistry::default());

    info!("atlas gRPC on {}", grpc_addr);

    if let Err(e) = serve_atlas(registry, grpc_listen_addr).await {
        eprintln!("robonix-atlas error: {e:?}");
        std::process::exit(1);
    }
}
