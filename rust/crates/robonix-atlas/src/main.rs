// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>

use clap::Parser;
use log::info;
use robonix_atlas::service::{AtlasRegistry, serve_atlas};
use std::sync::Arc;

const DEFAULT_LISTEN: &str = "0.0.0.0:50051";

#[derive(Parser, Debug)]
#[command(name = "robonix-atlas", about = "Robonix Atlas — capability registry")]
struct Args {
    /// Address Atlas binds its gRPC service on (default 0.0.0.0:50051).
    #[arg(long, env = "ROBONIX_ATLAS_LISTEN")]
    listen: Option<String>,

    /// Log filter (env_logger syntax: `info`, `robonix_atlas=debug`, …).
    /// Default: `robonix_atlas=info`. Also reads `RUST_LOG` if set.
    #[arg(long)]
    log: Option<String>,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();

    let log_filter = args
        .log
        .or_else(|| std::env::var("RUST_LOG").ok())
        .unwrap_or_else(|| "robonix_atlas=info".to_string());
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_filter)).init();

    info!("robonix-atlas starting (control plane)");

    let listen = args
        .listen
        .unwrap_or_else(|| DEFAULT_LISTEN.to_string());
    let listen_addr: std::net::SocketAddr = match listen.parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("invalid --listen '{listen}': {e}");
            std::process::exit(1);
        }
    };
    let registry = Arc::new(AtlasRegistry::default());

    info!("atlas gRPC on {listen}");

    if let Err(e) = serve_atlas(registry, listen_addr).await {
        eprintln!("robonix-atlas error: {e:?}");
        std::process::exit(1);
    }
}
