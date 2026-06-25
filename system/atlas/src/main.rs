// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>

use clap::Parser;
use robonix_atlas::contract_registry::{ContractRegistry, resolve_capabilities_roots};
use robonix_atlas::service::{AtlasRegistry, serve_atlas};
use robonix_scribe::{info, warn};
use std::sync::Arc;

const DEFAULT_LISTEN: &str = "0.0.0.0:50051";

#[derive(Parser, Debug)]
#[command(name = "robonix-atlas", about = "Robonix Atlas — capability registry")]
struct Args {
    /// Address Atlas binds its gRPC service on (default 0.0.0.0:50051).
    #[arg(long, env = "ROBONIX_ATLAS_LISTEN")]
    listen: Option<String>,

    /// Comma-separated list of directories atlas walks for contract
    /// TOMLs at startup. Each `*.toml` whose `[contract].id` is set
    /// becomes a `ContractDescriptor` served by `QueryContract` /
    /// `ListContracts`. Roots are merged in order: a duplicate id from
    /// a later root overrides earlier ones, which lets a per-package
    /// `<pkg>/capabilities/` re-declare a contract from the global
    /// `<robonix_source>/capabilities/`. When unset, atlas falls back
    /// to `$ROBONIX_SOURCE_PATH/capabilities`. When neither is set,
    /// the contract registry runs empty (clients see found=false on
    /// every QueryContract).
    #[arg(long, env = "ROBONIX_ATLAS_CAPABILITIES", value_delimiter = ',')]
    capabilities: Vec<String>,

    /// Log filter (env_logger syntax: `info`, `robonix_atlas=debug`, …).
    /// Default: `robonix_atlas=info`. Also reads `RUST_LOG` if set.
    #[arg(long)]
    log: Option<String>,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();

    // Log level is no longer consumed — Scribe writes everything and
    // filtering happens at read time (`rbnx logs --level`).  The CLI
    // arg and RUST_LOG env are kept for backward compat with existing
    // launch scripts.
    let _ = args.log.or_else(|| std::env::var("RUST_LOG").ok());

    robonix_scribe::init("atlas");
    info!("robonix-atlas starting (control plane)");

    let listen = args.listen.unwrap_or_else(|| DEFAULT_LISTEN.to_string());
    let listen_addr: std::net::SocketAddr = match listen.parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("invalid --listen '{listen}': {e}");
            std::process::exit(1);
        }
    };
    let roots = resolve_capabilities_roots(&args.capabilities);
    let contracts = if roots.is_empty() {
        warn!(
            "[atlas] contract registry: no capabilities dir configured \
             (pass --capabilities or set ROBONIX_SOURCE_PATH); QueryContract \
             will return found=false for every id"
        );
        ContractRegistry::default()
    } else {
        let refs: Vec<&std::path::Path> = roots.iter().map(|p| p.as_path()).collect();
        match ContractRegistry::load_from_capability_roots(&refs) {
            Ok(r) => r,
            Err(e) => {
                warn!(
                    "[atlas] contract registry: load failed ({e:#}); running with empty registry"
                );
                ContractRegistry::default()
            }
        }
    };
    let registry = Arc::new(AtlasRegistry::with_contracts(contracts));

    info!("atlas gRPC on {listen}");

    if let Err(e) = serve_atlas(registry, listen_addr).await {
        eprintln!("robonix-atlas error: {e:?}");
        std::process::exit(1);
    }
}
