// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Core Main Entry
//
// Main entry point for robonix-core service

use log::info;
use robonix_core::core::RobonixCore;
use robonix_core::logging::init_logger;
use robonix_core::node::create_node;
use robonix_core::server::{create_qos, create_servers, run_servers};
use std::sync::Arc;

fn main() {
    init_logger();

    info!("robonix core starting...");

    // Create Tokio runtime for async task execution
    // This must be created before RobonixCore::new() because TaskManager spawns background tasks
    let rt = tokio::runtime::Runtime::new().expect("Failed to create Tokio runtime");

    // Initialize core within runtime context (spawns background tasks)
    let core = rt.block_on(async {
        let core = Arc::new(RobonixCore::new());
        info!("robonix core initialized");
        core
    });

    let mut node = create_node();
    let service_qos = create_qos();
    info!("robonix core node started");

    let servers = match create_servers(&mut node, &service_qos) {
        Ok(servers) => servers,
        Err(e) => {
            eprintln!("failed to create servers: {}", e);
            std::process::exit(1);
        }
    };

    info!("all robonix modules initialized");
    info!("robonix core ready. waiting for requests...");

    // Run servers in Tokio runtime
    rt.block_on(run_servers(servers, core));
}
