// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Core Main Entry
//
// Main entry point for robonix-core service

use log::info;
use robonix_core::core::RobonixCore;
use robonix_core::logging::init_logger_with_buffer;
use robonix_core::node::create_node;
use robonix_core::server::{create_qos, create_servers, run_servers};
use robonix_core::web_gui::{
    LogBuffer, create_web_gui_state, index, logs_handler, primitives_handler, services_handler,
    skills_handler, status_handler, tasks_handler, tf_tree_handler,
};
use rocket::fs::FileServer;
use rocket::routes;
use std::net::TcpListener;
use std::sync::Arc;
use tokio::sync::Mutex;

fn main() {
    // Create log buffer first
    let log_buffer = Arc::new(LogBuffer::new(1000));

    // Initialize logger with buffer
    init_logger_with_buffer(Some(log_buffer.clone()));

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

    // Create TF monitor and start monitoring
    let tf_monitor = Arc::new(robonix_core::tf_monitor::TfMonitor::new());
    let node_arc = Arc::new(Mutex::new(node));

    // Start TF monitoring in background
    let tf_monitor_clone = tf_monitor.clone();
    let node_for_tf = node_arc.clone();
    rt.spawn(async move {
        let mut node_guard = node_for_tf.lock().await;
        if let Err(e) = tf_monitor_clone.start_monitoring(&mut *node_guard).await {
            eprintln!("Failed to start TF monitoring: {}", e);
        }
    });

    // Create web GUI state
    let web_gui_state = create_web_gui_state(core.clone(), node_arc, tf_monitor, log_buffer);

    // Get static directory path from environment or use default
    let static_dir = std::env::var("ROBONIX_WEB_STATIC_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| {
            // Default: relative to current working directory
            std::env::current_dir()
                .unwrap_or_else(|_| std::path::PathBuf::from("."))
                .join("web_gui")
                .join("static")
        });

    // Get port from environment or use default
    let base_port = std::env::var("ROBONIX_WEB_PORT")
        .ok()
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(8000);

    // Check if the specified port is available
    let port = if TcpListener::bind(("0.0.0.0", base_port)).is_ok() {
        base_port
    } else {
        eprintln!(
            "Error: Port {} is already in use. Please free the port or set ROBONIX_WEB_PORT to a different port.",
            base_port
        );
        eprintln!(
            "To find and kill the process using port {}, try one of these commands:",
            base_port
        );
        eprintln!("  fuser -k {}/tcp", base_port);
        eprintln!(
            "  ss -lptn 'sport = :{}' | grep -oP 'pid=\\K[0-9]+' | xargs kill -9",
            base_port
        );
        eprintln!(
            "  netstat -tlnp | grep ':{}' | awk '{{print $7}}' | cut -d'/' -f1 | xargs kill -9",
            base_port
        );
        std::process::exit(1);
    };

    info!("starting web GUI server on http://localhost:{}", port);

    // Start Rocket web server in a separate task
    let web_gui_state_clone = web_gui_state.clone();
    let static_dir_clone = static_dir.clone();
    let port_for_log = port;
    rt.spawn(async move {
        let config = rocket::Config::figment()
            .merge(("port", port))
            .merge(("address", "0.0.0.0"));

        let result = rocket::custom(config)
            .manage(web_gui_state_clone)
            .mount(
                "/",
                routes![
                    index,
                    status_handler,
                    tf_tree_handler,
                    tasks_handler,
                    skills_handler,
                    services_handler,
                    primitives_handler,
                    logs_handler,
                ],
            )
            .mount("/static", FileServer::from(static_dir_clone))
            .launch()
            .await;

        if let Err(e) = result {
            eprintln!("Web GUI server error: {}", e);
        }
    });

    info!("robonix core ready. waiting for requests...");
    info!("web GUI available at http://localhost:{}", port_for_log);

    // Run servers in Tokio runtime
    rt.block_on(run_servers(servers, core));
}
