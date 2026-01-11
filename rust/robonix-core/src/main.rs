// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Core Main Entry
//
// Main entry point for robonix-core service

use log::{debug, info};
use robonix_core::core::RobonixCore;
use robonix_core::logging::init_logger_with_buffer;
use robonix_core::node::create_node;
use robonix_core::server::{create_qos, create_servers, run_servers};
use robonix_core::web_gui::{
    LogBuffer, create_web_gui_state, image_handler, image_topics_handler, index, logs_handler,
    primitives_handler, semantic_map_handler, services_handler, skills_handler, status_handler,
    tasks_handler, tf_tree_handler, topics_handler,
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
    let rt = tokio::runtime::Runtime::new().expect("Failed to create Tokio runtime");

    // Create ROS2 node first (before creating RobonixCore)
    // Context must be kept alive for the node to work
    let (node, context) = create_node();
    let _context_arc = Arc::new(context); // Keep context alive
    let node_arc = Arc::new(Mutex::new(node));
    let service_qos = create_qos();
    info!("robonix core node started");

    // Initialize core within runtime context (spawns background tasks)
    // Pass the node to RobonixCore so TaskManager can use it
    let core = rt.block_on(async {
        let core = Arc::new(RobonixCore::new(node_arc.clone()));
        info!("robonix core initialized");
        core
    });

    // Get the node from the Arc for use in servers
    let servers = rt.block_on(async {
        let mut node_guard = node_arc.lock().await;
        match create_servers(&mut *node_guard, &service_qos) {
            Ok(servers) => servers,
            Err(e) => {
                eprintln!("failed to create servers: {}", e);
                std::process::exit(1);
            }
        }
    });

    info!("all robonix modules initialized");

    // Create TF monitor and start monitoring
    let tf_monitor = Arc::new(robonix_core::tf_monitor::TfMonitor::new());

    // Start TF monitoring in background
    let tf_monitor_clone = tf_monitor.clone();
    let node_for_tf = node_arc.clone();
    rt.spawn(async move {
        let mut node_guard = node_for_tf.lock().await;
        if let Err(e) = tf_monitor_clone.start_monitoring(&mut *node_guard).await {
            eprintln!("Failed to start TF monitoring: {}", e);
        }
    });

    // Create topic monitor and start monitoring
    let topic_monitor = Arc::new(robonix_core::topic_monitor::TopicMonitor::new());
    let topic_monitor_clone = topic_monitor.clone();
    let node_for_topics = node_arc.clone();
    rt.spawn(async move {
        let mut node_guard = node_for_topics.lock().await;
        if let Err(e) = topic_monitor_clone.start_monitoring(&mut *node_guard).await {
            eprintln!("Failed to start topic monitoring: {}", e);
        }
    });

    // Create image monitor
    let image_storage_dir = std::path::PathBuf::from("/tmp/robonix_images");
    let image_monitor = Arc::new(robonix_core::image_monitor::ImageMonitor::new(
        image_storage_dir,
    ));

    // Start image monitoring: discover image topics and subscribe to them
    let image_monitor_for_subscribe = image_monitor.clone();
    let topic_monitor_for_images = topic_monitor.clone();
    let node_for_images = node_arc.clone();
    rt.spawn(async move {
        let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(5));
        loop {
            interval.tick().await;

            // Get all topics from topic monitor
            let topics = topic_monitor_for_images.get_topics().await;

            // Find image topics (sensor_msgs/msg/Image)
            for topic in topics.topics {
                if topic.message_type.contains("sensor_msgs/msg/Image")
                    || topic.message_type.contains("Image")
                {
                    // Subscribe to image topic
                    let mut node_guard = node_for_images.lock().await;
                    if let Err(e) = image_monitor_for_subscribe
                        .register_image_topic(
                            &mut *node_guard,
                            topic.name.clone(),
                            topic.message_type.clone(),
                        )
                        .await
                    {
                        debug!("Failed to subscribe to image topic {}: {}", topic.name, e);
                    }
                }
            }
        }
    });

    // Check if both web GUI environment variables are set
    let static_dir_opt = std::env::var("ROBONIX_WEB_STATIC_DIR").ok();
    let port_opt = std::env::var("ROBONIX_WEB_PORT")
        .ok()
        .and_then(|p| p.parse::<u16>().ok());

    // Only start web GUI if both environment variables are set
    if let (Some(static_dir_str), Some(base_port)) = (static_dir_opt, port_opt) {
        let static_dir = std::path::PathBuf::from(static_dir_str);

        // Verify static directory exists
        if !static_dir.exists() || !static_dir.is_dir() {
            eprintln!(
                "Error: Static directory does not exist or is not a directory: {:?}",
                static_dir
            );
            eprintln!("Please set ROBONIX_WEB_STATIC_DIR to a valid directory path.");
            std::process::exit(1);
        }

        // Check if the specified port is available
        let port = if TcpListener::bind(("0.0.0.0", base_port)).is_ok() {
            base_port
        } else {
            eprintln!(
                "Error: Port {} is already in use. Please free the port or set ROBONIX_WEB_PORT to a different port.",
                base_port
            );
            std::process::exit(1);
        };

        // Create web GUI state
        let web_gui_state = create_web_gui_state(
            core.clone(),
            node_arc.clone(),
            tf_monitor.clone(),
            topic_monitor.clone(),
            log_buffer.clone(),
            image_monitor.clone(),
        );

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
                        topics_handler,
                        tasks_handler,
                        skills_handler,
                        services_handler,
                        primitives_handler,
                        logs_handler,
                        semantic_map_handler,
                        image_topics_handler,
                        image_handler,
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
    } else {
        info!("robonix core ready. waiting for requests...");
        info!("web GUI disabled (set ROBONIX_WEB_STATIC_DIR and ROBONIX_WEB_PORT to enable)");
    }

    // Run servers in Tokio runtime
    rt.block_on(run_servers(servers, core));
}
