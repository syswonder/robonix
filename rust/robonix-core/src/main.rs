// SPDX-License-Identifier: MulanPSL-2.0
// Robonix Core Main Entry
//
// Main entry point for robonix-server service

use log::{debug, info, warn};
use robonix_server::agent::{Agent, AgentConfig as LLMAgentConfig};
use robonix_server::core::RobonixCore;
use robonix_server::logging::init_logger_with_buffer;
use robonix_server::node::create_nodes;
use robonix_server::server::{create_qos, create_servers, run_servers};
use robonix_server::web::{
    LogBuffer, NodeLogState, NodeRegistry, agent_chat_handler, agent_reset_handler,
    create_web_state, get_config_handler, image_handler, image_topics_handler, index,
    log_subscriptions_delete, log_subscriptions_get, log_subscriptions_post, logs_handler,
    node_log_get, node_log_post, node_status_post, nodes_handler, primitives_handler,
    semantic_map_handler, services_handler, settings_page, skills_handler, status_handler,
    stt_handler, task_cancel_handler, tasks_handler, tf_tree_handler, topics_handler, tts_handler,
    update_config_handler,
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

    info!("robonix server starting...");

    // Create Tokio runtime for async task execution
    let rt = tokio::runtime::Runtime::new().expect("Failed to create Tokio runtime");

    // Create two ROS2 nodes on separate contexts: core_api for /rbnx/* servers (ping, register, query, task submit, etc.),
    // core_task for TaskManager (semantic_map, task_plan, executor) and monitors (TF, topic, image).
    // Two contexts = two DDS/event loops so a long-running task never blocks API requests.
    let (api_node, task_node, api_context, task_context) = create_nodes();
    let _api_context = api_context; // Keep API context alive
    let _task_context = task_context; // Keep task context alive
    let api_node_arc = Arc::new(Mutex::new(api_node));
    let task_node_arc = Arc::new(Mutex::new(task_node));
    let service_qos = create_qos();
    info!("robonix server nodes started (core_api, core_task)");

    // Initialize core with task node (TaskManager uses it for semantic_map, task_plan, executor)
    let core = rt.block_on(async {
        let core = Arc::new(RobonixCore::new(task_node_arc.clone()));
        info!("robonix server initialized");
        core
    });

    // Create all /rbnx/* service servers on the API node (keeps API responsive)
    let servers = rt.block_on(async {
        let mut node_guard = api_node_arc.lock().await;
        match create_servers(&mut *node_guard, &service_qos) {
            Ok(servers) => servers,
            Err(e) => {
                eprintln!("failed to create servers: {}", e);
                std::process::exit(1);
            }
        }
    });

    info!("all robonix server modules initialized");

    // Create TF monitor and start monitoring (uses task node)
    let tf_monitor = Arc::new(robonix_server::perception::tf_monitor::TfMonitor::new());

    // Start TF monitoring in background
    let tf_monitor_clone = tf_monitor.clone();
    let node_for_tf = task_node_arc.clone();
    rt.spawn(async move {
        let mut node_guard = node_for_tf.lock().await;
        if let Err(e) = tf_monitor_clone.start_monitoring(&mut *node_guard).await {
            eprintln!("Failed to start TF monitoring: {}", e);
        }
    });

    // Create topic monitor (no periodic task - topics discovered on-demand) (uses task node)
    let topic_monitor = Arc::new(robonix_server::perception::topic_monitor::TopicMonitor::new());
    // Initial discovery (synchronous, no spawn needed)
    let topic_monitor_init = topic_monitor.clone();
    rt.block_on(async {
        let mut node_guard = task_node_arc.lock().await;
        if let Err(e) = topic_monitor_init.start_monitoring(&mut *node_guard).await {
            eprintln!("Failed to start topic monitoring: {}", e);
        }
    });

    // Create image monitor (uses task node)
    let image_storage_dir = std::path::PathBuf::from("/tmp/robonix_images");
    let image_monitor =
        Arc::new(robonix_server::perception::image_monitor::ImageMonitor::new(image_storage_dir));

    // Start image monitoring: discover image topics and subscribe to them
    let image_monitor_for_subscribe = image_monitor.clone();
    let topic_monitor_for_images = topic_monitor.clone();
    let node_for_images = task_node_arc.clone();
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

    // Check if both web environment variables are set
    let web_dir_opt = std::env::var("ROBONIX_WEB_ASSETS_DIR").ok();
    let port_opt = std::env::var("ROBONIX_WEB_PORT")
        .ok()
        .and_then(|p| p.parse::<u16>().ok());

    // Only start web server if both environment variables are set
    if let (Some(web_dir_str), Some(base_port)) = (web_dir_opt, port_opt) {
        let web_dir = std::path::PathBuf::from(web_dir_str);

        // Verify web directory exists
        if !web_dir.exists() || !web_dir.is_dir() {
            eprintln!(
                "Error: Web directory does not exist or is not a directory: {:?}",
                web_dir
            );
            eprintln!(
                "Please set ROBONIX_WEB_ASSETS_DIR to the web directory path (e.g., /path/to/robonix-core/web)."
            );
            std::process::exit(1);
        }

        // Verify static subdirectory exists
        let static_dir = web_dir.join("static");
        if !static_dir.exists() || !static_dir.is_dir() {
            eprintln!(
                "Error: Static directory does not exist or is not a directory: {:?}",
                static_dir
            );
            eprintln!("Please ensure the web directory contains a 'static' subdirectory.");
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

        // Load config and create services
        let (agent_config, speech_config) = rt.block_on(async {
            use robonix_server::config::CoreConfig;
            match CoreConfig::load() {
                Ok(config) => (config.agent, config.speech),
                Err(e) => {
                    warn!("Failed to load core config, using defaults: {}", e);
                    (
                        LLMAgentConfig::default(),
                        robonix_server::config::SpeechConfig::default(),
                    )
                }
            }
        });

        let agent = Arc::new(tokio::sync::Mutex::new(Agent::new(
            core.clone(),
            agent_config,
            image_monitor.clone(),
        )));

        // Create TTS and STT services
        let tts_service = Arc::new(robonix_server::speech::TtsService::new(
            speech_config.clone(),
        ));
        let stt_service = Arc::new(robonix_server::speech::SttService::new(speech_config));

        let node_log_state = Arc::new(NodeLogState::new());
        let node_registry = Arc::new(NodeRegistry::new());

        // Create web state (node ref is for monitors; use task node)
        let web_state = create_web_state(
            core.clone(),
            task_node_arc.clone(),
            tf_monitor.clone(),
            topic_monitor.clone(),
            log_buffer.clone(),
            image_monitor.clone(),
            agent,
            tts_service,
            stt_service,
            web_dir.clone(),
            node_log_state,
            node_registry,
        );

        info!("starting web server on http://localhost:{}", port);

        // Start Rocket web server in a separate task
        let web_state_clone = web_state.clone();
        let static_dir_clone = static_dir.clone();
        let port_for_log = port;
        rt.spawn(async move {
            let config = rocket::Config::figment()
                .merge(("port", port))
                .merge(("address", "0.0.0.0"));

            let result = rocket::custom(config)
                .manage(web_state_clone)
                .mount(
                    "/",
                    routes![
                        index,
                        settings_page,
                        status_handler,
                        tf_tree_handler,
                        topics_handler,
                        tasks_handler,
                        task_cancel_handler,
                        skills_handler,
                        services_handler,
                        primitives_handler,
                        logs_handler,
                        log_subscriptions_post,
                        log_subscriptions_delete,
                        log_subscriptions_get,
                        node_log_post,
                        node_log_get,
                        node_status_post,
                        nodes_handler,
                        semantic_map_handler,
                        image_topics_handler,
                        image_handler,
                        get_config_handler,
                        update_config_handler,
                        agent_chat_handler,
                        agent_reset_handler,
                        tts_handler,
                        stt_handler,
                    ],
                )
                .mount("/static", FileServer::from(static_dir_clone))
                .launch()
                .await;

            if let Err(e) = result {
                eprintln!("Web server error: {}", e);
            }
        });

        info!("robonix core ready. waiting for requests...");
        info!("web available at http://localhost:{}", port_for_log);
    } else {
        info!("robonix core ready. waiting for requests...");
        info!("web disabled (set ROBONIX_WEB_ASSETS_DIR and ROBONIX_WEB_PORT to enable)");
    }

    // Run servers in Tokio runtime
    rt.block_on(run_servers(servers, core));
}
