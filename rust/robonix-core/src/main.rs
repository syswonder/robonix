use ansi_term::Colour;
use futures_util::stream::StreamExt;
use robonix_core::core::RobonixCore;
use robonix_core::primitive::primitive::{
    QueryPrimitiveRequest, QueryPrimitiveResponse, RegisterPrimitiveRequest,
    RegisterPrimitiveResponse,
};
use robonix_core::service::service::{
    QueryServiceRequest, QueryServiceResponse, RegisterServiceRequest, RegisterServiceResponse,
};
use robonix_core::skill_library::skill::{
    QuerySkillRequest, QuerySkillResponse, RegisterSkillRequest, RegisterSkillResponse,
};
use robonix_core::task_manager::{
    SubmitTaskRequest, SubmitTaskResponse, TaskResultRequest, TaskResultResponse,
    TaskStatusRequest, TaskStatusResponse,
};
use ros2_client::{
    AService, Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
    rustdds::{
        Duration, QosPolicies, QosPolicyBuilder,
        policy::{self, Deadline, Lifespan},
    },
};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::sync::Arc;
use tracing::{debug, error, info};
use tracing_subscriber::fmt::FmtContext;
use tracing_subscriber::fmt::format::{FormatEvent, FormatFields, Writer};
use tracing_subscriber::registry::LookupSpan;

// Ping Pong service types for testing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingPongRequest {
    pub message: String,
    pub sequence: u64,
}

impl ros2_client::Message for PingPongRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingPongResponse {
    pub message: String,
    pub sequence: u64,
    pub timestamp: u64,
}

impl ros2_client::Message for PingPongResponse {}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
                // Default: only show info+ for robonix_core, error+ for rustdds
                tracing_subscriber::EnvFilter::new("robonix_core=info,rustdds=error")
            }),
        )
        .with_target(false)
        .event_format(Formatter)
        .init();

    info!("robonix core starting...");

    let mut node = create_node();
    let service_qos = create_qos();
    info!("robonix core node started");

    let core = Arc::new(RobonixCore::new());

    // Get robonix core components
    let task_manager = core.get_task_manager();
    let skill_library = core.get_skill_library();
    let service_registry = core.get_service_registry();
    let primitive_registry = core.get_primitive_registry();

    // ===== robonix API Services =====

    // Primitive API: /rbnx/prm/register
    let register_primitive_server = node
        .create_server::<AService<RegisterPrimitiveRequest, RegisterPrimitiveResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/prm", "register").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "RegisterPrimitive"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("primitive register service created at /rbnx/prm/register");

    // Primitive API: /rbnx/prm/query
    let query_primitive_server = node
        .create_server::<AService<QueryPrimitiveRequest, QueryPrimitiveResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/prm", "query").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "QueryPrimitive"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("primitive query service created at /rbnx/prm/query");

    // Service API: /rbnx/srv/register
    let register_service_server = node
        .create_server::<AService<RegisterServiceRequest, RegisterServiceResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv", "register").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "RegisterService"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("service register service created at /rbnx/srv/register");

    // Service API: /rbnx/srv/query
    let query_service_server = node
        .create_server::<AService<QueryServiceRequest, QueryServiceResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv", "query").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "QueryService"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("service query service created at /rbnx/srv/query");

    // Skill API: /rbnx/skl/register
    let register_skill_server = node
        .create_server::<AService<RegisterSkillRequest, RegisterSkillResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/skl", "register").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "RegisterSkill"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("skill register service created at /rbnx/skl/register");

    // Skill API: /rbnx/skl/query
    let query_skill_server = node
        .create_server::<AService<QuerySkillRequest, QuerySkillResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/skl", "query").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "QuerySkill"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("skill query service created at /rbnx/skl/query");

    // Task API: /rbnx/task/submit
    let submit_task_server = node
        .create_server::<AService<SubmitTaskRequest, SubmitTaskResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/task", "submit").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "SubmitTask"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("task submit service created at /rbnx/task/submit");

    // Task API: /rbnx/task/status
    let task_status_server = node
        .create_server::<AService<TaskStatusRequest, TaskStatusResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/task", "status").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "TaskStatus"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("task status service created at /rbnx/task/status");

    // Task API: /rbnx/task/result
    let task_result_server = node
        .create_server::<AService<TaskResultRequest, TaskResultResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/task", "result").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "TaskResult"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("task result service created at /rbnx/task/result");

    // Ping Pong API: /rbnx/ping
    let ping_pong_server = node
        .create_server::<AService<PingPongRequest, PingPongResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx", "ping").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "PingPong"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("ping pong service created at /rbnx/ping");

    info!("all robonix modules initialized");
    info!("robonix core ready. waiting for requests...");

    // Run all service handlers
    smol::block_on(async {
        // robonix core component clones
        let task_manager_clone1 = task_manager.clone();
        let task_manager_clone2 = task_manager.clone();
        let task_manager_clone3 = task_manager.clone();
        let skill_library_clone1 = skill_library.clone();
        let skill_library_clone2 = skill_library.clone();
        let service_registry_clone1 = service_registry.clone();
        let service_registry_clone2 = service_registry.clone();
        let primitive_registry_clone1 = primitive_registry.clone();
        let primitive_registry_clone2 = primitive_registry.clone();

        // Handle primitive registration requests
        let register_primitive_task = smol::spawn(async move {
            let stream = register_primitive_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            info!(primitive_name = %req.name, "received primitive registration request");
                            let resp = primitive_registry_clone1.register_primitive(req).await;
                            if let Err(e) = register_primitive_server.async_send_response(req_id, resp).await {
                                error!("send primitive registration response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive primitive registration request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle primitive query requests
        let query_primitive_task = smol::spawn(async move {
            let stream = query_primitive_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(primitive_name = %req.name, "received primitive query request");
                            let resp = primitive_registry_clone2.query_primitive(req).await;
                            if let Err(e) = query_primitive_server
                                .async_send_response(req_id, resp)
                                .await
                            {
                                error!("send primitive query response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive primitive query request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle service registration requests
        let register_service_task = smol::spawn(async move {
            let stream = register_service_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            info!(service_name = %req.name, "received service registration request");
                            let resp = service_registry_clone1.register_service(req).await;
                            if let Err(e) = register_service_server.async_send_response(req_id, resp).await {
                                error!("send service registration response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive service registration request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle service query requests
        let query_service_task = smol::spawn(async move {
            let stream = query_service_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(service_name = %req.name, "received service query request");
                            let resp = service_registry_clone2.query_service(req).await;
                            if let Err(e) =
                                query_service_server.async_send_response(req_id, resp).await
                            {
                                error!("send service query response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive service query request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle skill registration requests
        let register_skill_task = smol::spawn(async move {
            let stream = register_skill_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            info!(skill_name = %req.name, "received skill registration request");
                            let resp = skill_library_clone1.register_skill(req).await;
                            if let Err(e) = register_skill_server
                                .async_send_response(req_id, resp)
                                .await
                            {
                                error!("send skill registration response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive skill registration request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle skill query requests
        let query_skill_task = smol::spawn(async move {
            let stream = query_skill_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(skill_name = %req.name, "received skill query request");
                            let resp = skill_library_clone2.query_skill(req).await;
                            if let Err(e) =
                                query_skill_server.async_send_response(req_id, resp).await
                            {
                                error!("send skill query response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive skill query request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle task submit requests
        let submit_task_task = smol::spawn(async move {
            let stream = submit_task_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            info!(description = %req.description, "received task submit request");
                            let resp = task_manager_clone1.submit_task(req).await;
                            if let Err(e) =
                                submit_task_server.async_send_response(req_id, resp).await
                            {
                                error!("send task submit response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive task submit request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle task status requests
        let task_status_task = smol::spawn(async move {
            let stream = task_status_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(task_id = %req.task_id, "received task status request");
                            let resp = task_manager_clone2.get_task_status(req).await;
                            if let Err(e) =
                                task_status_server.async_send_response(req_id, resp).await
                            {
                                error!("send task status response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive task status request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle task result requests
        let task_result_task = smol::spawn(async move {
            let stream = task_result_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(task_id = %req.task_id, "received task result request");
                            let resp = task_manager_clone3.get_task_result(req).await;
                            if let Err(e) =
                                task_result_server.async_send_response(req_id, resp).await
                            {
                                error!("send task result response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive task result request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle ping pong requests
        let ping_pong_task = smol::spawn(async move {
            let stream = ping_pong_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(sequence = req.sequence, message = %req.message, "received ping request");
                            let timestamp = std::time::SystemTime::now()
                                .duration_since(std::time::UNIX_EPOCH)
                                .unwrap()
                                .as_millis() as u64;
                            let resp = PingPongResponse {
                                message: format!("pong: {}", req.message),
                                sequence: req.sequence,
                                timestamp,
                            };
                            if let Err(e) = ping_pong_server.async_send_response(req_id, resp).await {
                                error!("send ping pong response error: {e:?}");
                            }
                        }
                        Err(e) => error!("receive ping pong request error: {e:?}"),
                    }
                })
                .await;
        });

        // Wait for all tasks (all run indefinitely)
        futures_util::future::select_all(vec![
            Box::pin(register_primitive_task),
            Box::pin(query_primitive_task),
            Box::pin(register_service_task),
            Box::pin(query_service_task),
            Box::pin(register_skill_task),
            Box::pin(query_skill_task),
            Box::pin(submit_task_task),
            Box::pin(task_status_task),
            Box::pin(task_result_task),
            Box::pin(ping_pong_task),
        ])
        .await;
    });
}

fn create_qos() -> QosPolicies {
    let service_qos: QosPolicies = {
        QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 1000 })
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: Duration::from_millis(100),
            })
            .durability(policy::Durability::Volatile)
            .deadline(Deadline(Duration::INFINITE))
            .lifespan(Lifespan {
                duration: Duration::INFINITE,
            })
            .liveliness(policy::Liveliness::Automatic {
                lease_duration: Duration::INFINITE,
            })
            .build()
    };
    service_qos
}

fn create_node() -> Node {
    let context = Context::new().unwrap();
    context
        .new_node(
            NodeName::new("/rbnx", "core").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap()
}

struct Formatter;

impl<S, N> FormatEvent<S, N> for Formatter
where
    S: tracing::Subscriber + for<'a> LookupSpan<'a>,
    N: for<'a> FormatFields<'a> + 'static,
{
    fn format_event(
        &self,
        ctx: &FmtContext<'_, S, N>,
        mut writer: Writer<'_>,
        event: &tracing::Event<'_>,
    ) -> fmt::Result {
        // Get timestamp in Linux kernel style [seconds.microseconds]
        static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
        let start = START.get_or_init(std::time::Instant::now);
        let elapsed = start.elapsed();
        let secs = elapsed.as_secs();
        let micros = elapsed.subsec_micros();
        let timestamp = format!("[{}.{:06}]", secs, micros);

        // Get log level and format it as a single colored letter
        let level = *event.metadata().level();
        let (level_char, level_color) = match level {
            tracing::Level::ERROR => ('E', Colour::Red),
            tracing::Level::WARN => ('W', Colour::Yellow),
            tracing::Level::INFO => ('I', Colour::Green),
            tracing::Level::DEBUG => ('D', Colour::Cyan),
            tracing::Level::TRACE => ('T', Colour::White),
        };

        // Get process name
        let proc_name = std::env::current_exe()
            .ok()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
            .unwrap_or_else(|| "eaios-core".to_string());

        // Write Linux-style log entry: timestamp procname[pid]: LEVEL message
        write!(
            writer,
            "{} {}[{}]: {} ",
            timestamp,
            proc_name,
            std::process::id(),
            level_color.paint(level_char.to_string())
        )?;

        // Format the message and fields
        ctx.format_fields(writer.by_ref(), event)?;
        writeln!(writer)
    }
}
