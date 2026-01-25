// SPDX-License-Identifier: MulanPSL-2.0
// Server Module
//
// Handles ROS2 service server creation and request handling

use crate::core::RobonixCore;
use crate::ros_idl::primitive::{
    QueryPrimitiveRequest, QueryPrimitiveResponse, RegisterPrimitiveRequest,
    RegisterPrimitiveResponse,
};
use crate::ros_idl::service_registry::{
    QueryServiceRequest, QueryServiceResponse, RegisterServiceRequest, RegisterServiceResponse,
};
use crate::ros_idl::skill::{
    QuerySkillRequest, QuerySkillResponse, RegisterSkillRequest, RegisterSkillResponse,
};
use crate::ros_idl::task::{
    SubmitTaskRequest, SubmitTaskResponse, TaskDataRequest, TaskDataResponse,
};
use crate::ros_idl::test::{PingPongRequest, PingPongResponse};
use futures_util::stream::StreamExt;
use log::{error, info, warn};
use ros2_client::{
    AService, Name, Node, Server, ServiceMapping, ServiceTypeName,
    rustdds::{
        Duration, QosPolicies, QosPolicyBuilder,
        policy::{self, Deadline, Lifespan},
    },
};
use std::sync::Arc;

/// Set thread to real-time priority with SCHED_FIFO policy
/// This ensures the thread has the highest priority and won't be preempted by other user threads
fn set_thread_realtime_priority() {
    #[cfg(target_os = "linux")]
    {
        use std::mem;
        unsafe {
            let mut param: libc::sched_param = mem::zeroed();
            // Set priority to maximum (99 is the highest for SCHED_FIFO)
            // Note: This requires CAP_SYS_NICE capability or running as root
            param.sched_priority = 99;

            let result =
                libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param);

            if result == 0 {
                info!("Thread set to SCHED_FIFO with priority 99 (highest priority)");
            } else {
                let errno = *libc::__errno_location();
                warn!(
                    "Failed to set thread to real-time priority (errno: {}). \
                    This may require root privileges or CAP_SYS_NICE capability. \
                    Thread will run with normal priority.",
                    errno
                );
            }
        }
    }

    #[cfg(not(target_os = "linux"))]
    {
        warn!("Real-time priority setting is only supported on Linux");
    }
}

/// Spawn a core service API handler on a dedicated high-priority thread
/// Each core ROS service API gets its own thread with real-time priority
fn spawn_core_api_thread<F, Fut>(name: &str, f: F)
where
    F: FnOnce() -> Fut + Send + 'static,
    Fut: std::future::Future<Output = ()> + Send + 'static,
{
    let name = name.to_string();
    let name_for_error = name.clone();
    std::thread::Builder::new()
        .name(format!("core-api-{}", name))
        .spawn(move || {
            // Set thread to real-time priority
            // set_thread_realtime_priority();

            // Create a new Tokio runtime for this thread
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect(&format!(
                    "Failed to create Tokio runtime for core API thread: {}",
                    name
                ));

            info!("Core API thread '{}' started", name);

            // Run the future
            rt.block_on(f());
        })
        .expect(&format!(
            "Failed to spawn core API thread: {}",
            name_for_error
        ));
}

pub fn create_qos() -> QosPolicies {
    // Match official ros2_client demo QoS settings
    QosPolicyBuilder::new()
        .history(policy::History::KeepLast { depth: 10 })
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
}

pub fn create_servers(
    node: &mut Node,
    service_qos: &QosPolicies,
) -> Result<Servers, Box<dyn std::error::Error>> {
    // Primitive API: /rbnx/prm/register
    let register_primitive_server =
        node.create_server::<AService<RegisterPrimitiveRequest, RegisterPrimitiveResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/prm", "register")?,
            &ServiceTypeName::new("robonix_sdk", "RegisterPrimitive"),
            service_qos.clone(),
            service_qos.clone(),
        )?;
    info!("primitive register service created at /rbnx/prm/register");

    // Primitive API: /rbnx/prm/query
    let query_primitive_server = node
        .create_server::<AService<QueryPrimitiveRequest, QueryPrimitiveResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/prm", "query")?,
            &ServiceTypeName::new("robonix_sdk", "QueryPrimitive"),
            service_qos.clone(),
            service_qos.clone(),
        )?;
    info!("primitive query service created at /rbnx/prm/query");

    // Service API: /rbnx/srv/register
    let register_service_server = node
        .create_server::<AService<RegisterServiceRequest, RegisterServiceResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv", "register")?,
            &ServiceTypeName::new("robonix_sdk", "RegisterService"),
            service_qos.clone(),
            service_qos.clone(),
        )?;
    info!("service register service created at /rbnx/srv/register");

    // Service API: /rbnx/srv/query
    let query_service_server = node
        .create_server::<AService<QueryServiceRequest, QueryServiceResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv", "query")?,
            &ServiceTypeName::new("robonix_sdk", "QueryService"),
            service_qos.clone(),
            service_qos.clone(),
        )?;
    info!("service query service created at /rbnx/srv/query");

    // Skill API: /rbnx/skl/register
    let register_skill_server = node
        .create_server::<AService<RegisterSkillRequest, RegisterSkillResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/skl", "register")?,
            &ServiceTypeName::new("robonix_sdk", "RegisterSkill"),
            service_qos.clone(),
            service_qos.clone(),
        )?;
    info!("skill register service created at /rbnx/skl/register");

    // Skill API: /rbnx/skl/query
    let query_skill_server = node
        .create_server::<AService<QuerySkillRequest, QuerySkillResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/skl", "query")?,
            &ServiceTypeName::new("robonix_sdk", "QuerySkill"),
            service_qos.clone(),
            service_qos.clone(),
        )?;
    info!("skill query service created at /rbnx/skl/query");

    // Task API: /rbnx/task/submit
    let submit_task_server = node
        .create_server::<AService<SubmitTaskRequest, SubmitTaskResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/task", "submit")?,
            &ServiceTypeName::new("robonix_sdk", "SubmitTask"),
            service_qos.clone(),
            service_qos.clone(),
        )?;
    info!("task submit service created at /rbnx/task/submit");

    // Task API: /rbnx/task/data
    let task_data_server = node.create_server::<AService<TaskDataRequest, TaskDataResponse>>(
        ServiceMapping::Enhanced,
        &Name::new("/rbnx/task", "data")?,
        &ServiceTypeName::new("robonix_sdk", "TaskData"),
        service_qos.clone(),
        service_qos.clone(),
    )?;
    info!("task data service created at /rbnx/task/data");

    // Ping Pong API: /rbnx/ping
    // WARNING: AService with custom Rust structs uses serde serialization,
    // which is incompatible with standard ROS2 clients that send CDR format.
    // This service will work with other ros2_client clients using AService,
    // but NOT with standard ROS2 clients (ros2 CLI, Python rclpy, C++ rclcpp).
    //
    // To support standard ROS2 clients, we would need to:
    // 1. Use generated ROS2 service types from .srv files (requires code generation)
    // 2. Or implement CDR serialization for custom message types
    //
    // For now, this service only works with robonix-cli (which also uses AService).
    let ping_pong_server = node.create_server::<AService<PingPongRequest, PingPongResponse>>(
        ServiceMapping::Enhanced,
        &Name::new("/rbnx", "ping")?,
        &ServiceTypeName::new("robonix_sdk", "PingPong"), // Matches robonix_sdk/srv/PingPong.srv
        service_qos.clone(),
        service_qos.clone(),
    )?;
    info!("ping pong service created at /rbnx/ping");

    Ok(Servers {
        register_primitive_server,
        query_primitive_server,
        register_service_server,
        query_service_server,
        register_skill_server,
        query_skill_server,
        submit_task_server,
        task_data_server,
        ping_pong_server,
    })
}

pub struct Servers {
    pub register_primitive_server:
        Server<AService<RegisterPrimitiveRequest, RegisterPrimitiveResponse>>,
    pub query_primitive_server: Server<AService<QueryPrimitiveRequest, QueryPrimitiveResponse>>,
    pub register_service_server: Server<AService<RegisterServiceRequest, RegisterServiceResponse>>,
    pub query_service_server: Server<AService<QueryServiceRequest, QueryServiceResponse>>,
    pub register_skill_server: Server<AService<RegisterSkillRequest, RegisterSkillResponse>>,
    pub query_skill_server: Server<AService<QuerySkillRequest, QuerySkillResponse>>,
    pub submit_task_server: Server<AService<SubmitTaskRequest, SubmitTaskResponse>>,
    pub task_data_server: Server<AService<TaskDataRequest, TaskDataResponse>>,
    pub ping_pong_server: Server<AService<PingPongRequest, PingPongResponse>>,
}

pub async fn run_servers(servers: Servers, core: Arc<RobonixCore>) {
    let task_manager = core.get_task_manager();
    let skill_library = core.get_skill_library();
    let service_registry = core.get_service_registry();
    let primitive_registry = core.get_primitive_registry();

    // Clone components for each handler
    let task_manager_clone1 = task_manager.clone();
    let task_manager_clone3 = task_manager.clone();
    let skill_library_clone1 = skill_library.clone();
    let skill_library_clone2 = skill_library.clone();
    let service_registry_clone1 = service_registry.clone();
    let service_registry_clone2 = service_registry.clone();
    let primitive_registry_clone1 = primitive_registry.clone();
    let primitive_registry_clone2 = primitive_registry.clone();

    // Handle primitive registration requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("prm-register", move || async move {
        let stream = servers.register_primitive_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!(
                            "received primitive [registration] request: primitive_name={}",
                            req.name
                        );
                        let resp = primitive_registry_clone1.register_primitive(req).await;
                        info!("sending primitive [registration] response: {resp:?}");
                        if let Err(e) = servers
                            .register_primitive_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send primitive [registration] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive primitive [registration] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle primitive query requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("prm-query", move || async move {
        let stream = servers.query_primitive_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!(
                            "received primitive [query] request: primitive_name={}",
                            req.name
                        );
                        let resp = primitive_registry_clone2.query_primitive(req).await;
                        info!("sending primitive [query] response: {resp:?}");
                        if let Err(e) = servers
                            .query_primitive_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send primitive [query] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive primitive [query] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle service registration requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("srv-register", move || async move {
        let stream = servers.register_service_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!(
                            "received service [registration] request: service_name={}",
                            req.name
                        );
                        let resp = service_registry_clone1.register_service(req).await;
                        info!("sending service [registration] response: {resp:?}");
                        if let Err(e) = servers
                            .register_service_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send service [registration] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive service [registration] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle service query requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("srv-query", move || async move {
        let stream = servers.query_service_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!(
                            "received service [query] request: service_name={}",
                            req.name
                        );
                        let resp = service_registry_clone2.query_service(req).await;
                        info!("sending service [query] response: {resp:?}");
                        if let Err(e) = servers
                            .query_service_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send service [query] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive service [query] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle skill registration requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("skl-register", move || async move {
        let stream = servers.register_skill_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!(
                            "received skill [registration] request: skill_name={}",
                            req.name
                        );
                        let resp = skill_library_clone1.register_skill(req).await;
                        info!("sending skill [registration] response: {resp:?}");
                        if let Err(e) = servers
                            .register_skill_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send skill [registration] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive skill [registration] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle skill query requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("skl-query", move || async move {
        let stream = servers.query_skill_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!("received skill [query] request: skill_name={}", req.name);
                        let resp = skill_library_clone2.query_skill(req).await;
                        info!("sending skill [query] response: {resp:?}");
                        if let Err(e) = servers
                            .query_skill_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send skill [query] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive skill [query] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle task submit requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("task-submit", move || async move {
        let stream = servers.submit_task_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!(
                            "received task [submit] request: description={}",
                            req.description
                        );
                        let resp = task_manager_clone1.submit_task(req).await;
                        info!("sending task [submit] response: {resp:?}");
                        if let Err(e) = servers
                            .submit_task_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send task [submit] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive task [submit] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle task data requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("task-data", move || async move {
        let stream = servers.task_data_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!("received task [data] request: task_id={}", req.task_id);
                        let resp = task_manager_clone3.get_task_data(req).await;
                        info!("sending task [data] response: {resp:?}");
                        if let Err(e) = servers
                            .task_data_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send task [data] response error: {e:?}");
                        }
                    }
                    Err(e) => error!("receive task [data] request error: {e:?}"),
                }
            })
            .await;
    });

    // Handle ping pong requests - Core API on dedicated high-priority thread
    spawn_core_api_thread("ping", move || async move {
        let stream = servers.ping_pong_server.receive_request_stream();
        stream
            .for_each(|result| async {
                match result {
                    Ok((req_id, req)) => {
                        info!("received ping request: {req:?}");
                        let timestamp = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_millis() as u64;
                        let resp = PingPongResponse {
                            message: format!("pong: {}", req.message),
                            sequence: req.sequence,
                            timestamp,
                        };
                        info!("sending pong response: {resp:?}",);
                        if let Err(e) = servers
                            .ping_pong_server
                            .async_send_response(req_id, resp)
                            .await
                        {
                            error!("send pong response error: {e:?}");
                        }
                    }
                    Err(e) => {
                        error!("receive pong request error: {e:?}");
                    }
                }
            })
            .await;
    });

    // All core API handlers are now running on dedicated high-priority threads
    // Wait indefinitely (threads run independently)
    info!("All core API service handlers started on dedicated high-priority threads");
    std::future::pending::<()>().await;
}
