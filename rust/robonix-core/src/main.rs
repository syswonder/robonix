use ansi_term::Colour;
use futures_util::stream::StreamExt;
use robonix_core::core::RobonixCore;
use robonix_core::messages::{PingRequest, PingResponse, QueryCapSklRequest, QueryCapSklResponse, RegisterCapSklRequest, RegisterCapSklResponse};
use robonix_core::mgmt::{QueryModelRequest, QueryModelResponse, RegisterModelRequest, RegisterModelResponse};
use robonix_core::perception::{
    AddEntityRequest, AddEntityResponse, AddSpatialMapEntryRequest, AddSpatialMapEntryResponse,
    GetMapStatusRequest, GetMapStatusResponse, GetSemanticMapRequest, GetSemanticMapResponse,
    GetSpatialMapRequest, GetSpatialMapResponse,
};
use robonix_core::planning::{
    CancelTaskRequest, CancelTaskResponse, CreateTaskRequest, CreateTaskResponse,
    GetTaskRequest, GetTaskResponse, ListTasksRequest, ListTasksResponse,
};
use ros2_client::{
    AService, Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
    rustdds::{
        Duration, QosPolicies, QosPolicyBuilder,
        policy::{self, Deadline, Lifespan},
    },
};
use std::fmt;
use std::sync::Arc;
use std::time::Instant;
#[allow(unused_imports)]
use tracing::{Level, debug, error, info, warn};
use tracing_subscriber::fmt::FmtContext;
use tracing_subscriber::fmt::format::{FormatEvent, FormatFields, Writer};
use tracing_subscriber::registry::LookupSpan;

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

    info!("Robonix Core starting...");

    let mut node = create_node();
    let service_qos = create_qos();
    info!("Robonix Core node started");

    let core = Arc::new(RobonixCore::new());
    
    // Get management module
    let mgmt = core.get_mgmt();
    
    // Create registration service for capabilities and skills
    let register_server = node
        .create_server::<AService<RegisterCapSklRequest, RegisterCapSklResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/mgmt", "register_cap_skl").unwrap(),
            &ServiceTypeName::new("robonix_core", "RegisterCapSkl"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();

    info!("Register capability/skill service created at /rbnx/srv/mgmt/register_cap_skl");

    // Create query service for capabilities and skills
    let query_server = node
        .create_server::<AService<QueryCapSklRequest, QueryCapSklResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/mgmt", "query_cap_skl").unwrap(),
            &ServiceTypeName::new("robonix_core", "QueryCapSkl"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();

    info!("Query capability/skill service created at /rbnx/srv/mgmt/query_cap_skl");

    // Create ping service for testing
    let ping_server = node
        .create_server::<AService<PingRequest, PingResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/mgmt", "ping").unwrap(),
            &ServiceTypeName::new("robonix_core", "Ping"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Ping service created at /rbnx/srv/mgmt/ping");

    // Get perception module
    let perception = core.get_perception();

    // Create perception services
    // Get semantic map service
    let get_semantic_map_server = node
        .create_server::<AService<GetSemanticMapRequest, GetSemanticMapResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/perception", "get_semantic_map").unwrap(),
            &ServiceTypeName::new("robonix_core", "GetSemanticMap"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Get semantic map service created at /rbnx/srv/perception/get_semantic_map");

    // Add entity service
    let add_entity_server = node
        .create_server::<AService<AddEntityRequest, AddEntityResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/perception", "add_entity").unwrap(),
            &ServiceTypeName::new("robonix_core", "AddEntity"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Add entity service created at /rbnx/srv/perception/add_entity");

    // Get spatial map service
    let get_spatial_map_server = node
        .create_server::<AService<GetSpatialMapRequest, GetSpatialMapResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/perception", "get_spatial_map").unwrap(),
            &ServiceTypeName::new("robonix_core", "GetSpatialMap"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Get spatial map service created at /rbnx/srv/perception/get_spatial_map");

    // Add spatial map entry service
    let add_spatial_map_entry_server = node
        .create_server::<AService<AddSpatialMapEntryRequest, AddSpatialMapEntryResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/perception", "add_spatial_map_entry").unwrap(),
            &ServiceTypeName::new("robonix_core", "AddSpatialMapEntry"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Add spatial map entry service created at /rbnx/srv/perception/add_spatial_map_entry");

    // Get map status service
    let get_map_status_server = node
        .create_server::<AService<GetMapStatusRequest, GetMapStatusResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/perception", "get_map_status").unwrap(),
            &ServiceTypeName::new("robonix_core", "GetMapStatus"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Get map status service created at /rbnx/srv/perception/get_map_status");

    info!("Perception module initialized");

    // Get planning and action modules
    let planning = core.get_planning();
    let action = core.get_action();

    // Create model registration service
    let register_model_server = node
        .create_server::<AService<RegisterModelRequest, RegisterModelResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/mgmt", "register_model").unwrap(),
            &ServiceTypeName::new("robonix_core", "RegisterModel"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Register model service created at /rbnx/srv/mgmt/register_model");

    // Create model query service
    let query_model_server = node
        .create_server::<AService<QueryModelRequest, QueryModelResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/mgmt", "query_model").unwrap(),
            &ServiceTypeName::new("robonix_core", "QueryModel"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Query model service created at /rbnx/srv/mgmt/query_model");

    // Create task services
    let create_task_server = node
        .create_server::<AService<CreateTaskRequest, CreateTaskResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/planning", "create_task").unwrap(),
            &ServiceTypeName::new("robonix_core", "CreateTask"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Create task service created at /rbnx/srv/planning/create_task");

    let get_task_server = node
        .create_server::<AService<GetTaskRequest, GetTaskResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/planning", "get_task").unwrap(),
            &ServiceTypeName::new("robonix_core", "GetTask"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Get task service created at /rbnx/srv/planning/get_task");

    let list_tasks_server = node
        .create_server::<AService<ListTasksRequest, ListTasksResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/planning", "list_tasks").unwrap(),
            &ServiceTypeName::new("robonix_core", "ListTasks"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("List tasks service created at /rbnx/srv/planning/list_tasks");

    let cancel_task_server = node
        .create_server::<AService<CancelTaskRequest, CancelTaskResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/srv/planning", "cancel_task").unwrap(),
            &ServiceTypeName::new("robonix_core", "CancelTask"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .unwrap();
    info!("Cancel task service created at /rbnx/srv/planning/cancel_task");

    info!("All modules initialized");
    info!("Robonix Core ready. Waiting for requests...");

    // run it!
    smol::block_on(async {
        let core_clone1 = core.clone();
        let core_clone2 = core.clone();
        let perception_clone1 = perception.clone();
        let perception_clone2 = perception.clone();
        let perception_clone3 = perception.clone();
        let perception_clone4 = perception.clone();
        let perception_clone5 = perception.clone();
        let mgmt_clone1 = mgmt.clone();
        let mgmt_clone4 = mgmt.clone();
        let mgmt_clone6 = mgmt.clone();
        let planning_clone1 = planning.clone();
        let planning_clone2 = planning.clone();
        let planning_clone3 = planning.clone();
        let planning_clone4 = planning.clone();
        let action_clone1 = action.clone();

        // Handle registration requests
        let register_task = smol::spawn(async move {
            let register_stream = register_server.receive_request_stream();
            register_stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            info!(
                                package = %req.package_name,
                                package_type = %req.package_type,
                                std_name = %req.std_name,
                                "Received registration request"
                            );
                            let resp = core_clone1.register(req).await;
                            let sr = register_server.async_send_response(req_id, resp).await;
                            if let Err(e) = sr {
                                error!("Send response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle query requests
        let query_task = smol::spawn(async move {
            let query_stream = query_server.receive_request_stream();
            query_stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            debug!(
                                std_name = %req.std_name,
                                "Received query request"
                            );
                            let resp = core_clone2.query(req).await;
                            let sr = query_server.async_send_response(req_id, resp).await;
                            if let Err(e) = sr {
                                error!("Send query response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive query request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle get semantic map requests
        let get_semantic_map_task = smol::spawn(async move {
            let stream = get_semantic_map_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let mut entities = Vec::new();
                            if let Some(entity_id) = &req.entity_id {
                                if let Some(entity) = perception_clone1.get_entity(entity_id).await {
                                    entities.push(entity);
                                }
                            } else if let Some(label) = &req.label {
                                entities = perception_clone1.find_entities_by_label(label).await;
                            } else if let Some(path) = &req.path {
                                entities = perception_clone1.find_entities_by_path(path).await;
                            } else {
                                // Get all entities
                                let semantic_map = perception_clone1.get_semantic_map();
                                let map = semantic_map.read().await;
                                entities = map.get_all_entities().iter().map(|e| (*e).clone()).collect();
                            }

                            let resp = GetSemanticMapResponse {
                                success: true,
                                error_message: String::new(),
                                entities,
                            };
                            if let Err(e) = get_semantic_map_server.async_send_response(req_id, resp).await {
                                error!("Send get semantic map response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive get semantic map request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle add entity requests (concurrent processing)
        let add_entity_task = smol::spawn(async move {
            let stream = add_entity_server.receive_request_stream();
            stream
                .for_each_concurrent(None, |result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let entity_id = req.entity.id.clone();
                            let entity_label = req.entity.label.clone();
                            info!(entity_id = %entity_id, entity_label = %entity_label, "Received add entity request");
                            let start = Instant::now();
                            
                            // Add entity - log before and after to track lock contention
                            debug!(entity_id = %entity_id, "Acquiring semantic map write lock");
                            let lock_start = Instant::now();
                            perception_clone2.add_entity(req.entity).await;
                            let lock_elapsed = lock_start.elapsed();
                            let total_elapsed = start.elapsed();
                            
                            if lock_elapsed.as_millis() > 100 {
                                warn!(entity_id = %entity_id, lock_ms = lock_elapsed.as_millis(), "Long lock wait detected");
                            }
                            
                            info!(entity_id = %entity_id, total_ms = total_elapsed.as_millis(), lock_ms = lock_elapsed.as_millis(), "Entity added successfully");
                            
                            let resp = AddEntityResponse {
                                success: true,
                                error_message: String::new(),
                            };
                            
                            let send_start = Instant::now();
                            match add_entity_server.async_send_response(req_id, resp).await {
                                Ok(_) => {
                                    let send_elapsed = send_start.elapsed();
                                    if send_elapsed.as_millis() > 50 {
                                        warn!(entity_id = %entity_id, send_ms = send_elapsed.as_millis(), "Slow response send");
                                    }
                                    debug!(entity_id = %entity_id, send_ms = send_elapsed.as_millis(), "Add entity response sent successfully");
                                }
                                Err(e) => {
                                    let send_elapsed = send_start.elapsed();
                                    error!(entity_id = %entity_id, send_ms = send_elapsed.as_millis(), error = ?e, "Send add entity response error");
                                }
                            }
                        }
                        Err(e) => error!(error = ?e, "Receive add entity request error"),
                    }
                })
                .await;
        });

        // Handle get spatial map requests
        let get_spatial_map_task = smol::spawn(async move {
            let stream = get_spatial_map_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let mut entries = perception_clone3.get_spatial_map_entries().await;
                            if let Some(frame_id) = &req.frame_id {
                                entries.retain(|e| e.frame_id == *frame_id);
                            }
                            let resp = GetSpatialMapResponse {
                                success: true,
                                error_message: String::new(),
                                entries,
                            };
                            if let Err(e) = get_spatial_map_server.async_send_response(req_id, resp).await {
                                error!("Send get spatial map response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive get spatial map request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle add spatial map entry requests
        let add_spatial_map_entry_task = smol::spawn(async move {
            let stream = add_spatial_map_entry_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            perception_clone4.add_spatial_map_entry(req.entry).await;
                            let resp = AddSpatialMapEntryResponse {
                                success: true,
                                error_message: String::new(),
                            };
                            if let Err(e) = add_spatial_map_entry_server.async_send_response(req_id, resp).await {
                                error!("Send add spatial map entry response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive add spatial map entry request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle get map status requests
        let get_map_status_task = smol::spawn(async move {
            let stream = get_map_status_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, _req)) => {
                            let is_updating = perception_clone5.is_map_updating().await;
                            let resp = GetMapStatusResponse { is_updating };
                            if let Err(e) = get_map_status_server.async_send_response(req_id, resp).await {
                                error!("Send get map status response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive get map status request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle model registration requests
        let register_model_task = smol::spawn(async move {
            let stream = register_model_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let resp = mgmt_clone1.register_model(req).await;
                            if let Err(e) = register_model_server.async_send_response(req_id, resp).await {
                                error!("Send register model response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive register model request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle model query requests
        let query_model_task = smol::spawn(async move {
            let stream = query_model_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let resp = mgmt_clone4.query_model(req).await;
                            if let Err(e) = query_model_server.async_send_response(req_id, resp).await {
                                error!("Send query model response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive query model request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle create task requests
        let create_task_task = smol::spawn(async move {
            let stream = create_task_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let task_id = planning_clone1.create_task(req.natural_language.clone()).await;
                            let resp = CreateTaskResponse {
                                success: true,
                                error_message: String::new(),
                                task_id: task_id.clone(),
                            };
                            if let Err(e) = create_task_server.async_send_response(req_id, resp).await {
                                error!("Send create task response error: {e:?}");
                            }
                            
                            // Automatically execute the full workflow after task creation
                            let planning_for_workflow = planning_clone1.clone();
                            let action_for_workflow = action_clone1.clone();
                            let task_id_for_workflow = task_id.clone();
                            smol::spawn(async move {
                                info!(task_id = %task_id_for_workflow, "Starting automatic workflow execution");
                                
                                // Step 1: Generate DSL
                                match planning_for_workflow.generate_dsl(&task_id_for_workflow).await {
                                    Ok(dsl_code) => {
                                        info!(task_id = %task_id_for_workflow, "DSL generated successfully");
                                        
                                        // Update state to Running before execution
                                        let _ = planning_for_workflow.update_task_state(
                                            &task_id_for_workflow,
                                            robonix_core::planning::task::TaskState::Running,
                                            None,
                                        ).await;
                                        
                                        // Step 2: Execute DSL
                                        match action_for_workflow.execute_dsl(&task_id_for_workflow, &dsl_code).await {
                                            Ok(()) => {
                                                info!(task_id = %task_id_for_workflow, "Task execution completed successfully");
                                                // Update task state to Completed
                                                let _ = planning_for_workflow.update_task_state(
                                                    &task_id_for_workflow,
                                                    robonix_core::planning::task::TaskState::Completed,
                                                    None,
                                                ).await;
                                            }
                                            Err(e) => {
                                                error!(task_id = %task_id_for_workflow, error = %e, "DSL execution failed");
                                                // Update task state to Failed
                                                let _ = planning_for_workflow.update_task_state(
                                                    &task_id_for_workflow,
                                                    robonix_core::planning::task::TaskState::Failed,
                                                    Some(e),
                                                ).await;
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        error!(task_id = %task_id_for_workflow, error = %e, "DSL generation failed");
                                        // Task state is already set to Failed in generate_dsl
                                    }
                                }
                            }).detach();
                        }
                        Err(e) => error!("Receive create task request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle get task requests
        let get_task_task = smol::spawn(async move {
            let stream = get_task_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let task = planning_clone2.get_task(&req.task_id).await;
                            let resp = GetTaskResponse {
                                success: task.is_some(),
                                error_message: if task.is_none() {
                                    format!("Task {} not found", req.task_id)
                                } else {
                                    String::new()
                                },
                                task,
                            };
                            if let Err(e) = get_task_server.async_send_response(req_id, resp).await {
                                error!("Send get task response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive get task request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle list tasks requests
        let list_tasks_task = smol::spawn(async move {
            let stream = list_tasks_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, _req)) => {
                            let tasks = planning_clone3.get_all_tasks().await;
                            let resp = ListTasksResponse {
                                success: true,
                                error_message: String::new(),
                                tasks,
                            };
                            if let Err(e) = list_tasks_server.async_send_response(req_id, resp).await {
                                error!("Send list tasks response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive list tasks request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle cancel task requests
        let cancel_task_task = smol::spawn(async move {
            let stream = cancel_task_server.receive_request_stream();
            stream
                .for_each(|result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let success = planning_clone4
                                .update_task_state(
                                    &req.task_id,
                                    robonix_core::planning::TaskState::Cancelled,
                                    None,
                                )
                                .await;
                            let resp = CancelTaskResponse {
                                success,
                                error_message: if success {
                                    String::new()
                                } else {
                                    format!("Task {} not found", req.task_id)
                                },
                            };
                            if let Err(e) = cancel_task_server.async_send_response(req_id, resp).await {
                                error!("Send cancel task response error: {e:?}");
                            }
                        }
                        Err(e) => error!("Receive cancel task request error: {e:?}"),
                    }
                })
                .await;
        });

        // Handle ping requests (concurrent processing)
        let ping_task = smol::spawn(async move {
            let stream = ping_server.receive_request_stream();
            stream
                .for_each_concurrent(None, |result| async {
                    match result {
                        Ok((req_id, req)) => {
                            let seq = req.sequence;
                            debug!(sequence = seq, "Received ping request");
                            let start = Instant::now();
                            let resp = mgmt_clone6.ping(req).await;
                            let elapsed = start.elapsed();
                            debug!(sequence = resp.sequence, elapsed_ms = elapsed.as_millis(), "Ping processed");
                            let send_start = Instant::now();
                            match ping_server.async_send_response(req_id, resp).await {
                                Ok(_) => {
                                    let send_elapsed = send_start.elapsed();
                                    if send_elapsed.as_millis() > 50 {
                                        warn!(sequence = seq, send_ms = send_elapsed.as_millis(), "Slow ping response send");
                                    }
                                    debug!(sequence = seq, send_ms = send_elapsed.as_millis(), "Ping response sent");
                                }
                                Err(e) => {
                                    let send_elapsed = send_start.elapsed();
                                    error!(sequence = seq, send_ms = send_elapsed.as_millis(), error = ?e, "Send ping response error");
                                }
                            }
                        }
                        Err(e) => error!(error = ?e, "Receive ping request error"),
                    }
                })
                .await;
        });

        // Wait for all tasks (all run indefinitely)
        futures_util::future::select_all(vec![
            Box::pin(register_task),
            Box::pin(query_task),
            Box::pin(get_semantic_map_task),
            Box::pin(add_entity_task),
            Box::pin(get_spatial_map_task),
            Box::pin(add_spatial_map_entry_task),
            Box::pin(get_map_status_task),
            Box::pin(register_model_task),
            Box::pin(query_model_task),
            Box::pin(create_task_task),
            Box::pin(get_task_task),
            Box::pin(list_tasks_task),
            Box::pin(cancel_task_task),
            Box::pin(ping_task),
        ]).await;
    });
    // .count() here just converts Stream to ordinary Future.
    // It would return the count of requestes processed, if the stream would end.
} // main

fn create_qos() -> QosPolicies {
    let service_qos: QosPolicies = {
        QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 1000 }) // Increased from 10 to handle high concurrency
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
        // Use a static Instant for the start time - we'll initialize it on first use
        static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
        let start = START.get_or_init(Instant::now);
        let elapsed = start.elapsed();
        let secs = elapsed.as_secs();
        let micros = elapsed.subsec_micros();
        // Format: [seconds.microseconds] with 6 digits for microseconds (always)
        // This ensures consistent alignment: [123.456789]
        let timestamp = format!("[{}.{:06}]", secs, micros);

        // Get log level and format it as a single colored letter
        let level = *event.metadata().level();
        let (level_char, level_color) = match level {
            Level::ERROR => ('E', Colour::Red),
            Level::WARN => ('W', Colour::Yellow),
            Level::INFO => ('I', Colour::Green),
            Level::DEBUG => ('D', Colour::Cyan),
            Level::TRACE => ('T', Colour::White),
        };

        // Get process name
        let proc_name = std::env::current_exe()
            .ok()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
            .unwrap_or_else(|| "robonix-core".to_string());

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
