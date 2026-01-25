// SPDX-License-Identifier: MulanPSL-2.0
// Task Manager Module
//
// Task Manager is the global scheduling and control core of the system,
// responsible for receiving user tasks, completing task parsing and planning,
// and coordinating task execution in the "planning-deduction-decision-execution-feedback" loop.

pub mod api;
pub mod exception;
pub mod executor;
pub mod queue;
pub mod task;

use crate::action::skill_library::SkillLibrary;
use crate::primitive::PrimitiveRegistry;
use crate::service::ServiceRegistry;
use crate::task_manager::exception::{RecoveryAction, apply_recovery_action};
use crate::task_manager::executor::ExecutionResult;
use log::{debug, error, info, trace, warn};
use ros2_client::{AService, Name, Node, ServiceMapping, ServiceTypeName};
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{Duration, interval, timeout};

/// Task Manager - Global scheduling and control core
pub struct TaskManager {
    task_store: Arc<task::TaskStore>,
    task_queue: Arc<queue::TaskQueue>,
    #[allow(dead_code)] // Executor runs in background, needs to be kept alive
    executor: Arc<executor::RtdlExecutor>,
    skill_library: Arc<SkillLibrary>,
    service_registry: Arc<ServiceRegistry>,
    primitive_registry: Arc<PrimitiveRegistry>,
    #[allow(dead_code)] // Node needs to be kept alive for ROS2 context
    node: Arc<Mutex<ros2_client::Node>>,
    // Runtime-specific state
    semantic_map_update_interval: Duration,
    semantic_map_cache: Arc<Mutex<serde_json::Value>>, // System-wide cache for semantic map
    task_plan_client: Arc<
        Mutex<
            Option<
                ros2_client::service::Client<
                    ros2_client::AService<
                        crate::ros_idl::service_types::PlanTaskRequest,
                        crate::ros_idl::service_types::PlanTaskResponse,
                    >,
                >,
            >,
        >,
    >,
}

impl TaskManager {
    pub fn new(
        skill_library: Arc<SkillLibrary>,
        service_registry: Arc<ServiceRegistry>,
        primitive_registry: Arc<PrimitiveRegistry>,
        node: Arc<Mutex<Node>>,
    ) -> Arc<Self> {
        let task_store = Arc::new(task::TaskStore::new());
        let task_queue = Arc::new(queue::TaskQueue::new());
        let executor = Arc::new(executor::RtdlExecutor::new(
            skill_library.clone(),
            node.clone(),
        ));

        let manager = Arc::new(Self {
            task_store: task_store.clone(),
            task_queue: task_queue.clone(),
            executor: executor.clone(),
            skill_library,
            service_registry: service_registry.clone(),
            primitive_registry,
            node: node.clone(),
            semantic_map_update_interval: Duration::from_secs(5), // Update every 5 seconds
            semantic_map_cache: Arc::new(Mutex::new(serde_json::json!([]))), // Initialize with empty array
            task_plan_client: Arc::new(Mutex::new(None)),
        });

        // Start runtime loop in background
        let manager_for_runtime = manager.clone();
        tokio::spawn(async move {
            manager_for_runtime.run_runtime_loop().await;
        });

        manager
    }

    pub fn get_task_store(&self) -> Arc<task::TaskStore> {
        self.task_store.clone()
    }

    pub fn get_skill_library(&self) -> Arc<SkillLibrary> {
        self.skill_library.clone()
    }

    pub fn get_service_registry(&self) -> Arc<ServiceRegistry> {
        self.service_registry.clone()
    }

    pub fn get_primitive_registry(&self) -> Arc<PrimitiveRegistry> {
        self.primitive_registry.clone()
    }

    pub fn get_semantic_map_cache(&self) -> Arc<Mutex<serde_json::Value>> {
        self.semantic_map_cache.clone()
    }

    /// Submit a new task
    pub async fn submit_task(&self, req: api::SubmitTaskRequest) -> api::SubmitTaskResponse {
        use log::debug;

        debug!(
            "submitting new task: description={}, params={}",
            req.description, req.params
        );

        // Parse params JSON string
        let params: serde_json::Value = match serde_json::from_str(&req.params) {
            Ok(v) => {
                debug!("parsed task params successfully");
                v
            }
            Err(e) => {
                debug!("failed to parse task params: {}, using empty object", e);
                serde_json::json!({}) // Default to empty object if parsing fails
            }
        };

        // Extract priority from params (default to 0)
        let priority = params.get("priority").and_then(|p| p.as_i64()).unwrap_or(0) as i32;
        debug!("task priority: {}", priority);

        // Create task in store (with priority)
        let task_id = self
            .task_store
            .create_task(req.description.clone(), params, priority)
            .await;
        debug!("created task in store: task_id={}", task_id);

        // Enqueue task
        let task = self
            .task_store
            .get_task(&task_id)
            .await
            .expect("Task should exist after creation");
        self.task_queue
            .enqueue(task_id.clone(), task.context.priority, task.created_at)
            .await;

        info!(
            "task {}: [PENDING] - task submitted (priority={}, description=\"{}\")",
            task_id, priority, req.description
        );
        debug!(
            "enqueued task: task_id={}, priority={}, queue_size={}",
            task_id,
            priority,
            self.task_queue.len().await
        );

        api::SubmitTaskResponse { task_id }
    }

    /// Get task data
    pub async fn get_task_data(&self, req: api::TaskDataRequest) -> api::TaskDataResponse {
        use log::debug;

        debug!("getting task result: task_id={}", req.task_id);

        // Build comprehensive task information
        let mut task_info = serde_json::json!({});

        // Get task from store
        if let Some(task) = self.task_store.get_task(&req.task_id).await {
            debug!("found task {} in task store", req.task_id);
            task_info["task_id"] = serde_json::json!(task.task_id);
            task_info["description"] = serde_json::json!(task.description);
            task_info["params"] = task.params.clone();
            task_info["state"] = serde_json::json!(format!("{:?}", task.state));
            task_info["created_at"] = serde_json::json!(task.created_at);
            task_info["updated_at"] = serde_json::json!(task.updated_at);
            task_info["priority"] = serde_json::json!(task.context.priority);
            task_info["retry_count"] = serde_json::json!(task.context.retry_count);
            task_info["rtdl_instruction_pointer"] =
                serde_json::json!(task.context.rtdl_instruction_pointer);

            if let Some(ref error_msg) = task.error_message {
                task_info["error_message"] = serde_json::json!(error_msg);
            }
            if let Some(ref result) = task.result {
                task_info["result"] = result.clone();
            }
            if let Some(ref rtdl) = task.context.rtdl {
                task_info["rtdl"] = serde_json::json!(rtdl);
            }
            if let Some(ref rtdl_type) = task.context.rtdl_type {
                task_info["rtdl_type"] = serde_json::json!(rtdl_type);
            }

            // Object graph information
            task_info["object_graph"] = task.context.object_graph.clone();
            task_info["object_graph_updated_at"] =
                serde_json::json!(task.context.object_graph_updated_at);
            if let Some(arr) = task.context.object_graph.as_array() {
                task_info["object_graph_count"] = serde_json::json!(arr.len());
            } else {
                task_info["object_graph_count"] = serde_json::json!(0);
            }

            // Exception information
            if let Some(ref exception) = task.context.last_exception {
                task_info["last_exception"] = serde_json::json!(exception);
            }
        } else {
            debug!("task {} not found in task store", req.task_id);
            task_info["error"] = serde_json::json!("Task not found");
        }

        // Serialize to JSON string
        let result = serde_json::to_string(&task_info).unwrap_or_else(|_| "{}".to_string());
        debug!(
            "task {} result serialized: {} bytes",
            req.task_id,
            result.len()
        );
        api::TaskDataResponse { data: result }
    }

    /// Start the runtime loop (called internally)
    async fn run_runtime_loop(self: Arc<Self>) {
        info!("task runtime started");
        debug!(
            "task runtime initialized: semantic_map_update_interval={:?}",
            self.semantic_map_update_interval
        );

        // Start semantic map cache update task
        let cache_clone = self.semantic_map_cache.clone();
        let service_registry_clone = self.service_registry.clone();
        let node_clone = self.node.clone();
        let update_interval = self.semantic_map_update_interval;

        debug!(
            "spawning semantic map cache update task with interval: {:?}",
            update_interval
        );
        tokio::spawn(async move {
            let mut interval = interval(update_interval);
            info!(
                "semantic map cache update task started (interval: {:?})",
                update_interval
            );
            loop {
                interval.tick().await;
                debug!("semantic map cache update tick");
                Self::update_semantic_map_cache(&cache_clone, &service_registry_clone, &node_clone)
                    .await;
            }
        });

        // Main runtime loop
        let mut tick_interval = interval(Duration::from_millis(500)); // 500ms tick
        debug!("main runtime loop started with 500ms tick interval");

        loop {
            tick_interval.tick().await;
            trace!("runtime loop tick");

            // Process task queue and execute tasks
            if let Err(e) = self.process_tasks().await {
                error!("error in task processing: {}", e);
            }
        }
    }

    /// Process tasks in the queue
    async fn process_tasks(&self) -> Result<(), String> {
        trace!("processing task queue");

        // Check if there's a higher priority task that should preempt current task
        if let Some(current_task_id) = self.task_queue.get_running_task().await {
            trace!("checking preemption for current task: {}", current_task_id);
            if let Some(task) = self.task_store.get_task(&current_task_id).await {
                trace!(
                    "current task {}: priority={}, state={:?}",
                    current_task_id, task.context.priority, task.state
                );
                let should_preempt = self.task_queue.should_preempt(task.context.priority).await;
                trace!(
                    "preemption check result for task {}: {}",
                    current_task_id, should_preempt
                );
                if should_preempt {
                    // Preempt current task
                    info!(
                        "preempting task {} for higher priority task",
                        current_task_id
                    );
                    self.suspend_task(&current_task_id).await?;
                } else {
                    trace!(
                        "no preemption needed for task {} (priority: {})",
                        current_task_id, task.context.priority
                    );
                }
            } else {
                trace!("current task {} not found", current_task_id);
            }
        } else {
            trace!("no current running task");
        }

        // Get next task to run
        let next_task_id = if let Some(running) = self.task_queue.get_running_task().await {
            // Continue running current task
            trace!("continuing with current task: {}", running);
            Some(running)
        } else {
            // Get next task from queue
            let queue_size = self.task_queue.len().await;
            trace!("no running task, checking queue (size: {})", queue_size);
            self.task_queue.dequeue().await
        };

        if let Some(task_id) = next_task_id {
            trace!("selected task to process: {}", task_id);
            self.task_queue
                .set_running_task(Some(task_id.clone()))
                .await;

            // Process the task based on its execution state
            if let Err(e) = self.process_task(&task_id).await {
                error!("error processing task {}: {}", task_id, e);
                // Mark task as failed
                self.task_store
                    .update_task(&task_id, |task| {
                        task.transition_state(task::TaskState::Failed);
                        task.context.record_exception(e.clone());
                    })
                    .await;
            }
        } else {
            trace!("no task to process");
        }

        Ok(())
    }

    /// Process a single task based on its execution state
    async fn process_task(&self, task_id: &str) -> Result<(), String> {
        let task = self
            .task_store
            .get_task(task_id)
            .await
            .ok_or_else(|| format!("Task not found: {}", task_id))?;

        debug!("processing task {} in state: {:?}", task_id, task.state);

        match task.state {
            task::TaskState::Pending => {
                // Update object_graph from cache before transitioning to Planning
                debug!(
                    "task {}: [PENDING] - updating object_graph from semantic_map cache",
                    task_id
                );
                let cache = self.semantic_map_cache.lock().await;
                let object_graph = cache.clone();
                drop(cache);

                let obj_count = if let Some(arr) = object_graph.as_array() {
                    arr.len()
                } else {
                    0
                };

                // [id:"label", id:"label", ...]
                let object_labels = |object_graph: &serde_json::Value| -> String {
                    if let Some(arr) = object_graph.as_array() {
                        arr.iter()
                            .map(|o| {
                                format!(
                                    "[{}:\"{}\"]",
                                    o.get("id").and_then(|i| i.as_str()).unwrap_or("unknown"),
                                    o.get("label").and_then(|l| l.as_str()).unwrap_or("unknown")
                                )
                            })
                            .collect::<Vec<_>>()
                            .join(", ")
                    } else {
                        "unknown".to_string()
                    }
                };

                // Transition to Planning state and update context with object_graph
                info!(
                    "task {}: [PENDING] -> [PLANNING] - starting task planning (object_graph: {} objects), sumary: \n{}",
                    task_id,
                    obj_count,
                    object_labels(&object_graph)
                );
                self.task_store
                    .update_task(task_id, |task| {
                        task.context.update_object_graph(object_graph);
                        task.transition_state(task::TaskState::Planning);
                    })
                    .await;
            }
            task::TaskState::Planning => {
                debug!("task {}: [PLANNING] - calling task_plan service", task_id);
                // Call task_plan service (object_graph already updated in context from Pending state)
                self.plan_task(task_id).await?;
            }
            task::TaskState::Running => {
                debug!("task {}: [RUNNING] - executing RTDL step", task_id);
                // Execute RTDL step
                self.execute_task(task_id).await?;
            }
            task::TaskState::Suspended => {
                // Task is suspended, don't process
                debug!("task {}: [SUSPENDED] - skipping (preempted)", task_id);
            }
            task::TaskState::Finished => {
                info!(
                    "task {}: [FINISHED] - task completed, removing from running queue",
                    task_id
                );
                // Task is done, remove from running
                self.task_queue.set_running_task(None).await;
            }
            task::TaskState::Failed => {
                info!(
                    "task {}: [FAILED] - task failed, removing from running queue",
                    task_id
                );
                // Task is done, remove from running
                self.task_queue.set_running_task(None).await;
            }
            task::TaskState::Cancelled => {
                info!(
                    "task {}: [CANCELLED] - task cancelled, removing from running queue",
                    task_id
                );
                self.task_queue.set_running_task(None).await;
            }
        }

        Ok(())
    }

    /// Plan task by calling task_plan service
    async fn plan_task(&self, task_id: &str) -> Result<(), String> {
        debug!("task {}: [PLANNING] - starting task planning", task_id);

        let task = self
            .task_store
            .get_task(task_id)
            .await
            .ok_or_else(|| format!("Task not found: {}", task_id))?;

        // Query task_plan service
        let query_resp = self
            .service_registry
            .query_started_service("srv::task_plan")
            .await;

        if query_resp.instances.is_empty() {
            error!("task_plan service not started (use 'rbnx deploy start' to start services)");
            return Err(
                "task_plan service not started (use 'rbnx deploy start' to start services)"
                    .to_string(),
            );
        }

        let service_instance = &query_resp.instances[0];
        let service_name = &service_instance.entry;
        // Parse metadata JSON string to get srv_type
        let metadata_value: serde_json::Value = serde_json::from_str(&service_instance.metadata)
            .unwrap_or_else(|_| serde_json::json!({}));
        let service_type = metadata_value
            .get("srv_type")
            .and_then(|v| v.as_str())
            .unwrap_or("robonix_sdk/srv/service/task_plan/PlanTask");

        let (rtdl, rtdl_type) = {
            let service_type_parsed = if let Some(last_slash) = service_type.rfind('/') {
                let package = "robonix_sdk";
                let type_name = &service_type[last_slash + 1..];
                ServiceTypeName::new(package, type_name)
            } else {
                error!("invalid service type format: {}", service_type);
                return Err(format!("Invalid service type format: {}", service_type));
            };

            let mut client_cache_guard = self.task_plan_client.lock().await;
            let client = if let Some(c) = client_cache_guard.as_ref() {
                c
            } else {
                let mut node_guard = self.node.lock().await;
                let service_qos = crate::server::create_qos();
                let service_name_parsed = match Name::parse(service_name) {
                    Ok(n) => n,
                    Err(e) => {
                        error!("failed to parse service name '{}': {}", service_name, e);
                        return Err(format!("Failed to parse service name: {}", e));
                    }
                };

                let new_client = match node_guard.create_client::<AService<
                    crate::ros_idl::service_types::PlanTaskRequest,
                    crate::ros_idl::service_types::PlanTaskResponse,
                >>(
                    ServiceMapping::Enhanced,
                    &service_name_parsed,
                    &service_type_parsed,
                    service_qos.clone(),
                    service_qos.clone(),
                ) {
                    Ok(c) => {
                        debug!("[task_plan] created new service client and cached it");
                        c
                    }
                    Err(e) => {
                        error!("failed to create task_plan service client: {}", e);
                        return Err(format!("Failed to create service client: {}", e));
                    }
                };
                *client_cache_guard = Some(new_client);
                client_cache_guard.as_ref().unwrap()
            };

            let object_graph_str = serde_json::to_string(&task.context.object_graph)
                .unwrap_or_else(|_| "[]".to_string());

            // Query all skills to get their names and start_args (input parameter schemas)
            let all_skills = self.skill_library.get_registry().get_all_skills().await;
            let skill_specs: Vec<serde_json::Value> = all_skills
                .into_iter()
                .map(|(_skill_id, instance)| {
                    // Parse start_args to get parameter schema
                    let start_args_schema: serde_json::Value =
                        serde_json::from_str(&instance.start_args)
                            .unwrap_or_else(|_| serde_json::json!({}));

                    serde_json::json!({
                        "name": instance.name,
                        "type": instance.r#type,
                        "start_topic": instance.start_topic,
                        "status_topic": instance.status_topic,
                        "start_args": start_args_schema,  // Input parameter schema
                        "provider": instance.provider,
                        "version": instance.version,
                    })
                })
                .collect();
            let skill_specs_str =
                serde_json::to_string(&skill_specs).unwrap_or_else(|_| "[]".to_string());

            let params = crate::ros_idl::object::Dict {
                keys: vec!["object_graph".to_string(), "skill_specs".to_string()],
                values: vec![object_graph_str.clone(), skill_specs_str],
            };

            let request = crate::ros_idl::service_types::PlanTaskRequest {
                description: task.description.clone(),
                params: params.clone(),
            };

            // Log request data
            let object_graph_obj: serde_json::Value =
                serde_json::from_str(&object_graph_str).unwrap_or_else(|_| serde_json::json!([]));
            let object_count = if let Some(arr) = object_graph_obj.as_array() {
                arr.len()
            } else {
                0
            };
            info!(
                "[task_plan] calling service: task_id={}, description=\"{}\", object_graph_count={}",
                task_id, request.description, object_count
            );
            debug!(
                "[task_plan] request details: description=\"{}\", object_graph={}",
                request.description,
                if object_graph_str.len() > 500 {
                    format!(
                        "{}... ({} chars)",
                        &object_graph_str[..500],
                        object_graph_str.len()
                    )
                } else {
                    object_graph_str.clone()
                }
            );

            // Record start time
            let start_time = std::time::Instant::now();

            let result = timeout(Duration::from_secs(60), client.async_call_service(request)).await;

            // Calculate elapsed time
            let elapsed = start_time.elapsed();

            match result {
                Ok(Ok(response)) => {
                    // Log response data and timing
                    info!(
                        "[task_plan] service response: task_id={}, rtdl_type={}, rtdl_length={}, elapsed={:?}",
                        task_id,
                        response.rtdl_type,
                        response.rtdl.len(),
                        elapsed
                    );
                    debug!(
                        "[task_plan] response details: rtdl_type={}, rtdl_preview={}",
                        response.rtdl_type,
                        if response.rtdl.len() > 500 {
                            format!(
                                "{}... ({} chars)",
                                &response.rtdl[..500],
                                response.rtdl.len()
                            )
                        } else {
                            response.rtdl.clone()
                        }
                    );
                    (response.rtdl, response.rtdl_type)
                }
                Ok(Err(e)) => {
                    error!(
                        "[task_plan] service call failed: task_id={}, error={:?}, elapsed={:?}",
                        task_id, e, elapsed
                    );
                    return Err(format!("Failed to call task_plan service: {:?}", e));
                }
                Err(_) => {
                    error!(
                        "[task_plan] service call timeout: task_id={}, elapsed={:?} (timeout=60s)",
                        task_id, elapsed
                    );
                    return Err("Task plan service call timeout after 60s".to_string());
                }
            }
        };

        // Update task with RTDL and transition to Running
        self.task_store
            .update_task(task_id, |task| {
                task.context.set_rtdl(rtdl.clone(), rtdl_type.clone());
                task.transition_state(task::TaskState::Running);
            })
            .await;

        info!(
            "task {}: [PLANNING] -> [RUNNING] - planning completed, starting execution",
            task_id
        );
        Ok(())
    }

    /// Execute a task
    async fn execute_task(&self, task_id: &str) -> Result<(), String> {
        let mut task = self
            .task_store
            .get_task(task_id)
            .await
            .ok_or_else(|| format!("Task not found: {}", task_id))?;

        let result = self.executor.execute_step(&mut task).await?;

        self.task_store
            .update_task(task_id, |t| {
                *t = task;
            })
            .await;

        match result {
            ExecutionResult::Completed => {
                self.task_store
                    .update_task(task_id, |task| {
                        task.transition_state(task::TaskState::Finished);
                    })
                    .await;
                self.task_queue.set_running_task(None).await;
                info!(
                    "task {}: [RUNNING] -> [FINISHED] - task completed successfully",
                    task_id
                );
            }
            ExecutionResult::InProgress => {
                // Continue execution
            }
            ExecutionResult::Exception(recovery_action) => {
                let state_transition = match recovery_action {
                    RecoveryAction::Replan => Some("PLANNING"),
                    RecoveryAction::Fail => Some("FAILED"),
                    _ => None,
                };

                self.task_store
                    .update_task(task_id, |task| {
                        apply_recovery_action(&mut task.context, recovery_action.clone());
                        match recovery_action {
                            RecoveryAction::Replan => {
                                task.transition_state(task::TaskState::Planning);
                            }
                            RecoveryAction::Fail => {
                                task.transition_state(task::TaskState::Failed);
                            }
                            _ => {
                                // Retry and Continue stay in Running state
                            }
                        }
                        task.updated_at = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_nanos() as u64;
                    })
                    .await;

                if let Some(new_state) = state_transition {
                    info!(
                        "task {}: [RUNNING] -> [{}] - exception occurred, recovery action: {:?}",
                        task_id, new_state, recovery_action
                    );
                } else {
                    debug!(
                        "task {}: [RUNNING] - exception occurred, recovery action: {:?} (staying in RUNNING)",
                        task_id, recovery_action
                    );
                }

                if matches!(recovery_action, RecoveryAction::Fail) {
                    self.task_queue.set_running_task(None).await;
                }
            }
        }

        Ok(())
    }

    /// Suspend a task (for preemption)
    async fn suspend_task(&self, task_id: &str) -> Result<(), String> {
        self.task_store
            .update_task(task_id, |task| {
                task.transition_state(task::TaskState::Suspended);
            })
            .await;

        self.task_queue.set_running_task(None).await;

        let task = self
            .task_store
            .get_task(task_id)
            .await
            .ok_or_else(|| format!("Task not found: {}", task_id))?;

        self.task_queue
            .enqueue(task_id.to_string(), task.context.priority, task.created_at)
            .await;

        info!(
            "task {}: [SUSPENDED] - task preempted and re-enqueued for later execution",
            task_id
        );
        Ok(())
    }

    /// Update semantic map cache from service
    async fn update_semantic_map_cache(
        cache: &Arc<Mutex<serde_json::Value>>,
        service_registry: &Arc<ServiceRegistry>,
        node: &Arc<Mutex<Node>>,
    ) {
        debug!("updating semantic map cache from service");

        let query_resp = service_registry
            .query_started_service("srv::semantic_map")
            .await;

        if query_resp.instances.is_empty() {
            debug!("no semantic_map service instances with status='started' found");
            return;
        }

        let service_instance = &query_resp.instances[0]; // TODO: we are choosing the first one here for now
        let service_name = &service_instance.entry;
        // Parse metadata JSON string to get srv_type
        let metadata_value: serde_json::Value = serde_json::from_str(&service_instance.metadata)
            .unwrap_or_else(|_| serde_json::json!({}));
        let service_type = metadata_value
            .get("srv_type")
            .and_then(|v| v.as_str())
            .unwrap_or("robonix_sdk/srv/service/semantic_map/QuerySemanticMap");

        let object_graph = {
            let mut node_guard = node.lock().await;
            let service_qos = crate::server::create_qos();

            let service_name_parsed = match Name::parse(service_name) {
                Ok(n) => n,
                Err(e) => {
                    error!("failed to parse service name '{}': {}", service_name, e);
                    return;
                }
            };

            let service_type_parsed = if let Some(last_slash) = service_type.rfind('/') {
                let package = "robonix_sdk";
                let type_name = &service_type[last_slash + 1..];
                ServiceTypeName::new(package, type_name)
            } else {
                error!(
                    "[semantic_map] invalid service type format: {}",
                    service_type
                );
                return;
            };

            let client = match node_guard.create_client::<AService<
                crate::ros_idl::service_types::QuerySemanticMapRequest,
                crate::ros_idl::service_types::QuerySemanticMapResponse,
            >>(
                ServiceMapping::Enhanced,
                &service_name_parsed,
                &service_type_parsed,
                service_qos.clone(),
                service_qos.clone(),
            ) {
                Ok(c) => c,
                Err(e) => {
                    error!("[semantic_map] failed to create service client: {}", e);
                    return;
                }
            };

            let request = crate::ros_idl::service_types::QuerySemanticMapRequest { types: vec![] };

            let timeout_duration = Duration::from_secs(30);
            let response = match timeout(timeout_duration, client.async_call_service(request)).await
            {
                Ok(Ok(response)) => response,
                Ok(Err(e)) => {
                    debug!("[semantic_map] service call failed: {:?}", e);
                    return;
                }
                Err(_) => {
                    warn!(
                        "[semantic_map] service call timeout after {:?}",
                        timeout_duration.as_secs()
                    );
                    return;
                }
            };

            match serde_json::to_value(&response.objects) {
                Ok(v) => v,
                Err(e) => {
                    error!("[semantic_map] failed to serialize objects to JSON: {}", e);
                    return;
                }
            }
        };

        let object_count = if let Some(arr) = object_graph.as_array() {
            arr.len()
        } else {
            0
        };

        let mut cache_guard = cache.lock().await;
        let old_count = if let Some(arr) = cache_guard.as_array() {
            arr.len()
        } else {
            0
        };
        *cache_guard = object_graph.clone();
        drop(cache_guard);

        // Print brief summary
        if object_count > 0 {
            info!(
                "[semantic_map] cache updated: {} objects total",
                object_count
            );

            // Print first 10 objects
            if let Some(arr) = object_graph.as_array() {
                let print_count = std::cmp::min(10, arr.len());
                let mut object_summaries = Vec::new();

                for i in 0..print_count {
                    if let Some(obj) = arr.get(i) {
                        let id = obj
                            .get("id")
                            .and_then(|v| v.as_str())
                            .unwrap_or("unknown")
                            .to_string();
                        let label = obj
                            .get("label")
                            .and_then(|v| v.as_str())
                            .unwrap_or("unknown")
                            .to_string();
                        object_summaries.push(format!("{}:{}", id, label));
                    }
                }

                if !object_summaries.is_empty() {
                    info!(
                        "[semantic_map] first {} objects: {}",
                        print_count,
                        object_summaries.join(", ")
                    );
                }
            }
        } else {
            debug!("[semantic_map] cache updated: 0 objects");
        }

        if object_count != old_count {
            info!(
                "[semantic_map] object count changed: {} -> {}",
                old_count, object_count
            );
        }
    }
}

impl std::fmt::Debug for TaskManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TaskManager").finish_non_exhaustive()
    }
}

// Re-export task types (from ros_idl)
pub use api::{SubmitTaskRequest, SubmitTaskResponse, TaskDataRequest, TaskDataResponse};
pub use task::{Task, TaskContext, TaskState};
