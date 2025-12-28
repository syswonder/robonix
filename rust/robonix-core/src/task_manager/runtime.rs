// SPDX-License-Identifier: MulanPSL-2.0
// Task Runtime Module
//
// Main runtime loop for task execution management:
// - Task queue scheduling
// - State machine transitions
// - Semantic map updates
// - RTDL execution
// - Exception handling

use crate::service::ServiceRegistry;
use crate::task_manager::context::{ExecutionState, TaskContextStore};
use crate::task_manager::exception::{RecoveryAction, apply_recovery_action};
use crate::task_manager::executor::{ExecutionResult, RtdlExecutor};
use crate::task_manager::queue::TaskQueue;
use log::{debug, error, info, trace};
use ros2_client::{AService, Name, Node, ServiceMapping, ServiceTypeName};
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{Duration, interval, timeout};

/// Task Runtime - Main runtime loop for task execution
pub struct TaskRuntime {
    context_store: Arc<TaskContextStore>,
    task_queue: Arc<TaskQueue>,
    executor: Arc<RtdlExecutor>,
    service_registry: Arc<ServiceRegistry>,
    node: Arc<Mutex<Node>>,
    semantic_map_update_interval: Duration,
    semantic_map_cache: Arc<Mutex<serde_json::Value>>, // System-wide cache for semantic map
    task_plan_client: Arc<
        Mutex<
            Option<
                ros2_client::service::Client<
                    AService<
                        crate::ros_idl::service_types::PlanTaskRequest,
                        crate::ros_idl::service_types::PlanTaskResponse,
                    >,
                >,
            >,
        >,
    >,
}

impl TaskRuntime {
    pub fn new(
        context_store: Arc<TaskContextStore>,
        task_queue: Arc<TaskQueue>,
        executor: Arc<RtdlExecutor>,
        service_registry: Arc<ServiceRegistry>,
        node: Arc<Mutex<Node>>,
    ) -> Self {
        Self {
            context_store,
            task_queue,
            executor,
            service_registry,
            node,
            semantic_map_update_interval: Duration::from_secs(5), // Update every 5 seconds
            semantic_map_cache: Arc::new(Mutex::new(serde_json::json!([]))), // Initialize with empty array
            task_plan_client: Arc::new(Mutex::new(None)),
        }
    }

    /// Start the runtime loop
    pub async fn run(&self) {
        info!("task runtime started");
        debug!(
            "task runtime initialized: semantic_map_update_interval={:?}",
            self.semantic_map_update_interval
        );

        // Start semantic map cache update task (periodically fetch from service and update cache)
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
            debug!("semantic map cache update task started");
            loop {
                interval.tick().await;
                trace!("semantic map cache update tick");
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
            debug!("checking preemption for current task: {}", current_task_id);
            if let Some(context) = self.context_store.get_context(&current_task_id).await {
                debug!(
                    "current task {} context: priority={}, state={:?}",
                    current_task_id, context.priority, context.execution_state
                );
                let should_preempt = self.task_queue.should_preempt(context.priority).await;
                debug!(
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
                        current_task_id, context.priority
                    );
                }
            } else {
                debug!("current task {} context not found", current_task_id);
            }
        } else {
            trace!("no current running task");
        }

        // Get next task to run
        let next_task_id = if let Some(running) = self.task_queue.get_running_task().await {
            // Continue running current task
            debug!("continuing with current task: {}", running);
            Some(running)
        } else {
            // Get next task from queue
            let queue_size = self.task_queue.len().await;
            trace!("no running task, checking queue (size: {})", queue_size);
            self.task_queue.dequeue().await
        };

        if let Some(task_id) = next_task_id {
            debug!("selected task to process: {}", task_id);
            self.task_queue
                .set_running_task(Some(task_id.clone()))
                .await;

            // Process the task based on its execution state
            if let Err(e) = self.process_task(&task_id).await {
                error!("error processing task {}: {}", task_id, e);
                // Mark task as failed
                self.context_store
                    .update_context(&task_id, |ctx| {
                        ctx.transition_state(ExecutionState::Failed);
                        ctx.record_exception(e.clone());
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
        let context = self
            .context_store
            .get_context(task_id)
            .await
            .ok_or_else(|| format!("Task context not found: {}", task_id))?;

        debug!(
            "processing task {} in state: {:?}",
            task_id, context.execution_state
        );

        match context.execution_state {
            ExecutionState::Init => {
                debug!("task {} transitioning from Init to Plan", task_id);
                // Transition to Plan state
                self.context_store
                    .update_context(task_id, |ctx| {
                        ctx.transition_state(ExecutionState::Plan);
                    })
                    .await;
            }
            ExecutionState::Plan => {
                debug!("task {} in Plan state, calling task_plan service", task_id);
                // Check if object_graph is empty, if so, update from cache
                let context = self
                    .context_store
                    .get_context(task_id)
                    .await
                    .ok_or_else(|| format!("Task context not found: {}", task_id))?;

                let obj_count = if let Some(arr) = context.object_graph.as_array() {
                    arr.len()
                } else {
                    0
                };

                if obj_count == 0 {
                    debug!(
                        "task {} object_graph is empty, updating from semantic_map cache",
                        task_id
                    );
                    // Update context from system cache
                    let cache = self.semantic_map_cache.lock().await;
                    let object_graph = cache.clone();
                    drop(cache);

                    self.context_store
                        .update_context(task_id, |ctx| {
                            ctx.update_object_graph(object_graph);
                        })
                        .await;
                }

                // Call task_plan service (using the snapshot in context)
                self.plan_task(task_id).await?;
            }
            ExecutionState::Execute => {
                debug!("task {} in Execute state, executing RTDL step", task_id);
                // Execute RTDL step
                self.execute_rtdl_step(task_id).await?;
            }
            ExecutionState::Suspended => {
                // Task is suspended, don't process
                debug!("task {} is suspended, skipping", task_id);
            }
            ExecutionState::Finish | ExecutionState::Failed => {
                debug!(
                    "task {} finished (state: {:?}), removing from running",
                    task_id, context.execution_state
                );
                // Task is done, remove from running
                self.task_queue.set_running_task(None).await;
            }
        }

        Ok(())
    }

    /// Plan task by calling task_plan service
    async fn plan_task(&self, task_id: &str) -> Result<(), String> {
        info!("Task Planning: Starting planning for task {}", task_id);
        debug!("plan_task called for task {}", task_id);

        let context = self
            .context_store
            .get_context(task_id)
            .await
            .ok_or_else(|| format!("Task context not found: {}", task_id))?;

        debug!(
            "retrieved task context: task_id={}, state={:?}, priority={}, retry_count={}",
            context.task_id, context.execution_state, context.priority, context.retry_count
        );

        debug!("Task Context:");
        debug!("  Description: {}", context.description);
        debug!(
            "  Object Graph: {}",
            serde_json::to_string_pretty(&context.object_graph)
                .unwrap_or_else(|_| "{}".to_string())
        );
        debug!(
            "  Object Graph Updated At: {}",
            context.object_graph_updated_at
        );

        let obj_count = if let Some(arr) = context.object_graph.as_array() {
            arr.len()
        } else {
            0
        };
        debug!(
            "task {} object graph contains {} objects",
            task_id, obj_count
        );

        // Query task_plan service (only started services)
        debug!("Step 1: Querying task_plan service...");
        debug!("querying service registry for task_plan service (started only)");
        let query_resp = self
            .service_registry
            .query_started_service("task_plan")
            .await;
        debug!(
            "service registry query returned {} started instances",
            query_resp.instances.len()
        );

        if query_resp.instances.is_empty() {
            debug!("no task_plan service instances with status='started' found");
            error!("task_plan service not started (use 'rbnx deploy start' to start services)");
            return Err(
                "task_plan service not started (use 'rbnx deploy start' to start services)"
                    .to_string(),
            );
        }

        // Use the first available task_plan service instance that is started
        let service_instance = &query_resp.instances[0];
        debug!(
            "selected task_plan service instance: provider={}, version={}, entry={}",
            service_instance.provider, service_instance.version, service_instance.entry
        );
        debug!("Found task_plan service:");
        debug!("  Provider: {}", service_instance.provider);
        debug!("  Version: {}", service_instance.version);
        debug!("  Entry: {}", service_instance.entry);
        debug!(
            "  Service Type: {}",
            service_instance
                .metadata
                .get("srv_type")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown")
        );

        debug!("service instance metadata: {:?}", service_instance.metadata);

        // Prepare task_plan request
        // params should include object_graph from context
        // Use Dict format (keys/values arrays) as per robonix_sdk/Dict definition
        debug!("preparing task_plan request parameters");

        // Serialize object_graph to JSON string for Dict value
        let object_graph_str =
            serde_json::to_string(&context.object_graph).unwrap_or_else(|_| "[]".to_string());

        // Create Dict with keys and values arrays
        let params = crate::ros_idl::object::Dict {
            keys: vec!["object_graph".to_string()],
            values: vec![object_graph_str],
        };

        let obj_count = if let Some(arr) = context.object_graph.as_array() {
            arr.len()
        } else {
            0
        };
        debug!(
            "task_plan request params: object_graph with {} objects",
            obj_count
        );

        debug!("Step 2: Preparing task_plan request...");
        debug!("  Description: {}", context.description);
        debug!("  Params: Dict with {} key(s)", params.keys.len());
        debug!(
            "task_plan request prepared: description_length={}, params_keys={:?}",
            context.description.len(),
            params.keys
        );
        // Print object_graph content (from semantic map service)
        let object_graph_pretty = serde_json::to_string_pretty(&context.object_graph)
            .unwrap_or_else(|_| "failed to serialize".to_string());
        debug!(
            "  object_graph (from semantic map service):\n{}",
            object_graph_pretty
        );

        // Call the task_plan service via ROS2
        let service_name = &service_instance.entry;
        let service_type = service_instance
            .metadata
            .get("srv_type")
            .and_then(|v| v.as_str())
            .unwrap_or("robonix_sdk/srv/service/task_plan/PlanTask");

        debug!(
            "Step 3: Calling task_plan service: entry={}, type={}",
            service_name, service_type
        );

        let (rtdl, rtdl_type) = {
            // Parse service type: "robonix_sdk/srv/service/task_plan/PlanTask"
            // But actual ROS2 service type is "robonix_sdk/srv/PlanTask"
            // Extract the service name from the end: "PlanTask"
            let service_type_parsed = if let Some(last_slash) = service_type.rfind('/') {
                let package = "robonix_sdk";
                let type_name = &service_type[last_slash + 1..]; // Extract "PlanTask"
                debug!(
                    "[task_plan] parsed service type: package={}, type_name={}",
                    package, type_name
                );
                ServiceTypeName::new(package, type_name)
            } else {
                error!("invalid service type format: {}", service_type);
                return Err(format!("Invalid service type format: {}", service_type));
            };

            // Use cached client or create new one
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

            let request = crate::ros_idl::service_types::PlanTaskRequest {
                description: context.description.clone(),
                params,
            };

            debug!(
                "calling task_plan service with description: {}",
                request.description
            );

            // Call service with timeout (task planning may take longer)
            // Note: async_call_service will handle service availability internally
            debug!("calling task_plan service (timeout: 60s)...");
            debug!(
                "[task_plan] calling service: entry={}, type={:?}",
                service_name, service_type_parsed
            );
            let call_start = std::time::Instant::now();
            match timeout(Duration::from_secs(60), client.async_call_service(request)).await {
                Ok(Ok(response)) => {
                    let call_duration = call_start.elapsed();
                    debug!("[task_plan] service call succeeded in {:?}", call_duration);
                    debug!(
                        "[task_plan] received RTDL: type={}, length={} chars",
                        response.rtdl_type,
                        response.rtdl.len()
                    );
                    debug!(
                        "[task_plan] RTDL code (first 500 chars):\n{}",
                        if response.rtdl.len() > 500 {
                            &response.rtdl[..500]
                        } else {
                            &response.rtdl
                        }
                    );
                    (response.rtdl, response.rtdl_type)
                }
                Ok(Err(e)) => {
                    let call_duration = call_start.elapsed();
                    error!(
                        "[task_plan] service call failed after {:?}: {:?}",
                        call_duration, e
                    );
                    return Err(format!("Failed to call task_plan service: {:?}", e));
                }
                Err(_) => {
                    let call_duration = call_start.elapsed();
                    error!(
                        "[task_plan] service call timeout after {:?} (60s limit)",
                        call_duration
                    );
                    return Err("Task plan service call timeout after 60s".to_string());
                }
            }
        };

        // Update context with RTDL and transition to Execute
        debug!("updating task context with RTDL and transitioning to Execute state");
        self.context_store
            .update_context(task_id, |ctx| {
                debug!(
                    "setting RTDL for task {}: type={}, length={} chars",
                    task_id,
                    &rtdl_type,
                    rtdl.len()
                );
                ctx.set_rtdl(rtdl.clone(), rtdl_type.clone());
                debug!(
                    "transitioning task {} from {:?} to Execute",
                    task_id, ctx.execution_state
                );
                ctx.transition_state(ExecutionState::Execute);
            })
            .await;

        info!(
            "Task planning completed for task {}. Transitioned to Execute state.",
            task_id
        );
        debug!("plan_task completed successfully for task {}", task_id);
        Ok(())
    }

    /// Execute a single RTDL step
    async fn execute_rtdl_step(&self, task_id: &str) -> Result<(), String> {
        debug!("executing RTDL step for task {}", task_id);

        let result = {
            let mut context = self
                .context_store
                .get_context(task_id)
                .await
                .ok_or_else(|| format!("Task context not found: {}", task_id))?;

            debug!(
                "task {} context: instruction_pointer={}, rtdl_type={:?}",
                task_id, context.rtdl_instruction_pointer, context.rtdl_type
            );

            self.executor.execute_step(&mut context).await?
        };

        match result {
            ExecutionResult::Completed => {
                debug!("task {} execution completed", task_id);
                // Task completed successfully
                self.context_store
                    .update_context(task_id, |ctx| {
                        ctx.transition_state(ExecutionState::Finish);
                    })
                    .await;
                self.task_queue.set_running_task(None).await;
                info!("task {} completed successfully", task_id);
            }
            ExecutionResult::InProgress => {
                debug!("task {} execution in progress, continuing", task_id);
                // Continue execution
            }
            ExecutionResult::Exception(recovery_action) => {
                debug!(
                    "task {} execution exception, recovery action: {:?}",
                    task_id, recovery_action
                );
                // Apply recovery action
                self.context_store
                    .update_context(task_id, |ctx| {
                        apply_recovery_action(ctx, recovery_action.clone());
                    })
                    .await;

                match recovery_action {
                    RecoveryAction::Fail => {
                        debug!("task {} failed, removing from running", task_id);
                        self.task_queue.set_running_task(None).await;
                    }
                    RecoveryAction::Retry => {
                        debug!("task {} will retry current instruction", task_id);
                    }
                    RecoveryAction::Replan => {
                        debug!("task {} will replan", task_id);
                    }
                    RecoveryAction::Continue => {
                        debug!("task {} continuing after exception", task_id);
                    }
                }
            }
        }

        Ok(())
    }

    /// Suspend a task (for preemption)
    async fn suspend_task(&self, task_id: &str) -> Result<(), String> {
        debug!("suspending task {} for preemption", task_id);

        self.context_store
            .update_context(task_id, |ctx| {
                debug!("task {} transitioning to Suspended state", task_id);
                ctx.transition_state(ExecutionState::Suspended);
            })
            .await;

        debug!("clearing running task after suspension");
        self.task_queue.set_running_task(None).await;

        // Re-enqueue the task so it can be resumed later
        let context = self
            .context_store
            .get_context(task_id)
            .await
            .ok_or_else(|| format!("Task context not found: {}", task_id))?;

        debug!(
            "re-enqueuing suspended task {} with priority={}, instruction_pointer={}",
            task_id, context.priority, context.rtdl_instruction_pointer
        );
        self.task_queue
            .enqueue(task_id.to_string(), context.priority, context.created_at)
            .await;

        debug!("task {} suspended and re-enqueued successfully", task_id);
        Ok(())
    }

    /// Update semantic map cache from service
    /// This function is called periodically to fetch latest data from semantic_map service and update system cache
    async fn update_semantic_map_cache(
        cache: &Arc<Mutex<serde_json::Value>>,
        service_registry: &Arc<ServiceRegistry>,
        node: &Arc<Mutex<Node>>,
    ) {
        trace!("updating semantic map cache from service");

        // Query semantic_map service (only started services)
        trace!("querying semantic_map service (started only)");
        let query_resp = service_registry.query_started_service("semantic_map").await;

        if query_resp.instances.is_empty() {
            trace!("no semantic_map service instances with status='started' found");
            return; // Silently return if service not started (will retry on next interval)
        }

        trace!(
            "found {} semantic_map service instances with status='started'",
            query_resp.instances.len()
        );

        // Use the first available semantic_map service instance that is started
        let service_instance = &query_resp.instances[0];
        let service_name = &service_instance.entry;
        let service_type = service_instance
            .metadata
            .get("srv_type")
            .and_then(|v| v.as_str())
            .unwrap_or("robonix_sdk/srv/service/semantic_map/QuerySemanticMap");

        trace!(
            "using semantic_map service: provider={}, entry={}, type={}",
            service_instance.provider, service_name, service_type
        );

        // Call the semantic_map service via ROS2
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

            // Parse service type: "robonix_sdk/srv/service/semantic_map/QuerySemanticMap"
            // But actual ROS2 service type is "robonix_sdk/srv/QuerySemanticMap"
            // Extract the service name from the end: "QuerySemanticMap"
            let service_type_parsed = if let Some(last_slash) = service_type.rfind('/') {
                let package = "robonix_sdk";
                let type_name = &service_type[last_slash + 1..]; // Extract "QuerySemanticMap"
                trace!(
                    "[semantic_map] parsed service type: original={}, package={}, type_name={}",
                    service_type, package, type_name
                );
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

            let request = crate::ros_idl::service_types::QuerySemanticMapRequest {
                types: vec![], // Empty types filter = get all objects
            };

            // Call service with timeout
            // Note: async_call_service will handle service availability internally
            // Use shorter timeout since service is already started (status='started')
            let call_start = std::time::Instant::now();
            let response =
                match timeout(Duration::from_secs(5), client.async_call_service(request)).await {
                    Ok(Ok(response)) => {
                        let call_duration = call_start.elapsed();
                        debug!(
                            "[semantic_map] service call succeeded in {:?}, received {} objects",
                            call_duration,
                            response.objects.len()
                        );
                        // Print object ids and labels
                        for obj in &response.objects {
                            debug!("[semantic_map] object: id={}, label={}", obj.id, obj.label);
                        }
                        response
                    }
                    Ok(Err(e)) => {
                        let call_duration = call_start.elapsed();
                        trace!(
                            "[semantic_map] service call failed after {:?}: {:?}",
                            call_duration, e
                        );
                        return; // Silently return, will retry on next interval
                    }
                    Err(_) => {
                        let call_duration = call_start.elapsed();
                        trace!(
                            "[semantic_map] service call timeout after {:?} (5s limit)",
                            call_duration
                        );
                        return; // Silently return, will retry on next interval
                    }
                };

            // Convert Vec<Object> to JSON Value
            let obj_count: usize = response.objects.len();
            debug!(
                "[semantic_map] received response: objects_count={}, stamp.sec={}, stamp.nanosec={}",
                obj_count, response.stamp.sec, response.stamp.nanosec
            );

            // Serialize Vec<Object> to JSON Value
            let objects_value: serde_json::Value = match serde_json::to_value(&response.objects) {
                Ok(v) => v,
                Err(e) => {
                    error!("[semantic_map] failed to serialize objects to JSON: {}", e);
                    return;
                }
            };

            objects_value
        };

        let obj_count: usize = if let Some(arr) = object_graph.as_array() {
            arr.len()
        } else {
            0
        };
        debug!("semantic map cache updated: {} objects", obj_count);

        // Update system cache
        let mut cache_guard = cache.lock().await;
        *cache_guard = object_graph;
    }
}
