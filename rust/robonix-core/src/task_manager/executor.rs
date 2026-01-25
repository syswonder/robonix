// SPDX-License-Identifier: MulanPSL-2.0
// RTDL Executor Module
//
// Executes RTDL (Robot Task Description Langauge) programs
// Currently supports simple list-style RTDL (skill invocation list)

use crate::action::skill_library::SkillLibrary;
use crate::task_manager::exception::{ExceptionHandler, ExceptionType, RecoveryAction};
use crate::task_manager::task::Task;
use futures_util::stream::StreamExt;
use log::{debug, error, info, warn};
use ros2_client::rustdds::{
    Duration as RustddsDuration, QosPolicyBuilder,
    policy::{self, Reliability},
};
use ros2_client::{MessageTypeName, Name, Publisher, Subscription};
use serde_json::Value;
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio::time::{Duration, timeout};

/// RTDL Instruction - Represents a single instruction in RTDL
#[derive(Debug, Clone)]
pub struct RtdlInstruction {
    pub object_id: String, // Object ID that executes this instruction (usually robot)
    pub instruction_type: String, // "skill", "service", "primitive", etc.
    pub name: String,      // Skill/service/primitive name
    pub params: Value,     // Instruction parameters (JSON)
}

/// RTDL Executor - Executes RTDL programs
pub struct RtdlExecutor {
    skill_library: Arc<SkillLibrary>,
    exception_handler: ExceptionHandler,
    node: Arc<tokio::sync::Mutex<ros2_client::Node>>,
}

impl RtdlExecutor {
    pub fn new(
        skill_library: Arc<SkillLibrary>,
        node: Arc<tokio::sync::Mutex<ros2_client::Node>>,
    ) -> Self {
        Self {
            skill_library,
            exception_handler: ExceptionHandler,
            node,
        }
    }

    /// Parse RTDL program (currently supports simple JSON list format)
    pub fn parse_rtdl(&self, rtdl: &str, rtdl_type: &str) -> Result<Vec<RtdlInstruction>, String> {
        debug!(
            "parsing RTDL: type={}, length={} chars",
            rtdl_type,
            rtdl.len()
        );

        match rtdl_type {
            "list" => {
                // Parse as JSON array of instructions
                let instructions: Vec<Value> = serde_json::from_str(rtdl)
                    .map_err(|e| format!("Failed to parse RTDL as JSON list: {}", e))?;

                debug!("parsed {} instructions from RTDL", instructions.len());

                let mut parsed = Vec::new();
                for (idx, inst) in instructions.iter().enumerate() {
                    let instruction = self.parse_instruction(inst, idx)?;
                    debug!(
                        "parsed instruction {}: type={}, name={}",
                        idx, instruction.instruction_type, instruction.name
                    );
                    parsed.push(instruction);
                }
                Ok(parsed)
            }
            _ => Err(format!("Unsupported RTDL type: {}", rtdl_type)),
        }
    }

    /// Parse a single instruction from JSON
    fn parse_instruction(&self, inst: &Value, idx: usize) -> Result<RtdlInstruction, String> {
        debug!("parsing instruction {}: raw={:?}", idx, inst);

        // Expected format: {"object_id": "robot_001", "type": "skill", "name": "pick", "params": {...}}
        let object_id = inst["object_id"]
            .as_str()
            .ok_or_else(|| format!("Instruction {} missing 'object_id' field (required to specify which object executes this instruction)", idx))?
            .to_string();
        debug!("instruction {} object_id: {}", idx, object_id);

        let instruction_type = inst["type"]
            .as_str()
            .ok_or_else(|| format!("Instruction {} missing 'type' field", idx))?
            .to_string();
        debug!("instruction {} type: {}", idx, instruction_type);

        let name = inst["name"]
            .as_str()
            .ok_or_else(|| format!("Instruction {} missing 'name' field", idx))?
            .to_string();
        debug!("instruction {} name: {}", idx, name);

        let params = inst
            .get("params")
            .cloned()
            .unwrap_or_else(|| Value::Object(serde_json::Map::new()));
        let param_count = if let Some(obj) = params.as_object() {
            obj.len()
        } else {
            0
        };
        debug!("instruction {} params: {} fields", idx, param_count);

        Ok(RtdlInstruction {
            object_id,
            instruction_type,
            name,
            params,
        })
    }

    /// Execute a single RTDL instruction
    /// Returns (ExceptionType, Option<error_message>) on failure
    pub async fn execute_instruction(
        &self,
        task: &mut Task,
        instruction: &RtdlInstruction,
    ) -> Result<(), (ExceptionType, Option<String>)> {
        debug!(
            "executing instruction: task_id={}, object_id={}, type={}, name={}, params={:?}",
            task.task_id,
            instruction.object_id,
            instruction.instruction_type,
            instruction.name,
            instruction.params
        );
        debug!(
            "task {} before instruction: state={:?}, retry_count={}, instruction_pointer={}",
            task.task_id,
            task.state,
            task.context.retry_count,
            task.context.rtdl_instruction_pointer
        );

        match instruction.instruction_type.as_str() {
            "skill" => {
                debug!("dispatching to skill execution handler");
                let result = self
                    .execute_skill(task, &instruction.name, &instruction.params)
                    .await;
                match &result {
                    Ok(_) => debug!("skill execution succeeded"),
                    Err((exception_type, error_msg)) => {
                        debug!(
                            "skill execution failed: {:?}, error={:?}",
                            exception_type, error_msg
                        );
                        // Save error message to task if available
                        if let Some(msg) = error_msg {
                            task.error_message = Some(msg.clone());
                        }
                    }
                }
                result
            }
            "service" => {
                debug!("service execution requested but not implemented");
                // TODO: Implement service execution
                error!("service execution not yet implemented");
                let error_msg = "Service execution not implemented".to_string();
                task.error_message = Some(error_msg.clone());
                Err((ExceptionType::ServiceUnavailable, Some(error_msg)))
            }
            "primitive" => {
                debug!("primitive execution requested but not implemented");
                // TODO: Implement primitive execution
                error!("primitive execution not yet implemented");
                let error_msg = "Primitive execution not implemented".to_string();
                task.error_message = Some(error_msg.clone());
                Err((ExceptionType::PrimitiveFailed, Some(error_msg)))
            }
            _ => {
                debug!("unknown instruction type: {}", instruction.instruction_type);
                warn!("unknown instruction type: {}", instruction.instruction_type);
                let error_msg =
                    format!("Unknown instruction type: {}", instruction.instruction_type);
                task.error_message = Some(error_msg.clone());
                Err((ExceptionType::Unknown(error_msg.clone()), Some(error_msg)))
            }
        }
    }

    /// Execute a skill instruction
    /// Returns (ExceptionType, Option<error_message>) on failure
    async fn execute_skill(
        &self,
        task: &Task,
        skill_name: &str,
        params: &Value,
    ) -> Result<(), (ExceptionType, Option<String>)> {
        debug!("querying skill library for skill: {}", skill_name);

        // Query skill from skill library
        let query_req = crate::ros_idl::skill::QuerySkillRequest {
            name: skill_name.to_string(),
            filter: "{}".to_string(),
        };

        let query_resp = self.skill_library.query_skill(query_req).await;

        if query_resp.instances.is_empty() {
            debug!("skill {} not found in skill library", skill_name);
            let error_msg = format!("Skill '{}' not found in skill library", skill_name);
            return Err((ExceptionType::SkillFailed, Some(error_msg)));
        }

        debug!(
            "found {} instances of skill {}",
            query_resp.instances.len(),
            skill_name
        );

        // Use the first available skill instance
        let skill_instance = &query_resp.instances[0];
        debug!(
            "using skill instance: skill_id={}, provider={}, entry={}",
            skill_instance.skill_id, skill_instance.provider, skill_instance.entry
        );

        // Actually invoke the skill via ROS2 topics
        info!(
            "executing skill: task_id={}, skill_name={}, skill_id={}, start_topic={}, status_topic={}",
            task.task_id,
            skill_name,
            skill_instance.skill_id,
            skill_instance.start_topic,
            skill_instance.status_topic
        );

        debug!("skill execution params: {:?}", params);

        // Generate unique skill execution ID
        let skill_exec_id = format!("{}_{}", skill_instance.skill_id, task.task_id);

        // Prepare start message: JSON with skill_id and params
        let start_msg_json = serde_json::json!({
            "skill_id": skill_exec_id,
            "params": params
        });
        let start_msg_data = serde_json::to_string(&start_msg_json).map_err(|e| {
            let error_msg = format!("Failed to serialize start message: {}", e);
            (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
        })?;

        // Get node for publishing/subscribing
        let mut node_guard = self.node.lock().await;

        // Create QoS for skill topics
        let topic_qos = QosPolicyBuilder::new()
            .history(policy::History::KeepLast { depth: 10 })
            .reliability(Reliability::Reliable {
                max_blocking_time: RustddsDuration::from_millis(100),
            })
            .durability(policy::Durability::Volatile)
            .build();

        // Create publisher for start_topic (std_msgs/msg/String)
        let start_topic_name = Name::parse(&skill_instance.start_topic).map_err(|e| {
            let error_msg = format!(
                "Failed to parse start_topic '{}': {}",
                skill_instance.start_topic, e
            );
            (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
        })?;
        let start_topic = node_guard
            .create_topic(
                &start_topic_name,
                MessageTypeName::new("std_msgs", "String"),
                &topic_qos,
            )
            .map_err(|e| {
                let error_msg = format!(
                    "Failed to create start_topic '{}': {}",
                    skill_instance.start_topic, e
                );
                (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
            })?;

        let start_publisher: Publisher<crate::ros_idl::skill::StdString> = node_guard
            .create_publisher(&start_topic, None)
            .map_err(|e| {
                let error_msg = format!(
                    "Failed to create publisher for start_topic '{}': {}",
                    skill_instance.start_topic, e
                );
                (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
            })?;

        // Create subscription for status_topic (std_msgs/msg/String)
        let status_topic_name = Name::parse(&skill_instance.status_topic).map_err(|e| {
            let error_msg = format!(
                "Failed to parse status_topic '{}': {}",
                skill_instance.status_topic, e
            );
            (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
        })?;
        let status_topic = node_guard
            .create_topic(
                &status_topic_name,
                MessageTypeName::new("std_msgs", "String"),
                &topic_qos,
            )
            .map_err(|e| {
                let error_msg = format!(
                    "Failed to create status_topic '{}': {}",
                    skill_instance.status_topic, e
                );
                (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
            })?;

        let status_subscription: Subscription<crate::ros_idl::skill::StdString> = node_guard
            .create_subscription(&status_topic, None)
            .map_err(|e| {
                let error_msg = format!(
                    "Failed to create subscription for status_topic '{}': {}",
                    skill_instance.status_topic, e
                );
                (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
            })?;

        drop(node_guard); // Release node lock

        // Spawn task to monitor status messages
        let skill_exec_id_clone = skill_exec_id.clone();
        let (status_tx, mut status_rx) = mpsc::channel::<std::string::String>(10);
        let status_subscription_arc = Arc::new(status_subscription);
        tokio::spawn(async move {
            let mut stream = Box::pin(status_subscription_arc.async_stream());
            while let Some(result) = stream.next().await {
                match result {
                    Ok((msg, _msg_info)) => {
                        let status_str = msg.data.clone();
                        // Filter messages for this skill execution
                        if let Ok(status_json) =
                            serde_json::from_str::<serde_json::Value>(&status_str)
                        {
                            if let Some(msg_skill_id) =
                                status_json.get("skill_id").and_then(|v| v.as_str())
                            {
                                if msg_skill_id == skill_exec_id_clone {
                                    let _ = status_tx.send(status_str.clone()).await;
                                }
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Error receiving status message: {}", e);
                    }
                }
            }
        });

        // Publish start message
        let start_msg = crate::ros_idl::skill::StdString {
            data: start_msg_data.clone(),
        };
        start_publisher.publish(start_msg).map_err(|e| {
            let error_msg = format!("Failed to publish start message: {}", e);
            (ExceptionType::Unknown(error_msg.clone()), Some(error_msg))
        })?;

        info!(
            "Published skill start message to {}: skill_exec_id={}, start_msg_data={}",
            skill_instance.start_topic, skill_exec_id, &start_msg_data
        );

        // Wait for skill completion (monitor status_topic)
        // Standard status format: {"skill_id": "...", "state": "running"|"finished"|"error", "result": {...}, "errno": 0, ...}
        let timeout_duration = Duration::from_secs(300); // 5 minutes timeout
        let start_time = std::time::Instant::now();

        loop {
            let remaining_timeout = timeout_duration.saturating_sub(start_time.elapsed());
            if remaining_timeout.is_zero() {
                error!(
                    "Skill {} execution timeout after {:?}",
                    skill_name, timeout_duration
                );
                let error_msg = format!(
                    "Skill {} execution timeout after {:?}",
                    skill_name, timeout_duration
                );
                return Err((ExceptionType::SkillFailed, Some(error_msg)));
            }

            match timeout(remaining_timeout, status_rx.recv()).await {
                Ok(Some(status_str)) => {
                    debug!("Received status message: {}", status_str);

                    // Parse status JSON
                    let status_json: serde_json::Value = match serde_json::from_str(&status_str) {
                        Ok(v) => v,
                        Err(e) => {
                            warn!(
                                "Failed to parse status message as JSON: {}, message: {}",
                                e, status_str
                            );
                            continue;
                        }
                    };

                    // Check skill_id matches
                    if let Some(msg_skill_id) = status_json.get("skill_id").and_then(|v| v.as_str())
                    {
                        if msg_skill_id != skill_exec_id {
                            continue; // Not for this execution
                        }
                    }

                    // Check state
                    let state = status_json
                        .get("state")
                        .and_then(|v| v.as_str())
                        .unwrap_or("unknown");

                    // Check errno if present (0 = success, non-zero = error)
                    let errno = status_json
                        .get("errno")
                        .and_then(|v| v.as_i64())
                        .unwrap_or(0);

                    // Extract error message from status JSON
                    let error_msg = status_json
                        .get("error")
                        .and_then(|v| v.as_str())
                        .or_else(|| status_json.get("message").and_then(|v| v.as_str()))
                        .or_else(|| status_json.get("error_message").and_then(|v| v.as_str()))
                        .map(|s| s.to_string())
                        .unwrap_or_else(|| {
                            if errno != 0 {
                                format!("Skill execution failed with errno={}", errno)
                            } else {
                                format!("Skill execution failed: state={}", state)
                            }
                        });

                    match state {
                        "finished" => {
                            if errno == 0 {
                                info!(
                                    "Skill {} execution completed successfully: skill_exec_id={}",
                                    skill_name, skill_exec_id
                                );
                                return Ok(());
                            } else {
                                error!(
                                    "Skill {} execution finished with error: errno={}, skill_exec_id={}, error={}",
                                    skill_name, errno, skill_exec_id, error_msg
                                );
                                return Err((ExceptionType::SkillFailed, Some(error_msg)));
                            }
                        }
                        "error" => {
                            error!(
                                "Skill {} execution failed: skill_exec_id={}, errno={}, error={}",
                                skill_name, skill_exec_id, errno, error_msg
                            );
                            return Err((ExceptionType::SkillFailed, Some(error_msg)));
                        }
                        "running" => {
                            debug!(
                                "Skill {} still running: skill_exec_id={}",
                                skill_name, skill_exec_id
                            );
                            // Continue waiting
                        }
                        _ => {
                            debug!(
                                "Skill {} status: state={}, skill_exec_id={}",
                                skill_name, state, skill_exec_id
                            );
                            // Continue waiting
                        }
                    }
                }
                Ok(None) => {
                    // Channel closed
                    let error_msg = format!("Status channel closed for skill {}", skill_name);
                    error!("{}", error_msg);
                    return Err((ExceptionType::SkillFailed, Some(error_msg)));
                }
                Err(_) => {
                    // Timeout
                    let error_msg = format!(
                        "Skill {} execution timeout after 5 minutes: skill_exec_id={}",
                        skill_name, skill_exec_id
                    );
                    error!("{}", error_msg);
                    return Err((ExceptionType::Timeout, Some(error_msg)));
                }
            }
        }
    }

    /// Execute RTDL program step by step
    pub async fn execute_step(&self, task: &mut Task) -> Result<ExecutionResult, String> {
        debug!("execute_step called for task {}", task.task_id);

        // Parse RTDL if not already parsed
        let rtdl = task
            .context
            .rtdl
            .as_ref()
            .ok_or_else(|| "RTDL not set".to_string())?;
        let rtdl_type = task
            .context
            .rtdl_type
            .as_ref()
            .ok_or_else(|| "RTDL type not set".to_string())?;

        debug!(
            "executing step for task {}: instruction_pointer={}, rtdl_type={}, rtdl_length={}",
            task.task_id,
            task.context.rtdl_instruction_pointer,
            rtdl_type,
            rtdl.len()
        );

        let instructions = self.parse_rtdl(rtdl, rtdl_type)?;
        debug!(
            "parsed {} instructions for task {}",
            instructions.len(),
            task.task_id
        );

        // Check if we've completed all instructions
        if task.context.rtdl_instruction_pointer >= instructions.len() {
            debug!(
                "task {} completed all instructions ({}/{})",
                task.task_id,
                task.context.rtdl_instruction_pointer,
                instructions.len()
            );
            return Ok(ExecutionResult::Completed);
        }

        // Execute current instruction
        let instruction = &instructions[task.context.rtdl_instruction_pointer];
        debug!(
            "executing instruction {}/{} for task {}: object_id={}, type={}, name={}, params={:?}",
            task.context.rtdl_instruction_pointer + 1,
            instructions.len(),
            task.task_id,
            instruction.object_id,
            instruction.instruction_type,
            instruction.name,
            instruction.params
        );

        match self.execute_instruction(task, instruction).await {
            Ok(()) => {
                debug!(
                    "instruction {} succeeded for task {}",
                    task.context.rtdl_instruction_pointer, task.task_id
                );
                // Instruction succeeded, advance pointer
                task.context.advance_instruction_pointer();
                task.updated_at = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;

                // Check if we've completed all instructions
                if task.context.rtdl_instruction_pointer >= instructions.len() {
                    debug!("task {} completed all instructions", task.task_id);
                    Ok(ExecutionResult::Completed)
                } else {
                    debug!(
                        "task {} has more instructions to execute ({}/{})",
                        task.task_id,
                        task.context.rtdl_instruction_pointer,
                        instructions.len()
                    );
                    Ok(ExecutionResult::InProgress)
                }
            }
            Err((exception_type, error_message)) => {
                debug!(
                    "instruction {} failed for task {}: {:?}, error={:?}",
                    task.context.rtdl_instruction_pointer,
                    task.task_id,
                    exception_type,
                    error_message
                );
                // Handle exception
                let error_msg = error_message.unwrap_or_else(|| {
                    format!(
                        "Instruction {} failed: type={}, name={}",
                        task.context.rtdl_instruction_pointer,
                        instruction.instruction_type,
                        instruction.name
                    )
                });

                // Save error message to task
                task.error_message = Some(error_msg.clone());

                let recovery_action = self.exception_handler.handle_exception(
                    &mut task.context,
                    exception_type,
                    error_msg,
                );

                debug!(
                    "recovery action for task {}: {:?}, retry_count={}",
                    task.task_id, recovery_action, task.context.retry_count
                );

                task.updated_at = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;

                Ok(ExecutionResult::Exception(recovery_action))
            }
        }
    }
}

/// Execution result for a single step
#[derive(Debug, Clone)]
pub enum ExecutionResult {
    /// Execution completed successfully
    Completed,
    /// Execution in progress (more instructions to execute)
    InProgress,
    /// Exception occurred, recovery action needed
    Exception(RecoveryAction),
}
