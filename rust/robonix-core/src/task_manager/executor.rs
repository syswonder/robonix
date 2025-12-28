// SPDX-License-Identifier: MulanPSL-2.0
// RTDL Executor Module
//
// Executes RTDL (Real-Time Decision Logic) programs
// Currently supports simple list-style RTDL (skill invocation list)

use crate::skill_library::SkillLibrary;
use crate::task_manager::context::TaskContext;
use crate::task_manager::exception::{ExceptionHandler, ExceptionType, RecoveryAction};
use log::{debug, error, info, warn};
use serde_json::Value;
use std::sync::Arc;

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
}

impl RtdlExecutor {
    pub fn new(skill_library: Arc<SkillLibrary>) -> Self {
        Self {
            skill_library,
            exception_handler: ExceptionHandler,
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
    pub async fn execute_instruction(
        &self,
        context: &mut TaskContext,
        instruction: &RtdlInstruction,
    ) -> Result<(), ExceptionType> {
        debug!(
            "executing instruction: task_id={}, object_id={}, type={}, name={}, params={:?}",
            context.task_id,
            instruction.object_id,
            instruction.instruction_type,
            instruction.name,
            instruction.params
        );
        debug!(
            "task {} context before instruction: state={:?}, retry_count={}, instruction_pointer={}",
            context.task_id,
            context.execution_state,
            context.retry_count,
            context.rtdl_instruction_pointer
        );

        match instruction.instruction_type.as_str() {
            "skill" => {
                debug!("dispatching to skill execution handler");
                let result = self
                    .execute_skill(context, &instruction.name, &instruction.params)
                    .await;
                match &result {
                    Ok(_) => debug!("skill execution succeeded"),
                    Err(e) => debug!("skill execution failed: {:?}", e),
                }
                result
            }
            "service" => {
                debug!("service execution requested but not implemented");
                // TODO: Implement service execution
                error!("service execution not yet implemented");
                Err(ExceptionType::Unknown(
                    "Service execution not implemented".to_string(),
                ))
            }
            "primitive" => {
                debug!("primitive execution requested but not implemented");
                // TODO: Implement primitive execution
                error!("primitive execution not yet implemented");
                Err(ExceptionType::Unknown(
                    "Primitive execution not implemented".to_string(),
                ))
            }
            _ => {
                debug!("unknown instruction type: {}", instruction.instruction_type);
                warn!("unknown instruction type: {}", instruction.instruction_type);
                Err(ExceptionType::Unknown(format!(
                    "Unknown instruction type: {}",
                    instruction.instruction_type
                )))
            }
        }
    }

    /// Execute a skill instruction
    async fn execute_skill(
        &self,
        context: &mut TaskContext,
        skill_name: &str,
        params: &Value,
    ) -> Result<(), ExceptionType> {
        debug!("querying skill library for skill: {}", skill_name);

        // Query skill from skill library
        let query_req = crate::ros_idl::skill::QuerySkillRequest {
            name: skill_name.to_string(),
            filter: "{}".to_string(),
        };

        let query_resp = self.skill_library.query_skill(query_req).await;

        if query_resp.instances.is_empty() {
            debug!("skill {} not found in skill library", skill_name);
            return Err(ExceptionType::SkillFailed);
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

        // TODO: Actually invoke the skill via ROS2
        // For now, we'll simulate skill execution
        info!(
            "executing skill: task_id={}, skill_name={}, skill_id={}",
            context.task_id, skill_name, skill_instance.skill_id
        );

        debug!("skill execution params: {:?}", params);

        // Simulate skill execution (replace with actual ROS2 skill invocation)
        // This is a placeholder - actual implementation would:
        // 1. Create ROS2 service client for the skill
        // 2. Call the skill service with params
        // 3. Wait for skill completion
        // 4. Handle errors/timeouts

        // For now, just return success
        debug!("skill {} execution completed successfully", skill_name);
        Ok(())
    }

    /// Execute RTDL program step by step
    pub async fn execute_step(&self, context: &mut TaskContext) -> Result<ExecutionResult, String> {
        debug!("execute_step called for task {}", context.task_id);

        // Parse RTDL if not already parsed
        let rtdl = context
            .rtdl
            .as_ref()
            .ok_or_else(|| "RTDL not set".to_string())?;
        let rtdl_type = context
            .rtdl_type
            .as_ref()
            .ok_or_else(|| "RTDL type not set".to_string())?;

        debug!(
            "executing step for task {}: instruction_pointer={}, rtdl_type={}, rtdl_length={}",
            context.task_id,
            context.rtdl_instruction_pointer,
            rtdl_type,
            rtdl.len()
        );

        let instructions = self.parse_rtdl(rtdl, rtdl_type)?;
        debug!(
            "parsed {} instructions for task {}",
            instructions.len(),
            context.task_id
        );

        // Check if we've completed all instructions
        if context.rtdl_instruction_pointer >= instructions.len() {
            debug!(
                "task {} completed all instructions ({}/{})",
                context.task_id,
                context.rtdl_instruction_pointer,
                instructions.len()
            );
            return Ok(ExecutionResult::Completed);
        }

        // Execute current instruction
        let instruction = &instructions[context.rtdl_instruction_pointer];
        debug!(
            "executing instruction {}/{} for task {}: object_id={}, type={}, name={}, params={:?}",
            context.rtdl_instruction_pointer + 1,
            instructions.len(),
            context.task_id,
            instruction.object_id,
            instruction.instruction_type,
            instruction.name,
            instruction.params
        );

        match self.execute_instruction(context, instruction).await {
            Ok(()) => {
                debug!(
                    "instruction {} succeeded for task {}",
                    context.rtdl_instruction_pointer, context.task_id
                );
                // Instruction succeeded, advance pointer
                context.advance_instruction_pointer();

                // Check if we've completed all instructions
                if context.rtdl_instruction_pointer >= instructions.len() {
                    debug!("task {} completed all instructions", context.task_id);
                    Ok(ExecutionResult::Completed)
                } else {
                    debug!(
                        "task {} has more instructions to execute ({}/{})",
                        context.task_id,
                        context.rtdl_instruction_pointer,
                        instructions.len()
                    );
                    Ok(ExecutionResult::InProgress)
                }
            }
            Err(exception_type) => {
                debug!(
                    "instruction {} failed for task {}: {:?}",
                    context.rtdl_instruction_pointer, context.task_id, exception_type
                );
                // Handle exception
                let error_msg = format!(
                    "Instruction {} failed: type={}, name={}",
                    context.rtdl_instruction_pointer,
                    instruction.instruction_type,
                    instruction.name
                );

                let recovery_action =
                    self.exception_handler
                        .handle_exception(context, exception_type, error_msg);

                debug!(
                    "recovery action for task {}: {:?}, retry_count={}",
                    context.task_id, recovery_action, context.retry_count
                );

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
