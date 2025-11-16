// SPDX-License-Identifier: MulanPSL-2.0
// Skill Executor Module
//
// This module executes skills by sending parameters to skill inputs and collecting outputs.

use crate::action::interpreter::{DSLInstruction, DSLProgram};
use crate::mgmt::ManagementModule;
use std::collections::HashMap;
use std::sync::Arc;
use tracing::{error, info, warn};

/// Skill Executor
pub struct SkillExecutor {
    mgmt: Option<Arc<ManagementModule>>,
}

impl SkillExecutor {
    pub fn new() -> Self {
        Self {
            mgmt: None,
        }
    }

    pub fn set_mgmt(&mut self, mgmt: Arc<ManagementModule>) {
        self.mgmt = Some(mgmt);
    }

    /// Execute a DSL program
    pub async fn execute_program(&self, program: &DSLProgram, task_id: &str) -> Result<(), String> {
        // Execute instructions sequentially for now
        for instruction in &program.instructions {
            match self.execute_instruction(instruction, task_id).await {
                Ok(()) => continue,
                Err(e) => {
                    error!(task_id = %task_id, error = %e, "Instruction execution failed");
                    return Err(e);
                }
            }
        }

        Ok(())
    }

    async fn execute_instruction(
        &self,
        instruction: &DSLInstruction,
        task_id: &str,
    ) -> Result<(), String> {
        info!(
            task_id = %task_id,
            skill = %instruction.skill_name,
            "Executing skill call"
        );
        self.execute_skill(&instruction.skill_name, &instruction.inputs, task_id).await
    }

    async fn execute_skill(
        &self,
        skill_name: &str,
        inputs: &HashMap<String, String>,
        _task_id: &str,
    ) -> Result<(), String> {
        let mgmt = self.mgmt.as_ref().ok_or("Management module not set")?;

        // Query skill to get input/output channels
        let query_req = crate::messages::QueryCapSklRequest {
            std_name: skill_name.to_string(),
            impl_id: String::new(), // Use default
            requirements: Vec::new(),
        };

        let query_resp = mgmt.query(query_req).await;
        if !query_resp.success {
            return Err(format!("Skill '{}' not found", skill_name));
        }

        info!(
            skill = %skill_name,
            input_channels = ?query_resp.input_channels,
            "Found skill, sending inputs"
        );

        // For each input parameter, send to the corresponding channel
        // This is a simplified implementation - in production, we need to:
        // 1. Parse input values according to ROS message types
        // 2. Create appropriate ROS messages
        // 3. Publish to the input channels
        // 4. Subscribe to output channels and collect results

        // For now, just log what would be sent
        for (param_name, param_value) in inputs {
            // Find corresponding channel
            if let Some(index) = query_resp.input_names.iter().position(|n| n == param_name) {
                if let Some(channel) = query_resp.input_channels.get(index) {
                    info!(
                        skill = %skill_name,
                        param = %param_name,
                        value = %param_value,
                        channel = %channel,
                        "Would send parameter to channel"
                    );
                    // TODO: Actually publish to the channel
                    // This requires:
                    // - Getting the ROS message type from query_resp.input_types[index]
                    // - Creating a publisher for that channel
                    // - Serializing the parameter value into the message
                    // - Publishing the message
                }
            } else {
                warn!(
                    skill = %skill_name,
                    param = %param_name,
                    "Parameter not found in skill inputs"
                );
            }
        }

        // TODO: Subscribe to output channels and wait for results
        // For now, just return success
        Ok(())
    }
}

