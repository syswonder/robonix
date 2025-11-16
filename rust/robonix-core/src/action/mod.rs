// SPDX-License-Identifier: MulanPSL-2.0
// Action Module
//
// This module interprets DSL code and executes skills.

pub mod executor;
pub mod interpreter;

use crate::mgmt::ManagementModule;
use executor::SkillExecutor;
use interpreter::DSLInterpreter;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tracing::{error, info};

/// Action Module
pub struct ActionModule {
    interpreter: Arc<DSLInterpreter>,
    executor: Arc<SkillExecutor>,
    mgmt: Option<Arc<ManagementModule>>,
}

impl ActionModule {
    pub fn new() -> Self {
        Self {
            interpreter: Arc::new(DSLInterpreter::new()),
            executor: Arc::new(SkillExecutor::new()),
            mgmt: None,
        }
    }

    pub fn set_mgmt(&mut self, mgmt: Arc<ManagementModule>) {
        self.mgmt = Some(mgmt.clone());
        // Use Arc::get_mut to get mutable reference
        if let Some(executor_mut) = Arc::get_mut(&mut self.executor) {
            executor_mut.set_mgmt(mgmt);
        }
    }

    /// Execute DSL code for a task
    pub async fn execute_dsl(&self, task_id: &str, dsl_code: &str) -> Result<(), String> {
        info!(task_id = %task_id, "Starting DSL execution");

        // Parse DSL
        let program = match self.interpreter.parse_dsl(dsl_code) {
            Ok(p) => p,
            Err(e) => {
                error!(task_id = %task_id, error = %e, "Failed to parse DSL");
                return Err(format!("DSL parsing failed: {}", e));
            }
        };

        // Execute program
        match self.executor.execute_program(&program, task_id).await {
            Ok(()) => {
                info!(task_id = %task_id, "DSL execution completed");
                Ok(())
            }
            Err(e) => {
                error!(task_id = %task_id, error = %e, "DSL execution failed");
                Err(e)
            }
        }
    }
}

// Service message types

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecuteDSLRequest {
    pub task_id: String,
    pub dsl_code: String,
}
impl ros2_client::Message for ExecuteDSLRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecuteDSLResponse {
    pub success: bool,
    pub error_message: String,
}
impl ros2_client::Message for ExecuteDSLResponse {}

