// SPDX-License-Identifier: MulanPSL-2.0
// Cognitive Planning Module
//
// This module manages tasks: natural language input -> LLM -> DSL -> skill execution

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::info;

// Task states
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TaskState {
    Pending,      // Task created, waiting for processing
    Generating,   // LLM is generating DSL code
    Parsing,      // Parsing DSL code
    Running,      // Executing skills
    Completed,    // Task completed successfully
    Failed,       // Task failed
    Cancelled,    // Task cancelled
}

// Task structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Task {
    pub task_id: String,
    pub natural_language: String, // Original natural language input
    pub dsl_code: Option<String>,  // Generated DSL code
    pub state: TaskState,
    pub error_message: Option<String>,
    pub created_at: u64,  // Unix timestamp in nanoseconds
    pub updated_at: u64,
}

// DSL instruction types (simplified temporary DSL)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DSLInstruction {
    CallSkill {
        skill_name: String,
        inputs: HashMap<String, String>, // Parameter name -> value (as string, will be parsed)
    },
    Sequence(Vec<DSLInstruction>), // Execute instructions sequentially
    Parallel(Vec<DSLInstruction>), // Execute instructions in parallel
    Condition {
        condition: String, // Simple condition expression
        then: Box<DSLInstruction>,
        else_: Option<Box<DSLInstruction>>,
    },
}

// Parsed DSL program
#[derive(Debug, Clone)]
pub struct DSLProgram {
    pub instructions: Vec<DSLInstruction>,
}

/// Cognitive Planning Module
pub struct PlanningModule {
    tasks: Arc<RwLock<HashMap<String, Task>>>,
    task_counter: Arc<AtomicU64>,
    #[allow(dead_code)]
    llm_models: Arc<RwLock<HashMap<String, crate::mgmt::LLMModel>>>, // Reference to management module
    #[allow(dead_code)]
    skills: Arc<RwLock<HashMap<String, crate::messages::Skill>>>,     // Reference to management module
}

impl PlanningModule {
    pub fn new() -> Self {
        Self {
            tasks: Arc::new(RwLock::new(HashMap::new())),
            task_counter: Arc::new(AtomicU64::new(0)),
            llm_models: Arc::new(RwLock::new(HashMap::new())),
            skills: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Set references to management module data
    pub fn set_llm_models(&self, _models: Arc<RwLock<HashMap<String, crate::mgmt::LLMModel>>>) {
        // Note: We can't directly replace Arc, so we'll need to update the design
        // For now, we'll access through management module
    }

    pub fn set_skills(&self, _skills: Arc<RwLock<HashMap<String, crate::messages::Skill>>>) {
        // Similar note as above
    }

    /// Create a new task from natural language
    pub async fn create_task(&self, natural_language: String) -> String {
        let counter = self.task_counter.fetch_add(1, Ordering::SeqCst);
        let task_id = format!("task_{}", counter);
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let task = Task {
            task_id: task_id.clone(),
            natural_language,
            dsl_code: None,
            state: TaskState::Pending,
            error_message: None,
            created_at: now,
            updated_at: now,
        };

        let mut tasks = self.tasks.write().await;
        tasks.insert(task_id.clone(), task);

        info!(task_id = %task_id, "Created new task");
        task_id
    }

    /// Get task by ID
    pub async fn get_task(&self, task_id: &str) -> Option<Task> {
        let tasks = self.tasks.read().await;
        tasks.get(task_id).cloned()
    }

    /// Get all tasks
    pub async fn get_all_tasks(&self) -> Vec<Task> {
        let tasks = self.tasks.read().await;
        tasks.values().cloned().collect()
    }

    /// Update task state
    pub async fn update_task_state(
        &self,
        task_id: &str,
        state: TaskState,
        error_message: Option<String>,
    ) -> bool {
        let mut tasks = self.tasks.write().await;
        if let Some(task) = tasks.get_mut(task_id) {
            task.state = state;
            task.error_message = error_message;
            task.updated_at = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            true
        } else {
            false
        }
    }

    /// Set DSL code for a task
    pub async fn set_task_dsl(&self, task_id: &str, dsl_code: String) -> bool {
        let mut tasks = self.tasks.write().await;
        if let Some(task) = tasks.get_mut(task_id) {
            task.dsl_code = Some(dsl_code);
            task.updated_at = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            true
        } else {
            false
        }
    }

    /// Parse DSL code (simplified parser)
    pub fn parse_dsl(&self, dsl_code: &str) -> Result<DSLProgram, String> {
        // Simple temporary DSL parser
        // Format: skill_name(param1=value1, param2=value2)
        // Or: sequence(instruction1, instruction2)
        // Or: parallel(instruction1, instruction2)

        // For now, implement a very simple parser
        // In production, this should be a proper DSL parser

        let mut instructions = Vec::new();

        // Split by lines and parse each
        for line in dsl_code.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue; // Skip empty lines and comments
            }

            // Try to parse as skill call: skill_name(param1=value1, param2=value2)
            if let Some(instruction) = self.parse_skill_call(line)? {
                instructions.push(instruction);
            }
        }

        Ok(DSLProgram { instructions })
    }

    fn parse_skill_call(&self, line: &str) -> Result<Option<DSLInstruction>, String> {
        // Simple parser for: skill_name(param1=value1, param2=value2)
        if let Some(open_paren) = line.find('(') {
            let skill_name = line[..open_paren].trim().to_string();
            if let Some(close_paren) = line.rfind(')') {
                let params_str = &line[open_paren + 1..close_paren];
                let mut inputs = HashMap::new();

                // Parse parameters
                for param in params_str.split(',') {
                    let param = param.trim();
                    if param.is_empty() {
                        continue;
                    }
                    if let Some(eq_pos) = param.find('=') {
                        let key = param[..eq_pos].trim().to_string();
                        let value = param[eq_pos + 1..].trim().to_string();
                        inputs.insert(key, value);
                    }
                }

                return Ok(Some(DSLInstruction::CallSkill {
                    skill_name,
                    inputs,
                }));
            }
        }

        // If no parentheses, treat as simple skill name
        if !line.is_empty() {
            return Ok(Some(DSLInstruction::CallSkill {
                skill_name: line.to_string(),
                inputs: HashMap::new(),
            }));
        }

        Ok(None)
    }

    /// Execute a DSL program
    pub async fn execute_dsl(
        &self,
        program: &DSLProgram,
        task_id: &str,
    ) -> Result<(), String> {
        // Update task state to running
        self.update_task_state(task_id, TaskState::Running, None).await;

        // Execute instructions sequentially for now
        for instruction in &program.instructions {
            match self.execute_instruction(instruction, task_id).await {
                Ok(()) => continue,
                Err(e) => {
                    self.update_task_state(task_id, TaskState::Failed, Some(e.clone()))
                        .await;
                    return Err(e);
                }
            }
        }

        // Task completed
        self.update_task_state(task_id, TaskState::Completed, None).await;
        Ok(())
    }

    async fn execute_instruction(
        &self,
        instruction: &DSLInstruction,
        task_id: &str,
    ) -> Result<(), String> {
        match instruction {
            DSLInstruction::CallSkill { skill_name, inputs: _ } => {
                info!(
                    task_id = %task_id,
                    skill = %skill_name,
                    "Executing skill call"
                );
                // In a real implementation, this would call the actual skill
                // For now, just log it
                Ok(())
            }
            DSLInstruction::Sequence(instructions) => {
                for inst in instructions {
                    Box::pin(self.execute_instruction(inst, task_id)).await?;
                }
                Ok(())
            }
            DSLInstruction::Parallel(instructions) => {
                // For now, execute sequentially
                // In production, this should be parallel
                for inst in instructions {
                    Box::pin(self.execute_instruction(inst, task_id)).await?;
                }
                Ok(())
            }
            DSLInstruction::Condition {
                condition: _,
                then,
                else_: _,
            } => {
                // Simple condition evaluation (always true for now)
                // In production, this should evaluate the condition
                Box::pin(self.execute_instruction(then, task_id)).await?;
                Ok(())
            }
        }
    }
}

// Service message types

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateTaskRequest {
    pub natural_language: String,
}
impl ros2_client::Message for CreateTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateTaskResponse {
    pub success: bool,
    pub error_message: String,
    pub task_id: String,
}
impl ros2_client::Message for CreateTaskResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetTaskRequest {
    pub task_id: String,
}
impl ros2_client::Message for GetTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetTaskResponse {
    pub success: bool,
    pub error_message: String,
    pub task: Option<Task>,
}
impl ros2_client::Message for GetTaskResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListTasksRequest {}
impl ros2_client::Message for ListTasksRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListTasksResponse {
    pub success: bool,
    pub error_message: String,
    pub tasks: Vec<Task>,
}
impl ros2_client::Message for ListTasksResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelTaskRequest {
    pub task_id: String,
}
impl ros2_client::Message for CancelTaskRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelTaskResponse {
    pub success: bool,
    pub error_message: String,
}
impl ros2_client::Message for CancelTaskResponse {}

