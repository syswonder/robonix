// SPDX-License-Identifier: MulanPSL-2.0
// Task Command Module
//
// Task command implementation for robonix-cli

use crate::config::Config;
use crate::output;
use crate::task::TaskClient;
use anyhow::Result;

pub async fn execute_create(config: Config, natural_language: String) -> Result<()> {
    output::action("Submitting", "task");

    let client = TaskClient::new(config)?;
    let response = client
        .submit(natural_language.clone(), serde_json::json!({}))
        .await?;

    output::success(&format!("Task submitted: {}", response.task_id));
    output::info(&format!("  Natural language: {}", natural_language));
    output::info(&format!("  Task ID: {}", response.task_id));

    Ok(())
}

pub async fn execute_get(config: Config, task_id: String) -> Result<()> {
    output::action("Getting", &format!("task {}", task_id));

    let client = TaskClient::new(config)?;

    // Get task data (comprehensive task information)
    let data_response = client.data(task_id.clone()).await?;
    // Parse JSON string and pretty print
    let result_value: serde_json::Value = serde_json::from_str(&data_response.data)
        .unwrap_or_else(|_| serde_json::json!({"error": "Failed to parse data JSON"}));

    // Display all task information in a structured way
    if let Some(task_id_val) = result_value.get("task_id") {
        output::info(&format!("Task ID: {}", task_id_val));
    }
    if let Some(desc) = result_value.get("description") {
        output::info(&format!("Description: {}", desc));
    }
    if let Some(state) = result_value.get("state") {
        output::info(&format!("State: {}", state));
    }
    if let Some(exec_state) = result_value.get("execution_state") {
        output::info(&format!("Execution State: {}", exec_state));
    }
    if let Some(priority) = result_value.get("priority") {
        output::info(&format!("Priority: {}", priority));
    }
    if let Some(retry_count) = result_value.get("retry_count") {
        output::info(&format!("Retry Count: {}", retry_count));
    }
    if let Some(rtdl) = result_value.get("rtdl") {
        output::info(&format!("RTDL: {}", rtdl));
    }
    if let Some(rtdl_type) = result_value.get("rtdl_type") {
        output::info(&format!("RTDL Type: {}", rtdl_type));
    }
    if let Some(rtdl_ptr) = result_value.get("rtdl_instruction_pointer") {
        output::info(&format!("RTDL Instruction Pointer: {}", rtdl_ptr));
    }
    if let Some(obj_count) = result_value.get("object_graph_count") {
        output::info(&format!("Object Graph Count: {}", obj_count));
    }
    if let Some(obj_graph) = result_value.get("object_graph") {
        output::info(&format!(
            "Object Graph: {}",
            serde_json::to_string_pretty(obj_graph)?
        ));
    }
    if let Some(exception) = result_value.get("last_exception") {
        output::warning(&format!("Last Exception: {}", exception));
    }
    if let Some(error_msg) = result_value.get("error_message") {
        output::error(&format!("Error Message: {}", error_msg));
    }
    if let Some(result) = result_value.get("result") {
        output::info(&format!(
            "Result: {}",
            serde_json::to_string_pretty(result)?
        ));
    }
    if let Some(created_at) = result_value.get("created_at") {
        output::info(&format!("Created At: {}", created_at));
    }
    if let Some(updated_at) = result_value.get("updated_at") {
        output::info(&format!("Updated At: {}", updated_at));
    }

    Ok(())
}

pub async fn execute_list(_config: Config) -> Result<()> {
    output::warning("List tasks functionality is not yet implemented in the new EAIOS API");
    output::info("Use 'task get <task_id>' to query individual tasks");
    Ok(())
}

pub async fn execute_cancel(config: Config, task_id: String) -> Result<()> {
    output::action("Cancelling", &format!("task {}", task_id));

    let client = TaskClient::new(config)?;
    let response = client.cancel(task_id.clone()).await?;

    if response.success {
        output::success(&format!("Task {} cancelled", task_id));
    } else {
        output::warning(&format!(
            "Task {} could not be cancelled (may already be finished, failed, or cancelled)",
            task_id
        ));
    }
    Ok(())
}
