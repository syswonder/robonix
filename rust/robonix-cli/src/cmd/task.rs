use crate::config::Config;
use crate::output;
use crate::task::TaskClient;
use anyhow::Result;

pub async fn execute_create(config: Config, natural_language: String) -> Result<()> {
    output::action("Submitting", "task");

    let client = TaskClient::new(config)?;
    let response = client.submit(natural_language.clone(), serde_json::json!({})).await?;

    output::success(&format!("Task submitted: {}", response.task_id));
    output::info(&format!("  Natural language: {}", natural_language));
    output::info(&format!("  Task ID: {}", response.task_id));

    Ok(())
}

pub async fn execute_get(config: Config, task_id: String) -> Result<()> {
    output::action("Getting", &format!("task {}", task_id));

    let client = TaskClient::new(config)?;
    
    // Get task status
    let status_response = client.status(task_id.clone()).await?;
    output::info(&format!("  Status: {}", status_response.status));
    
    // Get task result
    let result_response = client.result(task_id.clone()).await?;
    // Parse JSON string and pretty print
    let result_value: serde_json::Value = serde_json::from_str(&result_response.result)
        .unwrap_or_else(|_| serde_json::json!({"error": "Failed to parse result JSON"}));
    output::info(&format!("  Result: {}", serde_json::to_string_pretty(&result_value)?));

    Ok(())
}

pub async fn execute_list(_config: Config) -> Result<()> {
    output::warning("List tasks functionality is not yet implemented in the new EAIOS API");
    output::info("Use 'task get <task_id>' to query individual tasks");
    Ok(())
}

pub async fn execute_cancel(_config: Config, _task_id: String) -> Result<()> {
    output::warning("Cancel task functionality is not yet implemented in the new EAIOS API");
    Ok(())
}

