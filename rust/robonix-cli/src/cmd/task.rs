use crate::config::Config;
use crate::output;
use crate::task::TaskClient;
use anyhow::Result;

pub async fn execute_create(config: Config, natural_language: String) -> Result<()> {
    output::action("Creating", "task");

    let client = TaskClient::new(config)?;
    let response = client.create(natural_language.clone()).await?;

    if !response.success {
        output::error(&format!("Failed to create task: {}", response.error_message));
        anyhow::bail!("Task creation failed: {}", response.error_message);
    }

    output::success(&format!("Task created: {}", response.task_id));
    output::info(&format!("  Natural language: {}", natural_language));
    output::info(&format!("  Task ID: {}", response.task_id));

    Ok(())
}

pub async fn execute_get(config: Config, task_id: String) -> Result<()> {
    output::action("Getting", &format!("task {}", task_id));

    let client = TaskClient::new(config)?;
    let response = client.get(task_id.clone()).await?;

    if !response.success {
        output::error(&format!("Failed to get task: {}", response.error_message));
        anyhow::bail!("Task query failed: {}", response.error_message);
    }

    if let Some(task) = response.task {
        output::success(&format!("Task {} found", task_id));
        output::info(&format!("  Task ID: {}", task.task_id));
        output::info(&format!("  State: {:?}", task.state));
        output::info(&format!("  Natural language: {}", task.natural_language));
        if let Some(ref dsl_code) = task.dsl_code {
            output::info("  DSL code:");
            for line in dsl_code.lines() {
                output::sub_step(&format!("    {}", line));
            }
        } else {
            output::info("  DSL code: (not generated yet)");
        }
        if let Some(ref error) = task.error_message {
            output::warning(&format!("  Error: {}", error));
        }
        output::info(&format!("  Created at: {}", task.created_at));
        output::info(&format!("  Updated at: {}", task.updated_at));
    } else {
        output::error(&format!("Task {} not found", task_id));
        anyhow::bail!("Task not found");
    }

    Ok(())
}

pub async fn execute_list(config: Config) -> Result<()> {
    output::action("Listing", "tasks");

    let client = TaskClient::new(config)?;
    let response = client.list().await?;

    if !response.success {
        output::error(&format!("Failed to list tasks: {}", response.error_message));
        anyhow::bail!("Task list failed: {}", response.error_message);
    }

    if response.tasks.is_empty() {
        output::info("No tasks found");
    } else {
        output::info(&format!("Found {} task(s):", response.tasks.len()));
        for task in &response.tasks {
            output::sub_step(&format!("  Task ID: {}", task.task_id));
            output::sub_step(&format!("    State: {:?}", task.state));
            output::sub_step(&format!("    Natural language: {}", task.natural_language));
            if let Some(ref dsl_code) = task.dsl_code {
                let preview = if dsl_code.len() > 50 {
                    format!("{}...", &dsl_code[..50])
                } else {
                    dsl_code.clone()
                };
                output::sub_step(&format!("    DSL: {}", preview));
            }
            output::info("");
        }
    }

    Ok(())
}

pub async fn execute_cancel(config: Config, task_id: String) -> Result<()> {
    output::action("Cancelling", &format!("task {}", task_id));

    let client = TaskClient::new(config)?;
    let response = client.cancel(task_id.clone()).await?;

    if !response.success {
        output::error(&format!("Failed to cancel task: {}", response.error_message));
        anyhow::bail!("Task cancellation failed: {}", response.error_message);
    }

    output::success(&format!("Task {} cancelled", task_id));

    Ok(())
}

