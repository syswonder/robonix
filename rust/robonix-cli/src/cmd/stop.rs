use super::recipe_utils;
use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};
use crate::{output, Config};
use anyhow::Result;

pub async fn execute(config: Config, target: String) -> Result<()> {
    // Ensure daemon is running
    let client = DaemonClient::new()?;
    client.ensure_daemon_running().await?;

    // Get status from daemon
    let status_response = client
        .send_command(DaemonCommand::Status)
        .await?;

    let running_processes = match status_response {
        DaemonResponse::Status(procs) => procs,
        _ => {
            anyhow::bail!("Unexpected response from daemon");
        }
    };

    // Get all items from active recipe
    let all_items = recipe_utils::get_recipe_items(&config)?;

    // Filter running processes by pattern
    let processes_to_stop: Vec<_> = if target == "all" {
        // Stop all running processes that are in the recipe
        let recipe_std_names: std::collections::HashSet<String> = all_items
            .iter()
            .map(|item| format!("{}::{}", item.package_type, item.std_name))
            .collect();

        running_processes
            .into_iter()
            .filter(|proc| {
                let key = format!("{}::{}", proc.package_type, proc.std_name);
                recipe_std_names.contains(&key)
            })
            .collect()
    } else {
        // Filter by pattern
        let matching_items = recipe_utils::filter_items(&all_items, &target);
        let matching_keys: std::collections::HashSet<String> = matching_items
            .iter()
            .map(|item| format!("{}::{}", item.package_type, item.std_name))
            .collect();

        running_processes
            .into_iter()
            .filter(|proc| {
                let key = format!("{}::{}", proc.package_type, proc.std_name);
                matching_keys.contains(&key)
            })
            .collect()
    };

    if processes_to_stop.is_empty() {
        output::warning(&format!(
            "No matching running processes found for pattern: {}",
            target
        ));
        return Ok(());
    }

    output::action(
        "Stopping",
        &format!("{} process(es)", processes_to_stop.len()),
    );

    let mut stopped = 0;
    let mut errors = 0;

    for proc in &processes_to_stop {
        output::sub_step(&format!(
            "Stopping {} {}...",
            proc.package_type, proc.std_name
        ));
            match client
                .send_command(DaemonCommand::Stop {
                std_name: proc.std_name.clone(),
                package_type: proc.package_type.clone(),
            })
            .await
        {
                Ok(DaemonResponse::Ok(_)) => {
                output::check(&format!("Stopped {} {}", proc.package_type, proc.std_name));
                stopped += 1;
            }
                Ok(DaemonResponse::Error(e)) => {
                output::cross(&format!(
                    "Failed to stop {} {}: {}",
                    proc.package_type, proc.std_name, e
                ));
                errors += 1;
            }
            Err(e) => {
                output::cross(&format!(
                    "Failed to stop {} {}: {}",
                    proc.package_type, proc.std_name, e
                ));
                errors += 1;
            }
            _ => {
                output::cross(&format!(
                    "Unexpected response when stopping {} {}",
                    proc.package_type, proc.std_name
                ));
                errors += 1;
            }
        }
    }

    output::summary(&format!("Summary: {} stopped, {} errors", stopped, errors));

    Ok(())
}
