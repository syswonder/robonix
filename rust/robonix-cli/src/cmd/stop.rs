// SPDX-License-Identifier: MulanPSL-2.0
// Stop Command Module
//
// Stop command implementation for robonix-cli

use super::recipe_utils;
use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};
use crate::{Config, output};
use anyhow::Result;

pub async fn execute(config: Config, target: String) -> Result<()> {
    // Ensure daemon is running
    let client = DaemonClient::new()?;
    client.ensure_daemon_running().await?;

    // Get status from daemon
    let status_response = client.send_command(DaemonCommand::Status).await?;

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

    // Process stops in parallel for better performance
    let mut stop_tasks = Vec::new();
    for proc in &processes_to_stop {
        let proc = proc.clone();
        let task = tokio::spawn(async move {
            let client = match DaemonClient::new() {
                Ok(c) => c,
                Err(_) => {
                    return (false, true);
                }
            };
            let mut spinner = output::Spinner::new(format!(
                "Stopping {} {}...",
                proc.package_type, proc.std_name
            ));
            spinner.start();

            let result = client
                .send_command(DaemonCommand::Stop {
                    std_name: proc.std_name.clone(),
                    package_type: proc.package_type.clone(),
                })
                .await;

            let (success, error) = match result {
                Ok(DaemonResponse::Ok(_)) => {
                    spinner.finish_success(&format!(
                        "Stopped {} {}",
                        proc.package_type, proc.std_name
                    ));
                    (true, false)
                }
                Ok(DaemonResponse::OkWithDetails {
                    message,
                    pid,
                    pgid,
                    pids,
                }) => {
                    spinner.finish_success(&message);
                    if let Some(pgid) = pgid {
                        if let Some(ref pids_list) = pids {
                            if pids_list.len() > 1 {
                                output::sub_step(&format!(
                                    "  Process group {} ({} processes): {}",
                                    pgid,
                                    pids_list.len(),
                                    pids_list
                                        .iter()
                                        .map(|p| p.to_string())
                                        .collect::<Vec<_>>()
                                        .join(", ")
                                ));
                            } else {
                                output::sub_step(&format!("  PID: {}, PGID: {}", pid, pgid));
                            }
                        } else {
                            output::sub_step(&format!("  PID: {}, PGID: {}", pid, pgid));
                        }
                    } else {
                        output::sub_step(&format!("  PID: {}", pid));
                    }
                    (true, false)
                }
                Ok(DaemonResponse::Error(e)) => {
                    spinner.finish_error(&format!(
                        "Failed to stop {} {}: {}",
                        proc.package_type, proc.std_name, e
                    ));
                    (false, true)
                }
                Err(e) => {
                    spinner.finish_error(&format!(
                        "Failed to stop {} {}: {}",
                        proc.package_type, proc.std_name, e
                    ));
                    (false, true)
                }
                _ => {
                    spinner.finish_error(&format!(
                        "Unexpected response when stopping {} {}",
                        proc.package_type, proc.std_name
                    ));
                    (false, true)
                }
            };
            (success, error)
        });
        stop_tasks.push(task);
    }

    let mut stopped = 0;
    let mut errors = 0;
    for task in stop_tasks {
        if let Ok((success, error)) = task.await {
            if success {
                stopped += 1;
            }
            if error {
                errors += 1;
            }
        } else {
            errors += 1;
        }
    }

    output::summary(&format!("Summary: {} stopped, {} errors", stopped, errors));

    Ok(())
}
