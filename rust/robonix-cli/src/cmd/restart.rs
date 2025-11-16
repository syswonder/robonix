use super::recipe_utils;
use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};
use crate::{output, Config};
use anyhow::Result;
use tokio::time::{sleep, Duration};

pub async fn execute(config: Config, target: String) -> Result<()> {
    // Ensure daemon is running
    let client = DaemonClient::new()?;
    client.ensure_daemon_running().await?;

    // Get all items from active recipe
    let all_items = recipe_utils::get_recipe_items(&config)?;

    // Filter items by pattern
    let items_to_restart = if target == "all" {
        all_items
    } else {
        recipe_utils::filter_items(&all_items, &target)
    };

    if items_to_restart.is_empty() {
        output::warning(&format!("No matching items found for pattern: {}", target));
        return Ok(());
    }

    output::action(
        "Restarting",
        &format!("{} item(s)", items_to_restart.len()),
    );

    let mut restarted = 0;
    let mut started = 0;
    let mut errors = 0;

    // Step 1: Stop running processes
    let status_response = client
        .send_command(DaemonCommand::Status)
        .await?;

    let running_processes = match status_response {
        DaemonResponse::Status(procs) => procs,
        _ => {
            anyhow::bail!("Unexpected response from daemon");
        }
    };

    let items_to_stop: Vec<_> = items_to_restart
        .iter()
        .filter(|item| {
            running_processes.iter().any(|proc| {
                proc.std_name == item.std_name && proc.package_type == item.package_type
            })
        })
        .collect();

    if !items_to_stop.is_empty() {
        output::step("Stopping", &format!("{} process(es)", items_to_stop.len()));

        for item in &items_to_stop {
            output::sub_step(&format!(
                "Stopping {} {}...",
                item.package_type, item.std_name
            ));
            match client
                .send_command(DaemonCommand::Stop {
                    std_name: item.std_name.clone(),
                    package_type: item.package_type.clone(),
                })
                .await
            {
                Ok(DaemonResponse::Ok(_)) => {
                    output::check(&format!("Stopped {} {}", item.package_type, item.std_name));
                }
                Ok(DaemonResponse::Error(e)) => {
                    output::cross(&format!(
                        "Failed to stop {} {}: {}",
                        item.package_type, item.std_name, e
                    ));
                    errors += 1;
                }
                Err(e) => {
                    output::cross(&format!(
                        "Failed to stop {} {}: {}",
                        item.package_type, item.std_name, e
                    ));
                    errors += 1;
                }
                _ => {
                    output::cross(&format!(
                        "Unexpected response when stopping {} {}",
                        item.package_type, item.std_name
                    ));
                    errors += 1;
                }
            }
        }

        // Wait a bit for processes to fully stop
        output::sub_step("Waiting for processes to stop...");
        sleep(Duration::from_millis(500)).await;
    }

    // Step 2: Start processes
    output::step("Starting", &format!("{} item(s)", items_to_restart.len()));

    for item in &items_to_restart {
        output::sub_step(&format!(
            "Starting {} {}...",
            item.package_type, item.std_name
        ));
        match client
            .send_command(DaemonCommand::Start {
                package_name: item.package_name.clone(),
                std_name: item.std_name.clone(),
                package_type: item.package_type.clone(),
                package_path: item.package_path.clone(),
                start_script: item.start_script.clone(),
                robonix_msg_path: config.robonix_msg_path.clone(),
            })
            .await
        {
            Ok(DaemonResponse::Ok(_)) => {
                output::check(&format!("Started {} {}", item.package_type, item.std_name));
                if items_to_stop
                    .iter()
                    .any(|i| i.std_name == item.std_name && i.package_type == item.package_type)
                {
                    restarted += 1;
                } else {
                    started += 1;
                }
            }
            Ok(DaemonResponse::Error(e)) => {
                if e.contains("already running") {
                    // Already running, count as restarted if it was in stop list
                    if items_to_stop
                        .iter()
                        .any(|i| i.std_name == item.std_name && i.package_type == item.package_type)
                    {
                        restarted += 1;
                    } else {
                        started += 1;
                    }
                } else {
                    output::cross(&format!(
                        "Failed to start {} {}: {}",
                        item.package_type, item.std_name, e
                    ));
                    errors += 1;
                }
            }
            Err(e) => {
                output::cross(&format!(
                    "Failed to start {} {}: {}",
                    item.package_type, item.std_name, e
                ));
                errors += 1;
            }
            _ => {
                output::cross(&format!(
                    "Unexpected response when starting {} {}",
                    item.package_type, item.std_name
                ));
                errors += 1;
            }
        }
    }

    output::summary(&format!(
        "Summary: {} restarted, {} started, {} errors",
        restarted, started, errors
    ));

    Ok(())
}

