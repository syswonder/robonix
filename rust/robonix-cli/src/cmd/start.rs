use super::recipe_utils;
use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};
use crate::{output, Config};
use anyhow::Result;

pub async fn execute(config: Config, target: String) -> Result<()> {
    // Ensure daemon is running
    let client = DaemonClient::new()?;
    client.ensure_daemon_running().await?;

    // Get all items from active recipe
    let all_items = recipe_utils::get_recipe_items(&config)?;

    // Filter items by pattern
    let items_to_start = if target == "all" {
        all_items
    } else {
        recipe_utils::filter_items(&all_items, &target)
    };

    if items_to_start.is_empty() {
        output::warning(&format!("No matching items found for pattern: {}", target));
        return Ok(());
    }

    output::action("Starting", &format!("{} item(s)", items_to_start.len()));

    let mut started = 0;
    let mut skipped = 0;
    let mut errors = 0;

    for item in &items_to_start {
        // Skip items without start_script (primitives and services typically don't need start scripts)
        if item.start_script.is_none() {
            output::warning(&format!(
                "Skipping {} {} (no start_script defined)",
                item.package_type, item.std_name
            ));
            skipped += 1;
            continue;
        }

        let mut spinner = output::Spinner::new(format!(
            "Starting {} {}...",
            item.package_type, item.std_name
        ));
        spinner.start();

        let result = client
            .send_command(DaemonCommand::Start {
                package_name: item.package_name.clone(),
                std_name: item.std_name.clone(),
                package_type: item.package_type.clone(),
                package_path: item.package_path.clone(),
                start_script: item.start_script.as_ref().unwrap().clone(),
                robonix_sdk_path: config.robonix_sdk_path.clone(),
            })
            .await;

        match result {
            Ok(DaemonResponse::Ok(_)) => {
                spinner.finish_success(&format!("Started {} {}", item.package_type, item.std_name));
                started += 1;
            }
            Ok(DaemonResponse::OkWithDetails {
                message,
                pid,
                pgid,
                pids,
            }) => {
                spinner.finish_success(&message);

                // Display process group info first
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

                // Wait a bit more for process to fully start and spawn children
                tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;

                // Get and display process tree
                #[cfg(unix)]
                {
                    use crate::process::ProcessManager;
                    match ProcessManager::get_process_tree(pid) {
                        Ok(tree) => {
                            output::sub_step("Process tree:");
                            let tree_str = tree.format_tree("", true);
                            // Print each line
                            for line in tree_str.lines() {
                                if !line.trim().is_empty() {
                                    println!("    {}", line);
                                }
                            }
                        }
                        Err(e) => {
                            output::sub_step(&format!("  (tree fetch failed: {})", e));
                        }
                    }
                }
                #[cfg(not(unix))]
                {
                    // Already displayed above
                }

                started += 1;
            }
            Ok(DaemonResponse::Error(e)) => {
                if e.contains("already running") {
                    spinner.finish_success(&format!(
                        "Skipped {} {} (already running)",
                        item.package_type, item.std_name
                    ));
                    skipped += 1;
                } else {
                    spinner.finish_error(&format!(
                        "Failed to start {} {}: {}",
                        item.package_type, item.std_name, e
                    ));
                    errors += 1;
                }
            }
            Err(e) => {
                spinner.finish_error(&format!(
                    "Failed to start {} {}: {}",
                    item.package_type, item.std_name, e
                ));
                errors += 1;
            }
            _ => {
                spinner.finish_error(&format!(
                    "Unexpected response when starting {} {}",
                    item.package_type, item.std_name
                ));
                errors += 1;
            }
        }
    }

    output::summary(&format!(
        "Summary: {} started, {} skipped, {} errors",
        started, skipped, errors
    ));

    Ok(())
}
