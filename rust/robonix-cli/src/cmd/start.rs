// SPDX-License-Identifier: MulanPSL-2.0
// Start Command Module
//
// Start command implementation for robonix-cli

use super::recipe_utils;
use crate::Config;
use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};
use crate::database::PackageDatabase;
use crate::output;
use anyhow::Result;
use robonix_core::ros_idl::service_registry::RegisterServiceRequest;
use serde_yaml::Value;
use std::process::Command;
use std::time::Duration;

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
                tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;

                // Sync state to core: wait for service to be available and register it
                // This tells core that the service is now running and ready
                // Note: Initial registration happens via 'register' command, this is just state sync
                if item.package_type == "srv" {
                    output::sub_step(&format!(
                        "Syncing service {} state to core...",
                        item.std_name
                    ));
                    if let Err(e) = wait_and_register_service(
                        &client,
                        &item.package_name,
                        &item.std_name,
                        &config,
                    )
                    .await
                    {
                        output::warning(&format!(
                            "Failed to sync service {} state to core: {}",
                            item.std_name, e
                        ));
                    } else {
                        output::sub_step(&format!(
                            "Successfully synced service {} state to core",
                            item.std_name
                        ));
                    }
                }
                // Note: For primitives and skills, state sync can be added later if needed

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

/// Wait for ROS2 service to be available and register it to core
pub async fn wait_and_register_service(
    client: &DaemonClient,
    package_name: &str,
    service_name: &str,
    config: &Config,
) -> Result<()> {
    // Load package database to get manifest path
    let db = PackageDatabase::load(&config.package_storage_path)?;
    let pkg_info = db
        .find_by_name(package_name)
        .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

    // Load manifest to get service entry
    let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
    let manifest: Value = serde_yaml::from_str(&manifest_content)?;

    // Find service in manifest
    let service = find_service_in_manifest(&manifest, service_name)?
        .ok_or_else(|| anyhow::anyhow!("Service {} not found in manifest", service_name))?;

    let entry = service["entry"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("Service entry not found"))?;

    // Wait for ROS2 service to be available (check via ros2 service list)
    // ROS2 service discovery can take 5-15 seconds, especially on first startup
    output::sub_step(&format!("Waiting for service {} to be available...", entry));
    let max_wait = Duration::from_secs(20); // Increased from 10 to 20 seconds
    let check_interval = Duration::from_millis(500); // Increased from 200ms to 500ms for less frequent checks
    let start_time = std::time::Instant::now();

    let sdk_path = config
        .robonix_sdk_path
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("robonix_sdk_path not configured"))?;

    let mut last_log_elapsed = Duration::from_secs(0);
    let mut check_count = 0;

    while start_time.elapsed() < max_wait {
        // Check if service is available using ros2 service list
        // Use bash -c to source SDK and run ros2 command
        let output = Command::new("bash")
            .arg("-c")
            .arg(format!(
                "source {}/install/setup.bash && ros2 service list 2>/dev/null",
                sdk_path.display()
            ))
            .output();

        check_count += 1;

        if let Ok(output) = output {
            if output.status.success() {
                if let Ok(service_list) = String::from_utf8(output.stdout) {
                    if service_list.lines().any(|line| line.trim() == entry) {
                        let elapsed = start_time.elapsed();
                        output::sub_step(&format!(
                            "Service {} is available (found after {:.1}s, {} checks)",
                            entry,
                            elapsed.as_secs_f64(),
                            check_count
                        ));
                        break;
                    }
                }
            }
        }

        // Log progress every 5 seconds
        let elapsed = start_time.elapsed();
        if elapsed.as_secs() >= 5 && (elapsed - last_log_elapsed).as_secs() >= 5 {
            output::sub_step(&format!(
                "Still waiting for service {}... (elapsed: {:.1}s)",
                entry,
                elapsed.as_secs_f64()
            ));
            last_log_elapsed = elapsed;
        }

        tokio::time::sleep(check_interval).await;
    }

    if start_time.elapsed() >= max_wait {
        // Try one more time with verbose output for debugging
        let debug_output = Command::new("bash")
            .arg("-c")
            .arg(format!(
                "source {}/install/setup.bash && ros2 service list",
                sdk_path.display()
            ))
            .output();

        let debug_info = if let Ok(output) = debug_output {
            if let Ok(service_list) = String::from_utf8(output.stdout) {
                format!(
                    "Available services: {}",
                    service_list.lines().take(10).collect::<Vec<_>>().join(", ")
                )
            } else {
                "Failed to parse service list".to_string()
            }
        } else {
            "Failed to run ros2 service list".to_string()
        };

        return Err(anyhow::anyhow!(
            "Service {} did not become available within 20 seconds. {}",
            entry,
            debug_info
        ));
    }

    // Register service to core (this is the second registration to update status to "started")
    let srv_type = service["srv_type"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("Service srv_type not found"))?
        .to_string();

    let metadata_str = service["metadata"].as_str().unwrap_or("{}");
    serde_json::from_str::<serde_json::Value>(metadata_str)
        .map_err(|e| anyhow::anyhow!("Invalid metadata JSON: {}", e))?;

    let version = service["version"].as_str().unwrap_or("1.0.0").to_string();

    output::sub_step(&format!(
        "Registering service {} to core to update status to 'started'...",
        service_name
    ));

    let request = RegisterServiceRequest {
        name: service_name.to_string(),
        srv_type,
        entry: entry.to_string(),
        metadata: metadata_str.to_string(),
        provider: package_name.to_string(),
        version,
        node_id: config.effective_node_id(),
    };

    let request_json = serde_json::to_string(&request)?;
    let response = client
        .send_command(DaemonCommand::CallRegisterService {
            request: request_json,
        })
        .await?;

    match response {
        DaemonResponse::RegisterServiceResponse { response: _ } => {
            output::sub_step(&format!(
                "Synced service {} state to core (service is now available)",
                service_name
            ));
            Ok(())
        }
        DaemonResponse::Error(e) => Err(anyhow::anyhow!("State sync failed: {}", e)),
        _ => Err(anyhow::anyhow!("Unexpected response from daemon")),
    }
}

fn find_service_in_manifest<'a>(manifest: &'a Value, name: &str) -> Result<Option<&'a Value>> {
    if let Some(services) = manifest["services"].as_sequence() {
        for service in services {
            if let Some(service_name) = service["name"].as_str() {
                if service_name == name {
                    return Ok(Some(service));
                }
            }
        }
    }
    Ok(None)
}
