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
                started += 1;
            }
            Ok(DaemonResponse::Error(e)) => {
                if e.contains("already running") {
                    output::sub_step(&format!(
                        "Skipping {} {} (already running)",
                        item.package_type, item.std_name
                    ));
                    skipped += 1;
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
        "Summary: {} started, {} skipped, {} errors",
        started, skipped, errors
    ));

    Ok(())
}
