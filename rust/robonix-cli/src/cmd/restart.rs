use super::recipe_utils;
use crate::{output, Config, ProcessManager};
use anyhow::Result;
use tokio::time::{sleep, Duration};

pub async fn execute(config: Config, target: String) -> Result<()> {
    let log_dir = config.package_storage_path.join("logs");
    let process_manager = ProcessManager::new(log_dir)?;

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
    let running_processes = process_manager.get_running_processes();
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
            match process_manager
                .stop_process(&item.std_name, &item.package_type)
                .await
            {
                Ok(_) => {
                    output::check(&format!("Stopped {} {}", item.package_type, item.std_name));
                }
                Err(e) => {
                    output::cross(&format!(
                        "Failed to stop {} {}: {}",
                        item.package_type, item.std_name, e
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
        match process_manager
            .start_process(
                &item.package_name,
                &item.std_name,
                &item.package_type,
                &item.package_path,
                &item.start_script,
                config.robonix_msg_path.as_ref(),
            )
            .await
        {
            Ok(_) => {
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
            Err(e) => {
                output::cross(&format!(
                    "Failed to start {} {}: {}",
                    item.package_type, item.std_name, e
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

