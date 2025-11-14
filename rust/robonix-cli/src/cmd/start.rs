use super::recipe_utils;
use crate::{output, Config, ProcessManager};
use anyhow::Result;

pub async fn execute(config: Config, target: String) -> Result<()> {
    let log_dir = config.package_storage_path.join("logs");
    let process_manager = ProcessManager::new(log_dir)?;

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
        // Check if already running
        if process_manager.is_running(&item.std_name, &item.package_type) {
            output::sub_step(&format!(
                "Skipping {} {} (already running)",
                item.package_type, item.std_name
            ));
            skipped += 1;
            continue;
        }

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
            )
            .await
        {
            Ok(_) => {
                output::check(&format!("Started {} {}", item.package_type, item.std_name));
                started += 1;
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
        "Summary: {} started, {} skipped, {} errors",
        started, skipped, errors
    ));

    Ok(())
}
