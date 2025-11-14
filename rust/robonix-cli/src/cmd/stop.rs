use super::recipe_utils;
use crate::{Config, ProcessManager};
use anyhow::Result;

pub async fn execute(config: Config, target: String) -> Result<()> {
    let log_dir = config.package_storage_path.join("logs");
    let process_manager = ProcessManager::new(log_dir)?;

    // Get all items from active recipe
    let all_items = recipe_utils::get_recipe_items(&config)?;

    // Get all running processes
    let running_processes = process_manager.get_running_processes();

    // Filter running processes by pattern
    let processes_to_stop: Vec<crate::process::ProcessInfo> = if target == "all" {
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
        println!(
            "No matching running processes found for pattern: {}",
            target
        );
        return Ok(());
    }

    println!("Stopping {} process(es)...", processes_to_stop.len());

    let mut stopped = 0;
    let mut errors = 0;

    for proc in &processes_to_stop {
        println!("  Stopping {} {}...", proc.package_type, proc.std_name);
        match process_manager
            .stop_process(&proc.std_name, &proc.package_type)
            .await
        {
            Ok(_) => {
                println!("  ✓ Stopped {} {}", proc.package_type, proc.std_name);
                stopped += 1;
            }
            Err(e) => {
                eprintln!(
                    "  ✗ Failed to stop {} {}: {}",
                    proc.package_type, proc.std_name, e
                );
                errors += 1;
            }
        }
    }

    println!("\nSummary: {} stopped, {} errors", stopped, errors);

    Ok(())
}
