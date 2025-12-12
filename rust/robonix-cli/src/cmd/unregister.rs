use crate::{Config, PackageUnregistrar, ProcessManager};
use anyhow::Result;
use std::path::PathBuf;

pub async fn execute(config: Config, target: String) -> Result<()> {
    // Check if there are any running processes before unregistering
    let log_dir = config.package_storage_path.join("logs");
    let process_manager = ProcessManager::new(log_dir)?;
    let running_processes = process_manager.get_running_processes();

    if !running_processes.is_empty() {
        anyhow::bail!(
            "Cannot unregister while processes are running. Please stop all processes first using 'deploy stop'.\n\
            Running processes:\n{}",
            running_processes
                .iter()
                .map(|p| format!("  - {}::{} (PID: {})", p.package_type, p.std_name, p.pid))
                .collect::<Vec<_>>()
                .join("\n")
        );
    }

    let unregistrar = PackageUnregistrar::new(config)?;

    // Parse target format
    if target.ends_with(".yaml") || target.ends_with(".yml") {
        // Recipe file
        let recipe_path = PathBuf::from(&target);
        unregistrar.unregister_from_recipe(&recipe_path).await?;
    } else if target.contains('.') {
        // package.primitive, package.service, or package.skill format
        let parts: Vec<&str> = target.splitn(2, '.').collect();
        if parts.len() == 2 {
            let package_name = parts[0];
            let item_name = parts[1];

            if item_name.starts_with("prm::") {
                unregistrar
                    .unregister_primitive(package_name, item_name)
                    .await?;
            } else if item_name.starts_with("srv::") {
                unregistrar
                    .unregister_service(package_name, item_name)
                    .await?;
            } else if item_name.starts_with("skl::") {
                unregistrar
                    .unregister_skill(package_name, item_name)
                    .await?;
            } else {
                anyhow::bail!(
                    "Invalid format. Expected 'package.prm::name', 'package.srv::name', or 'package.skl::name', got: {}",
                    target
                );
            }
        } else {
            anyhow::bail!(
                "Invalid format. Expected 'package.prm::name', 'package.srv::name', or 'package.skl::name', got: {}",
                target
            );
        }
    } else {
        // Package name
        unregistrar.unregister_package(&target).await?;
    }

    Ok(())
}
