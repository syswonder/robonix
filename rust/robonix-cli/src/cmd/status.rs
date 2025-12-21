use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};
use crate::{Config, PackageDatabase, RecipeState};
use anyhow::Result;
use serde_yaml::Value;
use std::path::PathBuf;

pub async fn execute(config: Config) -> Result<()> {
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

    // Create a map for quick lookup
    let running_map: std::collections::HashMap<(String, String), (u32, PathBuf)> =
        running_processes
            .into_iter()
            .map(|proc| {
                let key = (proc.std_name.clone(), proc.package_type.clone());
                (key, (proc.pid, proc.log_file))
            })
            .collect();
    let db = PackageDatabase::load(&config.package_storage_path)?;

    // Show active recipe
    if let Ok(Some(recipe_state)) = RecipeState::load(&config.package_storage_path) {
        println!("\x1b[1mActive Recipe:\x1b[0m");
        println!("  Name: {}", recipe_state.recipe.name);
        if let Some(desc) = &recipe_state.recipe.description {
            println!("  Description: {}", desc);
        }
        println!("  Path: {}", recipe_state.recipe_path.display());
        println!("  Registered at: {}", recipe_state.registered_at);
        println!();
    } else {
        println!("\x1b[1mActive Recipe:\x1b[0m None");
        println!();
    }

    // Get all packages from active recipe only
    let recipe_state = RecipeState::load(&config.package_storage_path)?;
    let packages_to_show = if let Some(recipe_state) = &recipe_state {
        // Show packages from recipe
        recipe_state
            .recipe
            .packages
            .iter()
            .filter_map(|rp| db.find_by_name(&rp.name))
            .collect::<Vec<_>>()
    } else {
        // No active recipe, nothing to show
        println!("No active recipe. Register a recipe to see status.");
        return Ok(());
    };

    if packages_to_show.is_empty() {
        println!("No packages to display.");
        return Ok(());
    }

    // Collect all capabilities and skills with their status
    let mut all_items: Vec<(String, String, String, bool, Option<u32>, Option<String>)> =
        Vec::new();
    // Format: (package_name, std_name, type, is_running, pid, log_file)

    for pkg_info in &packages_to_show {
        // Load manifest
        let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
        let _manifest: Value = serde_yaml::from_str(&manifest_content)?;

        // Determine which primitives, services, and skills to show based on recipe
        // recipe_state is guaranteed to be Some at this point
        let (prims_to_show, srvs_to_show, skills_to_show) =
            if let Some(recipe_state) = &recipe_state {
                if let Some(recipe_pkg) = recipe_state
                    .recipe
                    .packages
                    .iter()
                    .find(|rp| rp.name == pkg_info.name)
                {
                    let prims = if let Some(prims) = &recipe_pkg.primitives {
                        prims.clone()
                    } else {
                        pkg_info.primitives.clone()
                    };
                    let srvs = if let Some(srvs) = &recipe_pkg.services {
                        srvs.clone()
                    } else {
                        pkg_info.services.clone()
                    };
                    let skills = if let Some(skills) = &recipe_pkg.skills {
                        skills.clone()
                    } else {
                        pkg_info.skills.clone()
                    };
                    (prims, srvs, skills)
                } else {
                    (
                        pkg_info.primitives.clone(),
                        pkg_info.services.clone(),
                        pkg_info.skills.clone(),
                    )
                }
            } else {
                // This should never happen due to early return above
                (
                    pkg_info.primitives.clone(),
                    pkg_info.services.clone(),
                    pkg_info.skills.clone(),
                )
            };

        // Add primitives
        for prim_name in &prims_to_show {
            let key = (prim_name.clone(), "prm".to_string());
            let (is_running, pid, log_file) =
                if let Some((proc_pid, proc_log_file)) = running_map.get(&key) {
                    (
                        true,
                        Some(*proc_pid),
                        Some(
                            proc_log_file
                                .file_name()
                                .and_then(|n| n.to_str())
                                .map(|s| s.to_string())
                                .unwrap_or_default(),
                        ),
                    )
                } else {
                    (false, None, None)
                };
            all_items.push((
                pkg_info.name.clone(),
                prim_name.clone(),
                "prm".to_string(),
                is_running,
                pid,
                log_file,
            ));
        }

        // Add services
        for srv_name in &srvs_to_show {
            let key = (srv_name.clone(), "srv".to_string());
            let (is_running, pid, log_file) =
                if let Some((proc_pid, proc_log_file)) = running_map.get(&key) {
                    (
                        true,
                        Some(*proc_pid),
                        Some(
                            proc_log_file
                                .file_name()
                                .and_then(|n| n.to_str())
                                .map(|s| s.to_string())
                                .unwrap_or_default(),
                        ),
                    )
                } else {
                    (false, None, None)
                };
            all_items.push((
                pkg_info.name.clone(),
                srv_name.clone(),
                "srv".to_string(),
                is_running,
                pid,
                log_file,
            ));
        }

        // Add skills
        for skill_name in &skills_to_show {
            let key = (skill_name.clone(), "skl".to_string());
            let (is_running, pid, log_file) =
                if let Some((proc_pid, proc_log_file)) = running_map.get(&key) {
                    (
                        true,
                        Some(*proc_pid),
                        Some(
                            proc_log_file
                                .file_name()
                                .and_then(|n| n.to_str())
                                .map(|s| s.to_string())
                                .unwrap_or_default(),
                        ),
                    )
                } else {
                    (false, None, None)
                };
            all_items.push((
                pkg_info.name.clone(),
                skill_name.clone(),
                "skl".to_string(),
                is_running,
                pid,
                log_file,
            ));
        }
    }

    if all_items.is_empty() {
        println!("No primitives, services, or skills to display.");
        return Ok(());
    }

    // Calculate column widths
    let max_package_len = all_items
        .iter()
        .map(|(pkg, _, _, _, _, _)| pkg.len())
        .max()
        .unwrap_or(0)
        .max(7);
    let max_std_name_len = all_items
        .iter()
        .map(|(_, name, _, _, _, _)| name.len())
        .max()
        .unwrap_or(0)
        .max(10);
    let max_log_file_len = all_items
        .iter()
        .filter_map(|(_, _, _, _, _, log)| log.as_ref())
        .map(|s| s.len())
        .max()
        .unwrap_or(0)
        .max(8);

    // Print header
    println!(
        "\x1b[1m{:<pkg_width$}  {:<name_width$}  Type  Status    PID     {:<log_width$}\x1b[0m",
        "Package",
        "Cap/Skill",
        "Log File",
        pkg_width = max_package_len,
        name_width = max_std_name_len,
        log_width = max_log_file_len
    );
    println!(
        "{}  {}  {}  {}  {}  {}",
        "─".repeat(max_package_len),
        "─".repeat(max_std_name_len),
        "────",
        "──────",
        "───────",
        "─".repeat(max_log_file_len)
    );

    // Print all items
    for (package_name, std_name, item_type, is_running, pid, log_file) in &all_items {
        let status = if *is_running {
            "\x1b[32mRunning\x1b[0m"
        } else {
            "\x1b[33mStopped\x1b[0m"
        };
        let pid_str = if let Some(p) = pid {
            p.to_string()
        } else {
            "-".to_string()
        };
        let log_file_str = log_file.as_deref().unwrap_or("-");
        println!(
            "{:<pkg_width$}  {:<name_width$}  {:<4}  {:<6}  {:<7}  {:<log_width$}",
            package_name,
            std_name,
            item_type,
            status,
            pid_str,
            log_file_str,
            pkg_width = max_package_len,
            name_width = max_std_name_len,
            log_width = max_log_file_len
        );
    }

    Ok(())
}
