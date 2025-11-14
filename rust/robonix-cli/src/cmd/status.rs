use crate::{Config, PackageDatabase, ProcessManager, RecipeState};
use anyhow::Result;
use serde_yaml::Value;

pub async fn execute(config: Config) -> Result<()> {
    let log_dir = config.package_storage_path.join("logs");
    let process_manager = ProcessManager::new(log_dir)?;
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

    // Get all packages from recipe or all installed packages
    let packages_to_show =
        if let Ok(Some(recipe_state)) = RecipeState::load(&config.package_storage_path) {
            // Show packages from recipe
            recipe_state
                .recipe
                .packages
                .iter()
                .filter_map(|rp| db.find_by_name(&rp.name))
                .collect::<Vec<_>>()
        } else {
            // Show all installed packages
            db.list_packages()
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

        // Determine which capabilities/skills to show based on recipe
        let (caps_to_show, skills_to_show) =
            if let Ok(Some(recipe_state)) = RecipeState::load(&config.package_storage_path) {
                if let Some(recipe_pkg) = recipe_state
                    .recipe
                    .packages
                    .iter()
                    .find(|rp| rp.name == pkg_info.name)
                {
                    let caps = if let Some(caps) = &recipe_pkg.capabilities {
                        caps.clone()
                    } else {
                        pkg_info.capabilities.clone()
                    };
                    let skills = if let Some(skills) = &recipe_pkg.skills {
                        skills.clone()
                    } else {
                        pkg_info.skills.clone()
                    };
                    (caps, skills)
                } else {
                    (pkg_info.capabilities.clone(), pkg_info.skills.clone())
                }
            } else {
                (pkg_info.capabilities.clone(), pkg_info.skills.clone())
            };

        // Add capabilities
        for cap_name in &caps_to_show {
            let is_running = process_manager.is_running(cap_name, "cap");
            let (pid, log_file) = if is_running {
                if let Some(proc) = process_manager
                    .get_running_processes()
                    .iter()
                    .find(|p| p.std_name == *cap_name && p.package_type == "cap")
                {
                    (
                        Some(proc.pid),
                        Some(
                            proc.log_file
                                .file_name()
                                .and_then(|n| n.to_str())
                                .map(|s| s.to_string())
                                .unwrap_or_default(),
                        ),
                    )
                } else {
                    (None, None)
                }
            } else {
                (None, None)
            };
            all_items.push((
                pkg_info.name.clone(),
                cap_name.clone(),
                "cap".to_string(),
                is_running,
                pid,
                log_file,
            ));
        }

        // Add skills
        for skill_name in &skills_to_show {
            let is_running = process_manager.is_running(skill_name, "skl");
            let (pid, log_file) = if is_running {
                if let Some(proc) = process_manager
                    .get_running_processes()
                    .iter()
                    .find(|p| p.std_name == *skill_name && p.package_type == "skl")
                {
                    (
                        Some(proc.pid),
                        Some(
                            proc.log_file
                                .file_name()
                                .and_then(|n| n.to_str())
                                .map(|s| s.to_string())
                                .unwrap_or_default(),
                        ),
                    )
                } else {
                    (None, None)
                }
            } else {
                (None, None)
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
        println!("No capabilities or skills to display.");
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
