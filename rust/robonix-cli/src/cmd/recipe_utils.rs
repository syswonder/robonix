use crate::{Config, PackageDatabase, RecipeState};
use anyhow::Result;
use serde_yaml::Value;

/// Item in recipe that can be started/stopped
#[derive(Debug, Clone)]
pub struct RecipeItem {
    pub package_name: String,
    pub std_name: String,
    pub package_type: String, // "cap" or "skl"
    pub start_script: String,
    pub package_path: std::path::PathBuf,
}

/// Get all items from active recipe
pub fn get_recipe_items(config: &Config) -> Result<Vec<RecipeItem>> {
    let db = PackageDatabase::load(&config.package_storage_path)?;

    let recipe_state = RecipeState::load(&config.package_storage_path)?
        .ok_or_else(|| anyhow::anyhow!("No active recipe. Please register a recipe first."))?;

    let mut items = Vec::new();

    for recipe_pkg in &recipe_state.recipe.packages {
        let pkg_info = db
            .find_by_name(&recipe_pkg.name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", recipe_pkg.name))?;

        // Load manifest
        let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
        let manifest: Value = serde_yaml::from_str(&manifest_content)?;

        // Determine which capabilities/skills to include
        let caps_to_include = if let Some(caps) = &recipe_pkg.capabilities {
            caps.clone()
        } else {
            pkg_info.capabilities.clone()
        };

        let skills_to_include = if let Some(skills) = &recipe_pkg.skills {
            skills.clone()
        } else {
            pkg_info.skills.clone()
        };

        // Add capabilities
        for cap_name in &caps_to_include {
            if let Some(cap) = manifest["capabilities"]
                .as_sequence()
                .and_then(|caps| caps.iter().find(|c| c["name"].as_str() == Some(cap_name)))
            {
                if let Some(start_script) = cap["start_script"].as_str() {
                    items.push(RecipeItem {
                        package_name: pkg_info.name.clone(),
                        std_name: cap_name.clone(),
                        package_type: "cap".to_string(),
                        start_script: start_script.to_string(),
                        package_path: pkg_info.path.clone(),
                    });
                }
            }
        }

        // Add skills
        for skill_name in &skills_to_include {
            if let Some(skill) = manifest["skills"].as_sequence().and_then(|skills| {
                skills
                    .iter()
                    .find(|s| s["name"].as_str() == Some(skill_name))
            }) {
                if let Some(start_script) = skill["start_script"].as_str() {
                    items.push(RecipeItem {
                        package_name: pkg_info.name.clone(),
                        std_name: skill_name.clone(),
                        package_type: "skl".to_string(),
                        start_script: start_script.to_string(),
                        package_path: pkg_info.path.clone(),
                    });
                }
            }
        }
    }

    Ok(items)
}

/// Match pattern against std_name
/// Supports:
/// - Exact match: "cap::vision.capture_rgb"
/// - Wildcard: "cap::vision.*", "*.capture_rgb", "*"
/// - Partial match: "vision" matches "cap::vision.capture_rgb"
pub fn matches_pattern(std_name: &str, pattern: &str) -> bool {
    if pattern == "*" || pattern == "all" {
        return true;
    }

    // Exact match
    if std_name == pattern {
        return true;
    }

    // Wildcard matching
    if pattern.contains('*') {
        let regex_pattern = pattern.replace(".", "\\.").replace("*", ".*");
        if let Ok(re) = regex::Regex::new(&format!("^{}$", regex_pattern)) {
            return re.is_match(std_name);
        }
    }

    // Partial match (contains) - only if pattern doesn't look like a full name
    if !pattern.contains("::") && !pattern.contains(".") {
        std_name.contains(pattern)
    } else {
        false
    }
}

/// Filter items by pattern
pub fn filter_items(items: &[RecipeItem], pattern: &str) -> Vec<RecipeItem> {
    items
        .iter()
        .filter(|item| matches_pattern(&item.std_name, pattern))
        .cloned()
        .collect()
}

/// Get all package names from active recipe
pub fn get_recipe_packages(config: &Config) -> Result<Vec<String>> {
    let recipe_state = RecipeState::load(&config.package_storage_path)?
        .ok_or_else(|| anyhow::anyhow!("No active recipe. Please register a recipe first."))?;
    
    Ok(recipe_state.recipe.packages.iter().map(|p| p.name.clone()).collect())
}
