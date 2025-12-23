use crate::{Config, PackageDatabase, RecipeState};
use anyhow::Result;
use serde_yaml::Value;

/// Item in recipe that can be started/stopped
#[derive(Debug, Clone)]
pub struct RecipeItem {
    pub package_name: String,
    pub std_name: String,
    pub package_type: String,         // "prm", "srv", or "skl"
    pub start_script: Option<String>, // Start script path (for primitives, services, and skills)
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

        // Add primitives
        let primitives_to_include = if let Some(primitives) = &recipe_pkg.primitives {
            primitives.clone()
        } else {
            pkg_info.primitives.clone()
        };

        for primitive_name in &primitives_to_include {
            if let Some(primitive) = manifest["primitives"].as_sequence().and_then(|primitives| {
                primitives
                    .iter()
                    .find(|p| p["name"].as_str() == Some(primitive_name))
            }) {
                let start_script = primitive["start_script"].as_str().map(|s| s.to_string());
                items.push(RecipeItem {
                    package_name: pkg_info.name.clone(),
                    std_name: primitive_name.clone(),
                    package_type: "prm".to_string(),
                    start_script,
                    package_path: pkg_info.path.clone(),
                });
            }
        }

        // Add services
        let services_to_include = if let Some(services) = &recipe_pkg.services {
            services.clone()
        } else {
            pkg_info.services.clone()
        };

        for service_name in &services_to_include {
            if let Some(service) = manifest["services"].as_sequence().and_then(|services| {
                services
                    .iter()
                    .find(|s| s["name"].as_str() == Some(service_name))
            }) {
                let start_script = service["start_script"].as_str().map(|s| s.to_string());
                items.push(RecipeItem {
                    package_name: pkg_info.name.clone(),
                    std_name: service_name.clone(),
                    package_type: "srv".to_string(),
                    start_script,
                    package_path: pkg_info.path.clone(),
                });
            }
        }

        // Add skills
        let skills_to_include = if let Some(skills) = &recipe_pkg.skills {
            skills.clone()
        } else {
            pkg_info.skills.clone()
        };

        for skill_name in &skills_to_include {
            if let Some(skill) = manifest["skills"].as_sequence().and_then(|skills| {
                skills
                    .iter()
                    .find(|s| s["name"].as_str() == Some(skill_name))
            }) {
                let start_script = skill["start_script"].as_str().map(|s| s.to_string());
                items.push(RecipeItem {
                    package_name: pkg_info.name.clone(),
                    std_name: skill_name.clone(),
                    package_type: "skl".to_string(),
                    start_script,
                    package_path: pkg_info.path.clone(),
                });
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

    Ok(recipe_state
        .recipe
        .packages
        .iter()
        .map(|p| p.name.clone())
        .collect())
}
