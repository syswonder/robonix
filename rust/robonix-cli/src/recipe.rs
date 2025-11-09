use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Recipe {
    pub name: String,
    pub description: Option<String>,
    pub packages: Vec<RecipePackage>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecipePackage {
    pub name: String,
    pub capabilities: Option<Vec<String>>,  // If None, register all capabilities
    pub skills: Option<Vec<String>>,        // If None, register all skills
    pub entity_name: Option<String>,        // Entity name to mount (e.g., "agilex_robot")
}

impl Recipe {
    pub fn load(path: &PathBuf) -> Result<Self> {
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read recipe: {}", path.display()))?;
        
        let recipe: Recipe = serde_yaml::from_str(&content)
            .with_context(|| format!("Failed to parse recipe: {}", path.display()))?;
        
        Ok(recipe)
    }

    pub fn save(&self, path: &PathBuf) -> Result<()> {
        let content = serde_yaml::to_string(self)
            .context("Failed to serialize recipe")?;
        
        std::fs::write(path, content)
            .with_context(|| format!("Failed to write recipe: {}", path.display()))?;
        
        Ok(())
    }
}

