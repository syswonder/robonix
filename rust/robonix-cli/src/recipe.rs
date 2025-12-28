// SPDX-License-Identifier: MulanPSL-2.0
// Recipe Module
//
// Recipe file parsing and management for robonix-cli

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
    pub primitives: Option<Vec<String>>, // If None, register all primitives
    pub services: Option<Vec<String>>,   // If None, register all services
    pub skills: Option<Vec<String>>,     // If None, register all skills
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
        let content = serde_yaml::to_string(self).context("Failed to serialize recipe")?;

        std::fs::write(path, content)
            .with_context(|| format!("Failed to write recipe: {}", path.display()))?;

        Ok(())
    }
}
