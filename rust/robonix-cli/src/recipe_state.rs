use crate::recipe::Recipe;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecipeState {
    pub recipe_path: PathBuf,
    pub recipe: Recipe,
    pub registered_at: String, // ISO 8601 timestamp
}

impl RecipeState {
    pub fn state_file_path(storage_path: &Path) -> PathBuf {
        storage_path.join("active_recipe.json")
    }

    pub fn load(storage_path: &Path) -> Result<Option<Self>> {
        let state_path = Self::state_file_path(storage_path);

        if !state_path.exists() {
            return Ok(None);
        }

        let content = std::fs::read_to_string(&state_path)
            .with_context(|| format!("Failed to read recipe state: {}", state_path.display()))?;

        let state: RecipeState = serde_json::from_str(&content)
            .with_context(|| format!("Failed to parse recipe state: {}", state_path.display()))?;

        Ok(Some(state))
    }

    pub fn save(&self, storage_path: &Path) -> Result<()> {
        let state_path = Self::state_file_path(storage_path);

        let content =
            serde_json::to_string_pretty(self).context("Failed to serialize recipe state")?;

        std::fs::write(&state_path, content)
            .with_context(|| format!("Failed to write recipe state: {}", state_path.display()))?;

        Ok(())
    }

    pub fn clear(storage_path: &Path) -> Result<()> {
        let state_path = Self::state_file_path(storage_path);
        if state_path.exists() {
            std::fs::remove_file(&state_path).with_context(|| {
                format!("Failed to remove recipe state: {}", state_path.display())
            })?;
        }
        Ok(())
    }
}
