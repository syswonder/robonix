use crate::config::Config;
use crate::database::PackageDatabase;
use crate::recipe::Recipe;
use crate::recipe_state::RecipeState;
use anyhow::Result;
use serde_yaml::Value;
use std::path::PathBuf;

pub struct PackageUnregistrar {
    config: Config,
}

impl PackageUnregistrar {
    pub fn new(config: Config) -> Result<Self> {
        Ok(Self { config })
    }

    /// Unregister a package (all capabilities and skills)
    pub async fn unregister_package(&self, package_name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let pkg_info = db
            .find_by_name(package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        // Load manifest
        let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
        let manifest: Value = serde_yaml::from_str(&manifest_content)?;

        println!("Unregistering package: {}", package_name);

        // TODO: Call core unregister service for all capabilities
        // For now, we'll just stop the processes

        // Unregister all capabilities
        if let Some(caps) = manifest["capabilities"].as_sequence() {
            for cap in caps {
                if let Some(std_name) = cap["name"].as_str() {
                    println!("  Unregistering capability: {}", std_name);
                    // TODO: Call core unregister service
                    // self.call_unregister_service(package_name, "cap", std_name).await?;
                }
            }
        }

        // Unregister all skills
        if let Some(skills) = manifest["skills"].as_sequence() {
            for skill in skills {
                if let Some(std_name) = skill["name"].as_str() {
                    println!("  Unregistering skill: {}", std_name);
                    // TODO: Call core unregister service
                    // self.call_unregister_service(package_name, "skl", std_name).await?;
                }
            }
        }

        println!("Package unregistration completed");
        Ok(())
    }

    /// Unregister a specific capability
    pub async fn unregister_capability(&self, package_name: &str, std_name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let _pkg_info = db
            .find_by_name(package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        println!(
            "Unregistering capability: {} from package: {}",
            std_name, package_name
        );

        // TODO: Call core unregister service
        // self.call_unregister_service(package_name, "cap", std_name).await?;

        println!("Capability unregistration completed");
        Ok(())
    }

    /// Unregister a specific skill
    pub async fn unregister_skill(&self, package_name: &str, std_name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let _pkg_info = db
            .find_by_name(package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        println!(
            "Unregistering skill: {} from package: {}",
            std_name, package_name
        );

        // TODO: Call core unregister service
        // self.call_unregister_service(package_name, "skl", std_name).await?;

        println!("Skill unregistration completed");
        Ok(())
    }

    /// Unregister from a recipe
    pub async fn unregister_from_recipe(&self, recipe_path: &PathBuf) -> Result<()> {
        let recipe = Recipe::load(recipe_path)?;
        let db = PackageDatabase::load(&self.config.package_storage_path)?;

        println!("Unregistering recipe: {}", recipe.name);
        if let Some(desc) = &recipe.description {
            println!("Description: {}", desc);
        }

        // Unregister each package in recipe
        for recipe_pkg in &recipe.packages {
            let pkg_info = db
                .find_by_name(&recipe_pkg.name)
                .ok_or_else(|| anyhow::anyhow!("Package not found: {}", recipe_pkg.name))?;

            // Load manifest
            let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
            let manifest: Value = serde_yaml::from_str(&manifest_content)?;

            // Unregister capabilities
            let caps_to_unregister = if let Some(caps) = &recipe_pkg.capabilities {
                caps.clone()
            } else {
                pkg_info.capabilities.clone()
            };

            for cap_name in caps_to_unregister {
                if let Some(cap) = find_capability_in_manifest(&manifest, &cap_name)? {
                    let std_name = cap["name"]
                        .as_str()
                        .ok_or_else(|| anyhow::anyhow!("Capability name not found"))?;

                    println!("  Unregistering capability: {}", std_name);
                    // TODO: Call core unregister service
                    // self.call_unregister_service(&pkg_info.name, "cap", std_name).await?;
                }
            }

            // Unregister skills
            let skills_to_unregister = if let Some(skills) = &recipe_pkg.skills {
                skills.clone()
            } else {
                pkg_info.skills.clone()
            };

            for skill_name in skills_to_unregister {
                if let Some(skill) = find_skill_in_manifest(&manifest, &skill_name)? {
                    let std_name = skill["name"]
                        .as_str()
                        .ok_or_else(|| anyhow::anyhow!("Skill name not found"))?;

                    println!("  Unregistering skill: {}", std_name);
                    // TODO: Call core unregister service
                    // self.call_unregister_service(&pkg_info.name, "skl", std_name).await?;
                }
            }
        }

        // Clear recipe state if this recipe was active
        if let Ok(Some(state)) = RecipeState::load(&self.config.package_storage_path) {
            if state.recipe_path == *recipe_path {
                RecipeState::clear(&self.config.package_storage_path)?;
            }
        }

        println!("Recipe unregistration completed");
        Ok(())
    }

    // TODO: Implement when core unregister service is available
    // async fn call_unregister_service(&self, package_name: &str, package_type: &str, std_name: &str) -> Result<()> {
    //     // Call robonix-core unregister service
    //     // This will be implemented when the service is available
    //     Ok(())
    // }
}

fn find_capability_in_manifest<'a>(manifest: &'a Value, name: &str) -> Result<Option<&'a Value>> {
    if let Some(caps) = manifest["capabilities"].as_sequence() {
        for cap in caps {
            if cap["name"].as_str() == Some(name) {
                return Ok(Some(cap));
            }
        }
    }
    Ok(None)
}

fn find_skill_in_manifest<'a>(manifest: &'a Value, name: &str) -> Result<Option<&'a Value>> {
    if let Some(skills) = manifest["skills"].as_sequence() {
        for skill in skills {
            if skill["name"].as_str() == Some(name) {
                return Ok(Some(skill));
            }
        }
    }
    Ok(None)
}
