// SPDX-License-Identifier: MulanPSL-2.0
// Unregister Module
//
// Package unregistration functionality for robonix-cli

use crate::config::Config;
use crate::database::PackageDatabase;
use crate::output;
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

    /// Unregister a package (all primitives, services, and skills)
    pub async fn unregister_package(&self, package_name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let pkg_info = db
            .find_by_name(package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        // Load manifest
        let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
        let manifest: Value = serde_yaml::from_str(&manifest_content)?;

        output::action("Unregistering", &format!("package '{}'", package_name));

        // TODO: Call core unregister service for all primitives, services, and skills
        // For now, we'll just stop the processes

        // Unregister all primitives
        if let Some(primitives) = manifest["primitives"].as_sequence() {
            for primitive in primitives {
                if let Some(std_name) = primitive["name"].as_str() {
                    output::sub_step(&format!("Unregistering primitive: {}", std_name));
                    // TODO: Call core unregister service
                    // self.call_unregister_primitive(package_name, std_name).await?;
                }
            }
        }

        // Unregister all services
        if let Some(services) = manifest["services"].as_sequence() {
            for service in services {
                if let Some(std_name) = service["name"].as_str() {
                    output::sub_step(&format!("Unregistering service: {}", std_name));
                    // TODO: Call core unregister service
                    // self.call_unregister_service(package_name, std_name).await?;
                }
            }
        }

        // Unregister all skills
        if let Some(skills) = manifest["skills"].as_sequence() {
            for skill in skills {
                if let Some(std_name) = skill["name"].as_str() {
                    output::sub_step(&format!("Unregistering skill: {}", std_name));
                    // TODO: Call core unregister service
                    // self.call_unregister_skill(package_name, std_name).await?;
                }
            }
        }

        output::success("Package unregistration completed");
        Ok(())
    }

    /// Unregister a specific primitive
    pub async fn unregister_primitive(&self, package_name: &str, std_name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let _pkg_info = db
            .find_by_name(package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        output::action(
            "Unregistering",
            &format!("primitive '{}' from package '{}'", std_name, package_name),
        );

        // TODO: Call core unregister service
        // self.call_unregister_primitive(package_name, std_name).await?;

        output::success("Primitive unregistration completed");
        Ok(())
    }

    /// Unregister a specific service
    pub async fn unregister_service(&self, package_name: &str, std_name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let _pkg_info = db
            .find_by_name(package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        output::action(
            "Unregistering",
            &format!("service '{}' from package '{}'", std_name, package_name),
        );

        // TODO: Call core unregister service
        // self.call_unregister_service(package_name, std_name).await?;

        output::success("Service unregistration completed");
        Ok(())
    }

    /// Unregister a specific skill
    pub async fn unregister_skill(&self, package_name: &str, std_name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let _pkg_info = db
            .find_by_name(package_name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", package_name))?;

        output::action(
            "Unregistering",
            &format!("skill '{}' from package '{}'", std_name, package_name),
        );

        // TODO: Call core unregister service
        // self.call_unregister_service(package_name, "skl", std_name).await?;

        output::success("Skill unregistration completed");
        Ok(())
    }

    /// Unregister from a recipe
    pub async fn unregister_from_recipe(&self, recipe_path: &PathBuf) -> Result<()> {
        let recipe = Recipe::load(recipe_path)?;
        let db = PackageDatabase::load(&self.config.package_storage_path)?;

        output::action("Unregistering", &format!("recipe '{}'", recipe.name));
        if let Some(desc) = &recipe.description {
            output::sub_step(&format!("Description: {}", desc));
        }

        // Unregister each package in recipe
        for recipe_pkg in &recipe.packages {
            let pkg_info = db
                .find_by_name(&recipe_pkg.name)
                .ok_or_else(|| anyhow::anyhow!("Package not found: {}", recipe_pkg.name))?;

            // Load manifest
            let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
            let manifest: Value = serde_yaml::from_str(&manifest_content)?;

            // Unregister primitives
            let primitives_to_unregister = if let Some(primitives) = &recipe_pkg.primitives {
                primitives.clone()
            } else {
                pkg_info.primitives.clone()
            };

            for primitive_name in primitives_to_unregister {
                if let Some(primitive) = find_primitive_in_manifest(&manifest, &primitive_name)? {
                    let std_name = primitive["name"]
                        .as_str()
                        .ok_or_else(|| anyhow::anyhow!("Primitive name not found"))?;

                    output::sub_step(&format!("Unregistering primitive: {}", std_name));
                    // TODO: Call core unregister service
                    // self.call_unregister_primitive(&pkg_info.name, std_name).await?;
                }
            }

            // Unregister services
            let services_to_unregister = if let Some(services) = &recipe_pkg.services {
                services.clone()
            } else {
                pkg_info.services.clone()
            };

            for service_name in services_to_unregister {
                if let Some(service) = find_service_in_manifest(&manifest, &service_name)? {
                    let std_name = service["name"]
                        .as_str()
                        .ok_or_else(|| anyhow::anyhow!("Service name not found"))?;

                    output::sub_step(&format!("Unregistering service: {}", std_name));
                    // TODO: Call core unregister service
                    // self.call_unregister_service(&pkg_info.name, std_name).await?;
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

                    output::sub_step(&format!("Unregistering skill: {}", std_name));
                    // TODO: Call core unregister service
                    // self.call_unregister_skill(&pkg_info.name, std_name).await?;
                }
            }
        }

        // Clear recipe state if this recipe was active
        if let Ok(Some(state)) = RecipeState::load(&self.config.package_storage_path) {
            if state.recipe_path == *recipe_path {
                RecipeState::clear(&self.config.package_storage_path)?;
            }
        }

        output::success("Recipe unregistration completed");
        Ok(())
    }

    // TODO: Implement when core unregister service is available
    // async fn call_unregister_service(&self, package_name: &str, package_type: &str, std_name: &str) -> Result<()> {
    //     // Call robonix-server unregister service
    //     // This will be implemented when the service is available
    //     Ok(())
    // }
}

fn find_primitive_in_manifest<'a>(manifest: &'a Value, name: &str) -> Result<Option<&'a Value>> {
    if let Some(primitives) = manifest["primitives"].as_sequence() {
        for primitive in primitives {
            if primitive["name"].as_str() == Some(name) {
                return Ok(Some(primitive));
            }
        }
    }
    Ok(None)
}

fn find_service_in_manifest<'a>(manifest: &'a Value, name: &str) -> Result<Option<&'a Value>> {
    if let Some(services) = manifest["services"].as_sequence() {
        for service in services {
            if service["name"].as_str() == Some(name) {
                return Ok(Some(service));
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
