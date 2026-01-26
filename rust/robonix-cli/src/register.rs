// SPDX-License-Identifier: MulanPSL-2.0
// Register Module
//
// Package registration functionality for robonix-cli

use crate::config::Config;
use crate::database::PackageDatabase;
use crate::output;
use crate::process::ProcessManager;
use crate::recipe::Recipe;
use crate::recipe_state::RecipeState;
use anyhow::Result;
use robonix_core::ros_idl::primitive::{RegisterPrimitiveRequest, RegisterPrimitiveResponse};
use robonix_core::ros_idl::service_registry::{RegisterServiceRequest, RegisterServiceResponse};
use robonix_core::ros_idl::skill::{RegisterSkillRequest, RegisterSkillResponse};
use serde_json;
use serde_yaml::Value;
use std::path::PathBuf;
use std::sync::Arc;

pub struct PackageRegistrar {
    config: Config,
    _process_manager: Arc<ProcessManager>,
}

impl PackageRegistrar {
    pub fn new(config: Config) -> Result<Self> {
        // Create log directory for process logs
        let log_dir = config.package_storage_path.join("logs");
        let process_manager = Arc::new(ProcessManager::new(log_dir)?);

        Ok(Self {
            config,
            _process_manager: process_manager,
        })
    }

    pub async fn register_from_recipe(&self, recipe_path: &PathBuf) -> Result<()> {
        let recipe = Recipe::load(recipe_path)?;
        let db = PackageDatabase::load(&self.config.package_storage_path)?;

        output::action("Registering", &format!("recipe '{}'", recipe.name));
        if let Some(desc) = &recipe.description {
            output::sub_step(&format!("Description: {}", desc));
        }

        // Register all primitives, services, and skills
        output::info("");
        for recipe_pkg in &recipe.packages {
            let pkg_info = db
                .find_by_name(&recipe_pkg.name)
                .ok_or_else(|| anyhow::anyhow!("Package not found: {}", recipe_pkg.name))?;

            // Load manifest
            let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
            let manifest: Value = serde_yaml::from_str(&manifest_content)?;

            // Register primitives
            let primitives_to_register = if let Some(primitives) = &recipe_pkg.primitives {
                primitives.clone()
            } else {
                pkg_info.primitives.clone()
            };

            for primitive_name in primitives_to_register {
                if let Some(primitive) = find_primitive_in_manifest(&manifest, &primitive_name)? {
                    self.register_primitive(&pkg_info.name, &pkg_info.path, primitive)
                        .await?;
                }
            }

            // Register services
            let services_to_register = if let Some(services) = &recipe_pkg.services {
                services.clone()
            } else {
                pkg_info.services.clone()
            };

            for service_name in services_to_register {
                if let Some(service) = find_service_in_manifest(&manifest, &service_name)? {
                    self.register_service(&pkg_info.name, &pkg_info.path, service)
                        .await?;
                }
            }

            // Register skills
            let skills_to_register = if let Some(skills) = &recipe_pkg.skills {
                skills.clone()
            } else {
                pkg_info.skills.clone()
            };

            for skill_name in skills_to_register {
                if let Some(skill) = find_skill_in_manifest(&manifest, &skill_name)? {
                    self.register_skill(&pkg_info.name, &pkg_info.path, skill)
                        .await?;
                }
            }
        }

        // Save recipe state with absolute path (realpath)
        let abs_recipe_path = recipe_path.canonicalize().unwrap_or_else(|_| {
            // If canonicalize fails (e.g., file doesn't exist), try to make it absolute
            // by joining with current directory
            std::env::current_dir()
                .ok()
                .and_then(|cwd| {
                    if recipe_path.is_absolute() {
                        Some(recipe_path.clone())
                    } else {
                        Some(cwd.join(recipe_path))
                    }
                })
                .unwrap_or_else(|| recipe_path.clone())
        });
        let recipe_state = RecipeState {
            recipe_path: abs_recipe_path,
            recipe: recipe.clone(),
            registered_at: chrono::Utc::now().to_rfc3339(),
        };
        recipe_state.save(&self.config.package_storage_path)?;

        output::success("Recipe registration completed");
        Ok(())
    }

    async fn register_primitive(
        &self,
        package_name: &str,
        _package_path: &PathBuf,
        primitive: &Value,
    ) -> Result<()> {
        let name = primitive["name"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Primitive name not found"))?
            .to_string();

        // Validate JSON but keep as string for ROS2 service
        let input_schema_str = primitive["input_schema"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Primitive input_schema not found"))?;
        serde_json::from_str::<serde_json::Value>(input_schema_str)
            .map_err(|e| anyhow::anyhow!("Invalid input_schema JSON: {}", e))?;

        let output_schema_str = primitive["output_schema"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Primitive output_schema not found"))?;
        serde_json::from_str::<serde_json::Value>(output_schema_str)
            .map_err(|e| anyhow::anyhow!("Invalid output_schema JSON: {}", e))?;

        let metadata_str = primitive["metadata"].as_str().unwrap_or("{}");
        serde_json::from_str::<serde_json::Value>(metadata_str)
            .map_err(|e| anyhow::anyhow!("Invalid metadata JSON: {}", e))?;

        let version = primitive["version"].as_str().unwrap_or("1.0.0").to_string();

        // Use package name as provider
        let provider = package_name.to_string();

        let request = RegisterPrimitiveRequest {
            name: name.clone(),
            input_schema: input_schema_str.to_string(),
            output_schema: output_schema_str.to_string(),
            metadata: metadata_str.to_string(),
            provider: provider.clone(),
            version,
        };

        self.call_primitive_register_service(request).await?;
        output::check(&format!(
            "Registered primitive: {} (provider: {})",
            name, provider
        ));
        Ok(())
    }

    async fn register_service(
        &self,
        package_name: &str,
        _package_path: &PathBuf,
        service: &Value,
    ) -> Result<()> {
        let name = service["name"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Service name not found"))?
            .to_string();

        let srv_type = service["srv_type"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Service srv_type not found"))?
            .to_string();

        let entry = service["entry"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Service entry not found"))?
            .to_string();

        let metadata_str = service["metadata"].as_str().unwrap_or("{}");
        // Validate JSON but keep as string for ROS2 service
        serde_json::from_str::<serde_json::Value>(metadata_str)
            .map_err(|e| anyhow::anyhow!("Invalid metadata JSON: {}", e))?;

        let version = service["version"].as_str().unwrap_or("1.0.0").to_string();

        // Use package name as provider
        let provider = package_name.to_string();

        let request = RegisterServiceRequest {
            name: name.clone(),
            srv_type,
            entry,
            metadata: metadata_str.to_string(),
            provider: provider.clone(),
            version,
        };

        self.call_service_register_service(request).await?;
        output::check(&format!(
            "Registered service: {} (provider: {})",
            name, provider
        ));
        Ok(())
    }

    async fn register_skill(
        &self,
        package_name: &str,
        package_path: &PathBuf,
        skill: &Value,
    ) -> Result<()> {
        let name = skill["name"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill name not found"))?
            .to_string();

        let start_topic = skill["start_topic"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill start_topic not found"))?
            .to_string();

        let status_topic = skill["status_topic"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill status_topic not found"))?
            .to_string();

        let skill_type = skill["type"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill type not found (must be 'basic' or 'rtdl')"))?
            .to_string();

        if skill_type != "basic" && skill_type != "rtdl" {
            anyhow::bail!("Skill type must be 'basic' or 'rtdl', got: {}", skill_type);
        }

        // Handle entry (for basic skills) or skill_dir/main_rtdl (for RTDL skills)
        let entry = if skill_type == "basic" {
            skill["entry"]
                .as_str()
                .ok_or_else(|| anyhow::anyhow!("Basic skill entry not found"))?
                .to_string()
        } else {
            String::new()
        };

        let (skill_dir, main_rtdl) = if skill_type == "rtdl" {
            let skill_dir_str = skill["skill_dir"]
                .as_str()
                .ok_or_else(|| anyhow::anyhow!("RTDL skill skill_dir not found"))?
                .to_string();
            // Make skill_dir absolute path
            let skill_dir_abs = if skill_dir_str.starts_with('/') {
                skill_dir_str
            } else {
                package_path
                    .join(&skill_dir_str)
                    .to_str()
                    .ok_or_else(|| anyhow::anyhow!("Invalid skill_dir path"))?
                    .to_string()
            };

            let main_rtdl_str = skill["main_rtdl"]
                .as_str()
                .ok_or_else(|| anyhow::anyhow!("RTDL skill main_rtdl not found"))?
                .to_string();

            (skill_dir_abs, main_rtdl_str)
        } else {
            (String::new(), String::new())
        };

        let start_args_str = skill["start_args"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill start_args not found"))?;
        // Validate JSON but keep as string for ROS2 service
        serde_json::from_str::<serde_json::Value>(start_args_str)
            .map_err(|e| anyhow::anyhow!("Invalid start_args JSON: {}", e))?;

        let status_str = skill["status"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill status not found"))?;
        serde_json::from_str::<serde_json::Value>(status_str)
            .map_err(|e| anyhow::anyhow!("Invalid status JSON: {}", e))?;

        let metadata_str = skill["metadata"].as_str().unwrap_or("{}");
        serde_json::from_str::<serde_json::Value>(metadata_str)
            .map_err(|e| anyhow::anyhow!("Invalid metadata JSON: {}", e))?;

        // Use package name as provider
        let provider = package_name.to_string();

        let version = skill["version"].as_str().unwrap_or("1.0.0").to_string();

        let request = RegisterSkillRequest {
            name: name.clone(),
            r#type: skill_type,
            start_topic,
            status_topic,
            entry,
            skill_dir,
            main_rtdl,
            start_args: start_args_str.to_string(),
            status: status_str.to_string(),
            metadata: metadata_str.to_string(),
            provider: provider.clone(),
            version,
        };

        let response = self.call_skill_register_service(request).await?;
        output::check(&format!(
            "Registered skill: {} (skill_id: {}, provider: {})",
            name, response.skill_id, provider
        ));
        Ok(())
    }

    async fn call_primitive_register_service(
        &self,
        request: RegisterPrimitiveRequest,
    ) -> Result<RegisterPrimitiveResponse> {
        use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};

        let daemon_client = DaemonClient::new()?;
        daemon_client.ensure_daemon_running().await?;

        let request_json = serde_json::to_string(&request)?;
        let response = daemon_client
            .send_command(DaemonCommand::CallRegisterPrimitive {
                request: request_json,
            })
            .await?;

        match response {
            DaemonResponse::RegisterPrimitiveResponse { response } => {
                let resp: RegisterPrimitiveResponse = serde_json::from_str(&response)?;
                if !resp.ok {
                    anyhow::bail!("Primitive registration failed");
                }
                Ok(resp)
            }
            DaemonResponse::Error(e) => anyhow::bail!("Daemon error: {}", e),
            _ => anyhow::bail!("Unexpected response type"),
        }
    }

    async fn call_service_register_service(
        &self,
        request: RegisterServiceRequest,
    ) -> Result<RegisterServiceResponse> {
        use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};

        let daemon_client = DaemonClient::new()?;
        daemon_client.ensure_daemon_running().await?;

        let request_json = serde_json::to_string(&request)?;
        let response = daemon_client
            .send_command(DaemonCommand::CallRegisterService {
                request: request_json,
            })
            .await?;

        match response {
            DaemonResponse::RegisterServiceResponse { response } => {
                let resp: RegisterServiceResponse = serde_json::from_str(&response)?;
                if !resp.ok {
                    anyhow::bail!("Service registration failed");
                }
                Ok(resp)
            }
            DaemonResponse::Error(e) => anyhow::bail!("Daemon error: {}", e),
            _ => anyhow::bail!("Unexpected response type"),
        }
    }

    async fn call_skill_register_service(
        &self,
        request: RegisterSkillRequest,
    ) -> Result<RegisterSkillResponse> {
        use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};

        let daemon_client = DaemonClient::new()?;
        daemon_client.ensure_daemon_running().await?;

        let request_json = serde_json::to_string(&request)?;
        let response = daemon_client
            .send_command(DaemonCommand::CallRegisterSkill {
                request: request_json,
            })
            .await?;

        match response {
            DaemonResponse::RegisterSkillResponse { response } => {
                let resp: RegisterSkillResponse = serde_json::from_str(&response)?;
                if !resp.ok {
                    anyhow::bail!("Skill registration failed");
                }
                Ok(resp)
            }
            DaemonResponse::Error(e) => anyhow::bail!("Daemon error: {}", e),
            _ => anyhow::bail!("Unexpected response type"),
        }
    }
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
