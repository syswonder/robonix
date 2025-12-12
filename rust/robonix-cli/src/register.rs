use crate::config::Config;
use crate::database::PackageDatabase;
use crate::output;
use crate::process::ProcessManager;
use crate::recipe::Recipe;
use crate::recipe_state::RecipeState;
use anyhow::Result;
use robonix_core::primitive::primitive::{RegisterPrimitiveRequest, RegisterPrimitiveResponse};
use robonix_core::service::service::{RegisterServiceRequest, RegisterServiceResponse};
use robonix_core::skill_library::skill::{RegisterSkillRequest, RegisterSkillResponse};
use ros2_client::{
    service::AService, Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
};
use rustdds::{policy, QosPolicyBuilder};
use serde_json;
use serde_yaml::Value;
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::Mutex;

pub struct PackageRegistrar {
    config: Config,
    node: Arc<Mutex<Option<Node>>>,
    primitive_client: Arc<
        Mutex<Option<ros2_client::service::Client<AService<RegisterPrimitiveRequest, RegisterPrimitiveResponse>>>>,
    >,
    service_client: Arc<
        Mutex<Option<ros2_client::service::Client<AService<RegisterServiceRequest, RegisterServiceResponse>>>>,
    >,
    skill_client: Arc<
        Mutex<Option<ros2_client::service::Client<AService<RegisterSkillRequest, RegisterSkillResponse>>>>,
    >,
    process_manager: Arc<ProcessManager>,
}

impl PackageRegistrar {
    pub fn new(config: Config) -> Result<Self> {
        // Create log directory for process logs
        let log_dir = config.package_storage_path.join("logs");
        let process_manager = Arc::new(ProcessManager::new(log_dir)?);

        Ok(Self {
            config,
            node: Arc::new(Mutex::new(None)),
            primitive_client: Arc::new(Mutex::new(None)),
            service_client: Arc::new(Mutex::new(None)),
            skill_client: Arc::new(Mutex::new(None)),
            process_manager,
        })
    }

    async fn ensure_clients(&self) -> Result<()> {
        let mut node_guard = self.node.lock().await;
        let mut primitive_client_guard = self.primitive_client.lock().await;
        let mut service_client_guard = self.service_client.lock().await;
        let mut skill_client_guard = self.skill_client.lock().await;

        if node_guard.is_none() {
            let context = Context::new()
                .map_err(|e| anyhow::anyhow!("Failed to create ROS2 context: {:?}", e))?;

            let mut node = context
                .new_node(
                    NodeName::new("/rbnx", "rbnx_cli").unwrap(),
                    NodeOptions::new().enable_rosout(false),
                )
                .map_err(|e| anyhow::anyhow!("Failed to create ROS2 node: {:?}", e))?;

            // Start spinner in background
            let spinner = node
                .spinner()
                .map_err(|e| anyhow::anyhow!("Failed to get node spinner: {:?}", e))?;
            tokio::spawn(async move {
                let _ = spinner.spin().await;
            });

            // Create service clients
            let service_qos = QosPolicyBuilder::new()
                .reliability(policy::Reliability::Reliable {
                    max_blocking_time: rustdds::Duration::from_millis(100),
                })
                .history(policy::History::KeepLast { depth: 1 })
                .build();

            // Primitive register client
            let primitive_client = node
                .create_client::<AService<RegisterPrimitiveRequest, RegisterPrimitiveResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/prm", "register").unwrap(),
                    &ServiceTypeName::new("robonix_core", "RegisterPrimitive"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| {
                    anyhow::anyhow!("Failed to create primitive register service client: {:?}", e)
                })?;

            // Service register client
            let service_client = node
                .create_client::<AService<RegisterServiceRequest, RegisterServiceResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv", "register").unwrap(),
                    &ServiceTypeName::new("robonix_core", "RegisterService"),
                    service_qos.clone(),
                    service_qos.clone(),
                )
                .map_err(|e| {
                    anyhow::anyhow!("Failed to create service register service client: {:?}", e)
                })?;

            // Skill register client
            let skill_client = node
                .create_client::<AService<RegisterSkillRequest, RegisterSkillResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/skl", "register").unwrap(),
                    &ServiceTypeName::new("robonix_core", "RegisterSkill"),
                    service_qos.clone(),
                    service_qos,
                )
                .map_err(|e| {
                    anyhow::anyhow!("Failed to create skill register service client: {:?}", e)
                })?;

            // Wait for services to be available (with timeout)
            tracing::debug!("Waiting for register services to be available...");
            let wait_future = primitive_client.wait_for_service(&node);
            let timeout_future = tokio::time::sleep(tokio::time::Duration::from_secs(5));
            tokio::select! {
                _ = wait_future => {
                    tracing::debug!("Register services are available");
                }
                _ = timeout_future => {
                    tracing::warn!("Register services not available after 5 seconds, continuing anyway...");
                }
            }

            *node_guard = Some(node);
            *primitive_client_guard = Some(primitive_client);
            *service_client_guard = Some(service_client);
            *skill_client_guard = Some(skill_client);
        }

        Ok(())
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

        // Save recipe state
        let recipe_state = RecipeState {
            recipe_path: recipe_path.clone(),
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
        package_path: &PathBuf,
        primitive: &Value,
    ) -> Result<()> {
        let name = primitive["name"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Primitive name not found"))?
            .to_string();

        let input_schema_str = primitive["input_schema"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Primitive input_schema not found"))?;
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

        let metadata_str = primitive["metadata"]
            .as_str()
            .unwrap_or("{}");
        serde_json::from_str::<serde_json::Value>(metadata_str)
            .map_err(|e| anyhow::anyhow!("Invalid metadata JSON: {}", e))?;

        // Use package name as provider
        let provider = package_name.to_string();

        let request = RegisterPrimitiveRequest {
            name: name.clone(),
            input_schema: input_schema_str.to_string(),
            output_schema: output_schema_str.to_string(),
            metadata: metadata_str.to_string(),
            provider: provider.clone(),
        };

        self.call_primitive_register_service(request).await?;
        output::check(&format!(
            "Registered primitive: {} (provider: {})",
            name,
            provider
        ));
        Ok(())
    }

    async fn register_service(
        &self,
        package_name: &str,
        package_path: &PathBuf,
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

        let metadata_str = service["metadata"]
            .as_str()
            .unwrap_or("{}");
        // Validate JSON but keep as string for ROS2 service
        serde_json::from_str::<serde_json::Value>(metadata_str)
            .map_err(|e| anyhow::anyhow!("Invalid metadata JSON: {}", e))?;

        // Use package name as provider
        let provider = package_name.to_string();

        let request = RegisterServiceRequest {
            name: name.clone(),
            srv_type,
            entry,
            metadata: metadata_str.to_string(),
            provider: provider.clone(),
        };

        self.call_service_register_service(request).await?;
        output::check(&format!(
            "Registered service: {} (provider: {})",
            name,
            provider
        ));
        Ok(())
    }

    async fn register_skill(
        &self,
        package_name: &str,
        package_path: &PathBuf,
        skill: &Value,
    ) -> Result<()> {
        let name_raw = skill["name"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill name not found"))?
            .to_string();
        // Automatically add 'skl::' prefix if not present
        let name = if name_raw.starts_with("skl::") {
            name_raw
        } else {
            format!("skl::{}", name_raw)
        };

        let start_topic = skill["start_topic"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill start_topic not found"))?
            .to_string();

        let status_topic = skill["status_topic"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill status_topic not found"))?
            .to_string();

        let skill_dir = skill["skill_dir"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill skill_dir not found"))?
            .to_string();
        // Make skill_dir absolute path
        let skill_dir = if skill_dir.starts_with('/') {
            skill_dir
        } else {
            package_path.join(&skill_dir)
                .to_str()
                .ok_or_else(|| anyhow::anyhow!("Invalid skill_dir path"))?
                .to_string()
        };

        let main_rtdl = skill["main_rtdl"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill main_rtdl not found"))?
            .to_string();

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

        let metadata_str = skill["metadata"]
            .as_str()
            .unwrap_or("{}");
        serde_json::from_str::<serde_json::Value>(metadata_str)
            .map_err(|e| anyhow::anyhow!("Invalid metadata JSON: {}", e))?;

        // Use package name as provider
        let provider = package_name.to_string();

        let version = skill["version"]
            .as_str()
            .unwrap_or("1.0.0")
            .to_string();

        let request = RegisterSkillRequest {
            name: name.clone(),
            start_topic,
            status_topic,
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
            name,
            response.skill_id,
            provider
        ));
        Ok(())
    }

    async fn call_primitive_register_service(
        &self,
        request: RegisterPrimitiveRequest,
    ) -> Result<RegisterPrimitiveResponse> {
        self.ensure_clients().await?;

        let client_guard = self.primitive_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Primitive register client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling primitive register service for: {}", request.name);
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!("Received response: ok={}", response.ok);
                if !response.ok {
                    anyhow::bail!("Primitive registration failed");
                }
                Ok(response)
            }
            Ok(Err(e)) => {
                tracing::error!("Service call error: {:?}", e);
                anyhow::bail!("Service call error: {:?}", e);
            }
            Err(_) => {
                tracing::error!("Service call timeout after 10 seconds");
                anyhow::bail!("Service call timeout after 10 seconds");
            }
        }
    }

    async fn call_service_register_service(
        &self,
        request: RegisterServiceRequest,
    ) -> Result<RegisterServiceResponse> {
        self.ensure_clients().await?;

        let client_guard = self.service_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Service register client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling service register service for: {}", request.name);
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!("Received response: ok={}", response.ok);
                if !response.ok {
                    anyhow::bail!("Service registration failed");
                }
                Ok(response)
            }
            Ok(Err(e)) => {
                tracing::error!("Service call error: {:?}", e);
                anyhow::bail!("Service call error: {:?}", e);
            }
            Err(_) => {
                tracing::error!("Service call timeout after 10 seconds");
                anyhow::bail!("Service call timeout after 10 seconds");
            }
        }
    }

    async fn call_skill_register_service(
        &self,
        request: RegisterSkillRequest,
    ) -> Result<RegisterSkillResponse> {
        self.ensure_clients().await?;

        let client_guard = self.skill_client.lock().await;
        let client = client_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("Skill register client not initialized"))?;

        let node_guard = self.node.lock().await;
        let _node = node_guard
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        tracing::info!("Calling skill register service for: {}", request.name);
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request),
        )
        .await;

        match call_result {
            Ok(Ok(response)) => {
                tracing::info!("Received response: ok={}, skill_id={}", response.ok, response.skill_id);
                if !response.ok {
                    anyhow::bail!("Skill registration failed");
                }
                Ok(response)
            }
            Ok(Err(e)) => {
                tracing::error!("Service call error: {:?}", e);
                anyhow::bail!("Service call error: {:?}", e);
            }
            Err(_) => {
                tracing::error!("Service call timeout after 10 seconds");
                anyhow::bail!("Service call timeout after 10 seconds");
            }
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
