use anyhow::Result;
use crate::config::Config;
use crate::database::PackageDatabase;
use crate::recipe::Recipe;
use serde_yaml::Value;
use std::path::PathBuf;
use robonix_core::messages::{RegisterRequest, RegisterResponse};
use ros2_client::{
    service::AService,
    Context, Name, Node, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
};
use rustdds::{policy, QosPolicyBuilder};
use std::sync::Arc;
use tokio::sync::Mutex;

pub struct PackageRegistrar {
    config: Config,
    node: Arc<Mutex<Option<Node>>>,
    register_client: Arc<Mutex<Option<ros2_client::service::Client<AService<RegisterRequest, RegisterResponse>>>>>,
}

impl PackageRegistrar {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            node: Arc::new(Mutex::new(None)),
            register_client: Arc::new(Mutex::new(None)),
        }
    }

    async fn ensure_client(&self) -> Result<()> {
        let mut node_guard = self.node.lock().await;
        let mut client_guard = self.register_client.lock().await;

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
            let spinner = node.spinner()
                .map_err(|e| anyhow::anyhow!("Failed to get node spinner: {:?}", e))?;
            tokio::spawn(async move {
                let _ = spinner.spin().await;
            });

            // Create service client
            let service_qos = QosPolicyBuilder::new()
                .reliability(policy::Reliability::Reliable {
                    max_blocking_time: rustdds::Duration::from_millis(100),
                })
                .history(policy::History::KeepLast { depth: 1 })
                .build();

            let client = node
                .create_client::<AService<RegisterRequest, RegisterResponse>>(
                    ServiceMapping::Enhanced,
                    &Name::new("/rbnx/srv", "register").unwrap(),
                    &ServiceTypeName::new("robonix_core", "Register"),
                    service_qos.clone(),
                    service_qos,
                )
                .map_err(|e| anyhow::anyhow!("Failed to create register service client: {:?}", e))?;

            // Wait for service to be available (with timeout)
            tracing::debug!("Waiting for register service to be available...");
            let wait_future = client.wait_for_service(&node);
            let timeout_future = tokio::time::sleep(tokio::time::Duration::from_secs(5));
            tokio::select! {
                _ = wait_future => {
                    tracing::debug!("Register service is available");
                }
                _ = timeout_future => {
                    tracing::warn!("Register service not available after 5 seconds, continuing anyway...");
                    // Continue anyway, the service call will fail if service is not available
                }
            }

            *node_guard = Some(node);
            *client_guard = Some(client);
        }

        Ok(())
    }

    pub async fn register_from_recipe(&self, recipe_path: &PathBuf) -> Result<()> {
        let recipe = Recipe::load(recipe_path)?;
        let db = PackageDatabase::load(&self.config.package_storage_path)?;

        println!("Registering recipe: {}", recipe.name);
        if let Some(desc) = &recipe.description {
            println!("Description: {}", desc);
        }

        // Register each package in recipe
        for recipe_pkg in &recipe.packages {
            let pkg_info = db.find_by_name(&recipe_pkg.name)
                .ok_or_else(|| anyhow::anyhow!("Package not found: {}", recipe_pkg.name))?;

            // Load manifest
            let manifest_content = std::fs::read_to_string(&pkg_info.manifest_path)?;
            let manifest: Value = serde_yaml::from_str(&manifest_content)?;

            // Register capabilities
            let caps_to_register = if let Some(caps) = &recipe_pkg.capabilities {
                caps.clone()
            } else {
                pkg_info.capabilities.clone()
            };

            for cap_name in caps_to_register {
                if let Some(cap) = find_capability_in_manifest(&manifest, &cap_name)? {
                    self.register_capability(&pkg_info.name, &pkg_info.path, cap).await?;
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
                    self.register_skill(&pkg_info.name, &pkg_info.path, skill).await?;
                }
            }
        }

        println!("Recipe registration completed successfully");
        Ok(())
    }

    async fn register_capability(
        &self,
        package_name: &str,
        package_path: &PathBuf,
        cap: &Value,
    ) -> Result<()> {
        let std_name = cap["name"].as_str()
            .ok_or_else(|| anyhow::anyhow!("Capability name not found"))?;
        
        // Description from spec, not needed in manifest
        let description = String::new();
        
        // Code path is the package path
        let code_path = package_path.to_str()
            .ok_or_else(|| anyhow::anyhow!("Invalid package path"))?
            .to_string();

        let (mut input_names, mut input_types, input_channels) = parse_io_params(cap, "inputs")?;
        let (mut output_names, mut output_types, output_channels) = parse_io_params(cap, "outputs")?;
        let (config_services, config_names) = parse_configs(cap)?;
        
        // Fill types from spec if empty (new simplified manifest format)
        fill_types_from_spec(std_name, "cap", &mut input_names, &mut input_types, &mut output_names, &mut output_types)?;

        let request = RegisterRequest {
            package_name: package_name.to_string(),
            package_type: "cap".to_string(),
            std_name: std_name.to_string(),
            description,
            code_path,
            input_names,
            input_ros_types: input_types,
            input_channels,
            output_names,
            output_ros_types: output_types,
            output_channels,
            config_services,
            config_names,
        };

        self.call_register_service(request).await?;
        println!("  Registered capability: {}", std_name);
        Ok(())
    }

    async fn register_skill(
        &self,
        package_name: &str,
        package_path: &PathBuf,
        skill: &Value,
    ) -> Result<()> {
        let std_name = skill["name"].as_str()
            .ok_or_else(|| anyhow::anyhow!("Skill name not found"))?;
        
        // Description from spec, not needed in manifest
        let description = String::new();
        
        // Code path is the package path
        let code_path = package_path.to_str()
            .ok_or_else(|| anyhow::anyhow!("Invalid package path"))?
            .to_string();

        let (mut input_names, mut input_types, input_channels) = parse_io_params(skill, "inputs")?;
        let (mut output_names, mut output_types, output_channels) = parse_io_params(skill, "outputs")?;
        let (config_services, config_names) = parse_configs(skill)?;
        
        // Fill types from spec if empty (new simplified manifest format)
        fill_types_from_spec(std_name, "skl", &mut input_names, &mut input_types, &mut output_names, &mut output_types)?;

        let request = RegisterRequest {
            package_name: package_name.to_string(),
            package_type: "skl".to_string(),
            std_name: std_name.to_string(),
            description,
            code_path,
            input_names,
            input_ros_types: input_types,
            input_channels,
            output_names,
            output_ros_types: output_types,
            output_channels,
            config_services,
            config_names,
        };

        self.call_register_service(request).await?;
        println!("  Registered skill: {}", std_name);
        Ok(())
    }

    async fn call_register_service(&self, request: RegisterRequest) -> Result<()> {
        // Ensure client is initialized
        self.ensure_client().await?;

        let client_guard = self.register_client.lock().await;
        let client = client_guard.as_ref()
            .ok_or_else(|| anyhow::anyhow!("Register client not initialized"))?;
        
        let node_guard = self.node.lock().await;
        let _node = node_guard.as_ref()
            .ok_or_else(|| anyhow::anyhow!("ROS2 node not initialized"))?;

        // Call service directly using ROS2 client
        tracing::info!("Calling register service for {}: {}", request.package_type, request.std_name);
        let call_result = tokio::time::timeout(
            tokio::time::Duration::from_secs(10),
            client.async_call_service(request)
        ).await;
        
        match call_result {
            Ok(Ok(response)) => {
                tracing::info!("Received response: success={}, error={}", response.success, response.error_message);
                if !response.success {
                    anyhow::bail!("Registration failed: {}", response.error_message);
                }
                Ok(())
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

fn parse_io_params(cap_or_skill: &Value, field: &str) -> Result<(Vec<String>, Vec<String>, Vec<String>)> {
    let mut names = Vec::new();
    let mut types = Vec::new();
    let mut channels = Vec::new();

    // Support both old format (list of objects) and new format (dict: name => channel)
    if let Some(params) = cap_or_skill[field].as_sequence() {
        // Old format: list of {name, type, channel}
        for param in params {
            if let Some(name) = param["name"].as_str() {
                names.push(name.to_string());
            } else {
                names.push(String::new());
            }

            if let Some(typ) = param["type"].as_str() {
                types.push(typ.to_string());
            } else {
                types.push(String::new());
            }

            if let Some(channel) = param["channel"].as_str() {
                channels.push(channel.to_string());
            } else {
                channels.push(String::new());
            }
        }
    } else if let Some(channel_map) = cap_or_skill[field].as_mapping() {
        // New format: dict {name: channel}
        // Note: names and types will be filled from spec during registration
        // For now, we just extract the channel mappings
        for (name_key, channel_value) in channel_map {
            if let (Some(name), Some(channel)) = (name_key.as_str(), channel_value.as_str()) {
                names.push(name.to_string());
                channels.push(channel.to_string());
                // Type will be filled from spec
                types.push(String::new());
            }
        }
    }

    Ok((names, types, channels))
}

fn parse_configs(cap_or_skill: &Value) -> Result<(Vec<String>, Vec<String>)> {
    let mut services = Vec::new();
    let mut names = Vec::new();

    // Support both old format (list) and new format (dict, though configs are usually empty)
    if let Some(configs) = cap_or_skill["configs"].as_sequence() {
        // Old format: list of {service, name}
        for config in configs {
            if let Some(service) = config["service"].as_str() {
                services.push(service.to_string());
            } else {
                services.push(String::new());
            }

            if let Some(name) = config["name"].as_str() {
                names.push(name.to_string());
            } else {
                names.push(String::new());
            }
        }
    } else if let Some(config_map) = cap_or_skill["configs"].as_mapping() {
        // New format: dict {name: service} (if needed in future)
        for (name_key, service_value) in config_map {
            if let (Some(name), Some(service)) = (name_key.as_str(), service_value.as_str()) {
                names.push(name.to_string());
                services.push(service.to_string());
            }
        }
    }

    Ok((services, names))
}

/// Fill types from spec for capabilities/skills using simplified manifest format
/// This function matches channel mappings from manifest with spec definitions
fn fill_types_from_spec(
    std_name: &str,
    package_type: &str,
    input_names: &mut Vec<String>,
    input_types: &mut Vec<String>,
    output_names: &mut Vec<String>,
    output_types: &mut Vec<String>,
) -> Result<()> {
    // Check if we need to fill types (new format where types are empty)
    let needs_filling = input_types.iter().any(|t| t.is_empty()) || output_types.iter().any(|t| t.is_empty());
    
    if !needs_filling {
        // Old format, types already provided
        return Ok(());
    }
    
    // Use robonix-core's spec registry to get types
    let spec_registry = robonix_core::spec::SpecRegistry::new();
    
    match package_type {
        "cap" => {
            let spec = spec_registry
                .capabilities
                .get(std_name)
                .ok_or_else(|| anyhow::anyhow!("Unknown capability spec: {}", std_name))?;
            
            // Match input names from manifest with spec and fill types
            for (i, input_name) in input_names.iter().enumerate() {
                if let Some(spec_input) = spec.inputs.iter().find(|s| s.name == *input_name) {
                    if i < input_types.len() && input_types[i].is_empty() {
                        input_types[i] = spec_input.ros_type.clone();
                    }
                }
            }
            
            // Match output names from manifest with spec and fill types
            for (i, output_name) in output_names.iter().enumerate() {
                if let Some(spec_output) = spec.outputs.iter().find(|s| s.name == *output_name) {
                    if i < output_types.len() && output_types[i].is_empty() {
                        output_types[i] = spec_output.ros_type.clone();
                    }
                }
            }
        }
        "skl" => {
            let spec = spec_registry
                .skills
                .get(std_name)
                .ok_or_else(|| anyhow::anyhow!("Unknown skill spec: {}", std_name))?;
            
            // Match input names from manifest with spec and fill types
            for (i, input_name) in input_names.iter().enumerate() {
                if let Some(spec_input) = spec.inputs.iter().find(|s| s.name == *input_name) {
                    if i < input_types.len() && input_types[i].is_empty() {
                        input_types[i] = spec_input.ros_type.clone();
                    }
                }
            }
            
            // Match output names from manifest with spec and fill types
            for (i, output_name) in output_names.iter().enumerate() {
                if let Some(spec_output) = spec.outputs.iter().find(|s| s.name == *output_name) {
                    if i < output_types.len() && output_types[i].is_empty() {
                        output_types[i] = spec_output.ros_type.clone();
                    }
                }
            }
        }
        _ => {
            anyhow::bail!("Invalid package type: {}", package_type);
        }
    }
    
    Ok(())
}
