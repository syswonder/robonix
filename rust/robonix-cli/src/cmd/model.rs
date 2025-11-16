use crate::config::Config;
use crate::model::ModelClient;
use crate::output;
use anyhow::Result;
use robonix_core::messages::ModelType;

pub async fn execute_register(
    config: Config,
    model_id: String,
    model_name: String,
    model_type: String,
    provider: String,
    api_endpoint: String,
    api_key: Option<String>,
    description: String,
    capabilities: Option<String>,
) -> Result<()> {
    output::action("Registering", &format!("AI model '{}'", model_id));

    // Parse model type
    let model_type_enum = match model_type.to_lowercase().as_str() {
        "llm" => ModelType::LLM,
        "vlm" => ModelType::VLM,
        _ => anyhow::bail!("Invalid model type: {}. Must be 'llm' or 'vlm'", model_type),
    };

    // Parse capabilities
    let capabilities_vec = if let Some(caps) = capabilities {
        caps.split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect()
    } else {
        Vec::new()
    };

    // Get API key from argument or environment
    let api_key_final = api_key.or_else(|| std::env::var("ROBONIX_MODEL_API_KEY").ok());

    if api_key_final.is_none() {
        output::error("API key is required. Please set the ROBONIX_MODEL_API_KEY environment variable or use the --api-key option");
        anyhow::bail!("API key is empty");
    }

    let client = ModelClient::new(config)?;
    client
        .register(
            model_id.clone(),
            model_name,
            model_type_enum,
            provider,
            api_endpoint,
            api_key_final,
            description,
            capabilities_vec,
        )
        .await?;

    output::success(&format!("AI model '{}' registered successfully", model_id));
    Ok(())
}

pub async fn execute_query(
    config: Config,
    model_id: Option<String>,
    model_type: Option<String>,
    capability: Option<String>,
) -> Result<()> {
    output::action("Querying", "AI models");

    // Parse model type if provided
    let model_type_enum = if let Some(mt) = model_type {
        Some(match mt.to_lowercase().as_str() {
            "llm" => ModelType::LLM,
            "vlm" => ModelType::VLM,
            _ => anyhow::bail!("Invalid model type: {}. Must be 'llm' or 'vlm'", mt),
        })
    } else {
        None
    };

    let client = ModelClient::new(config)?;
    let response = client.query(model_id, model_type_enum, capability).await?;

    if !response.success {
        anyhow::bail!("Query failed: {}", response.error_message);
    }

    if response.models.is_empty() {
        output::info("No AI models found matching the criteria");
    } else {
        output::info(&format!("Found {} AI model(s):", response.models.len()));
        for model in &response.models {
            output::sub_step(&format!("  Model ID: {}", model.model_id));
            output::sub_step(&format!("  Name: {}", model.model_name));
            output::sub_step(&format!("  Type: {:?}", model.model_type));
            output::sub_step(&format!("  Provider: {}", model.provider));
            output::sub_step(&format!("  Endpoint: {}", model.api_endpoint));
            output::sub_step(&format!("  Description: {}", model.description));
            if !model.capabilities.is_empty() {
                output::sub_step(&format!(
                    "  Capabilities: {}",
                    model.capabilities.join(", ")
                ));
            }
            output::info("");
        }
    }

    Ok(())
}
