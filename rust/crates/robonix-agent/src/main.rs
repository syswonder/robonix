mod react;
mod skills;
mod tools;
mod vlm;

use anyhow::Result;

const AGENT_NODE_ID: &str = "com.robonix.runtime.agent";

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("robonix_agent=info"),
    )
    .init();

    let server = std::env::var("ROBONIX_SERVER").unwrap_or_else(|_| "localhost:50051".into());
    let endpoint = if server.starts_with("http") {
        server
    } else {
        format!("http://{server}")
    };
    let mut sdk = robonix_sdk::RobonixClient::connect(&endpoint).await?;

    sdk.register_node(AGENT_NODE_ID, "robonix/sys/runtime/agent", "service", "")
        .await?;
    log::info!("registered as '{AGENT_NODE_ID}'");

    log::info!("discovering VLM service...");
    let vlm = vlm::VlmClient::discover(&mut sdk, AGENT_NODE_ID).await?;

    eprintln!("robonix-agent ready. Type 'quit' to exit.");
    react::run_react_loop(&mut sdk, vlm).await
}
