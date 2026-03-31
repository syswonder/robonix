mod chat_service;
mod react;
mod skills;
mod tools;
mod vlm;

use anyhow::Result;
use std::sync::Arc;
use tokio::sync::Mutex;

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

    let listen_port: u16 = std::env::var("AGENT_CHAT_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{listen_port}")).await?;
    let bound_port = listener.local_addr()?.port();

    sdk.declare_interface_full(
        AGENT_NODE_ID,
        "agent_chat",
        vec!["grpc".to_string()],
        serde_json::json!({"endpoint": format!("localhost:{bound_port}")}).to_string(),
        bound_port as u32,
        "robonix/sys/runtime/agent/agent_chat",
    )
    .await?;

    log::info!("AgentChat gRPC on :{bound_port}");
    eprintln!("robonix-agent ready. Connect with: rbnx chat");

    let sdk = Arc::new(Mutex::new(sdk));
    let vlm = Arc::new(Mutex::new(vlm));

    let svc = chat_service::AgentChatService::new(sdk, vlm);
    let incoming = tokio_stream::wrappers::TcpListenerStream::new(listener);

    tonic::transport::Server::builder()
        .add_service(chat_service::agent_chat_server(svc))
        .serve_with_incoming(incoming)
        .await?;

    Ok(())
}
