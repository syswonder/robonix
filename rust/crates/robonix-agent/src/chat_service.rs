use std::sync::Arc;
use tokio::sync::Mutex;
use tonic::{Request, Response, Status};

use crate::react;
use crate::vlm::VlmClient;
use robonix_sdk::RobonixClient;

pub mod pb {
    tonic::include_proto!("robonix.agent_chat");
}

fn wrap(e: pb::agent_chat_event::Event) -> pb::AgentChatEvent {
    pb::AgentChatEvent { event: Some(e) }
}

pub fn agent_chat_server(
    svc: AgentChatService,
) -> pb::agent_chat_server::AgentChatServer<AgentChatService> {
    pb::agent_chat_server::AgentChatServer::new(svc)
}

pub struct AgentChatService {
    sdk: Arc<Mutex<RobonixClient>>,
    vlm: Arc<Mutex<VlmClient>>,
    history: Arc<Mutex<Vec<crate::vlm::Message>>>,
}

impl AgentChatService {
    pub fn new(sdk: Arc<Mutex<RobonixClient>>, vlm: Arc<Mutex<VlmClient>>) -> Self {
        Self {
            sdk,
            vlm,
            history: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

#[tonic::async_trait]
impl pb::agent_chat_server::AgentChat for AgentChatService {
    type ChatStream =
        tokio_stream::wrappers::ReceiverStream<Result<pb::AgentChatEvent, Status>>;

    async fn chat(
        &self,
        request: Request<pb::AgentChatRequest>,
    ) -> Result<Response<Self::ChatStream>, Status> {
        let user_msg = request.into_inner().user_message;
        let (tx, rx) = tokio::sync::mpsc::channel(64);

        let sdk = self.sdk.clone();
        let vlm = self.vlm.clone();
        let history = self.history.clone();

        tokio::spawn(async move {
            let _ = tx
                .send(Ok(wrap(pb::agent_chat_event::Event::Status(
                    "thinking".into(),
                ))))
                .await;

            let event_tx = tx.clone();
            let result = react::run_single_turn(
                &sdk,
                &vlm,
                &history,
                &user_msg,
                move |ev| {
                    let tx = event_tx.clone();
                    let grpc_event = match ev {
                        react::ChatEvent::TextChunk(text) => {
                            wrap(pb::agent_chat_event::Event::TextChunk(text))
                        }
                        react::ChatEvent::ToolCall {
                            round,
                            tool_name,
                            arguments,
                        } => wrap(pb::agent_chat_event::Event::ToolCall(pb::ToolCallInfo {
                            round,
                            tool_name,
                            arguments,
                            result: String::new(),
                            completed: false,
                        })),
                        react::ChatEvent::ToolResult {
                            round,
                            tool_name,
                            result,
                        } => wrap(pb::agent_chat_event::Event::ToolCall(pb::ToolCallInfo {
                            round,
                            tool_name,
                            arguments: String::new(),
                            result,
                            completed: true,
                        })),
                    };
                    Box::pin(async move {
                        let _ = tx.send(Ok(grpc_event)).await;
                    })
                },
            )
            .await;

            match result {
                Ok(final_text) => {
                    // Signal stream end; the text was already streamed via TextChunk events
                    // but send FinalText as a termination marker with the full text
                    // so non-streaming clients still work.
                    let _ = tx
                        .send(Ok(wrap(pb::agent_chat_event::Event::FinalText(final_text))))
                        .await;
                }
                Err(e) => {
                    let _ = tx
                        .send(Ok(wrap(pb::agent_chat_event::Event::FinalText(format!(
                            "Error: {e:#}"
                        )))))
                        .await;
                }
            }
        });

        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
    }
}
