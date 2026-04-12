// SPDX-License-Identifier: MulanPSL-2.0
// pilot_service.rs — gRPC `SrvRuntimePilot` (contract facade)

use crate::contracts::{
    srv_runtime_executor_client::SrvRuntimeExecutorClient,
    srv_runtime_executor_list_tools_client::SrvRuntimeExecutorListToolsClient,
    srv_runtime_pilot_server::SrvRuntimePilot,
};
use crate::pilot::{Intent, PilotEvent, SessionStatusEvent};
use crate::pilot_wire::{self, PilotStreamBody};
use crate::planner::{self, ExecutorConn};
use crate::session::SessionManager;
use crate::session_state::SessionState;
use crate::vlm::VlmClient;
use robonix_sdk::RobonixClient;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{Mutex, watch};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};
use uuid::Uuid;

pub struct PilotServiceImpl {
    sdk: Arc<Mutex<RobonixClient>>,
    vlm: Arc<Mutex<VlmClient>>,
    sessions: SessionManager,
    executor_endpoint: String,
    /// Per-session cancellation senders. `abort_turn` Intent signals this without holding the session lock.
    cancels: Arc<Mutex<HashMap<String, watch::Sender<bool>>>>,
}

impl PilotServiceImpl {
    pub fn new(
        sdk: Arc<Mutex<RobonixClient>>,
        vlm: Arc<Mutex<VlmClient>>,
        executor_endpoint: String,
    ) -> Self {
        Self {
            sdk,
            vlm,
            sessions: SessionManager::default(),
            executor_endpoint,
            cancels: Arc::new(Mutex::new(HashMap::new())),
        }
    }
}

fn intent_is_abort_turn(intent: &Intent) -> bool {
    let j = intent.context_json.trim();
    if j.is_empty() {
        return false;
    }
    serde_json::from_str::<serde_json::Value>(j)
        .ok()
        .and_then(|v| v.get("abort_turn").and_then(|x| x.as_bool()))
        .unwrap_or(false)
}

#[tonic::async_trait]
impl SrvRuntimePilot for PilotServiceImpl {
    type StreamStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn stream(
        &self,
        request: Request<Intent>,
    ) -> Result<Response<Self::StreamStream>, Status> {
        let mut intent = request.into_inner();

        if intent_is_abort_turn(&intent) {
            let id = intent.session_id.clone();
            let ok = if let Some(tx) = self.cancels.lock().await.get(&id) {
                let _ = tx.send(true);
                true
            } else {
                false
            };
            log::debug!("[pilot] abort_turn intent for session {id} (signaled={ok})");
            let (_tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(1);
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        if intent.session_id.is_empty() {
            intent.session_id = Uuid::new_v4().to_string();
        }

        let session_arc = self.sessions.get_or_create(&intent.session_id).await;

        let (tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
        let sdk = Arc::clone(&self.sdk);
        let vlm = Arc::clone(&self.vlm);
        let executor_endpoint = self.executor_endpoint.clone();
        let session_id = intent.session_id.clone();
        let cancels = Arc::clone(&self.cancels);

        let (cancel_tx, cancel_rx) = watch::channel(false);
        cancels.lock().await.insert(session_id.clone(), cancel_tx);

        tokio::spawn(async move {
            let _ = tx
                .send(Ok(pilot_wire::pack(
                    &session_id,
                    PilotStreamBody::Status(SessionStatusEvent {
                        session_id: session_id.clone(),
                        state: SessionState::Active as u32,
                        message: String::new(),
                    }),
                )))
                .await;

            let channel = match tonic::transport::Endpoint::new(executor_endpoint.clone()) {
                Ok(ep) => match ep.connect().await {
                    Ok(ch) => ch,
                    Err(e) => {
                        let _ = tx
                            .send(Err(Status::unavailable(format!(
                                "cannot connect to Executor at '{executor_endpoint}': {e}"
                            ))))
                            .await;
                        cancels.lock().await.remove(&session_id);
                        return;
                    }
                },
                Err(e) => {
                    let _ = tx
                        .send(Err(Status::internal(format!(
                            "invalid Executor endpoint '{}': {e}",
                            executor_endpoint
                        ))))
                        .await;
                    cancels.lock().await.remove(&session_id);
                    return;
                }
            };

            let mut executor = ExecutorConn {
                graph: SrvRuntimeExecutorClient::new(channel.clone()),
                list_tools: SrvRuntimeExecutorListToolsClient::new(channel),
            };

            let mut session = session_arc.lock().await;
            if let Err(e) = planner::run_turn(
                &intent,
                &mut session,
                &sdk,
                &vlm,
                &mut executor,
                &tx,
                cancel_rx,
            )
            .await
            {
                log::error!("[pilot] turn error for session '{}': {e:#}", session_id);
                let _ = tx.send(Err(Status::internal(e.to_string()))).await;
            }

            cancels.lock().await.remove(&session_id);
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}
