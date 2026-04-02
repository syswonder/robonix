// SPDX-License-Identifier: MulanPSL-2.0
// pilot_service.rs — gRPC PilotService implementation

use crate::executor::executor_service_client::ExecutorServiceClient;
use crate::pilot::{
    pilot_service_server::PilotService, AbortSessionRequest, AbortSessionResponse,
    HandleIntentRequest, ListSessionsRequest, ListSessionsResponse, PilotEvent, SessionInfo,
    SessionStatusEvent,
};
use crate::pilot_wire::{self, PilotStreamBody};
use crate::planner;
use crate::session::SessionManager;
use crate::session_state::SessionState;
use crate::vlm::VlmClient;
use anyhow::Result;
use robonix_sdk::RobonixClient;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{watch, Mutex};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};
use uuid::Uuid;

pub struct PilotServiceImpl {
    sdk: Arc<Mutex<RobonixClient>>,
    vlm: Arc<Mutex<VlmClient>>,
    sessions: SessionManager,
    executor_endpoint: String,
    /// Per-session cancellation senders.  Stored outside the Session lock so
    /// `abort_session` can cancel without waiting for the turn to release it.
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

#[tonic::async_trait]
impl PilotService for PilotServiceImpl {
    type HandleIntentStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn handle_intent(
        &self,
        request: Request<HandleIntentRequest>,
    ) -> Result<Response<Self::HandleIntentStream>, Status> {
        let mut intent = request
            .into_inner()
            .intent
            .ok_or_else(|| Status::invalid_argument("missing intent"))?;

        // Assign a session_id if Liaison didn't provide one.
        if intent.session_id.is_empty() {
            intent.session_id = Uuid::new_v4().to_string();
        }

        let session_arc = self
            .sessions
            .get_or_create(&intent.session_id)
            .await;

        let (tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
        let sdk = Arc::clone(&self.sdk);
        let vlm = Arc::clone(&self.vlm);
        let executor_endpoint = self.executor_endpoint.clone();
        let session_id = intent.session_id.clone();
        let cancels = Arc::clone(&self.cancels);

        // Fresh cancel channel for this turn.
        let (cancel_tx, cancel_rx) = watch::channel(false);
        cancels.lock().await.insert(session_id.clone(), cancel_tx);

        tokio::spawn(async move {
            // Emit session-created / active status.
            let _ = tx.send(Ok(pilot_wire::pack(
                &session_id,
                PilotStreamBody::Status(SessionStatusEvent {
                    session_id: session_id.clone(),
                    state: SessionState::Active as u32,
                    message: String::new(),
                }),
            ))).await;

            // Connect to Executor.
            let mut executor = match ExecutorServiceClient::connect(executor_endpoint.clone()).await {
                Ok(c) => c,
                Err(e) => {
                    let _ = tx.send(Err(Status::unavailable(format!(
                        "cannot connect to Executor at '{}': {e}",
                        executor_endpoint
                    )))).await;
                    return;
                }
            };

            // Lock session and run one turn.
            let mut session = session_arc.lock().await;
            if let Err(e) = planner::run_turn(
                &intent,
                &mut session,
                &sdk,
                &vlm,
                &mut executor,
                &tx,
                cancel_rx,
            ).await {
                log::error!("[pilot] turn error for session '{}': {e:#}", session_id);
                let _ = tx.send(Err(Status::internal(e.to_string()))).await;
            }

            // Clean up cancel entry.
            cancels.lock().await.remove(&session_id);
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }

    async fn abort_session(
        &self,
        request: Request<AbortSessionRequest>,
    ) -> Result<Response<AbortSessionResponse>, Status> {
        let id = request.into_inner().session_id;
        // Signal the running turn via the cancel channel (no session lock needed).
        let ok = if let Some(tx) = self.cancels.lock().await.get(&id) {
            let _ = tx.send(true);
            true
        } else {
            false
        };
        Ok(Response::new(AbortSessionResponse { ok }))
    }

    async fn list_sessions(
        &self,
        _request: Request<ListSessionsRequest>,
    ) -> Result<Response<ListSessionsResponse>, Status> {
        let list = self.sessions.list().await;
        let sessions = list
            .into_iter()
            .map(|(id, state, created_at_ms, turn_count)| SessionInfo {
                session_id: id,
                state: state as u32,
                created_at_ms,
                turn_count,
            })
            .collect();
        Ok(Response::new(ListSessionsResponse { sessions }))
    }
}
