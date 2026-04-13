// SPDX-License-Identifier: MulanPSL-2.0
// pilot_service.rs — gRPC `SrvPilot` (contract facade)

use crate::contracts::{
    srv_executor_client::SrvExecutorClient,
    srv_executor_list_tools_client::SrvExecutorListToolsClient, srv_pilot_server::SrvPilot,
};
use crate::pilot::{PilotEvent, SessionStatusEvent, Task};
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
    /// Per-session cancellation senders. `abort_turn` Task signals this without holding the session lock.
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

fn task_is_abort_turn(task: &Task) -> bool {
    let j = task.context_json.trim();
    if j.is_empty() {
        return false;
    }
    serde_json::from_str::<serde_json::Value>(j)
        .ok()
        .and_then(|v| v.get("abort_turn").and_then(|x| x.as_bool()))
        .unwrap_or(false)
}

#[tonic::async_trait]
impl SrvPilot for PilotServiceImpl {
    type StreamStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn stream(&self, request: Request<Task>) -> Result<Response<Self::StreamStream>, Status> {
        let mut task = request.into_inner();

        if task_is_abort_turn(&task) {
            let id = task.session_id.clone();
            let ok = if let Some(tx) = self.cancels.lock().await.get(&id) {
                let _ = tx.send(true);
                true
            } else {
                false
            };
            log::debug!("[pilot] abort_turn task for session {id} (signaled={ok})");
            let (_tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(1);
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        if task.session_id.is_empty() {
            task.session_id = Uuid::new_v4().to_string();
        }

        let session_arc = self.sessions.get_or_create(&task.session_id).await;

        let (tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
        let sdk = Arc::clone(&self.sdk);
        let vlm = Arc::clone(&self.vlm);
        let executor_endpoint = self.executor_endpoint.clone();
        let session_id = task.session_id.clone();
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
                graph: SrvExecutorClient::new(channel.clone()),
                list_tools: SrvExecutorListToolsClient::new(channel),
            };

            let mut session = session_arc.lock().await;
            if let Err(e) = planner::run_turn(
                &task,
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

#[cfg(test)]
mod tests {
    use super::task_is_abort_turn;
    use crate::pilot::Task;

    fn task(ctx: &str) -> Task {
        Task {
            task_id: "t".into(),
            session_id: "s".into(),
            source: 0,
            text: String::new(),
            audio_data: Vec::new(),
            context_json: ctx.into(),
            timestamp_ms: 0,
        }
    }

    #[test]
    fn abort_turn_true() {
        assert!(task_is_abort_turn(&task(r#"{"abort_turn":true}"#)));
    }

    #[test]
    fn abort_turn_false_missing_field() {
        assert!(!task_is_abort_turn(&task(r#"{"foo":1}"#)));
    }

    #[test]
    fn abort_turn_false_explicit() {
        assert!(!task_is_abort_turn(&task(r#"{"abort_turn":false}"#)));
    }

    #[test]
    fn abort_turn_false_empty_or_invalid() {
        assert!(!task_is_abort_turn(&task("")));
        assert!(!task_is_abort_turn(&task("not json")));
    }
}
