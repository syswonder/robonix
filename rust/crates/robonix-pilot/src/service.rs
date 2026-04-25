// SPDX-License-Identifier: MulanPSL-2.0
// `SystemPilot` gRPC handler (contract `robonix/system/pilot`).
//
// One Stream RPC per Task. The handler:
//   * fast-paths abort_turn requests by signaling the in-flight session
//     without holding the session lock,
//   * spawns the planner in a tokio task, streaming `PilotEvent`s back to
//     the caller (Liaison) over the gRPC server-streaming response,
//   * connects to executor on demand via atlas (no static endpoint config).

use crate::pb::contracts::{
    system_executor_client::SystemExecutorClient,
    system_executor_list_tools_client::SystemExecutorListToolsClient,
    system_pilot_server::SystemPilot,
};
use crate::pb::pilot::{BatchResult, PilotEvent, SessionStatusEvent, Task, TaskGraph};
use crate::planner::{self, ExecutorConn};
use crate::vlm::{Message, VlmClient};
use anyhow::Context;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{Mutex, watch};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};
use uuid::Uuid;

/// Conversation state values that go on the wire in
/// `pb::pilot::SessionStatusEvent.state`. Defined here (not in the proto)
/// because pilot is the only writer.
#[derive(Clone, Copy)]
#[repr(u32)]
#[allow(dead_code)]
pub enum SessionState {
    Active = 0,
    Completed = 1,
    Failed = 2,
    WaitingInput = 3,
}

// ── PilotEvent wire helpers ────────────────────────────────────────────────
// `PilotEvent` carries one of N payloads tagged by `event_kind`. proto3 lacks
// a oneof here so we keep the discriminator explicit; planner + service both
// build events through `pack`.

pub const EVT_TEXT_CHUNK: u32 = 0;
pub const EVT_TASK_GRAPH: u32 = 1;
pub const EVT_BATCH_RESULT: u32 = 2;
pub const EVT_STATUS: u32 = 3;
pub const EVT_FINAL_TEXT: u32 = 4;

pub enum PilotStreamBody {
    TextChunk(String),
    FinalText(String),
    TaskGraph(TaskGraph),
    BatchResult(BatchResult),
    Status(SessionStatusEvent),
}

pub fn pack(session_id: &str, body: PilotStreamBody) -> PilotEvent {
    let mut e = PilotEvent {
        session_id: session_id.to_string(),
        ..Default::default()
    };
    match body {
        PilotStreamBody::TextChunk(s) => {
            e.event_kind = EVT_TEXT_CHUNK;
            e.text_chunk = s;
        }
        PilotStreamBody::TaskGraph(g) => {
            e.event_kind = EVT_TASK_GRAPH;
            e.task_graph = Some(g);
        }
        PilotStreamBody::BatchResult(b) => {
            e.event_kind = EVT_BATCH_RESULT;
            e.batch_result = Some(b);
        }
        PilotStreamBody::Status(s) => {
            e.event_kind = EVT_STATUS;
            e.status = Some(s);
        }
        PilotStreamBody::FinalText(s) => {
            e.event_kind = EVT_FINAL_TEXT;
            e.final_text = s;
        }
    }
    e
}

// ── Service ────────────────────────────────────────────────────────────────

/// LLM conversation history per `session_id`. Grows across turns; never
/// expired (turns trim themselves at MAX_HISTORY in planner).
type Histories = Arc<Mutex<HashMap<String, Arc<Mutex<Vec<Message>>>>>>;

pub struct PilotServiceImpl {
    atlas: Arc<Mutex<AtlasClient>>,
    vlm: VlmClient,
    histories: Histories,
    /// Per-session cancellation senders. `abort_turn` Task signals this
    /// without holding the history lock.
    cancels: Arc<Mutex<HashMap<String, watch::Sender<bool>>>>,
}

impl PilotServiceImpl {
    pub fn new(atlas: Arc<Mutex<AtlasClient>>, vlm: VlmClient) -> Self {
        Self {
            atlas,
            vlm,
            histories: Arc::new(Mutex::new(HashMap::new())),
            cancels: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    async fn get_or_create_history(&self, session_id: &str) -> Arc<Mutex<Vec<Message>>> {
        let mut map = self.histories.lock().await;
        map.entry(session_id.to_string())
            .or_insert_with(|| Arc::new(Mutex::new(Vec::new())))
            .clone()
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
impl SystemPilot for PilotServiceImpl {
    type StreamStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn stream(
        &self,
        request: Request<Task>,
    ) -> Result<Response<Self::StreamStream>, Status> {
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

        let history_arc = self.get_or_create_history(&task.session_id).await;
        let (tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
        let atlas = Arc::clone(&self.atlas);
        let vlm = self.vlm.clone();
        let session_id = task.session_id.clone();
        let cancels = Arc::clone(&self.cancels);

        let (cancel_tx, cancel_rx) = watch::channel(false);
        cancels.lock().await.insert(session_id.clone(), cancel_tx);

        tokio::spawn(async move {
            let _ = tx
                .send(Ok(pack(
                    &session_id,
                    PilotStreamBody::Status(SessionStatusEvent {
                        session_id: session_id.clone(),
                        state: SessionState::Active as u32,
                        message: String::new(),
                    }),
                )))
                .await;

            let mut executor = match build_executor_conn(&atlas).await {
                Ok(e) => e,
                Err(e) => {
                    let _ = tx
                        .send(Err(Status::unavailable(format!(
                            "cannot reach Executor via atlas: {e:#}"
                        ))))
                        .await;
                    cancels.lock().await.remove(&session_id);
                    return;
                }
            };

            let mut history = history_arc.lock().await;
            if let Err(e) = planner::run_turn(
                &task,
                &mut history,
                &vlm,
                &mut executor,
                &tx,
                cancel_rx,
            )
            .await
            {
                log::error!("[pilot] turn error for session '{session_id}': {e:#}");
                let _ = tx.send(Err(Status::internal(e.to_string()))).await;
            }

            cancels.lock().await.remove(&session_id);
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

/// Discover and connect to executor's two contracts. Both lookups go
/// through atlas so executor can move/restart without reconfiguring pilot.
async fn build_executor_conn(atlas: &Arc<Mutex<AtlasClient>>) -> anyhow::Result<ExecutorConn> {
    let mut atlas = atlas.lock().await;
    let (_, exec_ch) =
        atlas_client::connect_to_capability(&mut atlas, "robonix/system/executor")
            .await
            .context("connect_to_capability robonix/system/executor")?;
    let (_, list_ch) = atlas_client::connect_to_capability(
        &mut atlas,
        "robonix/system/executor/list_tools",
    )
    .await
    .context("connect_to_capability robonix/system/executor/list_tools")?;
    Ok(ExecutorConn {
        graph: SystemExecutorClient::new(exec_ch),
        list_tools: SystemExecutorListToolsClient::new(list_ch),
    })
}

#[cfg(test)]
mod tests {
    use super::task_is_abort_turn;
    use crate::pb::pilot::Task;

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
    fn abort_turn_detected() {
        assert!(task_is_abort_turn(&task(r#"{"abort_turn":true}"#)));
        assert!(!task_is_abort_turn(&task(r#"{"abort_turn":false}"#)));
        assert!(!task_is_abort_turn(&task(r#"{"foo":1}"#)));
        assert!(!task_is_abort_turn(&task("")));
        assert!(!task_is_abort_turn(&task("not json")));
    }
}
