// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// `RobonixSystemPilot` gRPC handler (contract `robonix/system/pilot`).

use crate::pb::contracts::{
    robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient,
    robonix_system_pilot_server::RobonixSystemPilot,
};
use crate::pb::pilot::{
    BatchResult, PilotEvent, PilotNodeState, Plan, SessionStatusEvent, Task, TaskStateEvent,
};
use crate::planner::{self, ExecutorConn};
use crate::vlm::{Message, VlmClient};
use anyhow::Context;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use tokio::sync::{Mutex, mpsc, watch};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};
use uuid::Uuid;

#[derive(Clone, Copy)]
#[repr(u32)]
#[allow(dead_code)]
pub enum SessionState {
    Active = 0,    // when pilot recieved a task
    Completed = 1, // the task is completed
    Failed = 2,    // the task failed
}

// `PilotEvent` carries one of N payloads tagged by `event_kind`. proto3 lacks
// a oneof here so we keep the discriminator explicit; planner + service both
// build events through `pack`.
pub const EVT_TEXT_CHUNK: u32 = 0;
pub const EVT_PLAN: u32 = 1;
pub const EVT_BATCH_RESULT: u32 = 2;
pub const EVT_STATUS: u32 = 3;
pub const EVT_FINAL_TEXT: u32 = 4;
pub const EVT_NODE_STATE: u32 = 5;
pub const EVT_TASK_STATE: u32 = 6;

pub enum PilotStreamBody {
    TextChunk(String),
    FinalText(String),
    Plan(Plan),
    BatchResult(BatchResult),
    Status(SessionStatusEvent),
    NodeState(PilotNodeState),
    TaskState(TaskStateEvent),
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
        PilotStreamBody::Plan(g) => {
            e.event_kind = EVT_PLAN;
            e.plan = Some(g);
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
        PilotStreamBody::NodeState(ns) => {
            e.event_kind = EVT_NODE_STATE;
            e.node_state = Some(ns);
        }
        PilotStreamBody::TaskState(ts) => {
            e.event_kind = EVT_TASK_STATE;
            e.task_state = Some(ts);
        }
    }
    e
}

/// LLM conversation history per `session_id`. Grows across turns; never
/// expired (turns trim themselves at MAX_HISTORY in planner).
type Histories = Arc<Mutex<HashMap<String, Arc<Mutex<Vec<Message>>>>>>;

pub struct PilotServiceImpl {
    /// `AtlasClient` is cheap to clone (its inner channel is just a handle);
    /// each Stream RPC clones it to discover executor concurrently without
    /// serialising on a single mutex.
    atlas: AtlasClient,
    /// Pilot's own provider_id; passed to atlas as `consumer_id` on every
    /// `ConnectCapability` so the channel record reflects who is using
    /// the executor.
    provider_id: String,
    vlm: VlmClient,
    histories: Histories,
    /// Per-session cancellation senders. `abort_turn` Task signals this
    /// without holding the history lock.
    cancels: Arc<Mutex<HashMap<String, watch::Sender<bool>>>>,
    /// Per-session steer queues. A Task submitted while a turn is already
    /// running for that session is pushed here as a mid-task steer instead of
    /// starting a second turn; the running `run_turn` drains it.
    steers: Arc<Mutex<HashMap<String, mpsc::Sender<Task>>>>,
    /// Per-session RTDL plan-id counter. Monotonic, never reused, and shared
    /// across every turn of the session so plan ids never reset to 1 on a new
    /// message.
    plan_seqs: Arc<Mutex<HashMap<String, Arc<AtomicU64>>>>,
}

impl PilotServiceImpl {
    pub fn new(atlas: AtlasClient, provider_id: String, vlm: VlmClient) -> Self {
        Self {
            atlas,
            provider_id,
            vlm,
            histories: Arc::new(Mutex::new(HashMap::new())),
            cancels: Arc::new(Mutex::new(HashMap::new())),
            steers: Arc::new(Mutex::new(HashMap::new())),
            plan_seqs: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    async fn get_or_create_history(&self, session_id: &str) -> Arc<Mutex<Vec<Message>>> {
        let mut map = self.histories.lock().await;
        map.entry(session_id.to_string())
            .or_insert_with(|| Arc::new(Mutex::new(Vec::new())))
            .clone()
    }

    /// The session's shared, monotonic RTDL plan-id counter, created on first
    /// use and persisted for the process lifetime so ids never reset per turn.
    async fn get_or_create_plan_seq(&self, session_id: &str) -> Arc<AtomicU64> {
        let mut map = self.plan_seqs.lock().await;
        map.entry(session_id.to_string())
            .or_insert_with(|| Arc::new(AtomicU64::new(0)))
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
impl RobonixSystemPilot for PilotServiceImpl {
    type SubmitTaskStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn submit_task(
        &self,
        request: Request<Task>,
    ) -> Result<Response<Self::SubmitTaskStream>, Status> {
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

        // Decide — under a single `steers` lock — whether this task is a mid-task
        // steer for an already-live turn or the start of a new turn. Doing the
        // check and the registration atomically prevents a check-then-insert race
        // where two near-simultaneous submits for one session both start a turn.
        let (steer_tx, steer_rx) = mpsc::channel::<Task>(32);
        let existing_steer = {
            let mut steers = self.steers.lock().await;
            match steers.get(&task.session_id) {
                Some(existing) => Some(existing.clone()),
                None => {
                    steers.insert(task.session_id.clone(), steer_tx.clone());
                    None
                }
            }
        };
        if let Some(existing) = existing_steer {
            // A turn is already live: hand this to its steer queue and return an
            // empty stream; events keep flowing on that turn's original stream.
            let id = task.session_id.clone();
            let ok = existing.send(task).await.is_ok();
            log::debug!("[pilot] steer task for session {id} (queued={ok})");
            let (_tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(1);
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        let history_arc = self.get_or_create_history(&task.session_id).await;
        let plan_seq = self.get_or_create_plan_seq(&task.session_id).await;
        // what is tokio's tx and rx:
        // https://docs.rs/tokio/latest/tokio/sync/mpsc/struct.Sender.html
        // https://tokio.rs/tokio/tutorial/channels
        // MPSC: Multiple Producer Single Consumer
        let (tx, rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
        let atlas = self.atlas.clone();
        let provider_id = self.provider_id.clone();
        let vlm = self.vlm.clone();
        let session_id = task.session_id.clone();
        let cancels = Arc::clone(&self.cancels);
        let steers = Arc::clone(&self.steers);

        let (cancel_tx, cancel_rx) = watch::channel(false);
        cancels.lock().await.insert(session_id.clone(), cancel_tx);
        // `steer_tx`/`steer_rx` were created above; the sender is already
        // registered in `self.steers` under the atomic check, and `steer_rx`
        // moves into the turn below to drain mid-task steers.

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

            let mut atlas_for_turn = atlas.clone();
            let mut executor = match build_executor_conn(atlas, &provider_id).await {
                Ok(e) => e,
                Err(e) => {
                    let _ = tx
                        .send(Err(Status::unavailable(format!(
                            "cannot reach Executor via atlas: {e:#}"
                        ))))
                        .await;
                    cancels.lock().await.remove(&session_id);
                    steers.lock().await.remove(&session_id);
                    return;
                }
            };

            let mut history = history_arc.lock().await;
            if let Err(e) = planner::run_turn(
                &task,
                &mut history,
                &vlm,
                &mut executor,
                &mut atlas_for_turn,
                &provider_id,
                &tx,
                cancel_rx,
                steer_rx,
                plan_seq,
            )
            .await
            {
                log::error!("[pilot] turn error for session '{session_id}': {e:#}");
                let _ = tx.send(Err(Status::internal(e.to_string()))).await;
            }

            cancels.lock().await.remove(&session_id);
            steers.lock().await.remove(&session_id);
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

/// Connect to executor's Execute RPC. Capability discovery (what's available
/// for the LLM to call) is done directly against atlas, not through executor.
async fn build_executor_conn(
    mut atlas: AtlasClient,
    consumer_id: &str,
) -> anyhow::Result<ExecutorConn> {
    let (_, _, exec_ch) = atlas_client::connect_to_capability(
        &mut atlas,
        consumer_id,
        "robonix/system/executor/execute",
    )
    .await
    .context("connect_to_capability robonix/system/executor/execute")?;
    Ok(ExecutorConn {
        graph: RobonixSystemExecutorExecuteClient::new(exec_ch),
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
