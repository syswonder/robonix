// SPDX-License-Identifier: MulanPSL-2.0
// examples/mock_pilot.rs — minimal RobonixSystemPilot stand-in for the voice demo.
//
// Replaces robonix-pilot for tests that only need to verify the
// Liaison ↔ Pilot wiring (no VLM, no Executor, no skill index).
//
// Behaviour:
//   * Registers as `com.robonix.demo.mock_pilot` in Atlas under
//     `robonix/system/pilot` (same contract as the real Pilot).
//   * Implements RobonixSystemPilot.SubmitTask:
//       - On `{"abort_turn":true}` / `{"session_end":true}` → close immediately.
//       - Otherwise → emit one TEXT_CHUNK + one FINAL_TEXT echoing the
//         incoming Task (text + user_id), so the demo client can verify the
//         user identity is propagated end-to-end.

use anyhow::Result;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_liaison::pb::contracts::robonix_system_pilot_server::{
    RobonixSystemPilot, RobonixSystemPilotServer,
};
use robonix_liaison::pb::pilot::{PilotEvent, Task};
use std::time::Duration;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const CAPABILITY_ID: &str = "com.robonix.demo.mock_pilot";
const NAMESPACE: &str = "robonix/system/pilot";
const CONTRACT_ID: &str = "robonix/system/pilot";

const EVT_TEXT_CHUNK: u32 = 0;
const EVT_FINAL_TEXT: u32 = 4;

fn extract_user_id(task: &Task) -> String {
    let v: serde_json::Value =
        serde_json::from_str(task.context_json.trim()).unwrap_or_else(|_| serde_json::json!({}));
    v.get("user_id")
        .and_then(|x| x.as_str())
        .unwrap_or("(none)")
        .to_string()
}

fn task_is_control(task: &Task) -> bool {
    let j = task.context_json.trim();
    if j.is_empty() {
        return false;
    }
    let v: serde_json::Value = match serde_json::from_str(j) {
        Ok(v) => v,
        Err(_) => return false,
    };
    v.get("abort_turn")
        .and_then(|x| x.as_bool())
        .unwrap_or(false)
        || v.get("session_end")
            .and_then(|x| x.as_bool())
            .unwrap_or(false)
}

#[derive(Default)]
struct MockPilot;

#[tonic::async_trait]
impl RobonixSystemPilot for MockPilot {
    type SubmitTaskStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn submit_task(
        &self,
        request: Request<Task>,
    ) -> Result<Response<Self::SubmitTaskStream>, Status> {
        let task = request.into_inner();
        let (tx, rx) = mpsc::channel::<Result<PilotEvent, Status>>(8);

        if task_is_control(&task) {
            log::info!(
                "[mock-pilot] control task ({}); closing stream",
                task.context_json
            );
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        let user_id = extract_user_id(&task);
        let echo = format!(
            "[mock-pilot] received from user_id='{user_id}' source={} text='{}' ctx={}",
            task.source, task.text, task.context_json
        );
        log::info!("{echo}");

        tokio::spawn(async move {
            let chunk = PilotEvent {
                event_kind: EVT_TEXT_CHUNK,
                session_id: task.session_id.clone(),
                text_chunk: format!("(mock) ack from {user_id} → "),
                plan: None,
                batch_result: None,
                status: None,
                final_text: String::new(),
            };
            let _ = tx.send(Ok(chunk)).await;
            tokio::time::sleep(Duration::from_millis(80)).await;
            let final_ev = PilotEvent {
                event_kind: EVT_FINAL_TEXT,
                session_id: task.session_id.clone(),
                text_chunk: String::new(),
                plan: None,
                batch_result: None,
                status: None,
                final_text: format!("you said \"{}\"", task.text),
            };
            let _ = tx.send(Ok(final_ev)).await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let atlas = std::env::var("ROBONIX_ATLAS").unwrap_or_else(|_| "127.0.0.1:50051".to_string());
    let atlas_http = if atlas.starts_with("http") {
        atlas
    } else {
        format!("http://{atlas}")
    };

    let port: u16 = std::env::var("MOCK_PILOT_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50071);
    let listen: std::net::SocketAddr = format!("0.0.0.0:{port}").parse()?;
    let advertised = format!("127.0.0.1:{port}");

    log::info!("[mock-pilot] connecting to Atlas at {atlas_http}");
    let mut atlas =
        AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2)).await?;
    atlas.register_service(CAPABILITY_ID, NAMESPACE, "").await?;
    atlas
        .declare_capability(
            CAPABILITY_ID,
            CONTRACT_ID,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/system/pilot.v1.toml",
                "robonix.contracts.RobonixSystemPilot",
                "/robonix.contracts.RobonixSystemPilot/SubmitTask",
            ),
        )
        .await?;
    log::info!("[mock-pilot] registered as '{CAPABILITY_ID}', listening on {advertised}");
    eprintln!("mock-pilot ready on :{port}");

    {
        let mut hb = atlas.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(CAPABILITY_ID).await {
                    log::warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    tonic::transport::Server::builder()
        .add_service(RobonixSystemPilotServer::new(MockPilot))
        .serve(listen)
        .await?;
    Ok(())
}
