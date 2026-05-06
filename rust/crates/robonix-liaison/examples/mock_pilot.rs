// SPDX-License-Identifier: MulanPSL-2.0
// examples/mock_pilot.rs — minimal SrvPilot stand-in for the voice demo.
//
// Replaces robonix-pilot for tests that only need to verify the
// Liaison ↔ Pilot wiring (no VLM, no Executor, no skill index).
//
// Behaviour:
//   * Registers as `com.robonix.demo.mock_pilot` in Atlas under
//     `robonix/srv/pilot` (same contract as the real Pilot, so Liaison
//     discovers it through `query_nodes_opts(contract_id="robonix/srv/pilot")`).
//   * Implements SrvPilot.Stream:
//       - On `{"abort_turn":true}` / `{"session_end":true}` → close immediately.
//       - Otherwise → emit one TEXT_CHUNK + one FINAL_TEXT echoing the
//         incoming Task (text + user_id), so the demo client can verify the
//         user identity is propagated end-to-end.

use anyhow::Result;
use robonix_interfaces::{
    contracts::srv_pilot_server::{SrvPilot, SrvPilotServer},
    pilot::{PilotEvent, Task},
};
use robonix_sdk::RobonixClient;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const NODE_ID: &str = "com.robonix.demo.mock_pilot";

const EVT_TEXT_CHUNK: u32 = 0;
const EVT_FINAL_TEXT: u32 = 4;

fn task_is_control(task: &Task) -> bool {
    let j = task.context_json.trim();
    if j.is_empty() {
        return false;
    }
    let v: serde_json::Value = match serde_json::from_str(j) {
        Ok(v) => v,
        Err(_) => return false,
    };
    v.get("abort_turn").and_then(|x| x.as_bool()).unwrap_or(false)
        || v.get("session_end").and_then(|x| x.as_bool()).unwrap_or(false)
}

#[derive(Default)]
struct MockPilot;

#[tonic::async_trait]
impl SrvPilot for MockPilot {
    type StreamStream = ReceiverStream<Result<PilotEvent, Status>>;

    async fn stream(&self, request: Request<Task>) -> Result<Response<Self::StreamStream>, Status> {
        let task = request.into_inner();
        let (tx, rx) = mpsc::channel::<Result<PilotEvent, Status>>(8);

        if task_is_control(&task) {
            log::info!("[mock-pilot] control task ({}); closing stream", task.context_json);
            return Ok(Response::new(ReceiverStream::new(rx)));
        }

        let echo = format!(
            "[mock-pilot] received from user_id='{}' source={} text='{}' ctx={}",
            task.user_id, task.source, task.text, task.context_json
        );
        log::info!("{}", echo);

        tokio::spawn(async move {
            let chunk = PilotEvent {
                event_kind: EVT_TEXT_CHUNK,
                session_id: task.session_id.clone(),
                text_chunk: format!("(mock) ack from {} → ", task.user_id),
                task_graph: None,
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
                task_graph: None,
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
    let mut sdk =
        RobonixClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2)).await?;
    sdk.register_node(NODE_ID, "robonix/srv/pilot", "service", "")
        .await?;
    sdk.declare_interface_full(
        NODE_ID,
        "pilot",
        vec!["grpc".to_string()],
        serde_json::json!({"endpoint": advertised}).to_string(),
        port as u32,
        "robonix/srv/pilot",
    )
    .await?;
    log::info!("[mock-pilot] registered as '{NODE_ID}', listening on {advertised}");
    eprintln!("mock-pilot ready on :{port}");

    tonic::transport::Server::builder()
        .add_service(SrvPilotServer::new(MockPilot))
        .serve(listen)
        .await?;
    Ok(())
}
