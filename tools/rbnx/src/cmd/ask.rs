// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx ask` — non-interactive sibling of `rbnx chat`. Same atlas
// + SubmitTask gRPC path, but no ratatui shell: prints events to
// stdout as they arrive and exits when the pilot stream closes.
//
// Use cases:
//   * scripted regression / smoke tests that need pilot in the loop
//     without a human typing into the TUI
//   * CI / agent-driven runs where stdout is the artifact
//   * quick one-shot prompts ("describe what's in front of the robot")
//
// Wire format is identical to chat: same Task message, same session
// semantics, same event stream. So if a prompt works in `rbnx ask`
// it will work in `rbnx chat`, and vice versa.

use anyhow::{Context, Result};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::io::{self, Write};
use std::time::{SystemTime, UNIX_EPOCH};
use tokio_stream::StreamExt;
use tonic::Request;
use uuid::Uuid;

use crate::pb::contracts::robonix_system_pilot_client::RobonixSystemPilotClient;
use crate::pb::pilot::{CapabilityCall, Plan, Task};

// PilotEvent.event_kind discriminants — must mirror service.rs / .msg.
const EVT_TEXT_CHUNK: u32 = 0;
const EVT_PLAN: u32 = 1;
const EVT_BATCH_RESULT: u32 = 2;
const EVT_STATUS: u32 = 3;
const EVT_FINAL_TEXT: u32 = 4;

const STATE_FAILED: u32 = 2;
// RtdlNodeState.state == SUCCEEDED (see lib/pilot/msg/RtdlNodeState.msg).
const NODE_STATE_SUCCEEDED: u32 = 2;
const CONSUMER_ID: &str = "rbnx-cli/ask";

pub async fn execute(server: &str, prompt: &str, json: bool) -> Result<()> {
    let mut atlas = AtlasClient::connect(server)
        .await
        .with_context(|| format!("connect to atlas at '{server}'"))?;
    let (channel_id, pilot_cap_id, channel) =
        atlas_client::connect_to_capability(&mut atlas, CONSUMER_ID, "robonix/system/pilot")
            .await
            .context("locate pilot via atlas")?;
    if !json {
        eprintln!("[ask] connected to pilot '{pilot_cap_id}' (channel {channel_id})");
    }
    let mut client = RobonixSystemPilotClient::new(channel);

    let session_id = Uuid::new_v4().to_string();
    let task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.clone(),
        source: 0, // INTENT_SOURCE_TEXT
        text: prompt.to_string(),
        audio_data: vec![],
        context_json: String::new(),
        timestamp_ms: now_ms(),
    };
    let mut stream = client
        .submit_task(Request::new(task))
        .await
        .context("SubmitTask RPC failed")?
        .into_inner();

    let stdout = io::stdout();
    let mut out = stdout.lock();
    let mut last_was_chunk = false;
    let mut had_failure = false;

    while let Some(event) = stream.next().await {
        let event = event.context("pilot stream error")?;
        if json {
            // One JSON object per event, line-delimited. Lets callers
            // pipe through `jq` without grappling with the TUI shape.
            let v = serde_json::json!({
                "event_kind": event.event_kind,
                "session_id": event.session_id,
                "text_chunk": event.text_chunk,
                "final_text": event.final_text,
                "plan": event.plan.as_ref().map(|p| serde_json::json!({
                    "round": p.round,
                    "root_index": p.root_index,
                    "calls": plan_calls(p).into_iter().map(|c| serde_json::json!({
                        "call_id": c.call_id,
                        "provider_id": c.provider_id,
                        "contract_id": c.contract_id,
                        "args_json": c.args_json,
                    })).collect::<Vec<_>>(),
                })),
                "batch_result": event.batch_result.as_ref().map(|b| serde_json::json!({
                    "round": b.round,
                    "any_failed": b.any_failed,
                    // One full per-node record (leaf and non-leaf); leaf nodes
                    // carry the capability call result, operator nodes the detail.
                    "results": b.results.iter().map(|r| serde_json::json!({
                        "node_index": r.node_index,
                        "node_kind": r.node_kind,
                        "state": r.state,
                        "op_id": r.op_id,
                        "description": r.description,
                        "operator_detail": r.operator_detail,
                        "leaf_result": r.leaf_result.as_ref().map(|lr| serde_json::json!({
                            "call_id": lr.call_id,
                            "contract_id": lr.contract_id,
                            "success": lr.success,
                            "output": lr.output,
                            "error": lr.error,
                        })),
                    })).collect::<Vec<_>>(),
                })),
                "status": event.status.as_ref().map(|s| serde_json::json!({
                    "state": s.state,
                    "message": s.message,
                })),
            });
            writeln!(out, "{v}")?;
            out.flush()?;
            continue;
        }

        // Plain-text mode: stream agent text inline; surface tool
        // calls + results on their own lines so a human reading the
        // log can follow the loop without parsing JSON.
        match event.event_kind {
            EVT_TEXT_CHUNK => {
                let chunk = event.text_chunk;
                if chunk.is_empty() {
                    continue;
                }
                if !last_was_chunk {
                    write!(out, "\n[agent] ")?;
                }
                write!(out, "{chunk}")?;
                out.flush()?;
                last_was_chunk = true;
            }
            EVT_PLAN => {
                if let Some(plan) = event.plan {
                    if last_was_chunk {
                        writeln!(out)?;
                        last_was_chunk = false;
                    }
                    let leaves: Vec<String> = plan_calls(&plan)
                        .into_iter()
                        .map(|c| {
                            c.contract_id
                                .rsplit_once('/')
                                .map(|(_, l)| l.to_string())
                                .unwrap_or_else(|| c.contract_id.clone())
                        })
                        .collect();
                    writeln!(out, "[plan r{}] {}", plan.round, leaves.join(", "))?;
                    out.flush()?;
                }
            }
            EVT_BATCH_RESULT => {
                if let Some(batch) = event.batch_result {
                    if last_was_chunk {
                        writeln!(out)?;
                        last_was_chunk = false;
                    }
                    for r in &batch.results {
                        let success = r.state == NODE_STATE_SUCCEEDED;
                        // Leaf nodes carry a capability call result; non-leaf
                        // (sequence/parallel) nodes carry an operator detail.
                        let (label, body) = match r.leaf_result.as_ref() {
                            Some(lr) => (
                                lr.contract_id
                                    .rsplit_once('/')
                                    .map(|(_, l)| l.to_string())
                                    .unwrap_or_else(|| lr.contract_id.clone()),
                                if lr.success {
                                    lr.output.clone()
                                } else {
                                    lr.error.clone()
                                },
                            ),
                            None if !r.description.is_empty() => {
                                (r.description.clone(), r.operator_detail.clone())
                            }
                            None => (format!("node{}", r.node_index), r.operator_detail.clone()),
                        };
                        let summary = compact_one_line(&body, 200);
                        let mark = if success { "✓" } else { "✗" };
                        writeln!(out, "  [{mark} {label}] {summary}")?;
                    }
                    out.flush()?;
                }
            }
            EVT_STATUS => {
                if let Some(s) = event.status {
                    if last_was_chunk {
                        writeln!(out)?;
                        last_was_chunk = false;
                    }
                    if s.state == STATE_FAILED {
                        had_failure = true;
                        writeln!(out, "[status FAILED] {}", s.message)?;
                    } else if !s.message.is_empty() {
                        writeln!(out, "[status] {}", s.message)?;
                    }
                    out.flush()?;
                }
            }
            EVT_FINAL_TEXT => {
                // text_chunk events already streamed the full text.
            }
            _ => {}
        }
    }

    if last_was_chunk {
        writeln!(out)?;
    }
    let _ = atlas.disconnect_capability(&channel_id).await;
    if had_failure {
        anyhow::bail!("pilot reported FAILED status");
    }
    Ok(())
}

fn compact_one_line(s: &str, n: usize) -> String {
    let flat: String = s
        .chars()
        .map(|c| if c == '\n' || c == '\r' { ' ' } else { c })
        .collect();
    if flat.chars().count() > n {
        let mut out: String = flat.chars().take(n).collect();
        out.push('…');
        out
    } else {
        flat
    }
}

fn plan_calls(plan: &Plan) -> Vec<&CapabilityCall> {
    plan.nodes
        .iter()
        .filter_map(|node| node.call.as_ref())
        .collect()
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}
