// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx chat` — minimal stdin/stdout chat client for direct pilot testing.
//
// Discovers pilot via atlas (`connect_to_capability(robonix/system/pilot)`),
// opens a SystemPilot.SubmitTask streaming RPC, reads user lines from stdin,
// and renders the streaming PilotEvents back as a typing-effect output.
//
// Liaison-bypass on purpose: when liaison merges back into this branch this
// command will move to talking through liaison instead. For now keeping it
// pilot-direct so local dev can drive the agent loop without the dialogue
// modality layer.

use anyhow::{Context, Result, bail};
use colored::*;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::io::{self, BufRead, Write};
use tokio_stream::StreamExt;
use tonic::Request;
use uuid::Uuid;

use crate::pb::contracts::system_pilot_client::SystemPilotClient;
use crate::pb::pilot::Task;

// PilotEvent.event_kind discriminants — must mirror service.rs / .msg.
const EVT_TEXT_CHUNK: u32 = 0;
const EVT_PLAN: u32 = 1;
const EVT_BATCH_RESULT: u32 = 2;
const EVT_STATUS: u32 = 3;
const EVT_FINAL_TEXT: u32 = 4;

// SessionState values surfaced via PilotEvent.status.state.
const STATE_ACTIVE: u32 = 0;
const STATE_COMPLETED: u32 = 1;
const STATE_FAILED: u32 = 2;

const CONSUMER_ID: &str = "rbnx-cli/chat";

pub async fn execute(server: &str) -> Result<()> {
    let mut atlas = AtlasClient::connect(server)
        .await
        .with_context(|| format!("connect to atlas at '{server}'"))?;

    let (channel_id, pilot_cap_id, channel) =
        atlas_client::connect_to_capability(&mut atlas, CONSUMER_ID, "robonix/system/pilot")
            .await
            .context("locate pilot via atlas")?;
    println!(
        "{} connected to pilot '{}' (atlas channel {})",
        "[chat]".green().bold(),
        pilot_cap_id.cyan(),
        channel_id.dimmed()
    );
    println!(
        "{} type messages, Ctrl-D / Ctrl-C to exit. /reset starts a new session.",
        "[chat]".green().bold()
    );
    let session_id = Uuid::new_v4().to_string();

    let mut client = SystemPilotClient::new(channel);
    let stdin = io::stdin();
    let mut stdout = io::stdout();
    let mut lines = stdin.lock().lines();
    let mut current_session = session_id;

    loop {
        print!("{} ", ">".bold().blue());
        stdout.flush().ok();

        let Some(line) = lines.next() else { break };
        let line = match line {
            Ok(l) => l,
            Err(e) => {
                eprintln!("[chat] stdin error: {e}");
                break;
            }
        };
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if trimmed == "/reset" {
            current_session = Uuid::new_v4().to_string();
            println!("{} new session", "[chat]".yellow().bold());
            continue;
        }

        let task = Task {
            task_id: Uuid::new_v4().to_string(),
            session_id: current_session.clone(),
            text: trimmed.to_string(),
            timestamp_ms: now_ms(),
            ..Default::default()
        };

        let resp = match client.submit_task(Request::new(task)).await {
            Ok(r) => r,
            Err(e) => {
                eprintln!("{} pilot RPC failed: {e}", "[chat]".red().bold());
                continue;
            }
        };
        let mut stream = resp.into_inner();

        while let Some(event) = stream.next().await {
            let event = match event {
                Ok(e) => e,
                Err(e) => {
                    eprintln!("\n{} stream error: {e}", "[chat]".red().bold());
                    break;
                }
            };
            match event.event_kind {
                EVT_TEXT_CHUNK => {
                    print!("{}", event.text_chunk);
                    stdout.flush().ok();
                }
                EVT_FINAL_TEXT => {
                    // pilot emits the consolidated text; we already rendered
                    // chunks live, so just newline-terminate the line.
                    println!();
                }
                EVT_PLAN => {
                    if let Some(plan) = event.plan {
                        let names: Vec<String> = plan
                            .calls
                            .iter()
                            .map(|c| {
                                c.contract_id
                                    .rsplit_once('/')
                                    .map(|(_, leaf)| leaf.to_string())
                                    .unwrap_or_else(|| c.contract_id.clone())
                            })
                            .collect();
                        println!(
                            "\n{} plan(round={}): [{}]",
                            "[plan]".cyan().bold(),
                            plan.round,
                            names.join(", ")
                        );
                    }
                }
                EVT_BATCH_RESULT => {
                    if let Some(batch) = event.batch_result {
                        for r in &batch.results {
                            let head = r
                                .contract_id
                                .rsplit_once('/')
                                .map(|(_, leaf)| leaf.to_string())
                                .unwrap_or_else(|| r.contract_id.clone());
                            if r.success {
                                let preview: String = r.output.chars().take(80).collect();
                                println!("{} {} → {}", "[ok]".green(), head, preview.dimmed());
                            } else {
                                println!("{} {} → {}", "[err]".red(), head, r.error);
                            }
                        }
                    }
                }
                EVT_STATUS => {
                    if let Some(s) = event.status {
                        match s.state {
                            STATE_FAILED => {
                                eprintln!(
                                    "{} session failed: {}",
                                    "[chat]".red().bold(),
                                    s.message
                                );
                            }
                            STATE_COMPLETED => {} // turn done; wait for next prompt
                            STATE_ACTIVE => {}
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
        println!();
    }

    let _ = atlas.disconnect_capability(&channel_id).await;
    println!("{} bye", "[chat]".green().bold());
    Ok(())
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

// keep silence-the-compiler if liaison comes in later and this file is gone
#[allow(dead_code)]
fn _ensure_used() -> Result<()> {
    bail!("placeholder")
}
