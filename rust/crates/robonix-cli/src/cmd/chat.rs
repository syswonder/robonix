// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx chat` — interactive ratatui TUI for talking to robonix-pilot.
//
// Layout / input handling / message rendering kept verbatim from the
// pre-dev-packaging TUI (the liaison branch builds on this exact shell);
// the only thing rewritten was the RPC layer:
//   - was: robonix-sdk QueryNodes → AgentChat.Chat (robonix.agent_chat proto)
//   - now: AtlasClient.connect_to_capability("robonix/system/pilot")
//          → SystemPilotClient.SubmitTask streaming PilotEvent
//
// Per-event mapping into the existing Role enum:
//   EVT_TEXT_CHUNK   → Role::Agent  (appended into the running message)
//   EVT_PLAN         → Role::ToolCall   (round + leaf names)
//   EVT_BATCH_RESULT → Role::ToolResult (one line per tool result)
//   EVT_STATUS       → Role::Status     (FAILED renders red-style status)
//   EVT_FINAL_TEXT   → ignored (chunks already rendered the text)
//
// When liaison merges back this command will move to liaison's dialogue
// front-end; until then chat hits pilot directly.

use anyhow::{Context, Result};
use crossterm::{
    ExecutableCommand,
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{self, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    Terminal,
    backend::CrosstermBackend,
    layout::{Constraint, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Wrap},
};
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use std::io;
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
const STATE_FAILED: u32 = 2;

const CONSUMER_ID: &str = "rbnx-cli/chat";

struct ChatMessage {
    role: Role,
    text: String,
}

enum Role {
    User,
    Agent,
    ToolCall,
    ToolResult,
    Status,
}

pub async fn execute(server: &str) -> Result<()> {
    // Atlas-side discovery + tonic Channel setup happens BEFORE we enter
    // the alternate screen so any connection error lands cleanly on the
    // user's normal terminal instead of leaving a half-drawn TUI.
    let mut atlas = AtlasClient::connect(server)
        .await
        .with_context(|| format!("connect to atlas at '{server}'"))?;
    let (channel_id, pilot_cap_id, channel) =
        atlas_client::connect_to_capability(&mut atlas, CONSUMER_ID, "robonix/system/pilot")
            .await
            .context("locate pilot via atlas")?;
    let client = SystemPilotClient::new(channel);

    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    terminal::enable_raw_mode()?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let banner = format!(
        "Connected to pilot '{pilot_cap_id}' (channel {channel_id}). \
         Type a message and press Enter."
    );
    let result = run_tui(&mut terminal, client, banner).await;

    terminal::disable_raw_mode()?;
    terminal.backend_mut().execute(LeaveAlternateScreen)?;
    let _ = atlas.disconnect_capability(&channel_id).await;

    result
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    mut client: SystemPilotClient<tonic::transport::Channel>,
    banner: String,
) -> Result<()> {
    let mut messages: Vec<ChatMessage> = vec![ChatMessage {
        role: Role::Status,
        text: banner,
    }];
    let mut input = String::new();
    let mut scroll: u16 = 0;
    let mut busy = false;
    let mut session_id = Uuid::new_v4().to_string();

    loop {
        draw(terminal, &messages, &input, scroll, busy)?;

        if event::poll(std::time::Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
                    break;
                }
                if busy {
                    match key.code {
                        KeyCode::PageUp => scroll = scroll.saturating_add(5),
                        KeyCode::PageDown => scroll = scroll.saturating_sub(5),
                        _ => {}
                    }
                    continue;
                }
                match key.code {
                    KeyCode::Enter => {
                        let msg = input.trim().to_string();
                        input.clear();
                        scroll = 0;
                        if msg.is_empty() {
                            continue;
                        }
                        if msg == "quit" || msg == "exit" {
                            break;
                        }
                        if msg == "/reset" {
                            session_id = Uuid::new_v4().to_string();
                            messages.push(ChatMessage {
                                role: Role::Status,
                                text: format!("New session: {session_id}"),
                            });
                            continue;
                        }
                        messages.push(ChatMessage {
                            role: Role::User,
                            text: msg.clone(),
                        });
                        busy = true;
                        draw(terminal, &messages, &input, scroll, busy)?;

                        match send_message(&mut client, &session_id, &msg, &mut messages, terminal, &input).await {
                            Ok(()) => {}
                            Err(e) => {
                                messages.push(ChatMessage {
                                    role: Role::Status,
                                    text: format!("Error: {e:#}"),
                                });
                            }
                        }
                        busy = false;
                    }
                    KeyCode::Char(c) => input.push(c),
                    KeyCode::Backspace => {
                        input.pop();
                    }
                    KeyCode::PageUp => scroll = scroll.saturating_add(5),
                    KeyCode::PageDown => scroll = scroll.saturating_sub(5),
                    _ => {}
                }
            }
        }
    }
    Ok(())
}

async fn send_message(
    client: &mut SystemPilotClient<tonic::transport::Channel>,
    session_id: &str,
    user_msg: &str,
    messages: &mut Vec<ChatMessage>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
) -> Result<()> {
    let task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        text: user_msg.to_string(),
        timestamp_ms: now_ms(),
        ..Default::default()
    };
    let resp = client
        .submit_task(Request::new(task))
        .await
        .context("SubmitTask RPC failed")?;
    let mut stream = resp.into_inner();

    while let Some(event) = stream.next().await {
        let event = event.context("stream error")?;
        match event.event_kind {
            EVT_TEXT_CHUNK => {
                let chunk = event.text_chunk;
                if chunk.is_empty() {
                    continue;
                }
                if let Some(last) = messages.last_mut() {
                    if matches!(last.role, Role::Agent) {
                        last.text.push_str(&chunk);
                        draw(terminal, messages, input, 0, true)?;
                        continue;
                    }
                }
                messages.push(ChatMessage {
                    role: Role::Agent,
                    text: chunk,
                });
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
                    messages.push(ChatMessage {
                        role: Role::ToolCall,
                        text: format!("[r{}] plan: {}", plan.round, names.join(", ")),
                    });
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
                        // Compact one-liner: collapse newlines, cap at ~80 chars,
                        // hide payload bulk. The full payload is always visible
                        // in the deploy-side log; the TUI is for reading flow,
                        // not auditing tool I/O.
                        let one_line = |s: &str, n: usize| -> String {
                            let flat: String = s
                                .chars()
                                .map(|c| if c == '\n' || c == '\r' { ' ' } else { c })
                                .collect();
                            let mut out: String = flat.chars().take(n).collect();
                            if flat.chars().count() > n {
                                out.push('…');
                            }
                            out
                        };
                        let text = if r.success {
                            if r.output.is_empty() {
                                format!("{head} → ok")
                            } else {
                                format!("{head} → {}", one_line(&r.output, 80))
                            }
                        } else {
                            format!("{head} ✗ {}", one_line(&r.error, 80))
                        };
                        messages.push(ChatMessage {
                            role: Role::ToolResult,
                            text,
                        });
                    }
                }
            }
            EVT_STATUS => {
                if let Some(s) = event.status {
                    if s.state == STATE_FAILED {
                        messages.push(ChatMessage {
                            role: Role::Status,
                            text: format!("session failed: {}", s.message),
                        });
                    } else if !s.message.is_empty() {
                        messages.push(ChatMessage {
                            role: Role::Status,
                            text: s.message,
                        });
                    }
                }
            }
            EVT_FINAL_TEXT => {
                // Already streamed via EVT_TEXT_CHUNK; nothing to do.
            }
            _ => {}
        }
        draw(terminal, messages, input, 0, true)?;
    }
    Ok(())
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn draw(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    messages: &[ChatMessage],
    input: &str,
    scroll: u16,
    busy: bool,
) -> Result<()> {
    terminal.draw(|f| {
        let area = f.area();
        let chunks = Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).split(area);

        let mut lines: Vec<Line> = Vec::new();
        for msg in messages {
            let (prefix, style) = match msg.role {
                Role::User => (
                    "You: ",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Role::Agent => ("Agent: ", Style::default().fg(Color::Green)),
                Role::ToolCall => ("> ", Style::default().fg(Color::Yellow)),
                Role::ToolResult => ("  = ", Style::default().fg(Color::DarkGray)),
                Role::Status => (
                    "",
                    Style::default()
                        .fg(Color::Magenta)
                        .add_modifier(Modifier::ITALIC),
                ),
            };
            for text_line in msg.text.lines() {
                lines.push(Line::from(vec![
                    Span::styled(prefix, style),
                    Span::styled(text_line.to_string(), style),
                ]));
            }
        }

        let total = lines.len() as u16;
        let visible = chunks[0].height.saturating_sub(2);
        let auto_scroll = if scroll == 0 {
            total.saturating_sub(visible)
        } else {
            total.saturating_sub(visible).saturating_sub(scroll)
        };

        let status = if busy { " [thinking...]" } else { "" };
        let history = Paragraph::new(lines)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(format!(" Robonix Agent{status} ")),
            )
            .wrap(Wrap { trim: false })
            .scroll((auto_scroll, 0));
        f.render_widget(history, chunks[0]);

        let input_widget = Paragraph::new(input.to_string()).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" > Type message (Enter to send, Ctrl+C quit) "),
        );
        f.render_widget(input_widget, chunks[1]);
    })?;
    Ok(())
}
