// SPDX-License-Identifier: MulanPSL-2.0
// cmd/chat.rs — TUI chat client (connects to robonix-pilot via PilotService)

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
use std::io;
use tokio_stream::StreamExt;
use uuid::Uuid;

pub mod pb {
    pub mod pilot {
        tonic::include_proto!("robonix.pilot");
    }
    pub mod executor {
        tonic::include_proto!("robonix.executor");
    }
}

struct ChatMessage {
    role: Role,
    text: String,
}

enum Role {
    User,
    Agent,
    ToolCall,
    Status,
}

pub async fn execute(server: &str) -> Result<()> {
    let atlas_endpoint = if server.starts_with("http") {
        server.to_string()
    } else {
        format!("http://{server}")
    };

    let pilot_endpoint = discover_pilot(&atlas_endpoint).await.unwrap_or_else(|e| {
        log::warn!("pilot discovery timed out ({e:#}), falling back to 127.0.0.1:50071");
        "http://127.0.0.1:50071".to_string()
    });
    let pilot_endpoint = localhost_to_ipv4_loopback(&pilot_endpoint);

    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    terminal::enable_raw_mode()?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_tui(&mut terminal, &pilot_endpoint).await;

    terminal::disable_raw_mode()?;
    terminal.backend_mut().execute(LeaveAlternateScreen)?;

    result
}

/// Try to discover Pilot via Atlas, retrying for up to `timeout_secs` seconds.
async fn discover_pilot(atlas_endpoint: &str) -> Result<String> {
    const RETRY_INTERVAL_MS: u64 = 2_000;
    const TIMEOUT_SECS: u64 = 60;

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(TIMEOUT_SECS);
    let mut attempt = 0u32;

    loop {
        attempt += 1;
        match try_discover_pilot_once(atlas_endpoint).await {
            Ok(ep) => return Ok(ep),
            Err(e) => {
                if std::time::Instant::now() >= deadline {
                    anyhow::bail!("pilot not found in Atlas after {TIMEOUT_SECS}s: {e:#}");
                }
                if attempt == 1 {
                    // Show a one-time notice so the user knows we're waiting.
                    eprintln!("[chat] waiting for Pilot to register in Atlas ({e:#})…");
                }
                tokio::time::sleep(std::time::Duration::from_millis(RETRY_INTERVAL_MS)).await;
            }
        }
    }
}

async fn try_discover_pilot_once(atlas_endpoint: &str) -> Result<String> {
    let mut sdk = robonix_sdk::RobonixClient::connect(atlas_endpoint).await?;
    let nodes = sdk
        .query_nodes_opts(robonix_sdk::QueryNodesOpts {
            contract_id: "robonix/sys/runtime/pilot".to_string(),
            ..Default::default()
        })
        .await?;

    for node in &nodes {
        for iface in &node.interfaces {
            if let Ok(meta) = serde_json::from_str::<serde_json::Value>(&iface.metadata_json) {
                if let Some(ep) = meta.get("endpoint").and_then(|v| v.as_str()) {
                    let uri = if ep.starts_with("http") {
                        ep.to_string()
                    } else {
                        format!("http://{ep}")
                    };
                    return Ok(localhost_to_ipv4_loopback(&uri));
                }
            }
        }
    }
    anyhow::bail!("no pilot interface found in Atlas registry")
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    pilot_endpoint: &str,
) -> Result<()> {
    let mut messages: Vec<ChatMessage> = vec![ChatMessage {
        role: Role::Status,
        text: format!("Connected to Pilot at {pilot_endpoint}. Type a message and press Enter."),
    }];
    let mut input = String::new();
    let mut scroll: u16 = 0;
    let mut busy = false;
    let session_id = Uuid::new_v4().to_string();

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
                        messages.push(ChatMessage { role: Role::User, text: msg.clone() });
                        busy = true;
                        draw(terminal, &messages, &input, scroll, busy)?;

                        match send_message(pilot_endpoint, &session_id, &msg, &mut messages, terminal, &input).await {
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
                    KeyCode::Backspace => { input.pop(); }
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
    pilot_endpoint: &str,
    session_id: &str,
    user_msg: &str,
    messages: &mut Vec<ChatMessage>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
) -> Result<()> {
    use pb::pilot::{HandleIntentRequest, Intent};

    const INTENT_SOURCE_TEXT: u32 = 0;
    const EVT_TEXT_CHUNK: u32 = 0;
    const EVT_TASK_GRAPH: u32 = 1;
    const EVT_BATCH_RESULT: u32 = 2;
    const EVT_STATUS: u32 = 3;
    const EVT_FINAL_TEXT: u32 = 4;

    let mut client =
        pb::pilot::pilot_service_client::PilotServiceClient::connect(pilot_endpoint.to_string())
            .await
            .context("failed to connect to Pilot")?;

    let intent = Intent {
        intent_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: user_msg.to_string(),
        audio_data: vec![],
        context_json: String::new(),
        timestamp_ms: now_ms(),
    };

    let mut stream = client
        .handle_intent(HandleIntentRequest {
            intent: Some(intent),
        })
        .await
        .context("HandleIntent RPC failed")?
        .into_inner();

    while let Some(event) = stream.next().await {
        let event = event.context("stream error")?;
        match event.event_kind {
            EVT_TEXT_CHUNK => {
                let t = event.text_chunk.clone();
                if let Some(last) = messages.last_mut() {
                    if matches!(last.role, Role::Agent) {
                        last.text.push_str(&t);
                    } else {
                        messages.push(ChatMessage { role: Role::Agent, text: t });
                    }
                } else {
                    messages.push(ChatMessage { role: Role::Agent, text: t });
                }
            }
            EVT_FINAL_TEXT => {
                let t = event.final_text.clone();
                let has_agent = messages.last().map_or(false, |m| matches!(m.role, Role::Agent));
                if !has_agent && !t.is_empty() {
                    messages.push(ChatMessage { role: Role::Agent, text: t });
                }
            }
            EVT_TASK_GRAPH => {
                if let Some(ref g) = event.task_graph {
                    for call in &g.calls {
                        messages.push(ChatMessage {
                            role: Role::ToolCall,
                            text: format!(
                                "[r{}] {}({})",
                                g.round, call.tool_name, call.args_json
                            ),
                        });
                    }
                }
            }
            EVT_BATCH_RESULT | EVT_STATUS | _ => {}
        }
        draw(terminal, messages, input, 0, true)?;
    }
    Ok(())
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
            let (prefix, indent, style) = match msg.role {
                Role::User    => ("You:   ", "       ", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Role::Agent   => ("Pilot: ", "       ", Style::default().fg(Color::Green)),
                Role::ToolCall => (">      ", "       ", Style::default().fg(Color::Yellow)),
                Role::Status  => ("",        "",        Style::default().fg(Color::Magenta).add_modifier(Modifier::ITALIC)),
            };
            for (i, text_line) in msg.text.lines().enumerate() {
                let lead = if i == 0 { prefix } else { indent };
                lines.push(Line::from(vec![
                    Span::styled(lead, style),
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
            .block(Block::default().borders(Borders::ALL).title(format!(" Robonix{status} ")))
            .wrap(Wrap { trim: false })
            .scroll((auto_scroll, 0));
        f.render_widget(history, chunks[0]);

        let input_widget = Paragraph::new(input.to_string())
            .block(Block::default().borders(Borders::ALL).title(" > Type message (Enter to send, Ctrl+C quit) "));
        f.render_widget(input_widget, chunks[1]);
    })?;
    Ok(())
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

/// Pilot binds IPv4 (`0.0.0.0`). Resolving `localhost` often prefers `::1`, so the
/// gRPC client hits IPv6 and gets connection refused — force IPv4 loopback.
fn localhost_to_ipv4_loopback(url: &str) -> String {
    url.replace("localhost", "127.0.0.1")
}
