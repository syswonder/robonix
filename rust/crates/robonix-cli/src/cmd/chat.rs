// SPDX-License-Identifier: MulanPSL-2.0
// cmd/chat.rs — TUI chat client (connects to robonix-pilot via SrvPilot)

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
use std::cell::RefCell;
use std::io;
use std::rc::Rc;
use tokio_stream::StreamExt;
use uuid::Uuid;

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
            contract_id: "robonix/srv/pilot".to_string(),
            ..Default::default()
        })
        .await?;

    for node in &nodes {
        for iface in &node.interfaces {
            if let Ok(meta) = serde_json::from_str::<serde_json::Value>(&iface.metadata_json)
                && let Some(ep) = meta.get("endpoint").and_then(|v| v.as_str())
            {
                let uri = if ep.starts_with("http") {
                    ep.to_string()
                } else {
                    format!("http://{ep}")
                };
                return Ok(localhost_to_ipv4_loopback(&uri));
            }
        }
    }
    anyhow::bail!("no pilot interface found in Atlas registry")
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    pilot_endpoint: &str,
) -> Result<()> {
    let messages: Rc<RefCell<Vec<ChatMessage>>> = Rc::new(RefCell::new(vec![ChatMessage {
        role: Role::Status,
        text: format!(
            "Connected to Pilot at {pilot_endpoint}. Enter = send, Esc = abort turn, Ctrl+C = quit."
        ),
    }]));
    let mut input = String::new();
    let mut scroll: u16 = 0;
    let mut busy = false;
    let session_id = Uuid::new_v4().to_string();

    loop {
        draw(terminal, &messages.borrow(), &input, scroll, busy)?;

        if event::poll(std::time::Duration::from_millis(50))?
            && let Event::Key(key) = event::read()?
        {
            if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
                let _ = notify_session_end(pilot_endpoint, &session_id).await;
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
                        let _ = notify_session_end(pilot_endpoint, &session_id).await;
                        break;
                    }
                    messages.borrow_mut().push(ChatMessage {
                        role: Role::User,
                        text: msg.clone(),
                    });
                    busy = true;
                    draw(terminal, &messages.borrow(), &input, scroll, busy)?;

                    match run_intent_with_esc_abort(
                        pilot_endpoint,
                        &session_id,
                        &msg,
                        Rc::clone(&messages),
                        terminal,
                        &input,
                        &mut scroll,
                    )
                    .await
                    {
                        Ok(()) => {}
                        Err(e) => {
                            messages.borrow_mut().push(ChatMessage {
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
    Ok(())
}

async fn notify_session_end(pilot_endpoint: &str, session_id: &str) -> Result<()> {
    use crate::pb::contracts::srv_pilot_client::SrvPilotClient;
    use crate::pb::pilot::Task;

    const INTENT_SOURCE_TEXT: u32 = 0;

    let mut client = SrvPilotClient::connect(pilot_endpoint.to_string())
        .await
        .context("failed to connect to Pilot for session_end")?;

    let task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: r#"{"session_end":true}"#.to_string(),
        timestamp_ms: now_ms(),
    };

    let mut stream = client
        .stream(tonic::Request::new(task))
        .await
        .context("Pilot Stream session_end failed")?
        .into_inner();

    while stream.next().await.is_some() {}
    Ok(())
}

async fn abort_pilot_session(pilot_endpoint: &str, session_id: &str) -> Result<()> {
    use crate::pb::contracts::srv_pilot_client::SrvPilotClient;
    use crate::pb::pilot::Task;

    const INTENT_SOURCE_TEXT: u32 = 0;

    let mut client = SrvPilotClient::connect(pilot_endpoint.to_string())
        .await
        .context("failed to connect to Pilot for abort_turn")?;

    let task = Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: r#"{"abort_turn":true}"#.to_string(),
        timestamp_ms: now_ms(),
    };

    let _ = client
        .stream(tonic::Request::new(task))
        .await
        .context("Pilot abort_turn Stream failed")?;
    Ok(())
}

/// Runs one `SrvPilot.Stream` while polling the keyboard: **Esc** calls
/// [`abort_pilot_session`] (abort_turn `Task`) so Pilot cancels the in-flight turn.
async fn run_intent_with_esc_abort(
    pilot_endpoint: &str,
    session_id: &str,
    user_msg: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
) -> Result<()> {
    use crate::pb::contracts::srv_pilot_client::SrvPilotClient;
    use crate::pb::pilot::{PilotEvent, Task};
    use tonic::Status;

    const INTENT_SOURCE_TEXT: u32 = 0;

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
    let pilot_ep = pilot_endpoint.to_string();
    let sid = session_id.to_string();
    let text = user_msg.to_string();

    let _stream_task = tokio::spawn(async move {
        let mut client = match SrvPilotClient::connect(pilot_ep.clone()).await {
            Ok(c) => c,
            Err(e) => {
                let _ = tx.send(Err(Status::unavailable(e.to_string()))).await;
                return;
            }
        };
        let task = Task {
            task_id: Uuid::new_v4().to_string(),
            session_id: sid,
            source: INTENT_SOURCE_TEXT,
            text,
            audio_data: vec![],
            context_json: String::new(),
            timestamp_ms: now_ms(),
        };
        let stream = match client.stream(tonic::Request::new(task)).await {
            Ok(r) => r.into_inner(),
            Err(e) => {
                let _ = tx.send(Err(e)).await;
                return;
            }
        };
        let mut stream = stream;
        while let Some(item) = stream.next().await {
            if tx.send(item).await.is_err() {
                break;
            }
        }
    });

    loop {
        tokio::select! {
            biased;
            item = rx.recv() => {
                match item {
                    None => break,
                    Some(Ok(event)) => {
                        apply_pilot_event(&messages, &event)?;
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                    }
                    Some(Err(e)) => {
                        messages.borrow_mut().push(ChatMessage {
                            role: Role::Status,
                            text: format!("Pilot stream error: {e}"),
                        });
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                        break;
                    }
                }
            }
            _ = tokio::time::sleep(std::time::Duration::from_millis(25)) => {
                if event::poll(std::time::Duration::ZERO)?
                    && let Event::Key(key) = event::read()? {
                        match key.code {
                            KeyCode::Esc => {
                                let _ = abort_pilot_session(pilot_endpoint, session_id).await;
                                messages.borrow_mut().push(ChatMessage {
                                    role: Role::Status,
                                    text: "Esc — abort_turn sent to Pilot (in-flight turn should stop)."
                                        .to_string(),
                                });
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            KeyCode::PageUp => {
                                *scroll = scroll.saturating_add(5);
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            KeyCode::PageDown => {
                                *scroll = scroll.saturating_sub(5);
                                draw(terminal, &messages.borrow(), input, *scroll, true)?;
                            }
                            _ => {}
                        }
                    }
            }
        }
    }
    Ok(())
}

fn apply_pilot_event(
    messages: &Rc<RefCell<Vec<ChatMessage>>>,
    event: &crate::pb::pilot::PilotEvent,
) -> Result<()> {
    const EVT_TEXT_CHUNK: u32 = 0;
    const EVT_TASK_GRAPH: u32 = 1;
    const EVT_FINAL_TEXT: u32 = 4;

    let mut m = messages.borrow_mut();
    match event.event_kind {
        EVT_TEXT_CHUNK => {
            let t = event.text_chunk.clone();
            if let Some(last) = m.last_mut() {
                if matches!(last.role, Role::Agent) {
                    last.text.push_str(&t);
                } else {
                    m.push(ChatMessage {
                        role: Role::Agent,
                        text: t,
                    });
                }
            } else {
                m.push(ChatMessage {
                    role: Role::Agent,
                    text: t,
                });
            }
        }
        EVT_FINAL_TEXT => {
            let t = event.final_text.clone();
            let has_agent = m.last().is_some_and(|x| matches!(x.role, Role::Agent));
            if !has_agent && !t.is_empty() {
                m.push(ChatMessage {
                    role: Role::Agent,
                    text: t,
                });
            }
        }
        EVT_TASK_GRAPH => {
            if let Some(ref g) = event.task_graph {
                for call in &g.calls {
                    m.push(ChatMessage {
                        role: Role::ToolCall,
                        text: format!("[r{}] {}({})", g.round, call.tool_name, call.args_json),
                    });
                }
            }
        }
        _ => {}
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
                Role::User => (
                    "You:   ",
                    "       ",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Role::Agent => ("Pilot: ", "       ", Style::default().fg(Color::Green)),
                Role::ToolCall => (">      ", "       ", Style::default().fg(Color::Yellow)),
                Role::Status => (
                    "",
                    "",
                    Style::default()
                        .fg(Color::Magenta)
                        .add_modifier(Modifier::ITALIC),
                ),
            };
            for (i, text_line) in msg.text.lines().enumerate() {
                let lead = if i == 0 { prefix } else { indent };
                lines.push(Line::from(vec![
                    Span::styled(lead, style),
                    Span::styled(text_line.to_string(), style),
                ]));
            }
        }

        let status = if busy { " [thinking...]" } else { "" };
        let block = Block::default()
            .borders(Borders::ALL)
            .title(format!(" Robonix{status} "));
        let inner = block.inner(chunks[0]);

        // Scroll offset must be in **wrapped** lines (not raw `\n`-split lines), or long replies
        // clip at the bottom while `total` stays small.
        let text_only = Paragraph::new(lines.clone()).wrap(Wrap { trim: false });
        let total_lines = text_only.line_count(inner.width) as u16;
        let visible = inner.height;
        let auto_scroll = if scroll == 0 {
            total_lines.saturating_sub(visible)
        } else {
            total_lines.saturating_sub(visible).saturating_sub(scroll)
        };

        let history = Paragraph::new(lines)
            .block(block)
            .wrap(Wrap { trim: false })
            .scroll((auto_scroll, 0));
        f.render_widget(history, chunks[0]);

        let input_widget = Paragraph::new(input.to_string()).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" > Enter = send · Esc = abort turn · Ctrl+C = quit "),
        );
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
