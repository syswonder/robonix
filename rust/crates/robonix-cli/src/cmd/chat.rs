// SPDX-License-Identifier: MulanPSL-2.0
// cmd/chat.rs — TUI chat client (connects to robonix-pilot via SystemPilot).
//
// dev-packaging note: this file deliberately tracks the structure of the
// `dev` branch chat.rs so the liaison/voice contributor can rebase with
// minimal conflict. Only the gRPC plumbing changed:
//   - SrvPilotClient → SystemPilotClient (proto namespace rename)
//   - Stream(Task)   → SubmitTask(Task)  (rpc rename)
//   - robonix_sdk::query_nodes → AtlasClient::connect_to_capability
//     (the sdk crate was deleted in dev-packaging)
//
// The Rc<RefCell> + spawned-stream + Esc-abort architecture, the
// audio_data / source fields on Task (used by liaison's voice path),
// PageUp/PageDown scroll, and the "Pilot:" prefix are kept verbatim
// from dev.

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
use std::cell::RefCell;
use std::io;
use std::rc::Rc;
use tokio_stream::StreamExt;
use tonic::transport::Channel;
use uuid::Uuid;

use crate::pb::contracts::system_pilot_client::SystemPilotClient;

const CONSUMER_ID: &str = "rbnx-cli/chat";

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

    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    terminal::enable_raw_mode()?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let banner = format!(
        "Connected to Pilot '{pilot_cap_id}' (channel {channel_id}). \
         Enter = send, Esc = abort turn, Ctrl+C = quit."
    );
    let result = run_tui(&mut terminal, channel, banner).await;

    terminal::disable_raw_mode()?;
    terminal.backend_mut().execute(LeaveAlternateScreen)?;
    let _ = atlas.disconnect_capability(&channel_id).await;

    result
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    channel: Channel,
    banner: String,
) -> Result<()> {
    let messages: Rc<RefCell<Vec<ChatMessage>>> = Rc::new(RefCell::new(vec![ChatMessage {
        role: Role::Status,
        text: banner,
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
                let _ = notify_session_end(channel.clone(), &session_id).await;
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
                        let _ = notify_session_end(channel.clone(), &session_id).await;
                        break;
                    }
                    messages.borrow_mut().push(ChatMessage {
                        role: Role::User,
                        text: msg.clone(),
                    });
                    busy = true;
                    draw(terminal, &messages.borrow(), &input, scroll, busy)?;

                    match run_intent_with_esc_abort(
                        channel.clone(),
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

async fn notify_session_end(channel: Channel, session_id: &str) -> Result<()> {
    use crate::pb::pilot::Task;

    const INTENT_SOURCE_TEXT: u32 = 0;

    let mut client = SystemPilotClient::new(channel);

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
        .submit_task(tonic::Request::new(task))
        .await
        .context("Pilot SubmitTask session_end failed")?
        .into_inner();

    while stream.next().await.is_some() {}
    Ok(())
}

async fn abort_pilot_session(channel: Channel, session_id: &str) -> Result<()> {
    use crate::pb::pilot::Task;

    const INTENT_SOURCE_TEXT: u32 = 0;

    let mut client = SystemPilotClient::new(channel);

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
        .submit_task(tonic::Request::new(task))
        .await
        .context("Pilot SubmitTask abort_turn failed")?;
    Ok(())
}

/// Runs one `SystemPilot.SubmitTask` while polling the keyboard: **Esc**
/// calls [`abort_pilot_session`] (abort_turn `Task`) so Pilot cancels the
/// in-flight turn.
async fn run_intent_with_esc_abort(
    channel: Channel,
    session_id: &str,
    user_msg: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
) -> Result<()> {
    use crate::pb::pilot::{PilotEvent, Task};
    use tonic::Status;

    const INTENT_SOURCE_TEXT: u32 = 0;

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
    let stream_channel = channel.clone();
    let sid = session_id.to_string();
    let text = user_msg.to_string();

    let _stream_task = tokio::spawn(async move {
        let mut client = SystemPilotClient::new(stream_channel);
        let task = Task {
            task_id: Uuid::new_v4().to_string(),
            session_id: sid,
            source: INTENT_SOURCE_TEXT,
            text,
            audio_data: vec![],
            context_json: String::new(),
            timestamp_ms: now_ms(),
        };
        let stream = match client.submit_task(tonic::Request::new(task)).await {
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
                                let _ = abort_pilot_session(channel.clone(), session_id).await;
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
    // event_kind discriminants — see PilotEvent.msg.
    const EVT_TEXT_CHUNK: u32 = 0;
    const EVT_PLAN: u32 = 1;
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
        EVT_PLAN => {
            // dev called this EVT_TASK_GRAPH with `event.task_graph` carrying
            // tool_name + args_json. dev-packaging renamed the message to
            // Plan/CapabilityCall; only contract_id + args_json are exposed,
            // so we leaf-strip contract_id back into a tool-name lookalike to
            // preserve the same `[r{round}] {name}({args})` line shape.
            if let Some(ref p) = event.plan {
                for call in &p.calls {
                    let leaf = call
                        .contract_id
                        .rsplit_once('/')
                        .map(|(_, l)| l.to_string())
                        .unwrap_or_else(|| call.contract_id.clone());
                    m.push(ChatMessage {
                        role: Role::ToolCall,
                        text: format!("[r{}] {}({})", p.round, leaf, call.args_json),
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
