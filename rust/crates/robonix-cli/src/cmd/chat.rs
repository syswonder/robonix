// SPDX-License-Identifier: MulanPSL-2.0
// cmd/chat.rs — TUI chat client (connects to robonix-liaison via SrvLiaison).
//
// Modalities (one TUI, one stream type per turn):
//
//   ┌────────────────┐ Enter ─────────► SrvLiaison.Stream(Task)
//   │ rbnx chat TUI  │                 (text → PilotEvent stream)
//   └────────────────┘ Ctrl+V ────────► SrvLiaison.StartVoiceSession(req)
//                                       (voice → VoiceEvent stream which
//                                        wraps PilotEvent in `pilot` field)
//
// Liaison handles user-id assignment, voice mic/ASR/voiceprint/TTS/speaker
// orchestration, and forwarding to Pilot. The TUI just renders events.

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
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use std::cell::RefCell;
use std::io;
use std::rc::Rc;
use tokio_stream::StreamExt;
use uuid::Uuid;

const CONSUMER_ID: &str = "rbnx-cli/chat";
const LIAISON_CONTRACT_ID: &str = "robonix/system/liaison";

struct ChatMessage {
    role: Role,
    text: String,
}

enum Role {
    User,
    Agent,
    ToolCall,
    Status,
    Voice,
}

const DEFAULT_LIAISON_FALLBACK: &str = "http://127.0.0.1:50081";

pub async fn execute(server: &str) -> Result<()> {
    let atlas_endpoint = if server.starts_with("http") {
        server.to_string()
    } else {
        format!("http://{server}")
    };

    let liaison_endpoint = if let Ok(ep) = std::env::var("ROBONIX_LIAISON_ENDPOINT") {
        log::info!("using ROBONIX_LIAISON_ENDPOINT={ep}");
        if ep.starts_with("http") {
            ep
        } else {
            format!("http://{ep}")
        }
    } else {
        discover_liaison(&atlas_endpoint).await.unwrap_or_else(|e| {
            log::warn!(
                "liaison discovery timed out ({e:#}), falling back to {DEFAULT_LIAISON_FALLBACK}"
            );
            DEFAULT_LIAISON_FALLBACK.to_string()
        })
    };
    let liaison_endpoint = localhost_to_ipv4_loopback(&liaison_endpoint);

    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    terminal::enable_raw_mode()?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_tui(&mut terminal, &liaison_endpoint).await;

    terminal::disable_raw_mode()?;
    terminal.backend_mut().execute(LeaveAlternateScreen)?;

    result
}

/// Try to discover Liaison via Atlas, retrying for up to `timeout_secs`.
async fn discover_liaison(atlas_endpoint: &str) -> Result<String> {
    const RETRY_INTERVAL_MS: u64 = 2_000;
    const TIMEOUT_SECS: u64 = 60;

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(TIMEOUT_SECS);
    let mut attempt = 0u32;

    loop {
        attempt += 1;
        match try_discover_once(atlas_endpoint, LIAISON_CONTRACT_ID).await {
            Ok(ep) => return Ok(ep),
            Err(e) => {
                if std::time::Instant::now() >= deadline {
                    anyhow::bail!("liaison not found in Atlas after {TIMEOUT_SECS}s: {e:#}");
                }
                if attempt == 1 {
                    eprintln!("[chat] waiting for Liaison to register in Atlas ({e:#})…");
                }
                tokio::time::sleep(std::time::Duration::from_millis(RETRY_INTERVAL_MS)).await;
            }
        }
    }
}

async fn try_discover_once(atlas_endpoint: &str, contract_id: &str) -> Result<String> {
    let mut atlas = AtlasClient::connect(atlas_endpoint).await?;
    let transport = atlas_pb::Transport::Grpc;
    let records = atlas.query_capabilities("", contract_id, transport).await?;
    for rec in &records {
        let has = rec
            .interfaces
            .iter()
            .any(|i| i.contract_id == contract_id && i.transport == transport as i32);
        if !has {
            continue;
        }
        let (_, endpoint, _) = atlas
            .connect_capability(CONSUMER_ID, &rec.capability_id, contract_id, transport)
            .await?;
        if endpoint.is_empty() {
            continue;
        }
        let uri = if endpoint.starts_with("http") {
            endpoint
        } else {
            format!("http://{endpoint}")
        };
        return Ok(localhost_to_ipv4_loopback(&uri));
    }
    anyhow::bail!("no {contract_id} interface found in Atlas registry")
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    liaison_endpoint: &str,
) -> Result<()> {
    let local_user = format!("local:{}", whoami_username());
    let messages: Rc<RefCell<Vec<ChatMessage>>> = Rc::new(RefCell::new(vec![ChatMessage {
        role: Role::Status,
        text: format!(
            "Connected to Liaison at {liaison_endpoint} as {local_user}. \
             Enter = send · Ctrl+V = start voice (5s) · Esc = abort turn · Ctrl+C = quit."
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
                let _ = notify_session_end(liaison_endpoint, &session_id, &local_user).await;
                break;
            }

            // Ctrl+V → push-to-talk voice session (5s default).
            if !busy
                && key.modifiers.contains(KeyModifiers::CONTROL)
                && key.code == KeyCode::Char('v')
            {
                busy = true;
                messages.borrow_mut().push(ChatMessage {
                    role: Role::Status,
                    text: "Ctrl+V — starting voice session…".to_string(),
                });
                draw(terminal, &messages.borrow(), &input, scroll, busy)?;
                if let Err(e) = run_voice_session_with_esc_abort(
                    liaison_endpoint,
                    &session_id,
                    &local_user,
                    Rc::clone(&messages),
                    terminal,
                    &input,
                    &mut scroll,
                )
                .await
                {
                    messages.borrow_mut().push(ChatMessage {
                        role: Role::Status,
                        text: format!("Voice error: {e:#}"),
                    });
                }
                busy = false;
                continue;
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
                        let _ =
                            notify_session_end(liaison_endpoint, &session_id, &local_user).await;
                        break;
                    }
                    messages.borrow_mut().push(ChatMessage {
                        role: Role::User,
                        text: msg.clone(),
                    });
                    busy = true;
                    draw(terminal, &messages.borrow(), &input, scroll, busy)?;

                    match run_text_intent_with_esc_abort(
                        liaison_endpoint,
                        &session_id,
                        &local_user,
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

// ── Liaison gRPC helpers ─────────────────────────────────────────────────────

fn build_text_task(session_id: &str, user_id: &str, text: &str) -> crate::pb::pilot::Task {
    use crate::pb::pilot::Task;
    const INTENT_SOURCE_TEXT: u32 = 0;
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: text.to_string(),
        audio_data: vec![],
        context_json: serde_json::json!({"user_id": user_id, "modality": "text"}).to_string(),
        timestamp_ms: now_ms(),
    }
}

fn build_control_task(
    session_id: &str,
    user_id: &str,
    extra_context_json: &str,
) -> crate::pb::pilot::Task {
    use crate::pb::pilot::Task;
    const INTENT_SOURCE_TEXT: u32 = 0;
    let mut ctx: serde_json::Value =
        serde_json::from_str(extra_context_json.trim()).unwrap_or_else(|_| serde_json::json!({}));
    if let Some(obj) = ctx.as_object_mut() {
        obj.entry("user_id").or_insert(serde_json::json!(user_id));
    }
    Task {
        task_id: Uuid::new_v4().to_string(),
        session_id: session_id.to_string(),
        source: INTENT_SOURCE_TEXT,
        text: String::new(),
        audio_data: vec![],
        context_json: ctx.to_string(),
        timestamp_ms: now_ms(),
    }
}

async fn notify_session_end(liaison_endpoint: &str, session_id: &str, user_id: &str) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;

    let mut client = SystemLiaisonClient::connect(liaison_endpoint.to_string())
        .await
        .context("failed to connect to Liaison for session_end")?;

    let task = build_control_task(session_id, user_id, r#"{"session_end":true}"#);
    let mut stream = client
        .submit_task(tonic::Request::new(task))
        .await
        .context("Liaison Stream session_end failed")?
        .into_inner();
    while stream.next().await.is_some() {}
    Ok(())
}

async fn abort_session(liaison_endpoint: &str, session_id: &str, user_id: &str) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;

    let mut client = SystemLiaisonClient::connect(liaison_endpoint.to_string())
        .await
        .context("failed to connect to Liaison for abort_turn")?;

    let task = build_control_task(session_id, user_id, r#"{"abort_turn":true}"#);
    let _ = client
        .submit_task(tonic::Request::new(task))
        .await
        .context("Liaison abort_turn Stream failed")?;
    Ok(())
}

// ── Text turn ────────────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
async fn run_text_intent_with_esc_abort(
    liaison_endpoint: &str,
    session_id: &str,
    user_id: &str,
    user_msg: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;
    use crate::pb::pilot::PilotEvent;
    use tonic::Status;

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<PilotEvent, Status>>(64);
    let liaison_ep = liaison_endpoint.to_string();
    let task = build_text_task(session_id, user_id, user_msg);

    let _stream_task = tokio::spawn(async move {
        let mut client = match SystemLiaisonClient::connect(liaison_ep.clone()).await {
            Ok(c) => c,
            Err(e) => {
                let _ = tx.send(Err(Status::unavailable(e.to_string()))).await;
                return;
            }
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
                            text: format!("Liaison stream error: {e}"),
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
                                let _ = abort_session(liaison_endpoint, session_id, user_id).await;
                                messages.borrow_mut().push(ChatMessage {
                                    role: Role::Status,
                                    text: "Esc — abort_turn sent (in-flight turn should stop)."
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

// ── Voice turn ───────────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
async fn run_voice_session_with_esc_abort(
    liaison_endpoint: &str,
    session_id: &str,
    user_id: &str,
    messages: Rc<RefCell<Vec<ChatMessage>>>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
    scroll: &mut u16,
) -> Result<()> {
    use crate::pb::contracts::system_liaison_client::SystemLiaisonClient;
    use crate::pb::liaison::{StartVoiceSessionRequest, VoiceEvent};
    use tonic::Status;

    let req = StartVoiceSessionRequest {
        session_id: session_id.to_string(),
        client_user_id: user_id.to_string(),
        record_seconds: voice_record_seconds(),
        language: voice_language(),
        tts_enabled: voice_tts_enabled(),
        mic_node_id: voice_node("ROBONIX_CHAT_MIC_NODE"),
        asr_node_id: voice_node("ROBONIX_CHAT_ASR_NODE"),
        voiceprint_node_id: String::new(),
        tts_node_id: voice_node("ROBONIX_CHAT_TTS_NODE"),
        speaker_node_id: voice_node("ROBONIX_CHAT_SPEAKER_NODE"),
        context_json: String::new(),
    };

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Result<VoiceEvent, Status>>(64);
    let liaison_ep = liaison_endpoint.to_string();

    let _stream_task = tokio::spawn(async move {
        let mut client = match SystemLiaisonClient::connect(liaison_ep.clone()).await {
            Ok(c) => c,
            Err(e) => {
                let _ = tx.send(Err(Status::unavailable(e.to_string()))).await;
                return;
            }
        };
        let stream = match client.start_voice_session(tonic::Request::new(req)).await {
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
                        apply_voice_event(&messages, &event)?;
                        draw(terminal, &messages.borrow(), input, 0, true)?;
                    }
                    Some(Err(e)) => {
                        messages.borrow_mut().push(ChatMessage {
                            role: Role::Status,
                            text: format!("Voice stream error: {e}"),
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
                                let _ = abort_session(liaison_endpoint, session_id, user_id).await;
                                messages.borrow_mut().push(ChatMessage {
                                    role: Role::Status,
                                    text: "Esc — abort_turn sent (Pilot stops; voice playback may still finish).".to_string(),
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

fn voice_record_seconds() -> u32 {
    std::env::var("ROBONIX_CHAT_VOICE_SECONDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0) // 0 → server default (5s)
}

fn voice_language() -> String {
    std::env::var("ROBONIX_CHAT_VOICE_LANG").unwrap_or_default()
}

fn voice_tts_enabled() -> bool {
    !matches!(
        std::env::var("ROBONIX_CHAT_VOICE_TTS").as_deref(),
        Ok("0") | Ok("false") | Ok("no") | Ok("off")
    )
}

fn voice_node(env_key: &str) -> String {
    std::env::var(env_key).unwrap_or_default()
}

// ── Event → ChatMessage rendering ────────────────────────────────────────────

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

fn apply_voice_event(
    messages: &Rc<RefCell<Vec<ChatMessage>>>,
    event: &crate::pb::liaison::VoiceEvent,
) -> Result<()> {
    // Mirror the kinds in voice.rs / VoiceEvent.msg
    const KIND_SESSION_STARTED: u32 = 0;
    const KIND_RECORDING_STARTED: u32 = 1;
    const KIND_RECORDING_DONE: u32 = 2;
    const KIND_ASR_PARTIAL: u32 = 3;
    const KIND_ASR_FINAL: u32 = 4;
    const KIND_USER_IDENTIFIED: u32 = 5;
    const KIND_PILOT: u32 = 6;
    const KIND_TTS_STARTED: u32 = 7;
    const KIND_TTS_DONE: u32 = 8;
    const KIND_SESSION_DONE: u32 = 9;
    const KIND_ERROR: u32 = 10;

    match event.event_kind {
        KIND_SESSION_STARTED | KIND_RECORDING_STARTED | KIND_RECORDING_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("voice · {}", event.status_message),
            });
        }
        KIND_ASR_PARTIAL => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("asr (partial, {:.2}): {}", event.confidence, event.text),
            });
        }
        KIND_ASR_FINAL => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::User,
                text: format!("(voice) {}", event.text),
            });
        }
        KIND_USER_IDENTIFIED => {
            let label = if event.status_message.is_empty() {
                format!("identified user → {}", event.user_id)
            } else {
                format!(
                    "identified user → {} · {}",
                    event.user_id, event.status_message
                )
            };
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: label,
            });
        }
        KIND_PILOT => {
            if let Some(ref pe) = event.pilot {
                apply_pilot_event(messages, pe)?;
            }
        }
        KIND_TTS_STARTED => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("tts · {}", event.status_message),
            });
        }
        KIND_TTS_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("tts done · {}", event.status_message),
            });
        }
        KIND_SESSION_DONE => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Status,
                text: "voice session done".to_string(),
            });
        }
        KIND_ERROR => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Status,
                text: format!("voice error: {}", event.error),
            });
        }
        _ => {
            messages.borrow_mut().push(ChatMessage {
                role: Role::Voice,
                text: format!("voice (kind={}) {}", event.event_kind, event.status_message),
            });
        }
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
                Role::Voice => (
                    "[v]    ",
                    "       ",
                    Style::default()
                        .fg(Color::Blue)
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
                .title(" > Enter = send · Ctrl+V = voice (5s) · Esc = abort · Ctrl+C = quit "),
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

/// Pilot/Liaison bind IPv4 (`0.0.0.0`). Resolving `localhost` often prefers
/// `::1`, so the gRPC client hits IPv6 and gets connection refused — force IPv4 loopback.
fn localhost_to_ipv4_loopback(url: &str) -> String {
    url.replace("localhost", "127.0.0.1")
}

/// Tiny username probe — avoids pulling in `whoami` for the CLI by reading
/// $USER/$USERNAME with a "user" fallback. Liaison uses the real `whoami`
/// crate when it stamps the canonical `local:<user>` identity.
fn whoami_username() -> String {
    std::env::var("USER")
        .or_else(|_| std::env::var("USERNAME"))
        .unwrap_or_else(|_| "user".to_string())
}
