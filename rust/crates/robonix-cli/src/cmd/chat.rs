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

pub mod pb {
    tonic::include_proto!("robonix.agent_chat");
}

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
    let endpoint = if server.starts_with("http") {
        server.to_string()
    } else {
        format!("http://{server}")
    };

    let agent_endpoint = discover_agent(&endpoint).await.unwrap_or_else(|e| {
        log::warn!("agent discovery failed ({e:#}), falling back to {endpoint}");
        endpoint.clone()
    });

    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    terminal::enable_raw_mode()?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_tui(&mut terminal, &agent_endpoint).await;

    terminal::disable_raw_mode()?;
    terminal.backend_mut().execute(LeaveAlternateScreen)?;

    result
}

async fn discover_agent(server_endpoint: &str) -> Result<String> {
    let mut sdk = robonix_sdk::RobonixClient::connect(server_endpoint).await?;
    let nodes = sdk
        .query_nodes_opts(robonix_sdk::QueryNodesOpts {
            abstract_interface_id: "robonix/sys/runtime/agent/agent_chat".to_string(),
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
                    return Ok(uri);
                }
            }
        }
    }
    anyhow::bail!("no agent_chat interface found in registry")
}

async fn run_tui(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    agent_endpoint: &str,
) -> Result<()> {
    let mut messages: Vec<ChatMessage> = vec![ChatMessage {
        role: Role::Status,
        text: format!("Connected to agent at {agent_endpoint}. Type a message and press Enter."),
    }];
    let mut input = String::new();
    let mut scroll: u16 = 0;
    let mut busy = false;

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
                        messages.push(ChatMessage {
                            role: Role::User,
                            text: msg.clone(),
                        });
                        busy = true;
                        draw(terminal, &messages, &input, scroll, busy)?;

                        match send_message(agent_endpoint, &msg, &mut messages, terminal, &input)
                            .await
                        {
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
    agent_endpoint: &str,
    user_msg: &str,
    messages: &mut Vec<ChatMessage>,
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    input: &str,
) -> Result<()> {
    let mut client = pb::agent_chat_client::AgentChatClient::connect(agent_endpoint.to_string())
        .await
        .context("failed to connect to agent")?;

    let request = tonic::Request::new(pb::AgentChatRequest {
        user_message: user_msg.to_string(),
    });

    let response = client.chat(request).await.context("chat RPC failed")?;
    let mut stream = response.into_inner();

    while let Some(event) = stream.next().await {
        let event = event.context("stream error")?;
        if let Some(ev) = event.event {
            match ev {
                pb::agent_chat_event::Event::Status(s) => {
                    messages.push(ChatMessage {
                        role: Role::Status,
                        text: s,
                    });
                }
                pb::agent_chat_event::Event::ToolCall(tc) => {
                    if tc.completed {
                        let truncated = if tc.result.len() > 200 {
                            format!("{}...", &tc.result[..200])
                        } else {
                            tc.result.clone()
                        };
                        messages.push(ChatMessage {
                            role: Role::ToolResult,
                            text: format!("{} -> {}", tc.tool_name, truncated),
                        });
                    } else {
                        messages.push(ChatMessage {
                            role: Role::ToolCall,
                            text: format!("[r{}] {}({})", tc.round, tc.tool_name, tc.arguments),
                        });
                    }
                }
                pb::agent_chat_event::Event::TextChunk(t) => {
                    if let Some(last) = messages.last_mut() {
                        if matches!(last.role, Role::Agent) {
                            last.text.push_str(&t);
                        } else {
                            messages.push(ChatMessage {
                                role: Role::Agent,
                                text: t,
                            });
                        }
                    } else {
                        messages.push(ChatMessage {
                            role: Role::Agent,
                            text: t,
                        });
                    }
                }
                pb::agent_chat_event::Event::FinalText(_t) => {
                    // FinalText is a termination marker; the text was already streamed
                    // via TextChunk events. Only add if no Agent message was built yet
                    // (fallback for non-streaming path).
                    let has_agent = messages
                        .last()
                        .map_or(false, |m| matches!(m.role, Role::Agent));
                    if !has_agent && !_t.is_empty() {
                        messages.push(ChatMessage {
                            role: Role::Agent,
                            text: _t,
                        });
                    }
                }
            }
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
