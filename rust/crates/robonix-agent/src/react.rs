use anyhow::Result;
use rmcp::ServiceExt;
use robonix_sdk::RobonixClient;
use serde_json::Value;
use std::collections::HashMap;
use std::io::Write;

use crate::skills::{self, AgentSkill};
use crate::tools;
use crate::vlm::{Message, ToolDef, VlmClient};

type McpClient = rmcp::service::RunningService<rmcp::RoleClient, ()>;

/// Default `QueryNodes.namespace` prefix for MCP tool providers (HAL `robonix/prm/...`).
/// Override with `ROBONIX_MCP_NAMESPACE_PREFIX` (empty string = no prefix filter).
fn mcp_namespace_prefix() -> String {
    match std::env::var("ROBONIX_MCP_NAMESPACE_PREFIX") {
        Ok(s) => s,
        Err(_) => "robonix/prm".to_string(),
    }
}

const MAX_HISTORY_MESSAGES: usize = 200;
const MAX_REPEATED_CALLS: usize = 3;
const MAX_CONSECUTIVE_EXPLORE: usize = 4;
const EXPLORE_TOOLS: &[&str] = &["list_dir", "read_file"];

/// Max tool-call rounds per single user input (prevents runaway loops).
/// Override with `ROBONIX_AGENT_MAX_TOOL_ROUNDS`.
const DEFAULT_MAX_TOOL_ROUNDS: usize = 64;

/// After any tool round, if the model replies with **text only** (no `tool_calls`), nudge it
/// to keep calling tools until it gives an **explicit** completion phrase (bounded per user line).
/// `ROBONIX_AGENT_TOOL_PERSIST_NUDGES` (default 5), `0` disables.
const DEFAULT_TOOL_PERSIST_NUDGES: u8 = 5;

fn max_tool_persist_nudges() -> u8 {
    match std::env::var("ROBONIX_AGENT_TOOL_PERSIST_NUDGES") {
        Ok(s) => s.parse().unwrap_or(DEFAULT_TOOL_PERSIST_NUDGES),
        Err(_) => DEFAULT_TOOL_PERSIST_NUDGES,
    }
}

fn max_tool_rounds() -> usize {
    match std::env::var("ROBONIX_AGENT_MAX_TOOL_ROUNDS") {
        Ok(s) => s.parse().unwrap_or(DEFAULT_MAX_TOOL_ROUNDS),
        Err(_) => DEFAULT_MAX_TOOL_ROUNDS,
    }
}

fn fold_smart_quotes(s: &str) -> String {
    s.chars()
        .map(|c| match c {
            '\u{2019}' | '\u{2018}' | '\u{02BC}' => '\'',
            '\u{201C}' | '\u{201D}' => '"',
            _ => c,
        })
        .collect()
}

/// Only skip the "keep calling tools" nudge when the model clearly says the robot task is done.
fn is_explicit_robot_task_done(content: &str) -> bool {
    let t = fold_smart_quotes(content.trim());
    if t.is_empty() {
        return false;
    }
    let l = t.to_lowercase();
    l.starts_with("task complete:")
        || l.starts_with("goal reached:")
        || l.contains("task is complete")
        || l.contains("task complete")
        || l.contains("goal reached")
        || l.contains("mission accomplished")
        || l.contains("objective achieved")
        || l.contains("successfully completed the")
        || l.contains("have finished")
        || l.contains("procedure complete")
        || t.contains("任务已完成")
        || t.contains("任务全部完成")
        || t.contains("目标已达成")
        || (l.starts_with("done.") && t.len() < 120)
        || (l.starts_with("done —") && t.len() < 200)
        || (l.starts_with("complete.") && t.len() < 120)
}

pub async fn run_react_loop(sdk: &mut RobonixClient, mut vlm: VlmClient) -> Result<()> {
    let mut history: Vec<Message> = Vec::new();

    loop {
        eprint!("You: ");
        std::io::stderr().flush()?;
        let input = read_line()?;
        if input.is_empty() || input == "quit" {
            // Trigger memory compaction at session end if available
            let mcp_tools_for_quit = load_mcp_tools(sdk).await.unwrap_or_default();
            if mcp_tools_for_quit.contains_key("compact_memory") {
                log::info!("[mcp] Compacting memory session before exit...");
                let _ = execute_mcp_tool(sdk, &mcp_tools_for_quit, "compact_memory", "{}").await;
            }
            break;
        }

        // Re-discover MCP tools and skills every turn: HAL nodes (e.g. tiago_bridge in Docker)
        // often register long after the agent process starts (Webots/Nav2 warmup).
        let mcp_tools = load_mcp_tools(sdk).await?;
        let merged_skills = match skills::load_merged_skills(sdk).await {
            Ok(s) => s,
            Err(e) => {
                log::warn!("failed to load merged skills: {e:#}");
                Vec::new()
            }
        };
        let mut memory_context = String::new();
        // Do not trigger silent memory recall for simple casual greetings or meta questions
        let is_casual = input.trim().to_lowercase();
        let skip_memory = is_casual == "hi" || is_casual == "hello" || is_casual.starts_with("who are you") || is_casual == "你是谁" || is_casual == "你好";
        
        if !skip_memory && mcp_tools.contains_key("search_memory") {
            let args = serde_json::json!({
                "query": input
            });
            let args_str = args.to_string();
            if let Ok(result) = execute_mcp_tool(sdk, &mcp_tools, "search_memory", &args_str).await {
                if !result.contains("No relevant memories found") {
                    memory_context = result;
                }
            }
        }

        let mcp_tool_defs = build_mcp_tool_defs(&mcp_tools);
        let mut all_tools = tools::builtin_tool_defs();
        all_tools.extend(mcp_tool_defs);
        let soul = skills::load_agent_soul();
        let system_prompt = build_system_prompt(soul.as_deref(), &mcp_tools, &merged_skills, &memory_context);
        log::info!(
            "tools for this turn: {} total ({} builtin + {} mcp), skills={}",
            all_tools.len(),
            tools::BUILTIN_NAMES.len(),
            mcp_tools.len(),
            merged_skills.len()
        );

        history.push(Message::user(&input));
        trim_history(&mut history);

        let mut round = 0;
        let mut last_call_sig = String::new();
        let mut repeat_count: usize = 0;
        let mut explore_streak: usize = 0;
        let mut tool_persist_nudges: u8 = 0;
        let max_persist = max_tool_persist_nudges();
        let max_rounds = max_tool_rounds();
        loop {
            let mut messages = vec![Message::system(&system_prompt)];
            messages.extend(history.clone());

            let (content, tool_calls) = vlm.chat(&messages, &all_tools).await?;

            if tool_calls.is_empty() {
                let text = content.unwrap_or_default();
                let trimmed = text.trim();

                // Physical robot tasks should not stop mid-task. If the model returns no `tool_calls`
                // after at least one tool round (including empty text), nudge it to continue chaining
                // tools until an explicit completion phrase.
                let need_tool_calls = trimmed.is_empty() || !is_explicit_robot_task_done(trimmed);
                if max_persist > 0
                    && round > 0
                    && tool_persist_nudges < max_persist
                    && need_tool_calls
                {
                    tool_persist_nudges += 1;
                    log::info!(
                        "[react] no tool_calls after tools (round {}, empty_content={}); nudging ({}/{})",
                        round,
                        trimmed.is_empty(),
                        tool_persist_nudges,
                        max_persist
                    );
                    if !trimmed.is_empty() {
                        history.push(Message::assistant(&text));
                    }
                    history.push(Message::user(
                        "Continue with **tool_calls only** next. Chain MCP tools in tight sense–act loops \
                         until the user goal is met or you must report a blocking error. Do not stop after \
                         one motion or one image. Prefer batched tool_calls (e.g. get_robot_pose + \
                         get_camera_image together) when supported. When truly finished, start with \
                         `Task complete:` or `Goal reached:`. If your assistant text was empty, still emit tool_calls.",
                    ));
                    trim_history(&mut history);
                    continue;
                }
                let out = if trimmed.is_empty() {
                    if round > 0 {
                        log::warn!("VLM still returned no text after nudge (or no nudge applied)");
                    }
                    "(The model returned no text. Say \"continue\" or rephrase your request.)"
                        .to_string()
                } else {
                    text
                };
                println!("Agent: {out}");
                history.push(Message::assistant(&out));
                break;
            }

            round += 1;
            if round > max_rounds {
                let msg = format!(
                    "(max tool-call rounds reached: {max_rounds}; set ROBONIX_AGENT_MAX_TOOL_ROUNDS to raise, or say continue.)"
                );
                eprintln!("[agent] {msg}");
                history.push(Message::assistant(&msg));
                break;
            }

            // --- Guard 1: identical tool calls repeated consecutively ---
            let call_sig = tool_calls
                .iter()
                .map(|tc| format!("{}:{}", tc.function.name, tc.function.arguments))
                .collect::<Vec<_>>()
                .join("|");
            if call_sig == last_call_sig {
                repeat_count += 1;
                if repeat_count >= MAX_REPEATED_CALLS {
                    let all_move_base = tool_calls
                        .iter()
                        .all(|tc| tc.function.name.as_str() == "move_base");
                    if all_move_base {
                        // Common failure mode for physical skills: the model keeps issuing identical
                        // motions without sensing. Don't stop; force a sense step instead.
                        let msg = "(repeated identical move_base calls detected; insert get_robot_pose + get_camera_image before any further motion)".to_string();
                        eprintln!("[agent] {msg}");
                        history.push(Message::user(&msg));
                        trim_history(&mut history);
                        last_call_sig.clear();
                        repeat_count = 0;
                        continue;
                    } else {
                        let msg = format!(
                            "(same tool call repeated {} times, stopping to avoid infinite loop)",
                            repeat_count + 1
                        );
                        eprintln!("[agent] {msg}");
                        history.push(Message::assistant(&msg));
                        break;
                    }
                }
            } else {
                last_call_sig = call_sig;
                repeat_count = 0;
            }

            // --- Guard 2: consecutive exploration tools (list_dir/read_file with varying args) ---
            let all_explore = tool_calls
                .iter()
                .all(|tc| EXPLORE_TOOLS.contains(&tc.function.name.as_str()));
            if all_explore {
                explore_streak += 1;
                if explore_streak >= MAX_CONSECUTIVE_EXPLORE {
                    let msg = format!(
                        "(filesystem exploration repeated {} rounds without progress, stopping — \
                        please tell the user what you have found so far)",
                        explore_streak
                    );
                    eprintln!("[agent] {msg}");
                    history.push(Message::tool_result(&tool_calls[0].id, &msg));
                    continue;
                }
            } else {
                explore_streak = 0;
            }

            history.push(Message::assistant_tool_calls(tool_calls.clone()));
            for tc in &tool_calls {
                log::info!(
                    "[round {round}] tool: {}({})",
                    tc.function.name,
                    tc.function.arguments
                );
                let result =
                    dispatch_tool(sdk, &mcp_tools, &tc.function.name, &tc.function.arguments)
                        .await?;
                log::info!("[round {round}] result: {}", truncate_log(&result, 200));
                history.push(extract_image_message(&tc.id, &result));
            }
        }
    }
    Ok(())
}

fn trim_history(history: &mut Vec<Message>) {
    if history.len() > MAX_HISTORY_MESSAGES {
        let excess = history.len() - MAX_HISTORY_MESSAGES;
        history.drain(..excess);
    }
}

fn truncate_log(s: &str, max: usize) -> String {
    if s.len() <= max {
        s.to_string()
    } else {
        // Find the nearest valid character boundary before `max`
        let mut max_idx = max;
        while max_idx > 0 && !s.is_char_boundary(max_idx) {
            max_idx -= 1;
        }
        format!("{}...", &s[..max_idx])
    }
}

/// If the tool result JSON contains `image_base64`, extract it into a dedicated
/// field so the VLM receives it as a proper image (not opaque text).
fn extract_image_message(tool_call_id: &str, result: &str) -> Message {
    if let Ok(val) = serde_json::from_str::<Value>(result) {
        if let Some(img) = val.get("image_base64").and_then(|v| v.as_str()) {
            let text_summary = val
                .as_object()
                .map(|obj| {
                    let mut summary = serde_json::Map::new();
                    for (k, v) in obj {
                        if k != "image_base64" {
                            summary.insert(k.clone(), v.clone());
                        }
                    }
                    summary.insert("image_base64".into(), Value::String("(attached)".into()));
                    Value::Object(summary).to_string()
                })
                .unwrap_or_else(|| "(image attached)".into());
            log::info!(
                "[image] extracted image_base64 ({} bytes) from tool result",
                img.len()
            );
            return Message::tool_result_with_image(tool_call_id, &text_summary, img.to_string());
        }
    }
    Message::tool_result(tool_call_id, result)
}

async fn dispatch_tool(
    sdk: &mut RobonixClient,
    mcp_tools: &HashMap<String, McpToolSpec>,
    name: &str,
    args: &str,
) -> Result<String> {
    if tools::BUILTIN_NAMES.contains(&name) {
        return tools::execute_builtin(name, args).await;
    }
    execute_mcp_tool(sdk, mcp_tools, name, args).await
}

fn build_system_prompt(
    soul: Option<&str>,
    mcp_tools: &HashMap<String, McpToolSpec>,
    skills: &[AgentSkill],
    memory_context: &str,
) -> String {
    let mut p = String::new();
    if let Some(s) = soul {
        let t = s.trim();
        if !t.is_empty() {
            p.push_str("## Agent SOUL\n\n");
            p.push_str(t);
            p.push_str("\n\n---\n\n");
        }
    }
    
    if !memory_context.is_empty() {
        p.push_str("## Relevant past memories (System Context)\n\n");
        p.push_str(memory_context);
        p.push_str("\n\n---\n\n");
    }

    p.push_str(
        "\
You are the Robonix system agent running on a robotic platform.
You have full conversation memory — refer back to earlier messages when the user follows up.

When the user asks you to do something, ACT immediately using your tools. Do not ask
the user to run commands themselves. Call tools, read the results, and continue until
the task is fully done. Only reply to the user when you have the final answer.

Rules:
- If a request requires multiple steps, execute them all before responding.
- When the user references something from earlier in the conversation, use that context.
- Keep responses concise and direct.
- If you are unsure about historical context, user preferences, or past decisions, use the `search_memory` tool.
- When you learn an important user preference or complete a major milestone, use `save_memory` to record it.
- When a tool returns a JSON result with a clear outcome (success/failure, error, etc.),
  report that result to the user immediately.
- Use **list_dir** / **read_file** for skill playbooks (see Skill workflow) or when the user
  asks for debugging. Do not browse the repo aimlessly unrelated to the task.

## Skill workflow (when the Skills index below is non-empty)

Robotics and stack-specific behaviour **must** follow the registered skills — not guesses.

1. **Select**: From the Skills index, pick the skill(s) whose `name` and `description` fit the
   user's goal **and** explicit constraints (e.g. no navigation / no map goals → prefer
   `object_search_wander`, not `navigation`).
2. **Read**: Before calling HAL/MCP tools for that goal, call **read_file** on the chosen
   skill's `path` (absolute path to `SKILL.md`). If several skills could apply, read those
   files and then decide.
   Skip this only if this message already contains the full inlined **Skill playbooks** for
   the same skill (see `ROBONIX_SKILLS_INJECT_BODIES`).
3. **Execute**: Follow the SKILL.md steps — which tools to use, which are forbidden, loops,
   safety limits — then invoke **builtin and MCP tools** accordingly.

**Skills** in the index are playbooks; the XML `path` is the file you must read.

## Builtin tools
- read_file: read a file's contents (path)
- write_file: create or overwrite a file (path, content)
- patch_file: replace a substring in a file (path, old, new)
- list_dir: list directory entries (path, default: cwd)
- run_command: run a shell command and return stdout/stderr (command)
",
    );
    if !mcp_tools.is_empty() {
        p.push_str("\n## MCP tools (from node interfaces, transport=mcp)\n\n");
        for (tool_name, spec) in mcp_tools {
            p.push_str(&format!(
                "- {} (node={}, interface={}, endpoint={}): {}\n",
                tool_name, spec.node_id, spec.interface_name, spec.endpoint, spec.description
            ));
        }
    } else {
        p.push_str(
            "\n## MCP tools\n\
             None discovered yet (robot HAL may still be starting). Do not invent ROS 1 / roslaunch \
             examples; only use the builtin tools above until MCP tools appear after the next user message.\n",
        );
    }

    let visible = skills::model_visible_skills(skills);
    if !visible.is_empty() {
        p.push_str(
            "\n## Skills index (registry + ~/.robonix/skills + ROBONIX_SKILLS_EXTRA_DIRS)\n\n",
        );
        p.push_str(&skills::format_skills_xml(&visible));
        p.push_str("\n\n");
        let inject = std::env::var("ROBONIX_SKILLS_INJECT_BODIES")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);
        if inject {
            p.push_str(&skills::format_skill_playbooks(&visible));
            p.push('\n');
        } else {
            p.push_str(
                "Full Playbooks are **not** inlined here — you **must** use **read_file** on the\n\
                 `path` from the index above before using robot/MCP tools for that task.\n\
                 (To embed excerpts in-prompt instead, run with `ROBONIX_SKILLS_INJECT_BODIES=true`;\n\
                 optional: `ROBONIX_SKILLS_INJECT_MAX_CHARS`, `ROBONIX_SKILLS_INJECT_PER_SKILL_CHARS`.)\n",
            );
        }
    }

    let hidden: Vec<&AgentSkill> = skills
        .iter()
        .filter(|s| skills::skill_disable_model_invocation(&s.metadata_json))
        .collect();
    if !hidden.is_empty() {
        p.push_str("\n## Skills not injected into model context\n\n");
        p.push_str("These are registered with `disable-model-invocation: true`. ");
        p.push_str("They are listed for bookkeeping only unless the user explicitly asks you to read them.\n\n");
        for s in hidden {
            p.push_str(&format!(
                "- **{}** (node={}): {} — `{}`\n",
                s.name, s.node_id, s.description, s.path
            ));
        }
    }

    p
}

#[derive(Clone, Debug)]
struct McpToolSpec {
    tool_name: String,
    description: String,
    node_id: String,
    interface_name: String,
    endpoint: String,
    input_schema: Value,
}

fn build_mcp_tool_defs(mcp_tools: &HashMap<String, McpToolSpec>) -> Vec<ToolDef> {
    mcp_tools
        .values()
        .map(|t| ToolDef::new(&t.tool_name, &t.description, t.input_schema.clone()))
        .collect()
}

fn read_line() -> Result<String> {
    let mut buf = String::new();
    std::io::stdin().read_line(&mut buf)?;
    Ok(buf.trim().to_string())
}

// ── MCP tool execution via rmcp client ──────────────────────────────────────

async fn execute_mcp_tool(
    _sdk: &mut RobonixClient,
    mcp_tools: &HashMap<String, McpToolSpec>,
    name: &str,
    args: &str,
) -> Result<String> {
    let spec = mcp_tools
        .get(name)
        .ok_or_else(|| anyhow::anyhow!("tool '{name}' not found in builtin or mcp registry"))?;

    let endpoint = &spec.endpoint;
    let args_value: Value = serde_json::from_str(args).unwrap_or_default();
    let args_obj = args_value.as_object().cloned();

    log::info!("[mcp] calling tool '{}' via endpoint '{}'", name, endpoint);

    let mut client: McpClient = connect_mcp(endpoint).await?;

    let result = client
        .call_tool(
            rmcp::model::CallToolRequestParams::new(name.to_string()).with_arguments(
                args_obj
                    .map(rmcp::model::JsonObject::from_iter)
                    .unwrap_or_default(),
            ),
        )
        .await
        .map_err(|e| anyhow::anyhow!("MCP call_tool failed: {e}"))?;

    let text: String = result
        .content
        .iter()
        .filter_map(|c| match c.raw {
            rmcp::model::RawContent::Text(ref t) => Some(t.text.as_str()),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join("\n");

    let is_error = result.is_error.unwrap_or(false);
    client.close().await.ok();

    if is_error {
        Ok(format!(
            r#"{{"status":"error","tool":"{name}","output":{}}}"#,
            serde_json::to_string(&text)?
        ))
    } else {
        Ok(text)
    }
}

async fn connect_mcp(endpoint: &str) -> Result<McpClient> {
    if endpoint.starts_with("stdio://") {
        let cmd_str = &endpoint["stdio://".len()..];
        let mut parts = cmd_str.split_whitespace();
        let program = parts.next().unwrap_or("python3");
        let args: Vec<&str> = parts.collect();
        let mut command = tokio::process::Command::new(program);
        command.args(&args);
        let transport = rmcp::transport::TokioChildProcess::new(command)
            .map_err(|e| anyhow::anyhow!("MCP stdio spawn '{cmd_str}' failed: {e}"))?;
        let client = ()
            .serve(transport)
            .await
            .map_err(|e| anyhow::anyhow!("MCP stdio init '{cmd_str}' failed: {e}"))?;
        return Ok(client);
    }
    if endpoint.is_empty() {
        anyhow::bail!("empty MCP endpoint");
    }
    let base = if endpoint.starts_with("http://") || endpoint.starts_with("https://") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    };
    // FastMCP (Python SDK) serves Streamable HTTP at the `/mcp` path by default.
    let uri = if base.contains("/mcp") {
        base
    } else {
        format!("{}/mcp", base.trim_end_matches('/'))
    };
    let transport =
        rmcp::transport::streamable_http_client::StreamableHttpClientTransport::from_uri(
            uri.clone(),
        );
    let client = ()
        .serve(transport)
        .await
        .map_err(|e| anyhow::anyhow!("MCP HTTP connect to '{uri}' failed: {e}"))?;
    Ok(client)
}

// ── MCP tool discovery ──────────────────────────────────────────────────────

async fn load_mcp_tools(sdk: &mut RobonixClient) -> Result<HashMap<String, McpToolSpec>> {
    let ns = mcp_namespace_prefix();
    let nodes = sdk.query_nodes(&ns, "", "mcp").await?;
    let mut out = HashMap::new();
    for n in nodes {
        for iface in &n.interfaces {
            if !iface.supported_transports.contains(&"mcp".to_string()) {
                continue;
            }
            let meta: Value = match serde_json::from_str(&iface.metadata_json) {
                Ok(v) => v,
                Err(_) => continue,
            };
            let endpoint = meta
                .get("endpoint")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string();
            if endpoint.is_empty() {
                log::warn!(
                    "[mcp] node '{}' interface '{}' has mcp transport but no endpoint in metadata",
                    n.node_id,
                    iface.name
                );
                continue;
            }
            let tools = meta
                .get("tools")
                .and_then(|v| v.as_array())
                .cloned()
                .unwrap_or_default();
            for t in &tools {
                let tool_name = t
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("unknown")
                    .to_string();
                let description = t
                    .get("description")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string();
                let input_schema = t
                    .get("input_schema")
                    .cloned()
                    .unwrap_or_else(|| serde_json::json!({"type": "object", "properties": {}}));
                out.insert(
                    tool_name.clone(),
                    McpToolSpec {
                        tool_name,
                        description,
                        node_id: n.node_id.clone(),
                        interface_name: iface.name.clone(),
                        endpoint: endpoint.clone(),
                        input_schema,
                    },
                );
            }
        }
    }
    Ok(out)
}
