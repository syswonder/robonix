use anyhow::Result;
use robonix_sdk::RobonixClient;
use rmcp::ServiceExt;
use serde_json::Value;
use std::io::Write;
use std::collections::HashMap;

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

const MAX_TOOL_ROUNDS: usize = 20;
const MAX_HISTORY_MESSAGES: usize = 200;
const MAX_REPEATED_CALLS: usize = 3;
const MAX_CONSECUTIVE_EXPLORE: usize = 4;
const EXPLORE_TOOLS: &[&str] = &["list_dir", "read_file"];

pub async fn run_react_loop(sdk: &mut RobonixClient, mut vlm: VlmClient) -> Result<()> {
    let mut all_tools = tools::builtin_tool_defs();
    let mcp_tools = load_mcp_tools(sdk).await?;
    let mcp_tool_defs = build_mcp_tool_defs(&mcp_tools);
    all_tools.extend(mcp_tool_defs);
    let system_prompt = build_system_prompt(&mcp_tools);
    log::info!(
        "loaded {} tool(s) ({} builtin + {} mcp)",
        all_tools.len(),
        tools::BUILTIN_NAMES.len(),
        mcp_tools.len()
    );

    let mut history: Vec<Message> = Vec::new();

    loop {
        eprint!("You: ");
        std::io::stderr().flush()?;
        let input = read_line()?;
        if input.is_empty() || input == "quit" {
            break;
        }

        history.push(Message::user(&input));
        trim_history(&mut history);

        let mut round = 0;
        let mut last_call_sig = String::new();
        let mut repeat_count: usize = 0;
        let mut explore_streak: usize = 0;
        loop {
            let mut messages = vec![Message::system(&system_prompt)];
            messages.extend(history.clone());

            let (content, tool_calls) = vlm.chat(&messages, &all_tools).await?;

            if tool_calls.is_empty() {
                let text = content.unwrap_or_default();
                println!("Agent: {text}");
                history.push(Message::assistant(&text));
                break;
            }

            round += 1;
            if round > MAX_TOOL_ROUNDS {
                let msg = "(max tool-call rounds reached, stopping)";
                eprintln!("[agent] {msg}");
                history.push(Message::assistant(msg));
                break;
            }

            // --- Guard 1: identical tool calls repeated consecutively ---
            let call_sig = tool_calls.iter()
                .map(|tc| format!("{}:{}", tc.function.name, tc.function.arguments))
                .collect::<Vec<_>>()
                .join("|");
            if call_sig == last_call_sig {
                repeat_count += 1;
                if repeat_count >= MAX_REPEATED_CALLS {
                    let msg = format!(
                        "(same tool call repeated {} times, stopping to avoid infinite loop)",
                        repeat_count + 1
                    );
                    eprintln!("[agent] {msg}");
                    history.push(Message::assistant(&msg));
                    break;
                }
            } else {
                last_call_sig = call_sig;
                repeat_count = 0;
            }

            // --- Guard 2: consecutive exploration tools (list_dir/read_file with varying args) ---
            let all_explore = tool_calls.iter().all(|tc| EXPLORE_TOOLS.contains(&tc.function.name.as_str()));
            if all_explore {
                explore_streak += 1;
                if explore_streak >= MAX_CONSECUTIVE_EXPLORE {
                    let msg = format!(
                        "(filesystem exploration repeated {} rounds without progress, stopping — \
                        please tell the user what you have found so far)",
                        explore_streak
                    );
                    eprintln!("[agent] {msg}");
                    history.push(Message::tool_result(
                        &tool_calls[0].id,
                        &msg,
                    ));
                    continue;
                }
            } else {
                explore_streak = 0;
            }

            history.push(Message::assistant_tool_calls(tool_calls.clone()));
            for tc in &tool_calls {
                log::info!("[round {round}] tool: {}({})", tc.function.name, tc.function.arguments);
                let result = dispatch_tool(sdk, &mcp_tools, &tc.function.name, &tc.function.arguments).await?;
                log::info!("[round {round}] result: {}", truncate_log(&result, 200));
                history.push(Message::tool_result(&tc.id, &result));
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
    if s.len() <= max { s.to_string() } else { format!("{}...", &s[..max]) }
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

fn build_system_prompt(mcp_tools: &HashMap<String, McpToolSpec>) -> String {
    let mut p = String::from("\
You are the Robonix system agent running on a robotic platform.
You have full conversation memory — refer back to earlier messages when the user follows up.

When the user asks you to do something, ACT immediately using your tools. Do not ask
the user to run commands themselves. Call tools, read the results, and continue until
the task is fully done. Only reply to the user when you have the final answer.

Rules:
- If a request requires multiple steps, execute them all before responding.
- When the user references something from earlier in the conversation, use that context.
- Keep responses concise and direct.
- When a tool returns a JSON result with a clear outcome (success/failure, error, etc.),
  report that result to the user immediately. Do NOT explore the filesystem to find
  additional information unless the user explicitly asks for debugging.
- Use list_dir/read_file only when you genuinely need specific file contents.
  Do NOT list directories aimlessly after a tool failure.

## Builtin tools
- read_file: read a file's contents (path)
- write_file: create or overwrite a file (path, content)
- patch_file: replace a substring in a file (path, old, new)
- list_dir: list directory entries (path, default: cwd)
- run_command: run a shell command and return stdout/stderr (command)
");
    if !mcp_tools.is_empty() {
        p.push_str("\n## MCP tools (from node interfaces, transport=mcp)\n\n");
        for (tool_name, spec) in mcp_tools {
            p.push_str(&format!(
                "- {} (node={}, interface={}, endpoint={}): {}\n",
                tool_name, spec.node_id, spec.interface_name, spec.endpoint, spec.description
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
    let spec = mcp_tools.get(name)
        .ok_or_else(|| anyhow::anyhow!("tool '{name}' not found in builtin or mcp registry"))?;

    let endpoint = &spec.endpoint;
    let args_value: Value = serde_json::from_str(args).unwrap_or_default();
    let args_obj = args_value.as_object().cloned();

    log::info!("[mcp] calling tool '{}' via endpoint '{}'", name, endpoint);

    let mut client: McpClient = connect_mcp(endpoint).await?;

    let result = client.call_tool(
        rmcp::model::CallToolRequestParams::new(name.to_string())
            .with_arguments(
                args_obj
                    .map(rmcp::model::JsonObject::from_iter)
                    .unwrap_or_default(),
            ),
    ).await
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
        Ok(format!(r#"{{"status":"error","tool":"{name}","output":{}}}"#, serde_json::to_string(&text)?))
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
        let client = ().serve(transport).await
            .map_err(|e| anyhow::anyhow!("MCP stdio init '{cmd_str}' failed: {e}"))?;
        return Ok(client);
    }
    if endpoint.is_empty() {
        anyhow::bail!("empty MCP endpoint");
    }
    let uri = if endpoint.starts_with("http://") || endpoint.starts_with("https://") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    };
    let transport =
        rmcp::transport::streamable_http_client::StreamableHttpClientTransport::from_uri(uri.clone());
    let client = ().serve(transport).await
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
                    n.node_id, iface.name
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
