use anyhow::{Context, Result};
use robonix_sdk::{NodeInfo, QueryNodesOpts, RobonixClient, SkillEntry};
use std::collections::HashMap;

const BUILTIN_TOOLS: &[(&str, &str)] = &[
    ("read_file", "Read a file and return its contents"),
    (
        "write_file",
        "Write content to a file (creates or overwrites)",
    ),
    (
        "patch_file",
        "Replace a substring in a file (first occurrence)",
    ),
    ("list_dir", "List files and directories at a path"),
    (
        "run_command",
        "Run a shell command and return stdout/stderr",
    ),
];

async fn connect(endpoint: &str) -> Result<RobonixClient> {
    let url = if endpoint.contains("://") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    };
    RobonixClient::connect(&url)
        .await
        .with_context(|| format!("cannot reach robonix-atlas at {endpoint}"))
}

fn fmt_interfaces(n: &NodeInfo) -> String {
    if n.interfaces.is_empty() {
        return "(no interfaces)".into();
    }
    n.interfaces
        .iter()
        .map(|i| {
            let ts = i.supported_transports.join(",");
            if i.metadata_json.is_empty() {
                format!("{}[{}]", i.name, ts)
            } else {
                format!("{}[{}] meta={}", i.name, ts, i.metadata_json)
            }
        })
        .collect::<Vec<_>>()
        .join("  ")
}

#[derive(Clone)]
struct McpToolEntry {
    node_id: String,
    endpoint: String,
    interface_name: String,
    tool_name: String,
    description: String,
}

/// MCP tools from node metadata. Deduplicates by `(node_id, tool_name)` — metadata can list the same
/// tool twice with different description lengths; we keep the longer text.
fn mcp_tools_from_nodes(nodes: &[NodeInfo]) -> Vec<McpToolEntry> {
    let mut by_key: HashMap<(String, String), McpToolEntry> = HashMap::new();
    for n in nodes {
        for i in &n.interfaces {
            if !i.supported_transports.iter().any(|t| t == "mcp") {
                continue;
            }
            let meta: serde_json::Value = match serde_json::from_str(&i.metadata_json) {
                Ok(v) => v,
                Err(_) => continue,
            };
            let endpoint = meta
                .get("endpoint")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string();
            let Some(tools) = meta.get("tools").and_then(|v| v.as_array()) else {
                continue;
            };
            for t in tools {
                let tool_name = t
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("?")
                    .to_string();
                let description = t
                    .get("description")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string();
                let key = (n.node_id.clone(), tool_name.clone());
                let entry = McpToolEntry {
                    node_id: n.node_id.clone(),
                    endpoint: endpoint.clone(),
                    interface_name: i.name.clone(),
                    tool_name,
                    description,
                };
                by_key
                    .entry(key)
                    .and_modify(|e| {
                        if entry.description.len() > e.description.len() {
                            *e = entry.clone();
                        }
                    })
                    .or_insert(entry);
            }
        }
    }
    let mut v: Vec<McpToolEntry> = by_key.into_values().collect();
    v.sort_by(|a, b| {
        (&a.node_id, &a.endpoint, &a.tool_name).cmp(&(&b.node_id, &b.endpoint, &b.tool_name))
    });
    v
}

/// Word-wrap a single paragraph to at most `width` columns (for terminal readability).
fn wrap_fill(s: &str, width: usize) -> Vec<String> {
    let w = width.max(16);
    let mut out = Vec::new();
    for para in s.split('\n') {
        let para = para.trim_end();
        if para.is_empty() {
            continue;
        }
        let mut line = String::new();
        for word in para.split_whitespace() {
            if line.is_empty() {
                line.push_str(word);
            } else if line.len() + 1 + word.len() <= w {
                line.push(' ');
                line.push_str(word);
            } else {
                out.push(std::mem::take(&mut line));
                line.push_str(word);
            }
        }
        if !line.is_empty() {
            out.push(line);
        }
    }
    if out.is_empty() && !s.trim().is_empty() {
        out.push(s.trim().to_string());
    }
    out
}

fn print_tools_human(skills: &[SkillEntry], nodes: &[NodeInfo], mcp: &[McpToolEntry]) {
    println!("Tools");
    println!("=====\n");

    println!("Built-in");
    println!("--------");
    for (name, desc) in BUILTIN_TOOLS {
        println!("  {name}");
        for line in wrap_fill(desc, 76) {
            println!("    {line}");
        }
    }

    if !skills.is_empty() {
        println!("\nService (skill catalog)");
        println!("------------------------");
        for s in skills {
            let tool_name = s.node_id.replace(['/', '-'], "_");
            let first_line = s
                .skill_md
                .lines()
                .next()
                .unwrap_or(&s.namespace)
                .trim_start_matches('#')
                .trim();
            let transport_str = nodes
                .iter()
                .find(|n| n.node_id == s.node_id)
                .map(|n| {
                    n.interfaces
                        .iter()
                        .map(|i| format!("{}:{}", i.name, i.supported_transports.join(",")))
                        .collect::<Vec<_>>()
                        .join(" ")
                })
                .unwrap_or_else(|| "?".into());
            println!("  {tool_name}  ({})", s.kind);
            println!("    transports: {transport_str}");
            for line in wrap_fill(first_line, 76) {
                println!("    {line}");
            }
        }
    }

    if !mcp.is_empty() {
        println!("\nMCP");
        println!("---");
        let mut i = 0;
        while i < mcp.len() {
            let t0 = &mcp[i];
            println!("\n  node:       {}", t0.node_id);
            println!("  endpoint:   {}", t0.endpoint);
            println!("  interface:  {}", t0.interface_name);
            let mut j = i;
            while j < mcp.len() && mcp[j].node_id == t0.node_id && mcp[j].endpoint == t0.endpoint {
                println!("\n    {}", mcp[j].tool_name);
                for line in wrap_fill(&mcp[j].description, 72) {
                    println!("      {line}");
                }
                j += 1;
            }
            i = j;
        }
    }

    let mcp_n = mcp.len();
    let total = BUILTIN_TOOLS.len() + skills.len() + mcp_n;
    println!(
        "\n{} tool(s) total ({} built-in + {} skill catalog + {} mcp)",
        total,
        BUILTIN_TOOLS.len(),
        skills.len(),
        mcp_n
    );
}

fn iface_summary(metadata_json: &str, transport: &str) -> String {
    let meta: serde_json::Value = match serde_json::from_str(metadata_json) {
        Ok(v) => v,
        Err(_) => return String::new(),
    };
    let mut parts = Vec::new();

    if let Some(ep) = meta.get("endpoint").and_then(|v| v.as_str()) {
        parts.push(format!("endpoint={ep}"));
    }

    if transport == "mcp" {
        if let Some(tools) = meta.get("tools").and_then(|v| v.as_array()) {
            let names: Vec<&str> = tools
                .iter()
                .filter_map(|t| t.get("name").and_then(|n| n.as_str()))
                .collect();
            if names.len() <= 3 {
                parts.push(format!("tools=[{}]", names.join(", ")));
            } else {
                parts.push(format!("tools={} total", names.len()));
            }
        }
    } else if transport.contains("grpc") {
        if let Some(rpc) = meta
            .get("contract")
            .and_then(|c| c.get("rpc_method"))
            .and_then(|v| v.as_str())
        {
            let short = rpc.rsplit('/').next().unwrap_or(rpc);
            parts.push(format!("rpc={short}"));
        }
    }

    parts.join("  ")
}

pub async fn nodes(
    endpoint: &str,
    distro_prefix: Option<&str>,
    container_id: Option<&str>,
    json: bool,
) -> Result<()> {
    let mut sdk = connect(endpoint).await?;
    let nodes = sdk
        .query_nodes_opts(QueryNodesOpts {
            namespace: String::new(),
            interface_name: String::new(),
            transport: String::new(),
            distro_prefix: distro_prefix.unwrap_or("").to_string(),
            container_id: container_id.unwrap_or("").to_string(),
            contract_id: String::new(),
        })
        .await?;

    if json {
        let out: Vec<serde_json::Value> = nodes
            .iter()
            .map(|n| {
                let mut obj = serde_json::json!({
                    "node_id": n.node_id,
                    "namespace": n.namespace,
                    "kind": n.kind,
                    "has_skill_md": n.has_skill_md,
                    "interfaces": n.interfaces.iter().map(|i| serde_json::json!({
                        "name": i.name,
                        "transports": i.supported_transports,
                        "metadata": i.metadata_json,
                    })).collect::<Vec<_>>(),
                });
                if !n.distro.is_empty() {
                    obj["distro"] = serde_json::Value::String(n.distro.clone());
                }
                if !n.container_id.is_empty() {
                    obj["container_id"] = serde_json::Value::String(n.container_id.clone());
                }
                if n.last_heartbeat_ms > 0 {
                    obj["last_heartbeat_ms"] =
                        serde_json::Value::Number(n.last_heartbeat_ms.into());
                }
                obj
            })
            .collect();
        println!("{}", serde_json::to_string_pretty(&out)?);
        return Ok(());
    }

    if nodes.is_empty() {
        println!("No nodes registered.");
        return Ok(());
    }

    for n in &nodes {
        let distro_tag = if n.distro.is_empty() {
            String::new()
        } else {
            format!(" [{}]", n.distro)
        };
        let container_tag = if n.container_id.is_empty() {
            String::new()
        } else {
            format!(" @{}", n.container_id)
        };
        let hb = if n.last_heartbeat_ms > 0 {
            format!(" hb_ms={}", n.last_heartbeat_ms)
        } else {
            String::new()
        };
        println!(
            "[{}] {} ({}){}{}{}",
            n.kind, n.node_id, n.namespace, distro_tag, container_tag, hb
        );
        if n.interfaces.is_empty() {
            println!("  (no interfaces)");
        }
        for i in &n.interfaces {
            let transport = i.supported_transports.join(",");
            let summary = iface_summary(&i.metadata_json, &transport);
            println!("  {:<18} {:<6} {}", i.name, transport, summary);
        }
    }
    println!("\n{} node(s) total", nodes.len());
    Ok(())
}

pub async fn describe(endpoint: &str, node_id: Option<&str>, json: bool) -> Result<()> {
    let mut sdk = connect(endpoint).await?;

    if let Some(id) = node_id {
        let nodes = sdk.query_nodes("", "", "").await?;
        let node = nodes.iter().find(|n| n.node_id == id);
        if let Some(n) = node {
            println!("[{}] {} ({})", n.kind, n.node_id, n.namespace);
            for i in &n.interfaces {
                println!("  {} <- {}", i.name, i.supported_transports.join(", "));
            }
            println!();
        }
        let md = sdk.query_skill_md(id).await?;
        if md.is_empty() {
            println!("Node '{}' has no SKILL.md.", id);
        } else {
            println!("{}", md);
        }
        return Ok(());
    }

    let nodes = sdk.query_nodes("", "", "").await?;
    let skills = sdk.query_all_skills().await?;

    if json {
        let out: Vec<serde_json::Value> = skills
            .iter()
            .map(|s| {
                let ifaces: Vec<serde_json::Value> = nodes
                    .iter()
                    .find(|n| n.node_id == s.node_id)
                    .map(|n| {
                        n.interfaces
                            .iter()
                            .map(|i| {
                                serde_json::json!({
                                    "name": i.name,
                                    "transports": i.supported_transports,
                                })
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                serde_json::json!({
                    "node_id": s.node_id,
                    "namespace": s.namespace,
                    "kind": s.kind,
                    "interfaces": ifaces,
                    "skill_md": s.skill_md,
                })
            })
            .collect();
        println!("{}", serde_json::to_string_pretty(&out)?);
        return Ok(());
    }

    if skills.is_empty() {
        println!("No skills registered.");
        return Ok(());
    }

    for s in &skills {
        let node = nodes.iter().find(|n| n.node_id == s.node_id);
        let iface_str = node.map(|n| fmt_interfaces(n)).unwrap_or_default();
        println!("== {} ({}) [{}] ==", s.node_id, s.namespace, s.kind);
        println!("  interfaces: {}", iface_str);
        let preview: String = s.skill_md.lines().take(5).collect::<Vec<_>>().join("\n");
        println!("{}", preview);
        if s.skill_md.lines().count() > 5 {
            println!(
                "  ... ({} lines total, use `rbnx describe --node {}` for full text)",
                s.skill_md.lines().count(),
                s.node_id
            );
        }
        println!();
    }
    println!("{} skill(s) total", skills.len());
    Ok(())
}

pub async fn tools(endpoint: &str, json: bool) -> Result<()> {
    let mut sdk = connect(endpoint).await?;
    let nodes = sdk.query_nodes("", "", "").await?;
    let skills = sdk.query_all_skills().await?;
    let mcp_entries = mcp_tools_from_nodes(&nodes);

    if json {
        let mut all = Vec::new();
        for (name, desc) in BUILTIN_TOOLS {
            all.push(serde_json::json!({
                "source": "builtin",
                "name": name,
                "description": desc,
                "transports": [],
            }));
        }
        for s in &skills {
            let ifaces: Vec<serde_json::Value> = nodes
                .iter()
                .find(|n| n.node_id == s.node_id)
                .map(|n| {
                    n.interfaces
                        .iter()
                        .map(|i| {
                            serde_json::json!({
                                "name": i.name,
                                "transports": i.supported_transports,
                            })
                        })
                        .collect()
                })
                .unwrap_or_default();
            all.push(serde_json::json!({
                "source": format!("node:{}", s.node_id),
                "name": s.node_id.replace(['/', '-'], "_"),
                "namespace": s.namespace,
                "kind": s.kind,
                "interfaces": ifaces,
                "description": s.skill_md.lines().next().unwrap_or(""),
            }));
        }
        for t in &mcp_entries {
            all.push(serde_json::json!({
                "source": "mcp",
                "name": t.tool_name,
                "node_id": t.node_id,
                "interface": t.interface_name,
                "endpoint": t.endpoint,
                "description": t.description,
            }));
        }
        println!("{}", serde_json::to_string_pretty(&all)?);
        return Ok(());
    }

    print_tools_human(&skills, &nodes, &mcp_entries);
    Ok(())
}

pub async fn inspect(endpoint: &str) -> Result<()> {
    let mut sdk = connect(endpoint).await?;
    let raw = sdk.inspect_runtime().await?;
    let parsed: serde_json::Value =
        serde_json::from_str(&raw).unwrap_or_else(|_| serde_json::Value::String(raw));
    println!("{}", serde_json::to_string_pretty(&parsed)?);
    Ok(())
}

pub async fn channels(endpoint: &str) -> Result<()> {
    let mut sdk = connect(endpoint).await?;
    let raw = sdk.inspect_runtime().await?;
    let parsed: serde_json::Value = serde_json::from_str(&raw)?;

    let chs: Vec<&serde_json::Value> = match parsed.get("channels") {
        Some(serde_json::Value::Object(map)) => map.values().collect(),
        Some(serde_json::Value::Array(arr)) => arr.iter().collect(),
        _ => vec![],
    };

    if chs.is_empty() {
        println!("No active channels.");
        return Ok(());
    }

    println!(
        "{:<38} {:<12} {:<24} {:<18} {}",
        "CHANNEL_ID", "TRANSPORT", "ENDPOINT", "PROVIDER", "CONSUMER"
    );
    println!("{}", "-".repeat(115));
    for ch in &chs {
        println!(
            "{:<38} {:<12} {:<24} {:<18} {}",
            ch.get("channel_id").and_then(|v| v.as_str()).unwrap_or("?"),
            ch.get("transport").and_then(|v| v.as_str()).unwrap_or("?"),
            ch.get("endpoint").and_then(|v| v.as_str()).unwrap_or("?"),
            ch.get("provider_node_id")
                .and_then(|v| v.as_str())
                .unwrap_or("?"),
            ch.get("consumer_id")
                .and_then(|v| v.as_str())
                .unwrap_or("?"),
        );
    }
    println!("\n{} channel(s) active", chs.len());
    Ok(())
}
