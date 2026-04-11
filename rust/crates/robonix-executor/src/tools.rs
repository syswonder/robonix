// SPDX-License-Identifier: MulanPSL-2.0
// tools.rs — tool catalogue (builtin + Atlas-discovered MCP)
//
// ExecutorService calls `load_tools` to build the VLM-facing tool list and
// populate the routing table used by dispatch.

use crate::pilot::ToolRouting;
use crate::routing_kind::RoutingKind;
use robonix_sdk::RobonixClient;
use serde_json::Value;
use std::collections::HashMap;

/// Fully-resolved tool spec handed to Pilot via ListTools.
#[derive(Clone, Debug)]
pub struct ToolEntry {
    pub name: String,
    pub description: String,
    pub input_schema: Value,
    pub routing: ToolRouting,
}

/// Discover all tools: built-in first, then Atlas MCP nodes.
pub async fn load_tools(sdk: &mut RobonixClient) -> anyhow::Result<Vec<ToolEntry>> {
    let mut out: Vec<ToolEntry> = builtin_tools();
    match load_mcp_tools(sdk).await {
        Ok(mcp) => {
            if mcp.is_empty() {
                log::debug!(
                    "no MCP tools from Atlas (nodes up? try ROBONIX_MCP_NAMESPACE_PREFIX=\"\" to match all namespaces)"
                );
            }
            out.extend(mcp);
        }
        Err(e) => log::warn!("Atlas MCP tool discovery failed: {e:#}"),
    }
    Ok(out)
}

// ── Builtin tools ─────────────────────────────────────────────────────────────

pub fn builtin_tools() -> Vec<ToolEntry> {
    vec![
        tool(
            "read_file",
            "Read a file and return its contents",
            serde_json::json!({
                "type": "object",
                "properties": { "path": {"type":"string","description":"Absolute or relative file path"} },
                "required": ["path"]
            }),
            RoutingKind::Builtin,
            "",
        ),
        tool(
            "write_file",
            "Write content to a file (creates or overwrites)",
            serde_json::json!({
                "type": "object",
                "properties": {
                    "path":    {"type":"string"},
                    "content": {"type":"string"}
                },
                "required": ["path","content"]
            }),
            RoutingKind::Builtin,
            "",
        ),
        tool(
            "patch_file",
            "Replace the first occurrence of a string in a file",
            serde_json::json!({
                "type": "object",
                "properties": {
                    "path": {"type":"string"},
                    "old":  {"type":"string"},
                    "new":  {"type":"string"}
                },
                "required": ["path","old","new"]
            }),
            RoutingKind::Builtin,
            "",
        ),
        tool(
            "list_dir",
            "List files and directories at a path",
            serde_json::json!({
                "type": "object",
                "properties": { "path": {"type":"string","description":"Directory path (default: current dir)"} }
            }),
            RoutingKind::Builtin,
            "",
        ),
        tool(
            "run_command",
            "Run a shell command and return stdout/stderr",
            serde_json::json!({
                "type": "object",
                "properties": { "command": {"type":"string"} },
                "required": ["command"]
            }),
            RoutingKind::Builtin,
            "",
        ),
    ]
}

fn tool(name: &str, desc: &str, schema: Value, kind: RoutingKind, endpoint: &str) -> ToolEntry {
    ToolEntry {
        name: name.into(),
        description: desc.into(),
        input_schema: schema,
        routing: ToolRouting {
            kind: kind as u32,
            endpoint: endpoint.into(),
            metadata_json: String::new(),
        },
    }
}

// ── Atlas-discovered MCP tools ────────────────────────────────────────────────

async fn load_mcp_tools(sdk: &mut RobonixClient) -> anyhow::Result<Vec<ToolEntry>> {
    // Empty prefix = match all registered nodes (QueryNodes treats empty namespace as wildcard).
    // Default empty so `robonix/srv/...` MCP providers are included alongside `robonix/prm/...`.
    let ns = std::env::var("ROBONIX_MCP_NAMESPACE_PREFIX").unwrap_or_default();
    let nodes = sdk.query_nodes(&ns, "", "mcp").await?;
    let mut out = Vec::new();

    for n in nodes {
        for iface in &n.interfaces {
            if !iface.supported_transports.contains(&"mcp".to_string()) {
                continue;
            }
            let meta: Value = match serde_json::from_str(&iface.metadata_json) {
                Ok(v) => v,
                Err(e) => {
                    log::warn!(
                        "skip MCP iface on node {}: metadata_json not JSON ({e}) snippet={:?}",
                        n.node_id,
                        iface.metadata_json.chars().take(120).collect::<String>()
                    );
                    continue;
                }
            };
            let endpoint = meta
                .get("endpoint")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string();
            if endpoint.is_empty() {
                continue;
            }
            // Same IPv4-only listen + localhost→::1 issue as gRPC clients.
            let endpoint = endpoint.replace("localhost", "127.0.0.1");
            for t in meta
                .get("tools")
                .and_then(|v| v.as_array())
                .cloned()
                .unwrap_or_default()
            {
                let name = t
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("unknown")
                    .to_string();
                let desc = t
                    .get("description")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string();
                let schema = t
                    .get("input_schema")
                    .cloned()
                    .unwrap_or_else(|| serde_json::json!({"type":"object","properties":{}}));
                out.push(tool(&name, &desc, schema, RoutingKind::Mcp, &endpoint));
            }
        }
    }
    Ok(out)
}

/// Build a name → ToolEntry lookup map for the dispatcher.
pub fn routing_map(tools: &[ToolEntry]) -> HashMap<String, ToolEntry> {
    tools.iter().map(|t| (t.name.clone(), t.clone())).collect()
}
