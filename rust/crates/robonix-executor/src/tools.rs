// SPDX-License-Identifier: MulanPSL-2.0
// Tool catalogue: builtin tools + Atlas-discovered MCP tools.
//
// `load_tools` is called by ExecutorService at every Stream / list_tools RPC
// so caps that registered after the last query are picked up next round.

use crate::pb::pilot::ToolRouting;
use crate::routing_kind::RoutingKind;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use serde_json::Value;
use std::collections::HashMap;

/// Fully-resolved tool spec returned to consumers via list_tools.
#[derive(Clone, Debug)]
pub struct ToolEntry {
    pub name: String,
    pub description: String,
    pub input_schema: Value,
    pub routing: ToolRouting,
}

pub async fn load_tools(atlas: &mut AtlasClient) -> anyhow::Result<Vec<ToolEntry>> {
    let mut out: Vec<ToolEntry> = builtin_tools();
    match load_mcp_tools(atlas).await {
        Ok(mcp) => {
            if mcp.is_empty() {
                log::debug!("no MCP tools from atlas (caps up? all MCP interfaces declared?)");
            }
            out.extend(mcp);
        }
        Err(e) => log::warn!("atlas MCP tool discovery failed: {e:#}"),
    }
    Ok(out)
}

// ── Builtin tools ─────────────────────────────────────────────────────────

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

// ── Atlas-discovered MCP tools ────────────────────────────────────────────

/// Query atlas for caps offering ANY MCP interface, then unpack each
/// `(contract_id, endpoint, McpParams)` triple into one `ToolEntry`.
///
/// Tool naming policy: contract_id leaf (last `/` segment) is the tool name.
/// Description + input_schema_json come from the cap's typed `McpParams`
/// stored at declare time — no opaque metadata_json round-trip.
async fn load_mcp_tools(atlas: &mut AtlasClient) -> anyhow::Result<Vec<ToolEntry>> {
    let records = atlas
        .query_capabilities("", "", atlas_pb::Transport::Mcp)
        .await?;
    let mut out = Vec::new();

    for rec in records {
        for ep in rec.endpoints {
            if ep.transport != atlas_pb::Transport::Mcp as i32 {
                continue;
            }
            let endpoint = ep.endpoint.replace("localhost", "127.0.0.1");
            if endpoint.is_empty() {
                continue;
            }
            let (description, input_schema_json) = match ep.params.and_then(|p| p.kind) {
                Some(atlas_pb::transport_params::Kind::Mcp(m)) => {
                    (m.description, m.input_schema_json)
                }
                _ => (String::new(), String::new()),
            };
            // Tool name = leaf of contract_id (e.g. "robonix/system/memory/search" → "search").
            // Memory's MCP server registers tools by their full leaf name though
            // (`search_memory`, `save_memory`, `compact_memory`); fall back to
            // metadata-tool-name on legacy callers if/when needed.
            let name = match ep.contract_id.rsplit_once('/') {
                Some((_, leaf)) => leaf.to_string(),
                None => ep.contract_id.clone(),
            };
            let schema: Value = if input_schema_json.is_empty() {
                serde_json::json!({"type":"object","properties":{}})
            } else {
                serde_json::from_str(&input_schema_json)
                    .unwrap_or_else(|_| serde_json::json!({"type":"object","properties":{}}))
            };
            out.push(tool(
                &name,
                &description,
                schema,
                RoutingKind::Mcp,
                &endpoint,
            ));
        }
    }
    Ok(out)
}

/// Build a name → ToolEntry lookup map for the dispatcher.
pub fn routing_map(tools: &[ToolEntry]) -> HashMap<String, ToolEntry> {
    tools.iter().map(|t| (t.name.clone(), t.clone())).collect()
}
