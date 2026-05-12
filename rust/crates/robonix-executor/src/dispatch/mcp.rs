// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// dispatch/mcp.rs — MCP tool dispatch
//
// TODO(executor maintainer): implement full MCP dispatch.
// Skeleton: connects to the MCP endpoint and calls the named tool.

use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use rmcp::ServiceExt;

type McpClient = rmcp::service::RunningService<rmcp::RoleClient, ()>;

/// MCP tool name = leaf of contract_id (e.g.
/// `robonix/system/memory/search` → `search`). Servers expose tools by
/// short name; the provider+contract grouping is Robonix-side bookkeeping.
/// TODO: use leaf node as tool name may  introduce collisions across providers - wheatfox
fn mcp_tool_name(contract_id: &str) -> &str {
    contract_id
        .rsplit_once('/')
        .map(|(_, leaf)| leaf)
        .unwrap_or(contract_id)
}

pub async fn execute(call: &CapabilityCall, endpoint: &str) -> CapabilityCallResult {
    let name = mcp_tool_name(&call.contract_id);
    let result = call_mcp(name, &call.args_json, endpoint).await;
    let mut out = CapabilityCallResult {
        call_id: call.call_id.clone(),
        provider_id: call.provider_id.clone(),
        contract_id: call.contract_id.clone(),
        ..Default::default()
    };
    match result {
        Ok(s) => {
            out.success = true;
            out.output = s;
        }
        Err(e) => {
            out.success = false;
            out.error = e.to_string();
        }
    }
    out
}

async fn call_mcp(name: &str, args_json: &str, endpoint: &str) -> anyhow::Result<String> {
    let mut client = connect_mcp(endpoint).await?;
    let args_val: serde_json::Value = serde_json::from_str(args_json)
        .map_err(|e| anyhow::anyhow!("invalid tool arguments JSON: {e}"))?;
    let args_obj = args_val.as_object().cloned();

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
        .filter_map(|c| match &c.raw {
            rmcp::model::RawContent::Text(t) => Some(t.text.as_str()),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join("\n");

    let is_error = result.is_error.unwrap_or(false);
    client.close().await.ok();

    if is_error {
        anyhow::bail!("mcp tool error: {}", text);
    }
    Ok(text)
}

async fn connect_mcp(endpoint: &str) -> anyhow::Result<McpClient> {
    if let Some(cmd_str) = endpoint.strip_prefix("stdio://") {
        let mut parts = cmd_str.split_whitespace();
        let program = parts.next().unwrap_or("python3");
        let args: Vec<&str> = parts.collect();
        let mut command = tokio::process::Command::new(program);
        command.args(&args);
        let transport = rmcp::transport::TokioChildProcess::new(command)
            .map_err(|e| anyhow::anyhow!("MCP stdio spawn failed: {e}"))?;
        return ().serve(transport).await.map_err(|e| anyhow::anyhow!("MCP init failed: {e}"));
    }
    let base = if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{}", endpoint)
    };
    let uri = if base.contains("/mcp") {
        base
    } else {
        format!("{}/mcp", base.trim_end_matches('/'))
    };
    let transport =
        rmcp::transport::streamable_http_client::StreamableHttpClientTransport::from_uri(
            uri.clone(),
        );
    ().serve(transport)
        .await
        .map_err(|e| anyhow::anyhow!("MCP HTTP connect to '{uri}' failed: {e}"))
}
