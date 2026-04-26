// SPDX-License-Identifier: MulanPSL-2.0
// Atlas-driven capability catalog for the LLM.
//
// Pilot needs to tell the LLM what it can ask for, but doesn't dial
// anything itself — dispatch is executor's job. So before each turn we
// just call `QueryCapabilities(transport=Mcp)` against atlas and read
// `McpParams.{description, input_schema_json}` straight from each
// returned `InterfaceMetadata`. No Connect, no channel bookkeeping —
// description + schema are public docs (atlas serves them in metadata).

use anyhow::Result;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;

/// LLM-facing tool name = the last `/`-segment of a contract_id.
/// `robonix/system/memory/search` → `search`.
pub fn llm_name(contract_id: &str) -> &str {
    contract_id
        .rsplit_once('/')
        .map(|(_, leaf)| leaf)
        .unwrap_or(contract_id)
}

/// Query atlas for every MCP-transport interface. Returns one
/// `(cap_id, InterfaceMetadata)` pair per LLM-callable contract; callers
/// pull description + input_schema_json out of `params.kind` themselves.
/// Interfaces with missing or non-MCP params are dropped with a warning.
pub async fn discover(
    atlas: &mut AtlasClient,
) -> Result<Vec<(String, atlas_pb::InterfaceMetadata)>> {
    let records = atlas
        .query_capabilities("", "", atlas_pb::Transport::Mcp)
        .await?;

    let mut out = Vec::new();
    for rec in records {
        for iface in rec.interfaces {
            if iface.transport != atlas_pb::Transport::Mcp as i32 {
                continue;
            }
            // Sanity: an MCP interface without McpParams is malformed —
            // skip rather than feed garbage to the LLM.
            let has_mcp = matches!(
                iface.params.as_ref().and_then(|p| p.kind.as_ref()),
                Some(atlas_pb::transport_params::Kind::Mcp(_))
            );
            if !has_mcp {
                log::warn!(
                    "[pilot/discovery] cap='{}' contract='{}' has no MCP params; skipping",
                    rec.capability_id,
                    iface.contract_id
                );
                continue;
            }
            out.push((rec.capability_id.clone(), iface));
        }
    }
    Ok(out)
}
