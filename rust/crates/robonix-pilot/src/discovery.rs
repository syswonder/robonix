// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>

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

/// One row per registered capability, summarised for the LLM-facing
/// "## Capability docs (lazy-load via read_file)" block in pilot's
/// system prompt. Includes only caps that registered with a non-empty
/// `capability_md_path`. The path is what we hand the LLM verbatim;
/// the executor's `read_file` builtin resolves it (it must be readable
/// from the executor's host workspace).
pub struct CapDoc {
    pub cap_id: String,
    pub namespace: String,
    pub md_path: String,
}

/// Returns a `CapDoc` per capability that has a non-empty
/// `capability_md_path`. Pilot lists these in the system prompt and
/// the LLM read_files them lazily.
pub async fn cap_md_index(atlas: &mut AtlasClient) -> Result<Vec<CapDoc>> {
    let records = atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await?;
    let mut out = Vec::new();
    for rec in records {
        if rec.capability_md_path.is_empty() {
            continue;
        }
        out.push(CapDoc {
            cap_id: rec.capability_id,
            namespace: rec.namespace,
            md_path: rec.capability_md_path,
        });
    }
    Ok(out)
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
