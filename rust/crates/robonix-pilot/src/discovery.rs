// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>

use anyhow::Result;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;

/// LLM-facing tool name = `<area>_<leaf>` of a contract_id, where
/// `<area>` is the segment immediately before the leaf.
/// Examples:
///   `robonix/primitive/camera/snapshot`        → `camera_snapshot`
///   `robonix/primitive/lidar/snapshot`         → `lidar_snapshot`
///   `robonix/primitive/chassis/move`           → `chassis_move`
///   `robonix/system/memory/search`             → `memory_search`
///   `robonix/service/navigation/navigate`      → `navigation_navigate`
///
/// Plain leaf-only used to be enough but multiple caps share leaves
/// (`snapshot` on camera AND lidar). The OpenAI tool-list collapses
/// duplicates and the LLM picks the wrong one. Prefixing with the
/// area segment disambiguates while staying short and human-readable.
///
/// Executor still routes via the *full* `contract_id` (the leaf is
/// the MCP-server-side tool name, which is unique within a single
/// driver's FastMCP server). This function only renames at the
/// LLM-↔-pilot boundary.
pub fn llm_name(contract_id: &str) -> String {
    let mut segs = contract_id.rsplit('/');
    let leaf = segs.next().unwrap_or(contract_id);
    let area = segs.next().unwrap_or("");
    if area.is_empty() {
        leaf.to_string()
    } else {
        format!("{area}_{leaf}")
    }
}

/// One row per registered capability, summarised for the LLM-facing
/// "## Capability docs (lazy-load via read_file)" block in pilot's
/// system prompt. Includes only caps that registered with a non-empty
/// `capability_md_path`. The path is what we hand the LLM verbatim;
/// the executor's `read_file` builtin resolves it (it must be readable
/// from the executor's host workspace).
pub struct CapDoc {
    pub provider_id: String,
    pub namespace: String,
    pub md_path: String,
}

/// Returns a `CapDoc` per capability that has a non-empty
/// `capability_md_path`. Pilot lists these in the system prompt and
/// the LLM read_files them lazily.
pub async fn cap_md_index(atlas: &mut AtlasClient) -> Result<Vec<CapDoc>> {
    let providers = atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await?;
    let mut out = Vec::new();
    for provider in providers {
        if provider.capability_md_path.is_empty() {
            continue;
        }
        out.push(CapDoc {
            provider_id: provider.id,
            namespace: provider.namespace,
            md_path: provider.capability_md_path,
        });
    }
    Ok(out)
}

/// Query atlas for every MCP-transport interface. Returns one
/// `(provider_id, Capability)` pair per LLM-callable contract; callers
/// pull description + input_schema_json out of `params.kind` themselves.
/// Interfaces with missing or non-MCP params are dropped with a warning.
pub async fn discover(atlas: &mut AtlasClient) -> Result<Vec<(String, atlas_pb::Capability)>> {
    let providers = atlas
        .query_capabilities("", "", atlas_pb::Transport::Mcp)
        .await?;

    let mut out = Vec::new();
    for provider in providers {
        for cap in provider.capabilities {
            if cap.transport != atlas_pb::Transport::Mcp as i32 {
                continue;
            }
            // Sanity: an MCP interface without McpParams is malformed —
            // skip rather than feed garbage to the LLM.
            let has_mcp = matches!(
                cap.params.as_ref().and_then(|p| p.kind.as_ref()),
                Some(atlas_pb::transport_params::Kind::Mcp(_))
            );
            if !has_mcp {
                log::warn!(
                    "[pilot/discovery] cap='{}' contract='{}' has no MCP params; skipping",
                    provider.id,
                    cap.contract_id
                );
                continue;
            }
            out.push((provider.id.clone(), cap));
        }
    }
    Ok(out)
}
