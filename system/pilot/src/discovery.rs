// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>

use anyhow::Result;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::warn;

/// LLM-facing tool name = `<area>_<leaf>` of a contract_id, where
/// `<area>` is the segment immediately before the leaf.
/// Examples:
///   `robonix/primitive/camera/snapshot`        → `camera_snapshot`
///   `robonix/primitive/lidar/snapshot`         → `lidar_snapshot`
///   `robonix/primitive/chassis/move`           → `chassis_move`
///   `robonix/service/memory/search`             → `memory_search`
///   `robonix/service/navigation/navigate`      → `navigation_navigate`
///
/// Plain leaf-only used to be enough but multiple providers share leaves
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

/// One row per provider that registered a CAPABILITY.md, summarised for the
/// LLM-facing "## Capability docs" block in pilot's system prompt. We expose
/// the `provider_id` (what the LLM passes to `read_capability_doc`), the
/// package `kind` (so skills can be flagged read-first), and a one-line
/// `description` lifted from the CAPABILITY.md frontmatter — enough for the
/// model to judge relevance without reading the full manual. The internal
/// `namespace` and any filesystem path are deliberately NOT exposed.
pub struct CapDoc {
    pub provider_id: String,
    pub kind: String,
    pub description: String,
}

/// Pull `kind` and `description` from a CAPABILITY.md YAML frontmatter block.
///
/// The package-level frontmatter is a leading `---` … `---` fence with simple
/// `key: value` lines (single-line values — see the CAPABILITY.md format spec).
/// Returns empty strings when there is no frontmatter or the keys are absent,
/// which is non-fatal: the provider still appears in the index, just without a
/// description until its CAPABILITY.md is updated.
fn parse_frontmatter(md: &str) -> (String, String) {
    let t = md.trim_start();
    let Some(rest) = t.strip_prefix("---") else {
        return (String::new(), String::new());
    };
    let Some(end) = rest.find("\n---") else {
        return (String::new(), String::new());
    };
    let (mut kind, mut description) = (String::new(), String::new());
    for line in rest[..end].lines() {
        let line = line.trim();
        if let Some(v) = line.strip_prefix("kind:") {
            kind = v.trim().trim_matches('"').to_string();
        } else if let Some(v) = line.strip_prefix("description:") {
            description = v.trim().trim_matches('"').to_string();
        }
    }
    (kind, description)
}

/// Returns a `CapDoc` per provider that registered non-empty CAPABILITY.md
/// *content*. Pilot lists these in the system prompt and instructs the LLM to
/// pull the full text on demand via the `read_capability_doc` builtin.
pub async fn cap_md_index(atlas: &mut AtlasClient) -> Result<Vec<CapDoc>> {
    let providers = atlas
        .query_capabilities("", "", atlas_pb::Transport::Unspecified)
        .await?;
    let mut out = Vec::new();
    for provider in providers {
        if provider.capability_md.trim().is_empty() {
            continue;
        }
        let (kind, description) = parse_frontmatter(&provider.capability_md);
        out.push(CapDoc {
            provider_id: provider.id,
            kind,
            description,
        });
    }
    Ok(out)
}

/// Query atlas for every MCP-transport capability. Returns one
/// `(provider_id, Capability)` pair per LLM-callable contract; callers
/// pull description + input_schema_json out of `params.kind` themselves.
/// Capabilities with missing or non-MCP params are dropped with a warning.
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
            // Sanity: an MCP capability without McpParams is malformed —
            // skip rather than feed garbage to the LLM.
            let has_mcp = matches!(
                cap.params.as_ref().and_then(|p| p.kind.as_ref()),
                Some(atlas_pb::transport_params::Kind::Mcp(_))
            );
            if !has_mcp {
                warn!(
                    "[pilot/discovery] provider='{}' contract='{}' has no MCP params; skipping",
                    provider.id, cap.contract_id
                );
                continue;
            }
            out.push((provider.id.clone(), cap));
        }
    }
    Ok(out)
}
