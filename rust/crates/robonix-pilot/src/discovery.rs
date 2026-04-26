// SPDX-License-Identifier: MulanPSL-2.0
// Atlas-driven capability catalog for the LLM.
//
// Pilot needs to tell the LLM what it can ask for, but doesn't need to
// dial anything itself — dispatch is executor's job. So before each turn
// we just call `QueryCapabilities(transport=Mcp)` against atlas and read
// `McpParams.{description, input_schema_json}` straight from each
// returned `InterfaceMetadata`. No Connect, no channel bookkeeping —
// description + schema are public docs (atlas serves them in metadata).

use anyhow::Result;
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;

/// One callable thing exposed to the LLM. cap_id + contract_id is the
/// Robonix-side identity; `name` (= contract_id leaf) is what the LLM sees.
#[derive(Debug, Clone)]
pub struct CapEntry {
    pub cap_id: String,
    pub contract_id: String,
    pub name: String,
    pub description: String,
    pub input_schema_json: String,
}

fn leaf_of(contract_id: &str) -> String {
    contract_id
        .rsplit_once('/')
        .map(|(_, leaf)| leaf.to_string())
        .unwrap_or_else(|| contract_id.to_string())
}

/// Query atlas for every MCP-transport capability and return the LLM-facing
/// catalog. Description + schema come straight from `InterfaceMetadata.params`
/// (MCP variant); no Connect/Disconnect roundtrip per cap.
pub async fn discover(atlas: &mut AtlasClient, _consumer_id: &str) -> Result<Vec<CapEntry>> {
    let records = atlas
        .query_capabilities("", "", atlas_pb::Transport::Mcp)
        .await?;

    let mut out = Vec::new();
    for rec in records {
        for iface in rec.interfaces {
            if iface.transport != atlas_pb::Transport::Mcp as i32 {
                continue;
            }
            let (description, input_schema_json) = match iface.params.and_then(|p| p.kind) {
                Some(atlas_pb::transport_params::Kind::Mcp(m)) => {
                    (m.description, m.input_schema_json)
                }
                _ => {
                    log::warn!(
                        "[pilot/discovery] cap='{}' contract='{}' has no MCP params; skipping",
                        rec.capability_id,
                        iface.contract_id
                    );
                    continue;
                }
            };
            out.push(CapEntry {
                name: leaf_of(&iface.contract_id),
                cap_id: rec.capability_id.clone(),
                contract_id: iface.contract_id,
                description,
                input_schema_json,
            });
        }
    }
    Ok(out)
}
