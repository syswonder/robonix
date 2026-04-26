// SPDX-License-Identifier: MulanPSL-2.0
// Atlas-driven capability catalog for the LLM.
//
// Pilot doesn't dispatch — but it needs to tell the LLM *what* it can ask
// for. So before each turn we query atlas for MCP-providing caps, peek at
// each cap's McpParams (description + input_schema_json) via a brief
// Connect/Disconnect, and assemble the catalog.
//
// The Connect here is purely for read access: bookkeeping is opened and
// closed inline; pilot never holds these channels for the actual call (the
// executor does, when it dispatches).

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

impl CapEntry {
    /// LLM-facing tool name = contract_id leaf (last `/` segment). All MCP
    /// servers expose tools by short name; the cap+contract grouping is
    /// internal to Robonix.
    fn from_metadata(cap_id: String, contract_id: String) -> Self {
        let name = contract_id
            .rsplit_once('/')
            .map(|(_, leaf)| leaf.to_string())
            .unwrap_or_else(|| contract_id.clone());
        Self {
            cap_id,
            contract_id,
            name,
            description: String::new(),
            input_schema_json: String::new(),
        }
    }
}

/// Query atlas for every MCP-transport capability and return the LLM-facing
/// catalog. Each entry's description + input_schema is fetched via a brief
/// `ConnectCapability` (released immediately).
pub async fn discover(atlas: &mut AtlasClient, consumer_id: &str) -> Result<Vec<CapEntry>> {
    let records = atlas
        .query_capabilities("", "", atlas_pb::Transport::Mcp)
        .await?;

    let mut out = Vec::new();
    for rec in records {
        for iface in rec.interfaces {
            if iface.transport != atlas_pb::Transport::Mcp as i32 {
                continue;
            }
            let mut entry =
                CapEntry::from_metadata(rec.capability_id.clone(), iface.contract_id.clone());

            // Brief Connect/Disconnect to read MCP params (description +
            // schema). Errors are warnings, not fatal — a cap that's gone
            // away between Query and Connect just gets skipped.
            match atlas
                .connect_capability(
                    consumer_id,
                    &rec.capability_id,
                    &iface.contract_id,
                    atlas_pb::Transport::Mcp,
                )
                .await
            {
                Ok((channel_id, _endpoint, params)) => {
                    if let Some(atlas_pb::transport_params::Kind::Mcp(m)) = params.kind {
                        entry.description = m.description;
                        entry.input_schema_json = m.input_schema_json;
                    }
                    let _ = atlas.disconnect_capability(&channel_id).await;
                }
                Err(e) => {
                    log::warn!(
                        "[pilot/discovery] connect to '{}/{}' failed: {e:#}; skipping",
                        rec.capability_id,
                        iface.contract_id
                    );
                    continue;
                }
            };
            out.push(entry);
        }
    }
    Ok(out)
}
