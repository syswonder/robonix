// SPDX-License-Identifier: MulanPSL-2.0
// Per-call async capability detection via required status/cancel sub-contracts.

use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;

#[derive(Debug, Clone)]
pub struct AsyncGroup {
    pub status_contract: String,
    pub cancel_contract: String,
}

fn contract_leaf(contract_id: &str) -> &str {
    contract_id
        .rsplit_once('/')
        .map(|(_, leaf)| leaf)
        .unwrap_or(contract_id)
}

/// Resolve the required async sub-contracts for `contract_id`.
/// Async capabilities must register both `<contract_id>/status` and
/// `<contract_id>/cancel`; registering only one is a provider configuration
/// error surfaced to the caller before dispatching the initial run call.
pub async fn resolve_async_group(
    atlas: &mut AtlasClient,
    provider_id: &str,
    contract_id: &str,
) -> Result<Option<AsyncGroup>, String> {
    if matches!(contract_leaf(contract_id), "status" | "cancel") {
        return Ok(None);
    }
    let status_contract = format!("{contract_id}/status");
    let cancel_contract = format!("{contract_id}/cancel");

    let providers = atlas
        .query_capabilities(provider_id, "", atlas_pb::Transport::Mcp)
        .await
        .map_err(|e| format!("query_capabilities({provider_id}) failed: {e:#}"))?;
    let Some(provider) = providers.into_iter().find(|p| p.id == provider_id) else {
        return Ok(None);
    };
    let has_status = provider
        .capabilities
        .iter()
        .any(|c| c.contract_id == status_contract);
    let has_cancel = provider
        .capabilities
        .iter()
        .any(|c| c.contract_id == cancel_contract);

    match (has_status, has_cancel) {
        (false, false) => Ok(None),
        (true, true) => Ok(Some(AsyncGroup {
            status_contract,
            cancel_contract,
        })),
        _ => Err(format!(
            "async capability '{contract_id}' on provider '{provider_id}' must register both '{status_contract}' and '{cancel_contract}'"
        )),
    }
}
