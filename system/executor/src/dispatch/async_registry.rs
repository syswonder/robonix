// SPDX-License-Identifier: MulanPSL-2.0
// Per-call async cap detection via sibling status/cancel contracts on the provider.

use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use robonix_scribe::warn;

#[derive(Debug, Clone)]
pub struct AsyncGroup {
    pub status_contract: String,
    pub cancel_contract: Option<String>,
}

fn contract_namespace(contract_id: &str) -> Option<&str> {
    contract_id.rsplit_once('/').map(|(ns, _)| ns)
}

fn contract_leaf(contract_id: &str) -> &str {
    contract_id
        .rsplit_once('/')
        .map(|(_, leaf)| leaf)
        .unwrap_or(contract_id)
}

/// When the target provider registers `{namespace}/status`, treat `contract_id` as async.
pub async fn resolve_async_group(
    atlas: &mut AtlasClient,
    provider_id: &str,
    contract_id: &str,
) -> Option<AsyncGroup> {
    if matches!(contract_leaf(contract_id), "status" | "cancel") {
        return None;
    }
    let ns = contract_namespace(contract_id)?;
    let status_contract = format!("{ns}/status");
    let cancel_contract = format!("{ns}/cancel");

    let providers = atlas
        .query_capabilities(provider_id, "", atlas_pb::Transport::Mcp)
        .await
        .ok()?;
    let provider = providers.into_iter().find(|p| p.id == provider_id)?;
    if !provider
        .capabilities
        .iter()
        .any(|c| c.contract_id == status_contract)
    {
        return None;
    }
    let has_cancel = provider
        .capabilities
        .iter()
        .any(|c| c.contract_id == cancel_contract);
    if !has_cancel {
        warn!(
            "[executor] provider '{provider_id}' namespace '{ns}' has status but no cancel contract"
        );
    }
    Some(AsyncGroup {
        status_contract,
        cancel_contract: has_cancel.then_some(cancel_contract),
    })
}
