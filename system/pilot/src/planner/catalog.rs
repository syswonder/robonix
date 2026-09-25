// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// The callable capabilities: which Atlas entries the model may call, and the
// map from the name the model uses to the provider and contract.

use super::*;

pub(super) type CapabilityTarget = (String, String);
pub(super) type CapabilityTargetMap = HashMap<String, CapabilityTarget>;

pub(crate) struct DisplayCapability<'a> {
    pub(crate) display_name: String,
    pub(crate) provider_id: &'a str,
    pub(crate) cap: &'a atlas_pb::Capability,
}

/// Convert Atlas rows to provider-qualified model names and sort them so an
/// unchanged catalog remains byte-identical even if discovery order varies.
pub(super) fn build_display_capabilities<'a>(
    cap_list: &'a [(String, atlas_pb::Capability)],
    non_llm_callable_contract_ids: &HashSet<String>,
) -> Vec<DisplayCapability<'a>> {
    let mut display = cap_list
        .iter()
        .filter(|(_, cap)| {
            !is_legacy_plan_control_contract(&cap.contract_id)
                && !non_llm_callable_contract_ids.contains(&cap.contract_id)
        })
        .map(|(provider_id, cap)| DisplayCapability {
            display_name: format!("{}.{}", provider_id, llm_name(&cap.contract_id)),
            provider_id: provider_id.as_str(),
            cap,
        })
        .collect::<Vec<_>>();
    display.sort_by(|left, right| left.display_name.cmp(&right.display_name));
    display
}

pub(super) fn is_legacy_plan_control_contract(contract_id: &str) -> bool {
    if !contract_id.starts_with("robonix/system/executor/builtin/") {
        return false;
    }
    matches!(
        contract_id.rsplit('/').next().unwrap_or_default(),
        "cancel_plan" | "cancel_all_plans" | "stop_plan_at" | "get_all_plans" | "get_plan_status"
    )
}

pub(super) fn build_capability_target_map(
    display_caps: &[DisplayCapability<'_>],
) -> CapabilityTargetMap {
    let mut out = HashMap::new();
    for cap in display_caps {
        out.insert(
            cap.display_name.clone(),
            (cap.provider_id.to_string(), cap.cap.contract_id.clone()),
        );
    }
    out
}
