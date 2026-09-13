// SPDX-License-Identifier: MulanPSL-2.0
// Poll async MCP capabilities via `<contract_id>/status` until terminal state.

use robonix_scribe::warn;
use std::time::Duration;

use crate::dispatch::{self, async_registry::AsyncGroup};
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::plan_runtime::{PlanRuntime, RunningAsyncCall};
use crate::rtdl_wire::{self, NodeEventContext};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use tokio::time;

const POLL_INTERVAL: Duration = Duration::from_secs(2);

/// Dispatch an async cap and poll status every 2s. The caller owns the single
/// terminal event so successful work can be verified before it is published.
pub async fn run_until_terminal(
    call: &CapabilityCall,
    group: &AsyncGroup,
    self_provider_id: &str,
    atlas: &mut AtlasClient,
    node: &NodeEventContext,
    runtime: &PlanRuntime,
) -> (CapabilityCallResult, u32) {
    let initial = dispatch::dispatch(call, self_provider_id, atlas, runtime, &node.plan_id).await;
    if !initial.success {
        return (initial, RtdlNodeStateEnum::Failed as u32);
    }

    let run_id = extract_run_id(&initial.output);
    let accepted = runtime
        .register_or_cancel_async_call(
            &node.plan_id,
            RunningAsyncCall {
                call_id: call.call_id.clone(),
                provider_id: call.provider_id.clone(),
                cancel_contract: group.cancel_contract.clone(),
                run_id: run_id.clone(),
            },
            self_provider_id,
            atlas,
        )
        .await;
    if !accepted {
        let result = canceled_result(call, "plan was cancelled before async polling began");
        return (result, RtdlNodeStateEnum::Canceled as u32);
    }

    let mut interval = time::interval(POLL_INTERVAL);
    interval.tick().await;

    loop {
        interval.tick().await;
        if runtime.is_cancelled(&node.plan_id).await {
            runtime
                .cancel_async_call_for_plan(&node.plan_id, &call.call_id, self_provider_id, atlas)
                .await;
            let result = canceled_result(call, "plan was cancelled");
            return (result, RtdlNodeStateEnum::Canceled as u32);
        }
        let status_out = match poll_status(
            self_provider_id,
            &call.provider_id,
            &group.status_contract,
            &run_id,
            atlas,
        )
        .await
        {
            Ok(s) => s,
            Err(e) => {
                let error = format!("status poll failed for {}: {e:#}", call.contract_id);
                warn!("[executor] {error}");
                let result = failed_result(call, &error);
                runtime
                    .unregister_async_call(&node.plan_id, &call.call_id)
                    .await;
                return (result, RtdlNodeStateEnum::Failed as u32);
            }
        };

        let (state, detail) = parse_status_json(&status_out);
        if rtdl_wire::is_terminal_state(state) {
            let result = terminal_result(call, state, &detail, &status_out);
            runtime
                .unregister_async_call(&node.plan_id, &call.call_id)
                .await;
            return (result, state);
        }
        // Record only live states here. The caller records the final state
        // after optional verification has completed.
        runtime
            .record_op_state(&node.plan_id, &node.op_id, state)
            .await;
    }
}

async fn poll_status(
    consumer_id: &str,
    provider_id: &str,
    status_contract: &str,
    run_id: &str,
    atlas: &mut AtlasClient,
) -> anyhow::Result<String> {
    let args = if run_id.is_empty() {
        "{}".to_string()
    } else {
        serde_json::json!({ "run_id": run_id }).to_string()
    };
    let status_call = CapabilityCall {
        call_id: format!("status-{}", uuid::Uuid::new_v4()),
        provider_id: provider_id.to_string(),
        contract_id: status_contract.to_string(),
        args_json: args,
    };
    let (channel_id, endpoint, _) = atlas
        .connect_capability(
            consumer_id,
            provider_id,
            status_contract,
            atlas_pb::Transport::Mcp,
        )
        .await?;
    let result = crate::dispatch::mcp::execute(&status_call, &endpoint).await;
    let _ = atlas.disconnect_capability(&channel_id).await;
    if result.success {
        Ok(result.output)
    } else {
        anyhow::bail!("{}", result.error)
    }
}

/// Read `run_id` from the async cap's initial MCP response JSON.
pub fn extract_run_id(output: &str) -> String {
    let Ok(v) = serde_json::from_str::<serde_json::Value>(output) else {
        return String::new();
    };
    v.get("run_id")
        .and_then(|x| x.as_str())
        .unwrap_or_default()
        .to_string()
}

/// Parse status MCP JSON: requires uppercase `state` enum; optional `detail` string.
pub fn parse_status_json(output: &str) -> (u32, String) {
    let Ok(v) = serde_json::from_str::<serde_json::Value>(output) else {
        warn!("[executor] status response is not valid JSON: {output}");
        return (RtdlNodeStateEnum::Running as u32, output.to_string());
    };

    let Some(state_str) = v.get("state").and_then(|s| s.as_str()) else {
        let error = format!("status response missing required 'state' field: {output}");
        warn!("[executor] {error}");
        return (RtdlNodeStateEnum::Failed as u32, error);
    };

    let detail = v
        .get("detail")
        .and_then(|x| x.as_str())
        .unwrap_or_default()
        .to_string();

    match parse_state_name(state_str) {
        Some(state) => (state, detail),
        None => {
            let error = format!("status response has unknown state '{state_str}': {output}");
            warn!("[executor] {error}");
            (RtdlNodeStateEnum::Failed as u32, error)
        }
    }
}

/// Convert status response state names into RTDL node state constants.
pub fn parse_state_name(s: &str) -> Option<u32> {
    match s.to_uppercase().as_str() {
        "PENDING" => Some(RtdlNodeStateEnum::Pending as u32),
        "RUNNING" => Some(RtdlNodeStateEnum::Running as u32),
        "SUCCEEDED" => Some(RtdlNodeStateEnum::Succeeded as u32),
        "FAILED" => Some(RtdlNodeStateEnum::Failed as u32),
        "CANCELED" | "CANCELLED" => Some(RtdlNodeStateEnum::Canceled as u32),
        "TIMEOUT" => Some(RtdlNodeStateEnum::Timeout as u32),
        "PAUSED" => Some(RtdlNodeStateEnum::Paused as u32),
        _ => None,
    }
}

fn terminal_result(
    call: &CapabilityCall,
    state: u32,
    detail: &str,
    raw: &str,
) -> CapabilityCallResult {
    let success = state == RtdlNodeStateEnum::Succeeded as u32;
    CapabilityCallResult {
        call_id: call.call_id.clone(),
        provider_id: call.provider_id.clone(),
        contract_id: call.contract_id.clone(),
        success,
        output: if success {
            raw.to_string()
        } else {
            String::new()
        },
        error: if success {
            String::new()
        } else {
            detail.to_string()
        },
    }
}

fn canceled_result(call: &CapabilityCall, error: &str) -> CapabilityCallResult {
    failed_result(call, error)
}

/// Build a failed capability result for async control-plane failures.
/// The original call identity is preserved so the terminal node event still
/// correlates with the user-requested async capability, not the status poll.
fn failed_result(call: &CapabilityCall, error: &str) -> CapabilityCallResult {
    CapabilityCallResult {
        call_id: call.call_id.clone(),
        provider_id: call.provider_id.clone(),
        contract_id: call.contract_id.clone(),
        success: false,
        output: String::new(),
        error: error.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_run_id_from_response() {
        assert_eq!(extract_run_id(r#"{"run_id":"r1","accepted":true}"#), "r1");
        assert_eq!(extract_run_id(r#"{"goal_id":"g1"}"#), "");
    }

    #[test]
    fn parse_status_requires_state_field() {
        let (s, d) = parse_status_json(r#"{"state":"SUCCEEDED","detail":"done"}"#);
        assert_eq!(s, RtdlNodeStateEnum::Succeeded as u32);
        assert_eq!(d, "done");
    }

    #[test]
    fn parse_status_missing_state_fails() {
        let (s, d) = parse_status_json(r#"{"known":true,"terminal":false}"#);
        assert_eq!(s, RtdlNodeStateEnum::Failed as u32);
        assert!(d.contains("missing required 'state' field"));
    }
}
