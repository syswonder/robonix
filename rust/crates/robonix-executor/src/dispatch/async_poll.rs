// SPDX-License-Identifier: MulanPSL-2.0
// Poll async MCP capabilities via sibling status contract until terminal state.

use std::time::Duration;

use crate::dispatch::{self, async_registry::AsyncGroup};
use crate::pb::executor::RtdlEvent;
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};
use crate::plan_runtime::{PlanRuntime, RunningAsyncCall};
use crate::rtdl_wire::{
    self, STATE_CANCELED, STATE_FAILED, STATE_RUNNING, STATE_SUCCEEDED, STATE_TIMEOUT,
};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb as atlas_pb;
use tokio::sync::mpsc::Sender;
use tokio::time;

const POLL_INTERVAL: Duration = Duration::from_secs(2);

/// Dispatch an async cap, poll status every 2s, stream node_state changes, return terminal result.
pub async fn run_until_terminal(
    call: &CapabilityCall,
    group: &AsyncGroup,
    self_provider_id: &str,
    atlas: &mut AtlasClient,
    tx: &Sender<Result<RtdlEvent, tonic::Status>>,
    plan_id: &str,
    runtime: &PlanRuntime,
) -> CapabilityCallResult {
    let initial = dispatch::dispatch(call, self_provider_id, atlas, runtime).await;
    if !initial.success {
        let _ = tx
            .send(Ok(rtdl_wire::node_state_from_result(
                initial.clone(),
                STATE_FAILED,
                String::new(),
            )))
            .await;
        return initial;
    }

    let run_id = extract_run_id(&initial.output);
    if let Some(cancel_contract) = &group.cancel_contract {
        let accepted = runtime
            .register_or_cancel_async_call(
                plan_id,
                RunningAsyncCall {
                    call_id: call.call_id.clone(),
                    provider_id: call.provider_id.clone(),
                    cancel_contract: cancel_contract.clone(),
                    run_id: run_id.clone(),
                },
                self_provider_id,
                atlas,
            )
            .await;
        if !accepted {
            let result = canceled_result(call, "plan was cancelled before async polling began");
            let _ = tx
                .send(Ok(rtdl_wire::node_state_from_result(
                    result.clone(),
                    STATE_CANCELED,
                    run_id,
                )))
                .await;
            return result;
        }
    }

    let _ = tx
        .send(Ok(rtdl_wire::node_state(
            call.call_id.clone(),
            call.provider_id.clone(),
            call.contract_id.clone(),
            run_id.clone(),
            STATE_RUNNING,
            initial.output.clone(),
            None,
        )))
        .await;
    let mut last_state = STATE_RUNNING;

    let mut interval = time::interval(POLL_INTERVAL);
    interval.tick().await;

    loop {
        interval.tick().await;
        if runtime.is_cancelled(plan_id).await {
            runtime
                .cancel_async_call_for_plan(plan_id, &call.call_id, self_provider_id, atlas)
                .await;
            let result = canceled_result(call, "plan was cancelled");
            let _ = tx
                .send(Ok(rtdl_wire::node_state_from_result(
                    result.clone(),
                    STATE_CANCELED,
                    run_id,
                )))
                .await;
            return result;
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
                log::warn!(
                    "[executor] status poll failed for {}: {e:#}",
                    call.contract_id
                );
                continue;
            }
        };

        let (state, detail) = parse_status_json(&status_out);
        if state != last_state {
            if rtdl_wire::is_terminal_state(state) {
                let result = terminal_result(call, state, &detail, &status_out);
                runtime.unregister_async_call(plan_id, &call.call_id).await;
                let _ = tx
                    .send(Ok(rtdl_wire::node_state_from_result(
                        result.clone(),
                        state,
                        run_id.clone(),
                    )))
                    .await;
                return result;
            }
            let _ = tx
                .send(Ok(rtdl_wire::node_state(
                    call.call_id.clone(),
                    call.provider_id.clone(),
                    call.contract_id.clone(),
                    run_id.clone(),
                    state,
                    detail.clone(),
                    None,
                )))
                .await;
            last_state = state;
        }
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
        log::warn!("[executor] status response is not valid JSON: {output}");
        return (STATE_RUNNING, output.to_string());
    };

    let Some(state_str) = v.get("state").and_then(|s| s.as_str()) else {
        let error = format!("status response missing required 'state' field: {output}");
        log::warn!("[executor] {error}");
        return (STATE_FAILED, error);
    };

    (
        parse_state_name(state_str),
        v.get("detail")
            .and_then(|x| x.as_str())
            .unwrap_or_default()
            .to_string(),
    )
}

pub fn parse_state_name(s: &str) -> u32 {
    match s.to_uppercase().as_str() {
        "PENDING" => rtdl_wire::STATE_PENDING,
        "RUNNING" => STATE_RUNNING,
        "SUCCEEDED" => STATE_SUCCEEDED,
        "FAILED" => STATE_FAILED,
        "CANCELED" | "CANCELLED" => STATE_CANCELED,
        "TIMEOUT" => STATE_TIMEOUT,
        "PAUSED" => rtdl_wire::STATE_PAUSED,
        other => {
            log::warn!("[executor] unknown status state '{other}', treating as RUNNING");
            STATE_RUNNING
        }
    }
}

fn terminal_result(
    call: &CapabilityCall,
    state: u32,
    detail: &str,
    raw: &str,
) -> CapabilityCallResult {
    let success = state == STATE_SUCCEEDED;
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
        assert_eq!(s, STATE_SUCCEEDED);
        assert_eq!(d, "done");
    }

    #[test]
    fn parse_status_missing_state_fails() {
        let (s, d) = parse_status_json(r#"{"known":true,"terminal":false}"#);
        assert_eq!(s, STATE_FAILED);
        assert!(d.contains("missing required 'state' field"));
    }
}
