// SPDX-License-Identifier: MulanPSL-2.0
// Flat `RtdlEvent` (`lib/executor/msg/RtdlEvent.msg`).

use crate::pb::executor::{RtdlEvent, RtdlNodeState, RtdlPlanComplete, RtdlPlanStarted};
use crate::pb::pilot::CapabilityCallResult;

pub const EVT_PLAN_STARTED: u32 = 0;
pub const EVT_NODE_STATE: u32 = 1;
pub const EVT_PLAN_COMPLETE: u32 = 2;

pub const STATE_PENDING: u32 = 0;
pub const STATE_RUNNING: u32 = 1;
pub const STATE_SUCCEEDED: u32 = 2;
pub const STATE_FAILED: u32 = 3;
pub const STATE_CANCELED: u32 = 4;
pub const STATE_TIMEOUT: u32 = 5;
pub const STATE_PAUSED: u32 = 6;

pub fn is_terminal_state(state: u32) -> bool {
    matches!(
        state,
        STATE_SUCCEEDED | STATE_FAILED | STATE_CANCELED | STATE_TIMEOUT
    )
}

pub fn plan_started(plan_id: String) -> RtdlEvent {
    RtdlEvent {
        event_kind: EVT_PLAN_STARTED,
        plan_started: Some(RtdlPlanStarted { plan_id }),
        ..Default::default()
    }
}

/// Emit an RTDL node (cap call) state change. When `result` is set, success/output/error
/// are copied into the event (required for terminal states).
pub fn node_state(
    call_id: String,
    provider_id: String,
    contract_id: String,
    run_id: String,
    state: u32,
    detail: String,
    result: Option<CapabilityCallResult>,
) -> RtdlEvent {
    let (success, output, error) = if let Some(r) = result {
        (r.success, r.output, r.error)
    } else {
        (false, String::new(), String::new())
    };
    RtdlEvent {
        event_kind: EVT_NODE_STATE,
        node_state: Some(RtdlNodeState {
            call_id,
            provider_id,
            contract_id,
            run_id,
            state,
            detail,
            success,
            output,
            error,
        }),
        ..Default::default()
    }
}

pub fn node_state_from_result(
    result: CapabilityCallResult,
    state: u32,
    run_id: String,
) -> RtdlEvent {
    let detail = if result.success {
        String::new()
    } else {
        result.error.clone()
    };
    node_state(
        result.call_id.clone(),
        result.provider_id.clone(),
        result.contract_id.clone(),
        run_id,
        state,
        detail,
        Some(result),
    )
}

pub fn plan_complete(plan_id: String, any_failed: bool) -> RtdlEvent {
    RtdlEvent {
        event_kind: EVT_PLAN_COMPLETE,
        plan_complete: Some(RtdlPlanComplete {
            plan_id,
            any_failed,
        }),
        ..Default::default()
    }
}
