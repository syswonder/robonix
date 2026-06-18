// SPDX-License-Identifier: MulanPSL-2.0
// Flat `RtdlEvent` (`lib/executor/msg/RtdlEvent.msg`).

use crate::pb::executor::{RtdlEvent, RtdlNodeState, RtdlPlanComplete, RtdlPlanStarted};
use crate::pb::pilot::{CapabilityCall, CapabilityCallResult};

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

#[derive(Clone)]
pub struct NodeEventContext {
    pub plan_id: String,
    pub node_index: u32,
    pub node_kind: u32,
}

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

pub fn node_state(
    plan_id: String,
    node_index: u32,
    node_kind: u32,
    state: u32,
    operator_detail: String,
    leaf_result: Option<CapabilityCallResult>,
) -> RtdlEvent {
    RtdlEvent {
        event_kind: EVT_NODE_STATE,
        node_state: Some(RtdlNodeState {
            plan_id,
            node_index,
            node_kind,
            state,
            operator_detail,
            leaf_result,
        }),
        ..Default::default()
    }
}

pub fn node_state_from_result(
    plan_id: &str,
    node_index: u32,
    node_kind: u32,
    call: &CapabilityCall,
    result: CapabilityCallResult,
    state: u32,
) -> RtdlEvent {
    let _ = call;
    node_state(
        plan_id.to_string(),
        node_index,
        node_kind,
        state,
        String::new(),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_state_uses_plan_and_node_fields() {
        let event = node_state(
            "p".into(),
            7,
            2,
            STATE_RUNNING,
            "operator detail".into(),
            None,
        );
        let ns = event.node_state.unwrap();

        assert_eq!(event.event_kind, EVT_NODE_STATE);
        assert_eq!(ns.plan_id, "p");
        assert_eq!(ns.node_index, 7);
        assert_eq!(ns.node_kind, 2);
        assert_eq!(ns.state, STATE_RUNNING);
        assert_eq!(ns.operator_detail, "operator detail");
        assert!(ns.leaf_result.is_none());
    }
}
