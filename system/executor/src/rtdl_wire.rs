// SPDX-License-Identifier: MulanPSL-2.0
// Flat `RtdlEvent` (`lib/executor/msg/RtdlEvent.msg`).

use crate::pb::executor::{RtdlEvent, RtdlPlanComplete, RtdlPlanStarted};
use crate::pb::pilot::{CapabilityCallResult, RtdlNodeState};

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
    pub op_id: String,
    pub description: String,
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
    node: &NodeEventContext,
    state: u32,
    operator_detail: String,
    leaf_result: Option<CapabilityCallResult>,
) -> RtdlEvent {
    RtdlEvent {
        event_kind: EVT_NODE_STATE,
        node_state: Some(RtdlNodeState {
            plan_id: node.plan_id.clone(),
            node_index: node.node_index,
            node_kind: node.node_kind,
            state,
            operator_detail,
            leaf_result,
            op_id: node.op_id.clone(),
            description: node.description.clone(),
        }),
        ..Default::default()
    }
}

pub fn node_state_from_result(
    node: &NodeEventContext,
    result: CapabilityCallResult,
    state: u32,
) -> RtdlEvent {
    node_state(node, state, String::new(), Some(result))
}

pub fn operator_node_state(node: &NodeEventContext, state: u32, detail: String) -> RtdlEvent {
    node_state(node, state, detail, None)
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
            &NodeEventContext {
                plan_id: "p".into(),
                node_index: 7,
                node_kind: 2,
                op_id: "op_1".into(),
                description: "take a camera snapshot".into(),
            },
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
        assert_eq!(ns.op_id, "op_1");
        assert_eq!(ns.description, "take a camera snapshot");
    }
}
