// SPDX-License-Identifier: MulanPSL-2.0
//! Construct RTDL events and publish operator state transitions.

use crate::active_plans::PlanRuntime;
use crate::pb::executor::rtdl_event::RtdlEventEnum;
use crate::pb::executor::{RtdlEvent, RtdlPlanComplete, RtdlPlanStarted};
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
use crate::pb::pilot::{CapabilityCallResult, Plan, RtdlNodeState};
use tokio::sync::mpsc::Sender;
use tonic::Status;

const RTDL_SEQUENCE: u32 = 0;
const RTDL_PARALLEL: u32 = 1;

#[derive(Clone)]
pub struct NodeEventContext {
    pub plan_id: String,
    pub node_index: u32,
    pub node_kind: u32,
    pub op_id: String,
    pub description: String,
}

/// Identify final node states; unknown wire values are not terminal.
pub fn is_terminal_state(state: u32) -> bool {
    matches!(
        RtdlNodeStateEnum::try_from(state as i32),
        Ok(RtdlNodeStateEnum::Succeeded
            | RtdlNodeStateEnum::Failed
            | RtdlNodeStateEnum::Canceled
            | RtdlNodeStateEnum::Timeout)
    )
}

/// Construct the first event of a registered plan.
pub fn plan_started(plan_id: String) -> RtdlEvent {
    RtdlEvent {
        event_kind: RtdlEventEnum::PlanStarted as u32,
        plan_started: Some(RtdlPlanStarted { plan_id }),
        ..Default::default()
    }
}

/// Preserve node identity while attaching operator details or a leaf result.
pub fn node_state(
    node: &NodeEventContext,
    state: u32,
    operator_detail: String,
    leaf_result: Option<CapabilityCallResult>,
) -> RtdlEvent {
    RtdlEvent {
        event_kind: RtdlEventEnum::NodeState as u32,
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

/// Construct the final event after plan execution and verification finish.
pub fn plan_complete(plan_id: String, any_failed: bool) -> RtdlEvent {
    RtdlEvent {
        event_kind: RtdlEventEnum::PlanComplete as u32,
        plan_complete: Some(RtdlPlanComplete {
            plan_id,
            any_failed,
        }),
        ..Default::default()
    }
}

/// Build the wire context copied into every node_state event.
pub(crate) fn node_event_context(plan: &Plan, node_index: usize) -> NodeEventContext {
    let node = &plan.nodes[node_index];
    NodeEventContext {
        plan_id: plan.plan_id.clone(),
        node_index: node_index as u32,
        node_kind: node.node_kind,
        op_id: node.op_id.clone(),
        description: node.description.clone(),
    }
}

/// Stream a state event for a non-leaf RTDL operator node and record it.
pub(crate) async fn send_operator_state(
    tx: &Sender<Result<RtdlEvent, Status>>,
    node: &NodeEventContext,
    state: u32,
    reason: &str,
    runtime: &PlanRuntime,
) {
    let op = match node.node_kind {
        RTDL_SEQUENCE => "sequence",
        RTDL_PARALLEL => "parallel",
        _ => "operator",
    };
    let detail = format!(
        "RTDL {op} op_id={} {reason}: {}",
        node.op_id, node.description
    );
    runtime
        .record_op_state(&node.plan_id, &node.op_id, state)
        .await;
    let _ = tx.send(Ok(operator_node_state(node, state, detail))).await;
}

/// Select and publish one non-leaf operator's final state after execution and verification.
pub(crate) async fn send_operator_final(
    tx: &Sender<Result<RtdlEvent, Status>>,
    node: &NodeEventContext,
    failed: bool,
    cancelled: bool,
    runtime: &PlanRuntime,
) {
    let (state, reason) = if cancelled {
        (RtdlNodeStateEnum::Canceled as u32, "was canceled")
    } else if failed {
        (
            RtdlNodeStateEnum::Failed as u32,
            "failed because one or more child nodes failed",
        )
    } else {
        (
            RtdlNodeStateEnum::Succeeded as u32,
            "completed successfully",
        )
    };
    send_operator_state(tx, node, state, reason, runtime).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Preserve all node metadata in the wire event.
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
            RtdlNodeStateEnum::Running as u32,
            "operator detail".into(),
            None,
        );
        let ns = event.node_state.unwrap();

        assert_eq!(event.event_kind, RtdlEventEnum::NodeState as u32);
        assert_eq!(ns.plan_id, "p");
        assert_eq!(ns.node_index, 7);
        assert_eq!(ns.node_kind, 2);
        assert_eq!(ns.state, RtdlNodeStateEnum::Running as u32);
        assert_eq!(ns.operator_detail, "operator detail");
        assert!(ns.leaf_result.is_none());
        assert_eq!(ns.op_id, "op_1");
        assert_eq!(ns.description, "take a camera snapshot");
    }

    /// Operator events retain their identity and carry no leaf result.
    #[tokio::test]
    async fn operator_terminal_event_carries_node_identity() {
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        let node = NodeEventContext {
            plan_id: "p".to_string(),
            node_index: 0,
            node_kind: RTDL_SEQUENCE,
            op_id: "op_1".to_string(),
            description: "run the ordered checks".to_string(),
        };

        let runtime = PlanRuntime::default();
        send_operator_state(
            &tx,
            &node,
            RtdlNodeStateEnum::Succeeded as u32,
            "completed successfully",
            &runtime,
        )
        .await;

        let event = rx.recv().await.unwrap().unwrap();
        let ns = event.node_state.unwrap();
        assert_eq!(event.event_kind, RtdlEventEnum::NodeState as u32);
        assert_eq!(ns.op_id, "op_1");
        assert_eq!(ns.description, "run the ordered checks");
        assert_eq!(ns.state, RtdlNodeStateEnum::Succeeded as u32);
        assert!(ns.leaf_result.is_none());
        assert!(ns.operator_detail.contains("RTDL sequence op_id=op_1"));
    }
}
