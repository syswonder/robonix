// SPDX-License-Identifier: MulanPSL-2.0
//! Complete-plan validation and incremental RTDL input construction.
use crate::pb::executor::ExecuteStreamInput;
use crate::pb::pilot::{CapabilityCall, Plan, RtdlNode};
use std::collections::HashSet;

// Match capabilities/lib/executor/msg/ExecuteStreamInput.msg.
pub(crate) const EXECUTE_STREAM_START: u32 = 0;
pub(crate) const EXECUTE_STREAM_NODE: u32 = 1;
pub(crate) const EXECUTE_STREAM_CLOSE: u32 = 2;
pub(crate) const EXECUTE_STREAM_END: u32 = 3;

/// Owns the growing arena and all input invariants independently of execution.
#[derive(Debug, Default)]
pub(crate) struct PlanInput {
    /// Plan metadata and node arena populated by complete or incremental input.
    plan: Plan,
    /// Whether plan initialization is complete, allowing node input after START.
    started: bool,
    /// Whether input is complete or sealed, so further messages are rejected.
    ended: bool,
    /// Parent arena index for each node; the root has no parent.
    parents: Vec<Option<usize>>,
    /// Parent-first traversal; wire arena indices are preserved for complete plans.
    order: Vec<usize>,
    /// Whether each node is closed to new children; leaf nodes are always closed.
    closed: Vec<bool>,
    /// Accepted, trimmed operator IDs used to reject duplicate node identities.
    op_ids: HashSet<String>,
    /// Accepted capability call IDs used to reject duplicate calls within the plan.
    call_ids: HashSet<String>,
}

impl PlanInput {
    pub(crate) fn plan(&self) -> &Plan {
        &self.plan
    }

    pub(crate) fn order(&self) -> &[usize] {
        &self.order
    }

    pub(crate) fn parent(&self, index: usize) -> Option<usize> {
        self.parents[index]
    }

    pub(crate) fn is_closed(&self, index: usize) -> bool {
        self.closed[index]
    }

    pub(crate) fn ended(&self) -> bool {
        self.ended
    }

    /// Stop admitting input and close every operator when execution is canceled.
    pub(crate) fn seal(&mut self) {
        self.ended = true;
        self.closed.fill(true);
    }

    /// Load a complete tree with all operators closed, preserving its original arena indices.
    pub(crate) fn from_plan(mut plan: Plan) -> Result<Self, String> {
        // The RPC layer authenticates this one-shot credential before validation.
        plan.auth_session_token.clear();
        let count = plan.nodes.len();
        if plan.plan_id.trim().is_empty() || count == 0 || count > 256 {
            return Err("Plan requires a nonempty plan_id and 1 to 256 nodes".into());
        }
        let root = plan.root_index as usize;
        if root >= count {
            return Err("Plan.root_index is out of bounds".into());
        }
        let mut input = Self {
            started: true,
            ended: true,
            parents: vec![None; count],
            closed: vec![true; count],
            ..Self::default()
        };
        for (index, node) in plan.nodes.iter().enumerate() {
            if node.children.iter().any(|child| *child as usize >= count) {
                return Err(format!("node {index} child index is out of bounds"));
            }
            input.validate_node(index, node)?;
        }
        let mut visited = vec![false; count];
        let mut pending = vec![(root, None, 1)];
        while let Some((index, parent, depth)) = pending.pop() {
            if visited[index] {
                return Err(format!("cycle or repeated child at node {index}"));
            }
            if depth > 32 {
                return Err("RTDL nesting exceeds 32".into());
            }
            visited[index] = true;
            input.parents[index] = parent;
            input.order.push(index);
            for child in plan.nodes[index].children.iter().rev() {
                pending.push((*child as usize, Some(index), depth + 1));
            }
        }
        if input.order.len() != count {
            return Err("Plan contains nodes unreachable from root_index".into());
        }
        input.plan = plan;
        Ok(input)
    }

    /// Check shared node constraints, then reserve identities only after all checks pass.
    fn validate_node(&mut self, index: usize, node: &RtdlNode) -> Result<(), String> {
        let op_id = node.op_id.trim();
        if op_id.is_empty() || node.description.trim().is_empty() {
            return Err(format!(
                "node {index} requires nonempty op_id and description"
            ));
        }
        if self.op_ids.contains(op_id) {
            return Err(format!("node {index} has duplicate op_id '{op_id}'"));
        }
        match node.node_kind {
            0 | 1 if node.call.is_none() => {}
            2 => {
                if !node.children.is_empty() {
                    return Err(format!("do node {index} must not have children"));
                }
                let call = node
                    .call
                    .as_ref()
                    .ok_or_else(|| format!("do node {index} must contain a call"))?;
                validate_call(index, call)?;
                let args: serde_json::Value = serde_json::from_str(&call.args_json)
                    .map_err(|error| format!("do node {index} has invalid args_json: {error}"))?;
                if !args.is_object() || self.call_ids.contains(&call.call_id) {
                    return Err(format!(
                        "do node {index} requires object args and unique call_id"
                    ));
                }
            }
            _ => {
                return Err(format!(
                    "node {index} has invalid node kind or operator call payload"
                ));
            }
        }
        if let Some(call) = &node.call {
            self.call_ids.insert(call.call_id.clone());
        }
        self.op_ids.insert(op_id.to_owned());
        Ok(())
    }

    /// Validate a message before mutating the arena; callers register accepted metadata.
    pub(crate) fn accept(&mut self, msg: ExecuteStreamInput) -> Result<(), String> {
        if self.ended {
            return Err("message after END".into());
        }
        if !self.started {
            if msg.message_kind != EXECUTE_STREAM_START || msg.plan_id.trim().is_empty() {
                return Err("stream must begin with START and a nonempty plan_id".into());
            }
            self.plan.plan_id = msg.plan_id;
            self.plan.session_id = msg.session_id;
            self.plan.round = msg.round;
            self.started = true;
            return Ok(());
        }
        match msg.message_kind {
            EXECUTE_STREAM_NODE => self.node(msg),
            EXECUTE_STREAM_CLOSE => {
                let index = msg.node_index as usize;
                if index >= self.closed.len() || self.closed[index] {
                    return Err("CLOSE requires an open operator".into());
                }
                for child in 0..self.closed.len() {
                    if !self.closed[child] {
                        let mut parent = self.parents[child];
                        while let Some(p) = parent {
                            if p == index {
                                return Err("CLOSE must be innermost first".into());
                            }
                            parent = self.parents[p];
                        }
                    }
                }
                self.closed[index] = true;
                Ok(())
            }
            EXECUTE_STREAM_END => {
                if self.plan.nodes.is_empty() || self.closed.iter().any(|closed| !closed) {
                    return Err("END requires a root and all operators closed".into());
                }
                self.ended = true;
                Ok(())
            }
            _ => Err("unexpected message kind or repeated START".into()),
        }
    }

    /// Roll back an arrival rejected by the runtime's atomic cancellation/admission check.
    pub(crate) fn discard_last_node(&mut self) {
        let node = self.plan.nodes.pop().expect("arrived node");
        self.op_ids.remove(node.op_id.trim());
        if let Some(call) = node.call {
            self.call_ids.remove(&call.call_id);
        }
        self.closed.pop();
        self.order.pop();
        if let Some(parent) = self.parents.pop().flatten() {
            self.plan.nodes[parent].children.pop();
        }
    }

    /// Append a child only to an arrived, still-open operator and reject ambiguous identity.
    fn node(&mut self, msg: ExecuteStreamInput) -> Result<(), String> {
        let index = self.plan.nodes.len();
        if msg.node_index as usize != index || index >= 256 {
            return Err("NODE index must be consecutive and fewer than 256 nodes".into());
        }
        let parent = if index == 0 {
            if msg.has_parent {
                return Err("root must not have a parent".into());
            }
            None
        } else {
            let p = msg.parent_index as usize;
            if !msg.has_parent || p >= index || self.closed[p] {
                return Err("NODE requires an arrived open operator parent".into());
            }
            Some(p)
        };
        let mut depth = 1;
        let mut ancestor = parent;
        while let Some(p) = ancestor {
            depth += 1;
            ancestor = self.parents[p];
        }
        if depth > 32 {
            return Err("RTDL nesting exceeds 32".into());
        }
        let node = msg.node.ok_or("NODE requires a node")?;
        if !node.children.is_empty() {
            return Err("NODE requires empty children".into());
        }
        self.validate_node(index, &node)?;
        self.closed.push(node.node_kind == 2);
        self.parents.push(parent);
        self.order.push(index);
        if let Some(p) = parent {
            self.plan.nodes[p].children.push(index as u32);
        }
        self.plan.nodes.push(node);
        Ok(())
    }
}

/// Require complete capability identity before dispatch or incremental admission.
fn validate_call(node_index: usize, call: &CapabilityCall) -> Result<(), String> {
    if call.call_id.is_empty() {
        return Err(format!("do node {node_index} call_id must not be empty"));
    }
    if call.provider_id.is_empty() {
        return Err(format!(
            "do node {node_index} provider_id must not be empty"
        ));
    }
    if call.contract_id.is_empty() {
        return Err(format!(
            "do node {node_index} contract_id must not be empty"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::pilot::{CapabilityCall, RtdlNode};

    /// Construct canonical protocol messages for state transition tests.
    fn message(kind: u32, index: u32, parent: Option<u32>, node_kind: u32) -> ExecuteStreamInput {
        ExecuteStreamInput {
            message_kind: kind,
            plan_id: "p".into(),
            node_index: index,
            has_parent: parent.is_some(),
            parent_index: parent.unwrap_or_default(),
            node: if kind == EXECUTE_STREAM_NODE {
                Some(RtdlNode {
                    node_kind,
                    op_id: index.to_string(),
                    description: "step".into(),
                    call: if node_kind == 2 {
                        Some(CapabilityCall {
                            call_id: index.to_string(),
                            provider_id: "executor".into(),
                            contract_id: "test/run".into(),
                            args_json: "{}".into(),
                        })
                    } else {
                        None
                    },
                    ..Default::default()
                })
            } else {
                None
            },
            ..Default::default()
        }
    }

    /// Reconstruct arrival-order siblings and enforce the complete START/NODE/CLOSE/END lifecycle.
    #[test]
    fn valid_nested_protocol_and_reconstructed_children() {
        let mut input = PlanInput::default();
        for msg in [
            message(EXECUTE_STREAM_START, 0, None, 0),
            message(EXECUTE_STREAM_NODE, 0, None, 0),
            message(EXECUTE_STREAM_NODE, 1, Some(0), 1),
            message(EXECUTE_STREAM_NODE, 2, Some(1), 2),
            message(EXECUTE_STREAM_CLOSE, 1, None, 0),
            message(EXECUTE_STREAM_NODE, 3, Some(0), 2),
            message(EXECUTE_STREAM_CLOSE, 0, None, 0),
            message(EXECUTE_STREAM_END, 0, None, 0),
        ] {
            input.accept(msg).unwrap();
        }
        assert_eq!(input.plan.nodes[0].children, [1, 3]);
        assert_eq!(input.plan.nodes[1].children, [2]);
        assert!(input.ended);
        assert!(
            input
                .accept(message(EXECUTE_STREAM_END, 0, None, 0))
                .is_err()
        );
    }

    /// Every invalid ordering fails without corrupting the accepted prefix.
    #[test]
    fn invalid_protocol_transitions() {
        let mut input = PlanInput::default();
        assert!(
            input
                .accept(message(EXECUTE_STREAM_NODE, 0, None, 0))
                .is_err()
        );
        input
            .accept(message(EXECUTE_STREAM_START, 0, None, 0))
            .unwrap();
        assert!(
            input
                .accept(message(EXECUTE_STREAM_START, 0, None, 0))
                .is_err()
        );
        assert!(
            input
                .accept(message(EXECUTE_STREAM_END, 0, None, 0))
                .is_err()
        );
        assert!(
            input
                .accept(message(EXECUTE_STREAM_NODE, 1, None, 0))
                .is_err()
        );
        assert!(
            input
                .accept(message(EXECUTE_STREAM_NODE, 0, Some(0), 0))
                .is_err()
        );
        input
            .accept(message(EXECUTE_STREAM_NODE, 0, None, 0))
            .unwrap();
        assert!(
            input
                .accept(message(EXECUTE_STREAM_NODE, 1, None, 0))
                .is_err()
        );
        assert!(
            input
                .accept(message(EXECUTE_STREAM_NODE, 1, Some(2), 0))
                .is_err()
        );
        input
            .accept(message(EXECUTE_STREAM_NODE, 1, Some(0), 0))
            .unwrap();
        assert!(
            input
                .accept(message(EXECUTE_STREAM_CLOSE, 0, None, 0))
                .is_err()
        );
        input
            .accept(message(EXECUTE_STREAM_NODE, 2, Some(1), 2))
            .unwrap();
        assert!(
            input
                .accept(message(EXECUTE_STREAM_CLOSE, 2, None, 0))
                .is_err()
        );
        assert!(
            input
                .accept(message(EXECUTE_STREAM_NODE, 3, Some(2), 2))
                .is_err()
        );
        input
            .accept(message(EXECUTE_STREAM_CLOSE, 1, None, 0))
            .unwrap();
        assert!(
            input
                .accept(message(EXECUTE_STREAM_CLOSE, 1, None, 0))
                .is_err()
        );
        assert!(
            input
                .accept(message(EXECUTE_STREAM_NODE, 3, Some(1), 2))
                .is_err()
        );
        assert!(
            input
                .accept(message(EXECUTE_STREAM_END, 0, None, 0))
                .is_err()
        );
        input
            .accept(message(EXECUTE_STREAM_CLOSE, 0, None, 0))
            .unwrap();
        input
            .accept(message(EXECUTE_STREAM_END, 0, None, 0))
            .unwrap();
    }

    /// Calls need complete identity/object args, and wire children cannot bypass reconstruction.
    #[test]
    fn invalid_node_payloads() {
        let mut input = PlanInput::default();
        input
            .accept(message(EXECUTE_STREAM_START, 0, None, 0))
            .unwrap();
        for mutate in [0, 1, 2, 3, 4] {
            let mut msg = message(EXECUTE_STREAM_NODE, 0, None, 2);
            let node = msg.node.as_mut().unwrap();
            match mutate {
                0 => node.children.push(1),
                1 => node.call = None,
                2 => node.call.as_mut().unwrap().args_json = "[]".into(),
                3 => node.op_id.clear(),
                _ => node.node_kind = 9,
            }
            assert!(
                PlanInput::from_plan(Plan {
                    plan_id: "p".into(),
                    nodes: vec![node.clone()],
                    ..Default::default()
                })
                .is_err()
            );
            assert!(input.accept(msg).is_err());
            assert!(input.plan.nodes.is_empty());
        }
    }

    /// Complete plans retain child order and wire indices even when the root is not first.
    #[test]
    fn complete_plan_preserves_arena_indices() {
        let leaf = message(EXECUTE_STREAM_NODE, 0, None, 2).node.unwrap();
        let mut root = message(EXECUTE_STREAM_NODE, 1, None, 0).node.unwrap();
        root.children = vec![2, 0];
        let operator = message(EXECUTE_STREAM_NODE, 2, None, 1).node.unwrap();
        let input = PlanInput::from_plan(Plan {
            plan_id: "p".into(),
            root_index: 1,
            nodes: vec![leaf, root, operator],
            ..Default::default()
        })
        .unwrap();
        assert_eq!(input.order, [1, 2, 0]);
        assert_eq!(input.parents, [Some(1), None, Some(1)]);
        assert_eq!(input.plan.root_index, 1);
        assert_eq!(input.plan.nodes[1].children, [2, 0]);
    }

    /// Reject ambiguous complete arenas instead of executing a node twice or leaving it pending.
    #[test]
    fn complete_plan_requires_one_tree() {
        for children in [vec![1, 1], vec![]] {
            let mut root = message(EXECUTE_STREAM_NODE, 0, None, 0).node.unwrap();
            root.children = children;
            let leaf = message(EXECUTE_STREAM_NODE, 1, None, 2).node.unwrap();
            assert!(
                PlanInput::from_plan(Plan {
                    plan_id: "p".into(),
                    nodes: vec![root, leaf],
                    ..Default::default()
                })
                .is_err()
            );
        }
    }
}

#[cfg(test)]
mod complete_plan_tests {
    use super::*;
    const RTDL_SEQUENCE: u32 = 0;
    const RTDL_PARALLEL: u32 = 1;
    const RTDL_DO: u32 = 2;

    /// Construct a valid call so tree tests isolate the targeted structural error.
    fn call(id: &str) -> CapabilityCall {
        CapabilityCall {
            call_id: id.to_string(),
            provider_id: "provider".to_string(),
            contract_id: "robonix/test/cap".to_string(),
            args_json: "{}".to_string(),
        }
    }

    fn node(kind: u32, children: Vec<u32>, call: Option<CapabilityCall>) -> RtdlNode {
        node_with_identity("op", "test node", kind, children, call)
    }

    /// Build a node with explicit identity for uniqueness and metadata checks.
    fn node_with_identity(
        op_id: &str,
        description: &str,
        kind: u32,
        children: Vec<u32>,
        call: Option<CapabilityCall>,
    ) -> RtdlNode {
        RtdlNode {
            node_kind: kind,
            children,
            call,
            op_id: op_id.to_string(),
            description: description.to_string(),
        }
    }

    /// Wrap an arena in a complete plan with the requested root index.
    fn plan(nodes: Vec<RtdlNode>, root_index: u32) -> Plan {
        Plan {
            plan_id: "p".to_string(),
            session_id: "s".to_string(),
            round: 0,
            nodes,
            root_index,
            auth_session_token: String::new(),
        }
    }

    /// Accept a tree containing both operators and complete capability calls.
    #[test]
    fn validates_sequence_and_parallel_nodes() {
        let p = plan(
            vec![
                node_with_identity("op_1", "run sequence", RTDL_SEQUENCE, vec![1, 2], None),
                node_with_identity("op_2", "call first cap", RTDL_DO, vec![], Some(call("p:0"))),
                node_with_identity("op_3", "run parallel", RTDL_PARALLEL, vec![3, 4], None),
                node_with_identity(
                    "op_4",
                    "call second cap",
                    RTDL_DO,
                    vec![],
                    Some(call("p:1")),
                ),
                node_with_identity("op_5", "call third cap", RTDL_DO, vec![], Some(call("p:2"))),
            ],
            0,
        );
        PlanInput::from_plan(p).unwrap();
    }

    /// Require an identity for every operator as well as every leaf.
    #[test]
    fn rejects_empty_op_id() {
        let p = plan(
            vec![node_with_identity("", "root", RTDL_SEQUENCE, vec![], None)],
            0,
        );
        assert!(PlanInput::from_plan(p).unwrap_err().contains("op_id"));
    }

    /// Require descriptive metadata even for operators with no children.
    #[test]
    fn rejects_empty_description() {
        let p = plan(
            vec![node_with_identity("op_1", "", RTDL_SEQUENCE, vec![], None)],
            0,
        );
        assert!(PlanInput::from_plan(p).unwrap_err().contains("description"));
    }

    /// Reject conflicting node identities before scheduling the plan.
    #[test]
    fn rejects_duplicate_op_id() {
        let p = plan(
            vec![
                node_with_identity("op_1", "root", RTDL_SEQUENCE, vec![1], None),
                node_with_identity("op_1", "child", RTDL_DO, vec![], Some(call("p:0"))),
            ],
            0,
        );
        assert!(
            PlanInput::from_plan(p)
                .unwrap_err()
                .contains("duplicate op_id")
        );
    }

    #[test]
    fn rejects_invalid_root() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![], None)], 3);
        assert!(PlanInput::from_plan(p).unwrap_err().contains("root_index"));
    }

    /// Reject child references outside the submitted arena.
    #[test]
    fn rejects_out_of_bounds_child() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![9], None)], 0);
        assert!(
            PlanInput::from_plan(p)
                .unwrap_err()
                .contains("out of bounds")
        );
    }

    /// Reject a child path that revisits an ancestor.
    #[test]
    fn rejects_cycle() {
        let p = plan(
            vec![
                node_with_identity("op_1", "root", RTDL_SEQUENCE, vec![1], None),
                node_with_identity("op_2", "child", RTDL_PARALLEL, vec![0], None),
            ],
            0,
        );
        assert!(PlanInput::from_plan(p).unwrap_err().contains("cycle"));
    }

    /// Reject do nodes without a capability call payload.
    #[test]
    fn rejects_do_without_call() {
        let p = plan(vec![node(RTDL_DO, vec![], None)], 0);
        assert!(
            PlanInput::from_plan(p)
                .unwrap_err()
                .contains("must contain a call")
        );
    }
}
