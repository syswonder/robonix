// SPDX-License-Identifier: MulanPSL-2.0
//! Shared RTDL validation, scheduling, and cleanup for complete and incremental plans.
use crate::pb::executor::{ExecuteStreamInput, RtdlEvent};
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum as State;
use crate::pb::pilot::{Plan, RtdlNode};
use crate::rtdl_wire;
use crate::service::{self, ExecutorServiceImpl};
use std::collections::HashSet;
use tokio::sync::mpsc;
use tokio::task::JoinSet;
use tokio_stream::wrappers::ReceiverStream;
use tokio_stream::{Stream, StreamExt};
use tonic::Status;

// Match capabilities/lib/executor/msg/ExecuteStreamInput.msg.
const EXECUTE_STREAM_START: u32 = 0;
const EXECUTE_STREAM_NODE: u32 = 1;
const EXECUTE_STREAM_CLOSE: u32 = 2;
const EXECUTE_STREAM_END: u32 = 3;

/// Owns the growing arena and all input invariants independently of execution.
#[derive(Debug, Default)]
pub(crate) struct Input {
    plan: Plan,
    started: bool,
    ended: bool,
    parents: Vec<Option<usize>>,
    /// Parent-first traversal; wire arena indices are preserved for complete plans.
    order: Vec<usize>,
    closed: Vec<bool>,
    op_ids: HashSet<String>,
    call_ids: HashSet<String>,
}

impl Input {
    /// Load a complete tree with all operators closed, preserving its original arena indices.
    pub(crate) fn from_plan(plan: Plan) -> Result<Self, String> {
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
                service::validate_call(index, call)?;
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
    fn discard_last_node(&mut self) {
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

/// Immediate outcomes release sequence siblings; final outcomes include overlap verification.
#[derive(Default)]
struct Execution {
    started: bool,
    immediate: Option<bool>,
    final_failed: Option<bool>,
    verifying: bool,
    canceled: bool,
}

enum Completion {
    Call(usize, service::NodeExecution),
    Verified(usize, bool),
}

pub(crate) struct Actor {
    input: Input,
    execution: Vec<Execution>,
    svc: ExecutorServiceImpl,
    tx: mpsc::Sender<Result<RtdlEvent, Status>>,
    tasks: JoinSet<Completion>,
    cancelled: bool,
    error: Option<Status>,
}

impl Actor {
    /// Reserve active state before returning headers and own all work until cleanup finishes.
    pub(crate) async fn start(
        svc: ExecutorServiceImpl,
        input: Input,
        stream: impl Stream<Item = Result<ExecuteStreamInput, Status>> + Unpin + Send + 'static,
    ) -> Result<ReceiverStream<Result<RtdlEvent, Status>>, Status> {
        svc.runtime
            .register_plan(&input.plan.plan_id)
            .await
            .map_err(Status::invalid_argument)?;
        let (tx, rx) = mpsc::channel(1024);
        let actor = Self {
            execution: (0..input.plan.nodes.len())
                .map(|_| Execution::default())
                .collect(),
            input,
            svc,
            tx,
            tasks: JoinSet::new(),
            cancelled: false,
            error: None,
        };
        tokio::spawn(actor.run(stream));
        Ok(ReceiverStream::new(rx))
    }

    /// Seal input on cancellation or transport loss and retain ownership until all work drains.
    async fn cancel(&mut self) {
        if self.cancelled {
            return;
        }
        self.cancelled = true;
        self.input.ended = true;
        self.input.closed.fill(true);
        self.svc
            .runtime
            .cancel_plan_control(
                &self.input.plan.plan_id,
                0,
                &self.svc.provider_id,
                &mut self.svc.atlas.clone(),
            )
            .await;
    }

    /// Return whether the parent permits execution and whether an earlier sequence child failed.
    fn permitted(&self, index: usize) -> (bool, bool) {
        let Some(parent) = self.input.parents[index] else {
            return (true, false);
        };
        let p = &self.execution[parent];
        if p.canceled {
            return (false, true);
        }
        if !p.started {
            return (false, false);
        }
        if self.input.plan.nodes[parent].node_kind == 1 {
            return (true, false);
        }
        for sibling in &self.input.plan.nodes[parent].children {
            if *sibling as usize == index {
                break;
            }
            match self.execution[*sibling as usize].immediate {
                Some(true) => return (false, true),
                None => return (false, false),
                Some(false) => {}
            }
        }
        (true, false)
    }

    /// Advance runnable nodes and closed operators; report transitions that may unlock more work.
    async fn schedule(&mut self) -> bool {
        let mut progressed = false;
        for position in 0..self.input.order.len() {
            let index = self.input.order[position];
            if self.execution[index].started {
                continue;
            }
            let (ready, blocked) = self.permitted(index);
            if !ready && !blocked && !self.cancelled {
                continue;
            }
            progressed = true;
            let ctx = service::node_event_context(&self.input.plan, index);
            if self.cancelled || blocked {
                let leaf = ctx.node_kind == 2;
                self.execution[index] = Execution {
                    started: true,
                    immediate: leaf.then_some(true),
                    final_failed: leaf.then_some(true),
                    canceled: true,
                    verifying: false,
                };
                if !leaf {
                    continue;
                }
                self.svc
                    .runtime
                    .record_op_state(&ctx.plan_id, &ctx.op_id, State::Canceled as u32)
                    .await;
                let _ = self
                    .tx
                    .send(Ok(rtdl_wire::node_state(
                        &ctx,
                        State::Canceled as u32,
                        "canceled before execution".into(),
                        None,
                    )))
                    .await;
                continue;
            }
            self.execution[index].started = true;
            let node = &self.input.plan.nodes[index];
            if let Some(call) = node.call.clone() {
                let svc = self.svc.clone();
                let tx = self.tx.clone();
                self.tasks.spawn(async move {
                    // Cancellation can race with task spawning. Check again before dispatch.
                    if svc.runtime.is_cancelled(&ctx.plan_id).await {
                        svc.runtime
                            .record_op_state(&ctx.plan_id, &ctx.op_id, State::Canceled as u32)
                            .await;
                        let _ = tx
                            .send(Ok(rtdl_wire::node_state(
                                &ctx,
                                State::Canceled as u32,
                                "canceled before execution".into(),
                                None,
                            )))
                            .await;
                        return Completion::Call(
                            index,
                            service::NodeExecution {
                                failed: true,
                                verifier: None,
                            },
                        );
                    }
                    let result = service::execute_call(
                        &call,
                        ctx,
                        tx,
                        svc.atlas,
                        svc.provider_id,
                        svc.runtime,
                        svc.verification,
                    )
                    .await;
                    Completion::Call(index, result)
                });
            } else {
                service::send_operator_state(
                    &self.tx,
                    &ctx,
                    State::Running as u32,
                    "running",
                    &self.svc.runtime,
                )
                .await;
            }
        }
        for position in (0..self.input.order.len()).rev() {
            let index = self.input.order[position];
            let node = &self.input.plan.nodes[index];
            if node.node_kind == 2
                || !self.execution[index].started
                || self.execution[index].final_failed.is_some()
                || !self.input.closed[index]
            {
                continue;
            }
            let children = &node.children;
            if children
                .iter()
                .all(|c| self.execution[*c as usize].immediate.is_some())
            {
                let failed = self.cancelled
                    || self.execution[index].canceled
                    || children
                        .iter()
                        .any(|c| self.execution[*c as usize].immediate == Some(true));
                progressed |= self.execution[index].immediate != Some(failed);
                self.execution[index].immediate = Some(failed);
            }
            let ctx = service::node_event_context(&self.input.plan, index);
            if children
                .iter()
                .all(|c| self.execution[*c as usize].final_failed.is_some())
            {
                let failed = self.cancelled
                    || self.execution[index].canceled
                    || children
                        .iter()
                        .any(|c| self.execution[*c as usize].final_failed == Some(true));
                self.execution[index].final_failed = Some(failed);
                progressed = true;
                service::send_operator_final(
                    &self.tx,
                    &ctx,
                    failed,
                    self.cancelled || self.execution[index].canceled,
                    &self.svc.runtime,
                )
                .await;
                if self
                    .svc
                    .runtime
                    .is_cancelled(&self.input.plan.plan_id)
                    .await
                {
                    self.cancel().await;
                }
            } else if self.execution[index].immediate.is_some() && !self.execution[index].verifying
            {
                self.execution[index].verifying = true;
                progressed = true;
                service::send_operator_state(
                    &self.tx,
                    &ctx,
                    State::Verifying as u32,
                    "waiting for descendant verification",
                    &self.svc.runtime,
                )
                .await;
            }
        }
        progressed
    }

    /// Consume input and task results concurrently, cleaning up on EOF, cancellation, and errors.
    async fn run(
        mut self,
        mut stream: impl Stream<Item = Result<ExecuteStreamInput, Status>> + Unpin,
    ) {
        self.svc.runtime.record_plan_ops(&self.input.plan).await;
        let _ = self
            .tx
            .send(Ok(rtdl_wire::plan_started(self.input.plan.plan_id.clone())))
            .await;
        let mut input_eof = false;
        // Poll external cancellation while idle; locally runnable work never waits for this tick.
        let mut tick = tokio::time::interval(std::time::Duration::from_millis(10));
        loop {
            if self
                .svc
                .runtime
                .is_cancelled(&self.input.plan.plan_id)
                .await
            {
                self.cancel().await;
            }
            let progressed = self.schedule().await;
            if !self.cancelled
                && self
                    .svc
                    .runtime
                    .is_cancelled(&self.input.plan.plan_id)
                    .await
            {
                self.cancel().await;
                continue;
            }
            if self.input.ended
                && self.tasks.is_empty()
                && self.execution.iter().all(|e| e.final_failed.is_some())
            {
                break;
            }
            if progressed {
                continue;
            }
            tokio::select! {
                biased;
                _ = self.tx.closed(), if !self.cancelled => { self.cancel().await; }
                result = self.tasks.join_next(), if !self.tasks.is_empty() => {
                    match result {
                        Some(Ok(Completion::Call(index, outcome))) => {
                            self.execution[index].immediate = Some(outcome.failed);
                            if let Some(verifier) = outcome.verifier {
                                self.tasks.spawn(async move { Completion::Verified(index, service::wait_for_verifier(verifier).await) });
                            } else {
                                self.execution[index].final_failed = Some(outcome.failed);
                            }
                        }
                        Some(Ok(Completion::Verified(index, failed))) => { self.execution[index].final_failed = Some(failed || self.execution[index].immediate == Some(true)); }
                        Some(Err(error)) => {
                            self.error = Some(Status::internal(format!("execution task failed: {error}")));
                            self.cancel().await;
                            // A panicking task has no index; mark unresolved leaves after draining survivors.
                        }
                        None => {},
                    }
                }
                incoming = stream.next(), if !input_eof && !self.cancelled => {
                    match incoming {
                        Some(Ok(msg)) => {
                            let is_node = msg.message_kind == EXECUTE_STREAM_NODE;
                            match self.input.accept(msg) {
                                Ok(()) => {
                                    if is_node {
                                        let node = self.input.plan.nodes.last().expect("accepted NODE");
                                        if self.svc.runtime.record_stream_node(&self.input.plan.plan_id, node).await {
                                            self.execution.push(Execution::default());
                                        } else {
                                            self.input.discard_last_node();
                                            self.cancel().await;
                                        }
                                    }
                                }
                                Err(error) => { self.error = Some(Status::invalid_argument(error)); self.cancel().await; }
                            }
                        }
                        None => {
                            input_eof = true;
                            if !self.input.ended { self.error = Some(Status::invalid_argument("EOF before END")); self.cancel().await; }
                        }
                        Some(Err(error)) => { self.error = Some(error); self.cancel().await; }
                    }
                }
                _ = tick.tick() => {},
            }
            if self.error.is_some() && self.tasks.is_empty() {
                for e in &mut self.execution {
                    if e.started && e.immediate.is_none() {
                        e.immediate = Some(true);
                        e.final_failed = Some(true);
                    }
                }
            }
        }
        self.cancelled |= self
            .svc
            .runtime
            .complete_plan(&self.input.plan.plan_id)
            .await;
        if let Some(error) = self.error {
            let _ = self.tx.send(Err(error)).await;
        } else {
            let failed = self.cancelled
                || self
                    .execution
                    .get(self.input.plan.root_index as usize)
                    .is_some_and(|e| e.final_failed == Some(true));
            let _ = self
                .tx
                .send(Ok(rtdl_wire::plan_complete(
                    self.input.plan.plan_id,
                    failed,
                )))
                .await;
        }
    }
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
        let mut input = Input::default();
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
        let mut input = Input::default();
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
        let mut input = Input::default();
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
                Input::from_plan(Plan {
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
        let input = Input::from_plan(Plan {
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
                Input::from_plan(Plan {
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
mod scheduling_tests {
    use super::*;
    use crate::config::VerificationRule;
    use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
    use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecuteServer;
    use crate::pb::contracts::robonix_system_executor_execute_stream_client::RobonixSystemExecutorExecuteStreamClient;
    use crate::pb::contracts::robonix_system_executor_execute_stream_server::RobonixSystemExecutorExecuteStreamServer;
    use crate::pb::pilot::{CapabilityCall, RtdlNode};
    use crate::verification::VerificationPolicy;
    use robonix_atlas::{
        client::AtlasClient,
        service::{AtlasRegistry, serve_atlas},
    };
    use std::{sync::Arc, time::Duration};

    /// Both RPCs preserve sequence advancement and final failures in either verification mode.
    #[tokio::test]
    async fn overlap_preserves_immediate_sequence_result() {
        for (overlap, incremental) in [(false, false), (false, true), (true, false), (true, true)] {
            let atlas_addr = std::net::TcpListener::bind("127.0.0.1:0")
                .unwrap()
                .local_addr()
                .unwrap();
            let atlas_task =
                tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
            let atlas = AtlasClient::connect_with_retry(
                format!("http://{atlas_addr}"),
                50,
                Duration::from_millis(10),
            )
            .await
            .unwrap();
            let contract = "robonix/system/executor/builtin/get_all_plans";
            let svc = ExecutorServiceImpl::new(
                atlas,
                "executor".into(),
                Arc::new(VerificationPolicy::new(
                    overlap,
                    vec![VerificationRule {
                        target_contract_id: contract.into(),
                        target_provider_id: None,
                        verifier_provider_id: "missing_verifier".into(),
                        verifier_args: serde_json::json!({}),
                    }],
                )),
            );
            let addr = std::net::TcpListener::bind("127.0.0.1:0")
                .unwrap()
                .local_addr()
                .unwrap();
            let server = tokio::spawn(
                tonic::transport::Server::builder()
                    .add_service(RobonixSystemExecutorExecuteServer::new(svc.clone()))
                    .add_service(RobonixSystemExecutorExecuteStreamServer::new(svc.clone()))
                    .serve(addr),
            );
            let channel = AtlasClient::connect_with_retry(
                format!("http://{addr}"),
                50,
                Duration::from_millis(10),
            )
            .await
            .unwrap();
            drop(channel);
            let nodes = vec![
                RtdlNode {
                    node_kind: 0,
                    children: vec![1, 2],
                    op_id: "root".into(),
                    description: "sequence".into(),
                    ..Default::default()
                },
                RtdlNode {
                    node_kind: 2,
                    op_id: "verified".into(),
                    description: "verify".into(),
                    call: Some(CapabilityCall {
                        call_id: "c1".into(),
                        provider_id: "executor".into(),
                        contract_id: contract.into(),
                        args_json: "{}".into(),
                    }),
                    ..Default::default()
                },
                RtdlNode {
                    node_kind: 2,
                    op_id: "next".into(),
                    description: "next".into(),
                    call: Some(CapabilityCall {
                        call_id: "c2".into(),
                        provider_id: "executor".into(),
                        contract_id: "robonix/system/executor/builtin/list_dir".into(),
                        args_json: r#"{"path":"."}"#.into(),
                    }),
                    ..Default::default()
                },
            ];
            let mut response = if incremental {
                let mut client =
                    RobonixSystemExecutorExecuteStreamClient::connect(format!("http://{addr}"))
                        .await
                        .unwrap();
                let mut messages = vec![ExecuteStreamInput {
                    message_kind: EXECUTE_STREAM_START,
                    plan_id: "verify".into(),
                    ..Default::default()
                }];
                for (index, mut node) in nodes.into_iter().enumerate() {
                    node.children.clear();
                    messages.push(ExecuteStreamInput {
                        message_kind: EXECUTE_STREAM_NODE,
                        node_index: index as u32,
                        has_parent: index != 0,
                        node: Some(node),
                        ..Default::default()
                    });
                }
                messages.push(ExecuteStreamInput {
                    message_kind: EXECUTE_STREAM_CLOSE,
                    ..Default::default()
                });
                messages.push(ExecuteStreamInput {
                    message_kind: EXECUTE_STREAM_END,
                    ..Default::default()
                });
                client
                    .execute_stream(tokio_stream::iter(messages))
                    .await
                    .unwrap()
                    .into_inner()
            } else {
                let mut client =
                    RobonixSystemExecutorExecuteClient::connect(format!("http://{addr}"))
                        .await
                        .unwrap();
                client
                    .execute(Plan {
                        plan_id: "verify".into(),
                        nodes,
                        ..Default::default()
                    })
                    .await
                    .unwrap()
                    .into_inner()
            };
            let events = tokio::time::timeout(Duration::from_secs(5), async {
                let mut events = Vec::new();
                while let Some(event) = response.message().await.unwrap() {
                    events.push(event);
                }
                events
            })
            .await
            .unwrap();
            assert_eq!(
                events.iter().filter(|e| e.plan_complete.is_some()).count(),
                1
            );
            assert!(
                events
                    .last()
                    .unwrap()
                    .plan_complete
                    .as_ref()
                    .unwrap()
                    .any_failed
            );
            let states: Vec<_> = events
                .iter()
                .filter_map(|e| e.node_state.as_ref())
                .map(|s| (s.node_index, s.state))
                .collect();
            assert!(states.contains(&(1, State::Failed as u32)));
            assert!(states.contains(&(
                2,
                if overlap {
                    State::Succeeded
                } else {
                    State::Canceled
                } as u32
            )));
            assert_eq!(states.last(), Some(&(0, State::Failed as u32)));
            assert_eq!(
                serde_json::from_str::<serde_json::Value>(&svc.runtime.active_plans_json().await)
                    .unwrap()["count"],
                0
            );
            server.abort();
            atlas_task.abort();
        }
    }
}
