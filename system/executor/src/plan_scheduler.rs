// SPDX-License-Identifier: MulanPSL-2.0
//! Schedule RTDL sequence/parallel nodes and drain calls and verifiers before cleanup.
use crate::active_plans::PlanRuntime;
use crate::capability_execution;
use crate::pb::executor::{ExecuteStreamInput, RtdlEvent};
use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum as State;
use crate::plan_input::{EXECUTE_STREAM_NODE, PlanInput};
use crate::rtdl_events;
use crate::verification::VerificationPolicy;
use robonix_atlas::client::AtlasClient;
use robonix_scribe::{info, warn};
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio::task::JoinSet;
use tokio_stream::wrappers::ReceiverStream;
use tokio_stream::{Stream, StreamExt};
use tonic::Status;

/// Scheduling progress for one node, separating immediate and verified outcomes.
#[derive(Default)]
struct Execution {
    /// Whether the node has been started or marked canceled before execution.
    started: bool,
    /// Outcome used to release sequence siblings: `None` is pending, `Some(true)` is failure.
    immediate: Option<bool>,
    /// Outcome including background verification: `None` is pending, `Some(true)` is failure.
    final_failed: Option<bool>,
    /// Whether an operator has emitted VERIFYING while awaiting descendant verification.
    verifying: bool,
    /// Whether the scheduler skipped this node due to cancellation or a failed sibling.
    canceled: bool,
}

enum Completion {
    Call(usize, capability_execution::NodeExecution),
    Verified(usize, bool),
    Controlled(usize, bool),
}

/// Execute a completed control subtree, including repeated visits to its body.
#[derive(Clone)]
struct ControlExecution {
    /// Immutable arena after the control subtree has closed.
    plan: Arc<crate::pb::pilot::Plan>,
    /// Channel shared with the surrounding streamed plan.
    tx: mpsc::Sender<Result<RtdlEvent, Status>>,
    /// Atlas connection for capability calls.
    atlas: AtlasClient,
    /// Consumer identity for capability calls.
    provider_id: String,
    /// Plan context and cancellation state.
    runtime: PlanRuntime,
    /// Verification policy for descendants.
    verification: Arc<VerificationPolicy>,
}

impl ControlExecution {
    fn run(self, index: usize) -> Pin<Box<dyn Future<Output = bool> + Send>> {
        Box::pin(async move {
            let node = &self.plan.nodes[index];
            let ctx = rtdl_events::node_event_context(&self.plan, index);
            if self.runtime.is_cancelled(&ctx.plan_id).await {
                self.runtime
                    .record_op_state(&ctx.plan_id, &ctx.op_id, State::Canceled as u32)
                    .await;
                let _ = self
                    .tx
                    .send(Ok(rtdl_events::node_state(
                        &ctx,
                        State::Canceled as u32,
                        "canceled before execution".into(),
                        None,
                    )))
                    .await;
                return true;
            }
            if node.node_kind == 2 {
                let mut call = node.call.clone().expect("validated do call");
                let args: serde_json::Value =
                    serde_json::from_str(&call.args_json).expect("validated args");
                let vars = self.runtime.context(&ctx.plan_id).await.snapshot().await;
                match crate::expression::eval_value(&args, &vars) {
                    Ok(value) => call.args_json = value.to_string(),
                    Err(error) => {
                        self.fail(&ctx, &format!("argument evaluation: {error}"))
                            .await;
                        return true;
                    }
                }
                let outcome = capability_execution::execute_call(
                    &call,
                    &node.bind,
                    ctx,
                    self.tx.clone(),
                    self.atlas.clone(),
                    self.provider_id.clone(),
                    self.runtime.clone(),
                    self.verification.clone(),
                )
                .await;
                return outcome.failed
                    || match outcome.verifier {
                        Some(verifier) => capability_execution::wait_for_verifier(verifier).await,
                        None => false,
                    };
            }
            rtdl_events::send_operator_state(
                &self.tx,
                &ctx,
                State::Running as u32,
                "running",
                &self.runtime,
            )
            .await;
            let (failed, reported) = match node.node_kind {
                0 => {
                    let mut failed = false;
                    for &child in &node.children {
                        if self.clone().run(child as usize).await {
                            failed = true;
                            break;
                        }
                    }
                    (failed, false)
                }
                1 => {
                    let mut tasks = JoinSet::new();
                    for &child in &node.children {
                        tasks.spawn(self.clone().run(child as usize));
                    }
                    let mut failed = false;
                    while let Some(result) = tasks.join_next().await {
                        failed |= result.unwrap_or(true);
                    }
                    (failed, false)
                }
                3..=5 => self.clone().run_condition_or_loop(index, &ctx).await,
                _ => (true, false),
            };
            let cancelled = self.runtime.is_cancelled(&ctx.plan_id).await;
            if !reported {
                rtdl_events::send_operator_final(
                    &self.tx,
                    &ctx,
                    failed || cancelled,
                    cancelled,
                    &self.runtime,
                )
                .await;
            }
            failed || cancelled
        })
    }

    async fn fail(&self, ctx: &rtdl_events::NodeEventContext, detail: &str) {
        warn!(
            "[executor/rtdl] plan_id={} node={} op_id={} failed: {}",
            ctx.plan_id, ctx.node_index, ctx.op_id, detail
        );
        self.runtime
            .record_op_state(&ctx.plan_id, &ctx.op_id, State::Failed as u32)
            .await;
        let _ = self
            .tx
            .send(Ok(rtdl_events::node_state(
                ctx,
                State::Failed as u32,
                detail.into(),
                None,
            )))
            .await;
    }

    async fn run_condition_or_loop(
        &self,
        index: usize,
        ctx: &rtdl_events::NodeEventContext,
    ) -> (bool, bool) {
        let node = &self.plan.nodes[index];
        let kind = match node.node_kind {
            3 => "if",
            4 => "repeat",
            _ => "while",
        };
        let limit = match node.node_kind {
            3 => 1,
            4 => node.repeat_count,
            _ => node.max_iterations,
        };
        let condition: serde_json::Value = if node.node_kind == 4 {
            serde_json::Value::Bool(true)
        } else {
            serde_json::from_str(&node.condition_json).expect("validated condition")
        };
        info!(
            "[executor/rtdl] plan_id={} node={} op_id={} {kind} start limit={limit}",
            ctx.plan_id, index, ctx.op_id
        );
        let mut iterations = 0;
        loop {
            if self.runtime.is_cancelled(&ctx.plan_id).await {
                return (true, false);
            }
            let selected = if node.node_kind == 4 {
                if iterations < limit { Some(0) } else { None }
            } else {
                let vars = self.runtime.context(&ctx.plan_id).await.snapshot().await;
                match crate::expression::eval_condi(&condition, &vars) {
                    Ok(value) => {
                        let branch = if value {
                            Some(0)
                        } else if node.node_kind == 3 && node.children.len() == 2 {
                            Some(1)
                        } else {
                            None
                        };
                        info!(
                            "[executor/rtdl] plan_id={} node={} op_id={} {kind} condition={value} completed_iterations={iterations} selected={}",
                            ctx.plan_id,
                            index,
                            ctx.op_id,
                            match branch {
                                Some(0) if node.node_kind == 3 => "then",
                                Some(1) => "else",
                                Some(_) => "body",
                                None => "none",
                            }
                        );
                        branch
                    }
                    Err(error) => {
                        self.fail(ctx, &format!("condition evaluation: {error}"))
                            .await;
                        return (true, true);
                    }
                }
            };
            let Some(selected) = selected else {
                info!(
                    "[executor/rtdl] plan_id={} node={} op_id={} {kind} exit completed_iterations={iterations} reason={}",
                    ctx.plan_id,
                    index,
                    ctx.op_id,
                    if node.node_kind == 4 {
                        "count completed"
                    } else {
                        "condition false"
                    }
                );
                return (false, false);
            };
            if iterations >= limit {
                self.fail(ctx, "while condition remains true at max_iterations")
                    .await;
                return (true, true);
            }
            info!(
                "[executor/rtdl] plan_id={} node={} op_id={} {kind} iteration={} limit={limit} child={} start",
                ctx.plan_id,
                index,
                ctx.op_id,
                iterations + 1,
                node.children[selected]
            );
            let failed = self.clone().run(node.children[selected] as usize).await;
            iterations += 1;
            info!(
                "[executor/rtdl] plan_id={} node={} op_id={} {kind} iteration={iterations} child={} complete failed={failed}",
                ctx.plan_id, index, ctx.op_id, node.children[selected]
            );
            if failed || node.node_kind == 3 {
                info!(
                    "[executor/rtdl] plan_id={} node={} op_id={} {kind} exit completed_iterations={iterations} reason={}",
                    ctx.plan_id,
                    index,
                    ctx.op_id,
                    if failed {
                        "child failed"
                    } else {
                        "branch completed"
                    }
                );
                return (failed, false);
            }
        }
    }
}

/// Own one plan's scheduling state and tasks until all work has been drained.
pub(crate) struct PlanScheduler {
    /// Validated plan input and the closure state of its operators.
    input: PlanInput,
    /// Scheduling progress indexed by the corresponding node's arena position.
    execution: Vec<Execution>,
    /// Atlas connection cloned for capability dispatch and cancellation.
    atlas: AtlasClient,
    /// Executor's own provider ID used for consumer identity and builtin dispatch.
    provider_id: String,
    /// Shared active-plan state used to publish progress and observe cancellation.
    runtime: PlanRuntime,
    /// Rules and timing policy for verification of successful capability results.
    verification: Arc<VerificationPolicy>,
    /// Sends execution events or a terminal error to the requesting RPC stream.
    tx: mpsc::Sender<Result<RtdlEvent, Status>>,
    /// In-flight capability calls and verification waits drained before plan cleanup.
    tasks: JoinSet<Completion>,
    /// Whether this scheduler has begun cancellation and sealed further input.
    cancelled: bool,
    /// Input, transport, or task error retained for reporting after cleanup.
    error: Option<Status>,
}

impl PlanScheduler {
    /// Reserve active state before returning headers and own all work until cleanup finishes.
    pub(crate) async fn start(
        atlas: AtlasClient,
        provider_id: String,
        runtime: PlanRuntime,
        verification: Arc<VerificationPolicy>,
        input: PlanInput,
        stream: impl Stream<Item = Result<ExecuteStreamInput, Status>> + Unpin + Send + 'static,
    ) -> Result<ReceiverStream<Result<RtdlEvent, Status>>, Status> {
        runtime
            .register_plan(&input.plan().plan_id)
            .await
            .map_err(Status::invalid_argument)?;
        let (tx, rx) = mpsc::channel(1024);
        let scheduler = Self {
            execution: (0..input.plan().nodes.len())
                .map(|_| Execution::default())
                .collect(),
            input,
            atlas,
            provider_id,
            runtime,
            verification,
            tx,
            tasks: JoinSet::new(),
            cancelled: false,
            error: None,
        };
        tokio::spawn(scheduler.run(stream));
        Ok(ReceiverStream::new(rx))
    }

    /// Seal input on cancellation or transport loss and retain ownership until all work drains.
    async fn cancel(&mut self) {
        if self.cancelled {
            return;
        }
        self.cancelled = true;
        self.input.seal();
        self.runtime
            .cancel_plan_control(
                &self.input.plan().plan_id,
                0,
                &self.provider_id,
                &mut self.atlas.clone(),
            )
            .await;
    }

    /// Return whether the parent permits execution and whether an earlier sequence child failed.
    fn permitted(&self, index: usize) -> (bool, bool) {
        let Some(parent) = self.input.parent(index) else {
            return (true, false);
        };
        let p = &self.execution[parent];
        if p.canceled {
            return (false, true);
        }
        if !p.started {
            return (false, false);
        }
        if self.input.plan().nodes[parent].node_kind == 1 {
            return (true, false);
        }
        for sibling in &self.input.plan().nodes[parent].children {
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

    /// Descendants of a control node are owned by its recursive execution task.
    fn inside_control(&self, index: usize) -> bool {
        let mut parent = self.input.parent(index);
        while let Some(p) = parent {
            if matches!(self.input.plan().nodes[p].node_kind, 3..=5) {
                return true;
            }
            parent = self.input.parent(p);
        }
        false
    }

    fn is_descendant_of(&self, index: usize, ancestor: usize) -> bool {
        let mut parent = self.input.parent(index);
        while let Some(p) = parent {
            if p == ancestor {
                return true;
            }
            parent = self.input.parent(p);
        }
        false
    }

    /// Advance runnable nodes and closed operators; report transitions that may unlock more work.
    async fn schedule(&mut self) -> bool {
        let mut progressed = false;
        for position in 0..self.input.order().len() {
            let index = self.input.order()[position];
            if self.execution[index].started || self.inside_control(index) {
                continue;
            }
            if matches!(self.input.plan().nodes[index].node_kind, 3..=5)
                && !self.input.is_closed(index)
            {
                continue;
            }
            let (ready, blocked) = self.permitted(index);
            if !ready && !blocked && !self.cancelled {
                continue;
            }
            progressed = true;
            let ctx = rtdl_events::node_event_context(self.input.plan(), index);
            if self.cancelled || blocked {
                let leaf = ctx.node_kind == 2;
                let controlled = matches!(ctx.node_kind, 3..=5);
                self.execution[index] = Execution {
                    started: true,
                    immediate: (leaf || controlled).then_some(true),
                    final_failed: (leaf || controlled).then_some(true),
                    canceled: true,
                    verifying: false,
                };
                if controlled {
                    for descendant in 0..self.execution.len() {
                        if self.is_descendant_of(descendant, index) {
                            self.execution[descendant].immediate = Some(true);
                            self.execution[descendant].final_failed = Some(true);
                        }
                    }
                }
                if !leaf && !controlled {
                    continue;
                }
                self.runtime
                    .record_op_state(&ctx.plan_id, &ctx.op_id, State::Canceled as u32)
                    .await;
                let _ = self
                    .tx
                    .send(Ok(rtdl_events::node_state(
                        &ctx,
                        State::Canceled as u32,
                        "canceled before execution".into(),
                        None,
                    )))
                    .await;
                continue;
            }
            self.execution[index].started = true;
            let node = &self.input.plan().nodes[index];
            if matches!(node.node_kind, 3..=5) {
                let control = ControlExecution {
                    plan: Arc::new(self.input.plan().clone()),
                    tx: self.tx.clone(),
                    atlas: self.atlas.clone(),
                    provider_id: self.provider_id.clone(),
                    runtime: self.runtime.clone(),
                    verification: self.verification.clone(),
                };
                self.tasks
                    .spawn(async move { Completion::Controlled(index, control.run(index).await) });
            } else if let Some(call) = node.call.clone() {
                let bind = node.bind.clone();
                let atlas = self.atlas.clone();
                let provider_id = self.provider_id.clone();
                let runtime = self.runtime.clone();
                let verification = self.verification.clone();
                let tx = self.tx.clone();
                self.tasks.spawn(async move {
                    // Cancellation can race with task spawning. Check again before dispatch.
                    if runtime.is_cancelled(&ctx.plan_id).await {
                        runtime
                            .record_op_state(&ctx.plan_id, &ctx.op_id, State::Canceled as u32)
                            .await;
                        let _ = tx
                            .send(Ok(rtdl_events::node_state(
                                &ctx,
                                State::Canceled as u32,
                                "canceled before execution".into(),
                                None,
                            )))
                            .await;
                        return Completion::Call(
                            index,
                            capability_execution::NodeExecution {
                                failed: true,
                                verifier: None,
                            },
                        );
                    }
                    let mut call = call;
                    let args: serde_json::Value =
                        serde_json::from_str(&call.args_json).expect("validated args");
                    let vars = runtime.context(&ctx.plan_id).await.snapshot().await;
                    match crate::expression::eval_value(&args, &vars) {
                        Ok(value) => call.args_json = value.to_string(),
                        Err(error) => {
                            runtime
                                .record_op_state(&ctx.plan_id, &ctx.op_id, State::Failed as u32)
                                .await;
                            let _ = tx
                                .send(Ok(rtdl_events::node_state(
                                    &ctx,
                                    State::Failed as u32,
                                    format!("argument evaluation: {error}"),
                                    None,
                                )))
                                .await;
                            return Completion::Call(
                                index,
                                capability_execution::NodeExecution {
                                    failed: true,
                                    verifier: None,
                                },
                            );
                        }
                    }
                    let result = capability_execution::execute_call(
                        &call,
                        &bind,
                        ctx,
                        tx,
                        atlas,
                        provider_id,
                        runtime,
                        verification,
                    )
                    .await;
                    Completion::Call(index, result)
                });
            } else {
                rtdl_events::send_operator_state(
                    &self.tx,
                    &ctx,
                    State::Running as u32,
                    "running",
                    &self.runtime,
                )
                .await;
            }
        }
        for position in (0..self.input.order().len()).rev() {
            let index = self.input.order()[position];
            let node = &self.input.plan().nodes[index];
            if node.node_kind == 2
                || self.inside_control(index)
                || !self.execution[index].started
                || self.execution[index].final_failed.is_some()
                || !self.input.is_closed(index)
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
            let ctx = rtdl_events::node_event_context(self.input.plan(), index);
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
                rtdl_events::send_operator_final(
                    &self.tx,
                    &ctx,
                    failed,
                    self.cancelled || self.execution[index].canceled,
                    &self.runtime,
                )
                .await;
                if self.runtime.is_cancelled(&self.input.plan().plan_id).await {
                    self.cancel().await;
                }
            } else if self.execution[index].immediate.is_some() && !self.execution[index].verifying
            {
                self.execution[index].verifying = true;
                progressed = true;
                rtdl_events::send_operator_state(
                    &self.tx,
                    &ctx,
                    State::Verifying as u32,
                    "waiting for descendant verification",
                    &self.runtime,
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
        let started_at = std::time::Instant::now();
        info!(
            "[executor/rtdl] plan_id={} session_id={} round={} plan start",
            self.input.plan().plan_id,
            self.input.plan().session_id,
            self.input.plan().round
        );
        self.runtime.record_plan_ops(self.input.plan()).await;
        let _ = self
            .tx
            .send(Ok(rtdl_events::plan_started(
                self.input.plan().plan_id.clone(),
            )))
            .await;
        let mut input_eof = false;
        // Poll external cancellation while idle; locally runnable work never waits for this tick.
        let mut tick = tokio::time::interval(std::time::Duration::from_millis(10));
        loop {
            if self.runtime.is_cancelled(&self.input.plan().plan_id).await {
                self.cancel().await;
            }
            let progressed = self.schedule().await;
            if !self.cancelled && self.runtime.is_cancelled(&self.input.plan().plan_id).await {
                self.cancel().await;
                continue;
            }
            if self.input.ended()
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
                                self.tasks.spawn(async move { Completion::Verified(index, capability_execution::wait_for_verifier(verifier).await) });
                            } else {
                                self.execution[index].final_failed = Some(outcome.failed);
                            }
                        }
                        Some(Ok(Completion::Verified(index, failed))) => { self.execution[index].final_failed = Some(failed || self.execution[index].immediate == Some(true)); }
                        Some(Ok(Completion::Controlled(index, failed))) => {
                            self.execution[index].immediate = Some(failed);
                            self.execution[index].final_failed = Some(failed);
                            for descendant in 0..self.execution.len() {
                                if self.inside_control(descendant) && self.is_descendant_of(descendant, index) {
                                    self.execution[descendant].immediate = Some(false);
                                    self.execution[descendant].final_failed = Some(false);
                                }
                            }
                        }
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
                                        let node = self.input.plan().nodes.last().expect("accepted NODE");
                                        if self.runtime.record_stream_node(&self.input.plan().plan_id, node).await {
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
                            if !self.input.ended() { self.error = Some(Status::invalid_argument("EOF before END")); self.cancel().await; }
                        }
                        Some(Err(error)) => { self.error = Some(error); self.cancel().await; }
                    }
                }
                _ = tick.tick() => {},
            }
            if self.error.is_some() && self.tasks.is_empty() {
                for e in &mut self.execution {
                    e.immediate.get_or_insert(true);
                    e.final_failed.get_or_insert(true);
                }
            }
        }
        self.cancelled |= self.runtime.complete_plan(&self.input.plan().plan_id).await;
        let failed = self.cancelled
            || self
                .execution
                .get(self.input.plan().root_index as usize)
                .is_some_and(|e| e.final_failed == Some(true));
        info!(
            "[executor/rtdl] plan_id={} plan end state={} nodes={} calls={} duration_ms={}",
            self.input.plan().plan_id,
            if self.cancelled {
                "Canceled"
            } else if failed || self.error.is_some() {
                "Failed"
            } else {
                "Succeeded"
            },
            self.input.plan().nodes.len(),
            self.input
                .plan()
                .nodes
                .iter()
                .filter(|node| node.call.is_some())
                .count(),
            started_at.elapsed().as_millis()
        );
        if let Some(error) = self.error {
            let _ = self.tx.send(Err(error)).await;
        } else {
            let _ = self
                .tx
                .send(Ok(rtdl_events::plan_complete(
                    self.input.plan().plan_id.clone(),
                    failed,
                )))
                .await;
        }
    }
}

#[cfg(test)]
mod scheduling_tests {
    use super::*;
    use crate::config::VerificationRule;
    use crate::grpc_server::ExecutorServiceImpl;
    use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
    use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecute;
    use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecuteServer;
    use crate::pb::contracts::robonix_system_executor_execute_stream_client::RobonixSystemExecutorExecuteStreamClient;
    use crate::pb::contracts::robonix_system_executor_execute_stream_server::RobonixSystemExecutorExecuteStreamServer;
    use crate::pb::pilot::Plan;
    use crate::pb::pilot::rtdl_node_state::RtdlNodeStateEnum;
    use crate::pb::pilot::{CapabilityCall, RtdlNode};
    use crate::plan_input::{EXECUTE_STREAM_CLOSE, EXECUTE_STREAM_END, EXECUTE_STREAM_START};
    use crate::verification::VerificationPolicy;
    use robonix_atlas::{
        client::AtlasClient,
        service::{AtlasRegistry, serve_atlas},
    };
    use std::net::SocketAddr;
    use std::time::Duration;
    use tonic::Request;
    const RTDL_SEQUENCE: u32 = 0;
    const RTDL_DO: u32 = 2;

    fn reserve_address() -> SocketAddr {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("reserve port");
        listener.local_addr().expect("reserved address")
    }

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

    /// A sequence reports verification in progress before its final failed state.
    #[tokio::test]
    async fn overlap_stream_finalizes_the_leaf_before_failing_its_sequence() {
        let atlas_addr = reserve_address();
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let atlas = AtlasClient::connect_with_retry(
            format!("http://{atlas_addr}"),
            50,
            Duration::from_millis(10),
        )
        .await
        .expect("connect test Atlas");
        let contract_id = "robonix/system/executor/builtin/get_all_plans";
        let service = ExecutorServiceImpl::new(
            atlas,
            "executor".into(),
            Arc::new(VerificationPolicy::new(
                true,
                vec![VerificationRule {
                    target_contract_id: contract_id.into(),
                    target_provider_id: None,
                    verifier_provider_id: "missing_verifier".into(),
                    verifier_args: serde_json::json!({}),
                }],
            )),
        );
        let plan = Plan {
            plan_id: "overlap-plan".into(),
            session_id: "test".into(),
            round: 0,
            nodes: vec![
                RtdlNode {
                    node_kind: RTDL_SEQUENCE,
                    children: vec![1, 2],
                    call: None,
                    op_id: "sequence".into(),
                    description: "run the verified sequence".into(),
                    ..Default::default()
                },
                RtdlNode {
                    node_kind: RTDL_DO,
                    children: Vec::new(),
                    call: Some(CapabilityCall {
                        call_id: "overlap-plan:1".into(),
                        provider_id: "executor".into(),
                        contract_id: contract_id.into(),
                        args_json: "{}".into(),
                    }),
                    op_id: "op_1".into(),
                    description: "list active plans".into(),
                    ..Default::default()
                },
                RtdlNode {
                    node_kind: RTDL_DO,
                    children: Vec::new(),
                    call: Some(CapabilityCall {
                        call_id: "overlap-plan:2".into(),
                        provider_id: "executor".into(),
                        contract_id: "robonix/system/executor/builtin/list_dir".into(),
                        args_json: r#"{"path":"."}"#.into(),
                    }),
                    op_id: "op_2".into(),
                    description: "continue with the next node".into(),
                    ..Default::default()
                },
            ],
            root_index: 0,
        };
        let mut stream = service
            .execute(Request::new(plan))
            .await
            .unwrap()
            .into_inner();
        let mut states = Vec::new();
        let mut plan_failed = None;
        while let Some(event) = stream.next().await {
            let event = event.unwrap();
            if let Some(state) = event.node_state {
                states.push((state.node_index, state.state));
            }
            if let Some(complete) = event.plan_complete {
                plan_failed = Some(complete.any_failed);
            }
        }

        assert_eq!(
            states.first(),
            Some(&(0, RtdlNodeStateEnum::Running as u32))
        );
        assert!(states.contains(&(1, RtdlNodeStateEnum::Verifying as u32)));
        assert!(states.contains(&(0, RtdlNodeStateEnum::Verifying as u32)));
        assert!(states.contains(&(1, RtdlNodeStateEnum::Failed as u32)));
        assert!(states.contains(&(2, RtdlNodeStateEnum::Succeeded as u32)));
        assert_eq!(states.last(), Some(&(0, RtdlNodeStateEnum::Failed as u32)));
        assert_eq!(plan_failed, Some(true));
        atlas_server.abort();
    }
}
