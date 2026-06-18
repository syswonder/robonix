// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// gRPC contract handlers for executor plan execution and cancellation.

use crate::dispatch::{async_poll, async_registry};
use crate::pb::contracts::robonix_system_executor_cancel_all_plans_server::RobonixSystemExecutorCancelAllPlans;
use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecute;
use crate::pb::executor::{CancelAllResponse, RtdlEvent};
use crate::pb::pilot::{CapabilityCall, Plan};
use crate::plan_runtime::PlanRuntime;
use crate::rtdl_wire::{self, NodeEventContext};
use robonix_atlas::client::AtlasClient;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use tokio::sync::mpsc::Sender;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

const RTDL_SEQUENCE: u32 = 0;
const RTDL_PARALLEL: u32 = 1;
const RTDL_DO: u32 = 2;

/// `AtlasClient` is cheap to clone — each Execute RPC clones it so per-plan
/// dispatch runs without serialising on a single mutex.
#[derive(Clone)]
pub struct ExecutorServiceImpl {
    atlas: AtlasClient,
    /// Executor's own provider_id. Two roles:
    ///   1. consumer_id passed to atlas on every ConnectCapability so the
    ///      channel record reflects who is using each downstream provider.
    ///   2. self-detection: when a CapabilityCall in the plan targets this
    ///      provider_id, dispatch short-circuits to the in-process builtin
    ///      handlers instead of going through MCP loopback.
    provider_id: String,
    runtime: PlanRuntime,
}

impl ExecutorServiceImpl {
    pub fn new(atlas: AtlasClient, provider_id: String) -> Self {
        Self {
            atlas,
            provider_id,
            runtime: PlanRuntime::default(),
        }
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutorExecute for ExecutorServiceImpl {
    type ExecuteStream = ReceiverStream<Result<RtdlEvent, Status>>;

    async fn execute(
        &self,
        request: Request<Plan>,
    ) -> Result<Response<Self::ExecuteStream>, Status> {
        let plan = request.into_inner();
        validate_plan(&plan).map_err(Status::invalid_argument)?;
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let atlas = self.atlas.clone();
        let provider_id = self.provider_id.clone();
        let runtime = self.runtime.clone();

        tokio::spawn(async move {
            let plan_id = plan.plan_id.clone();
            let plan = Arc::new(plan);
            runtime.register_plan(&plan_id).await;
            let _ = tx.send(Ok(rtdl_wire::plan_started(plan_id.clone()))).await;
            let any_failed = execute_node(
                Arc::clone(&plan),
                plan.root_index as usize,
                tx.clone(),
                atlas,
                provider_id,
                runtime.clone(),
            )
            .await;
            let cancelled = runtime.is_cancelled(&plan_id).await;
            runtime.complete_plan(&plan_id).await;

            let _ = tx
                .send(Ok(rtdl_wire::plan_complete(
                    plan_id,
                    any_failed || cancelled,
                )))
                .await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

type ExecuteNodeFuture = Pin<Box<dyn Future<Output = bool> + Send + 'static>>;

fn execute_node(
    plan: Arc<Plan>,
    node_index: usize,
    tx: Sender<Result<RtdlEvent, Status>>,
    atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
) -> ExecuteNodeFuture {
    Box::pin(async move {
        if runtime.is_cancelled(&plan.plan_id).await {
            return true;
        }
        let node = &plan.nodes[node_index];
        match node.node_kind {
            RTDL_SEQUENCE => {
                let mut any_failed = false;
                for child in &node.children {
                    if runtime.is_cancelled(&plan.plan_id).await {
                        any_failed = true;
                        break;
                    }
                    any_failed |= execute_node(
                        Arc::clone(&plan),
                        *child as usize,
                        tx.clone(),
                        atlas.clone(),
                        provider_id.clone(),
                        runtime.clone(),
                    )
                    .await;
                    if any_failed {
                        break;
                    }
                }
                any_failed
            }
            RTDL_PARALLEL => {
                let mut handles = Vec::with_capacity(node.children.len());
                for child in &node.children {
                    let child_plan = Arc::clone(&plan);
                    let child_tx = tx.clone();
                    let child_atlas = atlas.clone();
                    let child_provider_id = provider_id.clone();
                    let child_runtime = runtime.clone();
                    let child_index = *child as usize;
                    handles.push(tokio::spawn(async move {
                        execute_node(
                            child_plan,
                            child_index,
                            child_tx,
                            child_atlas,
                            child_provider_id,
                            child_runtime,
                        )
                        .await
                    }));
                }
                let mut any_failed = false;
                for handle in handles {
                    match handle.await {
                        Ok(child_failed) => any_failed |= child_failed,
                        Err(e) => {
                            any_failed = true;
                            log::warn!("[executor] parallel branch task failed: {e}");
                        }
                    }
                }
                any_failed
            }
            RTDL_DO => {
                let call = node
                    .call
                    .as_ref()
                    .expect("validated do node must contain call");
                execute_call(
                    call,
                    NodeEventContext {
                        plan_id: plan.plan_id.clone(),
                        node_index: node_index as u32,
                        node_kind: node.node_kind,
                    },
                    tx,
                    atlas,
                    provider_id,
                    runtime,
                )
                .await
            }
            _ => {
                log::warn!(
                    "[executor] invalid node_kind={} reached after validation",
                    node.node_kind
                );
                true
            }
        }
    })
}

/// Dispatch one RTDL `do` node and stream node_state events.
async fn execute_call(
    call: &CapabilityCall,
    node: NodeEventContext,
    tx: Sender<Result<RtdlEvent, Status>>,
    mut atlas: AtlasClient,
    provider_id: String,
    runtime: PlanRuntime,
) -> bool {

    log::info!(
        "[executor] dispatching call_id={} provider='{}' contract='{}'",
        call.call_id,
        call.provider_id,
        call.contract_id,
    );

    let async_group = if call.provider_id == provider_id {
        None
    } else {
        async_registry::resolve_async_group(&mut atlas, &call.provider_id, &call.contract_id).await
    };

    let result = if let Some(group) = async_group {
        async_poll::run_until_terminal(call, &group, &provider_id, &mut atlas, &tx, &node, &runtime)
            .await
    } else {
        let r = crate::dispatch::dispatch(call, &provider_id, &mut atlas, &runtime).await;
        let state = if r.success {
            rtdl_wire::STATE_SUCCEEDED
        } else {
            rtdl_wire::STATE_FAILED
        };
        let _ = tx
            .send(Ok(rtdl_wire::node_state_from_result(
                &node.plan_id,
                node.node_index,
                node.node_kind,
                call,
                r.clone(),
                state,
            )))
            .await;
        r
    };
    let failed = !result.success;

    if result.success {
        let preview: String = result.output.chars().take(120).collect();
        let ellipsis = if result.output.len() > 120 { "..." } else { "" };
        log::info!(
            "[executor] '{}' ok: {}{}",
            call.contract_id,
            preview,
            ellipsis
        );
    } else {
        log::warn!("[executor] '{}' failed: {}", call.contract_id, result.error);
    }

    failed
}

#[tonic::async_trait]
impl RobonixSystemExecutorCancelAllPlans for ExecutorServiceImpl {
    async fn cancel_all(
        &self,
        _request: Request<crate::pb::executor::CancelAllRequest>,
    ) -> Result<Response<CancelAllResponse>, Status> {
        let mut atlas = self.atlas.clone();
        let success = self
            .runtime
            .cancel_all_plans(&self.provider_id, &mut atlas)
            .await;
        Ok(Response::new(CancelAllResponse { success }))
    }
}

/// Validate Plan arena shape before spawning execution work.
fn validate_plan(plan: &Plan) -> Result<(), String> {
    if plan.nodes.is_empty() {
        return Err("Plan.nodes must not be empty".to_string());
    }
    let root = plan.root_index as usize;
    if root >= plan.nodes.len() {
        return Err(format!(
            "Plan.root_index {} is out of bounds for {} nodes",
            plan.root_index,
            plan.nodes.len()
        ));
    }

    for (idx, node) in plan.nodes.iter().enumerate() {
        match node.node_kind {
            RTDL_SEQUENCE | RTDL_PARALLEL => {
                for child in &node.children {
                    if *child as usize >= plan.nodes.len() {
                        return Err(format!("node {idx} child index {child} is out of bounds"));
                    }
                }
            }
            RTDL_DO => {
                if !node.children.is_empty() {
                    return Err(format!("do node {idx} must not have children"));
                }
                let Some(call) = node.call.as_ref() else {
                    return Err(format!("do node {idx} must contain a call"));
                };
                validate_call(idx, call)?;
            }
            other => return Err(format!("node {idx} has invalid node_kind {other}")),
        }
    }

    let mut colors = vec![VisitColor::White; plan.nodes.len()];
    visit_for_cycles(root, plan, &mut colors)
}

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

#[derive(Clone, Copy, PartialEq, Eq)]
enum VisitColor {
    White,
    Gray,
    Black,
}

/// DFS cycle check on the plan arena following only sequence/parallel child edges.
///
/// Uses White/Gray/Black marks: entering a Gray node means a back-edge to an ancestor.
/// `RTDL_DO` nodes have no children in this graph. Returns `Ok` when the subgraph from
/// `index` is acyclic; otherwise an error naming the node where the cycle was found.
fn visit_for_cycles(index: usize, plan: &Plan, colors: &mut [VisitColor]) -> Result<(), String> {
    match colors[index] {
        VisitColor::Gray => return Err(format!("cycle detected at node {index}")),
        VisitColor::Black => return Ok(()),
        VisitColor::White => {}
    }
    colors[index] = VisitColor::Gray;
    let node = &plan.nodes[index];
    if matches!(node.node_kind, RTDL_SEQUENCE | RTDL_PARALLEL) {
        for child in &node.children {
            visit_for_cycles(*child as usize, plan, colors)?;
        }
    }
    colors[index] = VisitColor::Black;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE, validate_plan};
    use crate::pb::pilot::{CapabilityCall, Plan, RtdlNode};

    fn call(id: &str) -> CapabilityCall {
        CapabilityCall {
            call_id: id.to_string(),
            provider_id: "provider".to_string(),
            contract_id: "robonix/test/cap".to_string(),
            args_json: "{}".to_string(),
        }
    }

    fn node(kind: u32, children: Vec<u32>, call: Option<CapabilityCall>) -> RtdlNode {
        RtdlNode {
            node_kind: kind,
            children,
            call,
        }
    }

    fn plan(nodes: Vec<RtdlNode>, root_index: u32) -> Plan {
        Plan {
            plan_id: "p".to_string(),
            session_id: "s".to_string(),
            round: 0,
            nodes,
            root_index,
        }
    }

    #[test]
    fn validates_sequence_and_parallel_nodes() {
        let p = plan(
            vec![
                node(RTDL_SEQUENCE, vec![1, 2], None),
                node(RTDL_DO, vec![], Some(call("p:0"))),
                node(RTDL_PARALLEL, vec![3, 4], None),
                node(RTDL_DO, vec![], Some(call("p:1"))),
                node(RTDL_DO, vec![], Some(call("p:2"))),
            ],
            0,
        );
        validate_plan(&p).unwrap();
    }

    #[test]
    fn rejects_invalid_root() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![], None)], 3);
        assert!(validate_plan(&p).unwrap_err().contains("root_index"));
    }

    #[test]
    fn rejects_out_of_bounds_child() {
        let p = plan(vec![node(RTDL_SEQUENCE, vec![9], None)], 0);
        assert!(validate_plan(&p).unwrap_err().contains("out of bounds"));
    }

    #[test]
    fn rejects_cycle() {
        let p = plan(
            vec![
                node(RTDL_SEQUENCE, vec![1], None),
                node(RTDL_PARALLEL, vec![0], None),
            ],
            0,
        );
        assert!(validate_plan(&p).unwrap_err().contains("cycle"));
    }

    #[test]
    fn rejects_do_without_call() {
        let p = plan(vec![node(RTDL_DO, vec![], None)], 0);
        assert!(
            validate_plan(&p)
                .unwrap_err()
                .contains("must contain a call")
        );
    }
}
