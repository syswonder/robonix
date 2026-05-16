// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// gRPC contract handler: RobonixSystemExecutor.Execute(Plan) → stream CapabilityCallEvent.

use crate::dispatch;
use crate::exec_wire;
use crate::pb::contracts::robonix_system_executor_server::RobonixSystemExecutor;
use crate::pb::executor::CapabilityCallEvent;
use crate::pb::pilot::{CapabilityCall, Plan};
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
}

impl ExecutorServiceImpl {
    pub fn new(atlas: AtlasClient, provider_id: String) -> Self {
        Self { atlas, provider_id }
    }
}

#[tonic::async_trait]
impl RobonixSystemExecutor for ExecutorServiceImpl {
    type ExecuteStream = ReceiverStream<Result<CapabilityCallEvent, Status>>;

    async fn execute(
        &self,
        request: Request<Plan>,
    ) -> Result<Response<Self::ExecuteStream>, Status> {
        let plan = request.into_inner();
        validate_plan(&plan).map_err(Status::invalid_argument)?;
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        let atlas = self.atlas.clone();
        let provider_id = self.provider_id.clone();

        tokio::spawn(async move {
            let plan_id = plan.plan_id.clone();
            let plan = Arc::new(plan);
            let any_failed = execute_node(
                Arc::clone(&plan),
                plan.root_index as usize,
                tx.clone(),
                atlas,
                provider_id,
            )
            .await;

            let _ = tx.send(Ok(exec_wire::complete(plan_id, any_failed))).await;
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

type ExecuteNodeFuture = Pin<Box<dyn Future<Output = bool> + Send + 'static>>;

fn execute_node(
    plan: Arc<Plan>,
    node_index: usize,
    tx: Sender<Result<CapabilityCallEvent, Status>>,
    atlas: AtlasClient,
    provider_id: String,
) -> ExecuteNodeFuture {
    Box::pin(async move {
        let node = &plan.nodes[node_index];
        match node.node_kind {
            RTDL_SEQUENCE => {
                let mut any_failed = false;
                for child in &node.children {
                    any_failed |= execute_node(
                        Arc::clone(&plan),
                        *child as usize,
                        tx.clone(),
                        atlas.clone(),
                        provider_id.clone(),
                    )
                    .await;
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
                    let child_index = *child as usize;
                    handles.push(tokio::spawn(async move {
                        execute_node(
                            child_plan,
                            child_index,
                            child_tx,
                            child_atlas,
                            child_provider_id,
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
                execute_call(call, tx, atlas, provider_id).await
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

/// Dispatch one RTDL `do` node and stream its started/result events.
async fn execute_call(
    call: &CapabilityCall,
    tx: Sender<Result<CapabilityCallEvent, Status>>,
    atlas: AtlasClient,
    provider_id: String,
) -> bool {
    // `tokio::sync::mpsc::Sender` is concurrency-safe when cloned (parallel branches).
    // Outbound events ride the Execute server-stream to the gRPC client (Pilot).
    let _ = tx
        .send(Ok(exec_wire::started(
            call.call_id.clone(),
            call.provider_id.clone(),
            call.contract_id.clone(),
        )))
        .await;

    log::info!(
        "[executor] dispatching call_id={} provider='{}' contract='{}'",
        call.call_id,
        call.provider_id,
        call.contract_id,
    );
    let mut atlas_for_call = atlas.clone();
    let result = dispatch::dispatch(call, &provider_id, &mut atlas_for_call).await;
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

    let _ = tx.send(Ok(exec_wire::result(result))).await;
    failed
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
