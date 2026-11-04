// SPDX-License-Identifier: MulanPSL-2.0
//! Ownership of a single Pilot → Executor input/output stream during generation.
use crate::pb::contracts::robonix_system_executor_control_plan_client::RobonixSystemExecutorControlPlanClient;
use crate::pb::contracts::robonix_system_executor_execute_stream_client::RobonixSystemExecutorExecuteStreamClient;
use crate::pb::executor::{ControlPlanRequest, ExecuteStreamInput, RtdlEvent};
use crate::pb::pilot::Plan;
use crate::rtdl_stream::{
    EXECUTE_STREAM_CLOSE, EXECUTE_STREAM_END, EXECUTE_STREAM_NODE, EXECUTE_STREAM_START,
};
use anyhow::{Result, ensure};
use std::collections::{HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Streaming, transport::Channel};

pub(crate) enum SubmissionInterrupt {
    Stale,
    Failed,
}

/// Admission context checked before each streamed node is submitted.
pub(crate) struct SubmissionGuard<'a> {
    /// Executor connection receiving this response's tree.
    pub client: &'a mut RobonixSystemExecutorExecuteStreamClient<Channel>,
    /// Calls already running in other trees and unavailable for redispatch.
    pub forbidden: &'a HashSet<String>,
    /// Execution changes that can invalidate a reply before submission.
    pub revision: &'a AtomicU64,
    /// Execution revision observed when this planning request began.
    pub expected_revision: u64,
}

/// One submitted tree's input, observed output, and exact accepted node arena.
#[derive(Default)]
pub(crate) struct StreamExecution {
    /// Open incremental input; dropped after sealing or requesting cancellation.
    input: Option<mpsc::Sender<ExecuteStreamInput>>,
    /// Execution output retained for the session driver after generation ends.
    pub(crate) output: Option<Streaming<RtdlEvent>>,
    /// Events received during generation and awaiting session processing.
    pub(crate) events: VecDeque<RtdlEvent>,
    /// Only the nodes actually submitted to Executor, with their assigned ids.
    pub(crate) plan: Plan,
    /// Operators already closed, indexed by their accepted arena position.
    closed: Vec<bool>,
    /// Whether Executor has reported plan_complete.
    pub(crate) complete: bool,
    /// Whether output was lost before terminal execution evidence arrived.
    pub(crate) transport_failed: bool,
}

impl StreamExecution {
    /// Recheck execution failure and revision at each submission boundary.
    /// Steering and cancellation are handled by the generation loop after submission returns.
    pub(crate) async fn submit(
        &mut self,
        messages: Vec<ExecuteStreamInput>,
        guard: SubmissionGuard<'_>,
    ) -> Result<Option<SubmissionInterrupt>> {
        for message in messages {
            if self.plan.nodes.is_empty()
                && guard.revision.load(Ordering::Acquire) != guard.expected_revision
            {
                return Ok(Some(SubmissionInterrupt::Stale));
            }
            // Poll ready Executor events before sending the next node in a large text delta.
            loop {
                tokio::select! {
                    biased;
                    event = self.next(), if self.output.is_some() => {
                        match event? {
                            Some(event) => { if self.remember(event) { return Ok(Some(SubmissionInterrupt::Failed)); } },
                            None => anyhow::bail!("Executor EOF while submitting nodes"),
                        }
                    }
                    _ = std::future::ready(()) => break,
                }
            }
            self.send(message, guard.client, guard.forbidden).await?;
        }
        Ok(None)
    }

    /// Send accepted messages once and mirror only the actual submitted arena.
    async fn send(
        &mut self,
        message: ExecuteStreamInput,
        client: &mut RobonixSystemExecutorExecuteStreamClient<Channel>,
        forbidden: &HashSet<String>,
    ) -> Result<()> {
        if message.message_kind == EXECUTE_STREAM_START {
            ensure!(self.input.is_none(), "stream already started");
            let (tx, rx) = mpsc::channel(1024);
            tx.send(message.clone()).await?;
            let response = tokio::time::timeout(
                std::time::Duration::from_secs(5),
                client.execute_stream(ReceiverStream::new(rx)),
            )
            .await??;
            self.output = Some(response.into_inner());
            self.input = Some(tx);
            self.plan = Plan {
                plan_id: message.plan_id,
                session_id: message.session_id,
                round: message.round,
                ..Default::default()
            };
            return Ok(());
        }
        if message.message_kind == EXECUTE_STREAM_NODE {
            let node = message.node.as_ref().expect("parser NODE");
            if let Some(call) = &node.call {
                ensure!(
                    !forbidden.contains(&crate::planner::call_signature(call)),
                    "capability call is already in flight"
                );
            }
        }
        self.input
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("stream is closed"))?
            .send(message.clone())
            .await?;
        match message.message_kind {
            EXECUTE_STREAM_NODE => {
                let node = message.node.expect("parser NODE");
                if message.has_parent {
                    self.plan.nodes[message.parent_index as usize]
                        .children
                        .push(message.node_index);
                }
                self.closed.push(node.node_kind == 2);
                self.plan.nodes.push(node);
            }
            EXECUTE_STREAM_CLOSE => self.closed[message.node_index as usize] = true,
            _ => {}
        }
        Ok(())
    }

    /// Receive one execution event while token generation is still active.
    pub(crate) async fn next(&mut self) -> Result<Option<RtdlEvent>, tonic::Status> {
        match &mut self.output {
            Some(stream) => stream.message().await,
            None if self.plan.plan_id.is_empty() => std::future::pending().await,
            None => Ok(None),
        }
    }

    /// Remember events until the final prefix Plan has been published to the forest.
    pub(crate) fn remember(&mut self, event: RtdlEvent) -> bool {
        let failed = event
            .node_state
            .as_ref()
            .is_some_and(|n| matches!(n.state, 3..=5));
        if event.plan_complete.is_some() {
            self.complete = true;
        }
        self.events.push_back(event);
        failed
    }

    /// Close only submitted operators, then END and half-close the request transport.
    pub(crate) async fn seal(&mut self) -> Result<()> {
        let Some(input) = self.input.take() else {
            return Ok(());
        };
        for index in (0..self.closed.len()).rev() {
            if !self.closed[index] {
                input
                    .send(ExecuteStreamInput {
                        message_kind: EXECUTE_STREAM_CLOSE,
                        node_index: index as u32,
                        ..Default::default()
                    })
                    .await?;
                self.closed[index] = true;
            }
        }
        input
            .send(ExecuteStreamInput {
                message_kind: EXECUTE_STREAM_END,
                ..Default::default()
            })
            .await?;
        Ok(())
    }

    /// Drain terminal events after an invalid response or execution failure, never resending input.
    pub(crate) async fn drain(&mut self) -> Result<()> {
        while self.output.is_some() && !self.complete {
            match self.next().await {
                Ok(Some(event)) => {
                    self.remember(event);
                }
                Ok(None) => {
                    self.transport_failed = true;
                    anyhow::bail!("Executor EOF before plan_complete");
                }
                Err(error) => {
                    self.transport_failed = true;
                    return Err(error.into());
                }
            }
        }
        Ok(())
    }

    /// Request cancellation before closing input, without sending CLOSE or END
    /// that could release more execution. Retain observed results for history.
    pub(crate) async fn cancel(
        &mut self,
        control: &mut RobonixSystemExecutorControlPlanClient<Channel>,
        auth_session_token: &str,
    ) -> Result<()> {
        if self.plan.plan_id.is_empty() {
            return Ok(());
        }
        let response = tokio::time::timeout(
            std::time::Duration::from_secs(6),
            control.control_plan(ControlPlanRequest {
                session_token: auth_session_token.to_owned(),
                action: "cancel".into(),
                plan_id: self.plan.plan_id.clone(),
                wait_ms: 5000,
            }),
        )
        .await;
        // Also close input when the control RPC fails, so Executor observes EOF.
        self.input.take();
        let response = response??.into_inner();
        ensure!(
            response.success,
            "Executor cancellation failed: {}",
            response.error
        );
        if !response.completed {
            robonix_scribe::warn!(
                "[pilot/stream] cancellation wait timed out: {}",
                response.message
            );
        }
        // Read already produced events; synchronous calls may still be finishing after timeout.
        match tokio::time::timeout(std::time::Duration::from_secs(1), self.drain()).await {
            Ok(Ok(())) => {}
            Ok(Err(error)) => {
                robonix_scribe::warn!("[pilot/stream] cancellation drain failed: {error:#}")
            }
            Err(error) => {
                robonix_scribe::warn!("[pilot/stream] cancellation drain timed out: {error}")
            }
        }
        if self.complete || self.transport_failed {
            self.output.take();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pb::contracts::robonix_system_executor_control_plan_server::{
        RobonixSystemExecutorControlPlan, RobonixSystemExecutorControlPlanServer,
    };
    use crate::pb::contracts::robonix_system_executor_execute_stream_server::{
        RobonixSystemExecutorExecuteStream, RobonixSystemExecutorExecuteStreamServer,
    };
    use crate::pb::executor::{ControlPlanResponse, RtdlPlanComplete};
    use std::sync::Arc;
    use tokio::sync::Mutex;

    /// Control endpoint that verifies input stays open until cancellation arrives.
    struct CancelControl {
        /// Input receiver used to detect premature CLOSE, END, or EOF.
        input: Arc<Mutex<mpsc::Receiver<ExecuteStreamInput>>>,
        /// Whether to simulate a failed cancellation RPC.
        reject: bool,
        /// Whether cancellation has already finished all accepted work.
        completed: bool,
    }

    #[tonic::async_trait]
    impl RobonixSystemExecutorControlPlan for CancelControl {
        /// Check ordering at the RPC boundary, then accept or reject cancellation.
        async fn control_plan(
            &self,
            request: tonic::Request<ControlPlanRequest>,
        ) -> Result<tonic::Response<ControlPlanResponse>, tonic::Status> {
            let request = request.into_inner();
            assert_eq!(request.action, "cancel");
            assert_eq!(request.plan_id, "partial");
            assert!(matches!(
                self.input.lock().await.try_recv(),
                Err(mpsc::error::TryRecvError::Empty)
            ));
            if self.reject {
                return Err(tonic::Status::unavailable("control unavailable"));
            }
            Ok(tonic::Response::new(ControlPlanResponse {
                success: true,
                completed: self.completed,
                ..Default::default()
            }))
        }
    }

    /// Neither successful nor failed cancellation may seal a partial plan first.
    #[tokio::test]
    async fn cancellation_precedes_input_closure_even_when_control_fails() {
        for reject in [false, true] {
            let (tx, rx) = mpsc::channel(8);
            let input = Arc::new(Mutex::new(rx));
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let address = listener.local_addr().unwrap();
            let incoming = futures_util::stream::unfold(listener, |listener| async {
                let stream = listener.accept().await.map(|(stream, _)| stream);
                Some((stream, listener))
            });
            let server = tokio::spawn(
                tonic::transport::Server::builder()
                    .add_service(RobonixSystemExecutorControlPlanServer::new(CancelControl {
                        input: Arc::clone(&input),
                        reject,
                        completed: true,
                    }))
                    .serve_with_incoming(incoming),
            );
            let mut control =
                RobonixSystemExecutorControlPlanClient::connect(format!("http://{address}"))
                    .await
                    .unwrap();
            let mut live = StreamExecution {
                input: Some(tx),
                plan: Plan {
                    plan_id: "partial".into(),
                    ..Default::default()
                },
                closed: vec![false],
                ..Default::default()
            };

            let result = live.cancel(&mut control, "").await;

            assert_eq!(result.is_err(), reject);
            assert!(live.input.is_none());
            assert!(matches!(
                input.lock().await.try_recv(),
                Err(mpsc::error::TryRecvError::Disconnected)
            ));
            server.abort();
        }
    }

    /// An accepted synchronous call that reports completion after the drain window.
    struct DeferredExecution {
        /// Releases the terminal event only after cancellation has returned.
        finish: Arc<tokio::sync::Notify>,
    }

    #[tonic::async_trait]
    impl RobonixSystemExecutorExecuteStream for DeferredExecution {
        type ExecuteStreamStream = ReceiverStream<Result<RtdlEvent, tonic::Status>>;

        async fn execute_stream(
            &self,
            _request: tonic::Request<Streaming<ExecuteStreamInput>>,
        ) -> Result<tonic::Response<Self::ExecuteStreamStream>, tonic::Status> {
            let (tx, rx) = mpsc::channel(1);
            let finish = Arc::clone(&self.finish);
            tokio::spawn(async move {
                finish.notified().await;
                let _ = tx
                    .send(Ok(RtdlEvent {
                        event_kind: 2,
                        plan_complete: Some(RtdlPlanComplete {
                            plan_id: "partial".into(),
                            any_failed: false,
                        }),
                        ..Default::default()
                    }))
                    .await;
            });
            Ok(tonic::Response::new(ReceiverStream::new(rx)))
        }
    }

    #[tokio::test]
    async fn cancellation_drain_timeout_keeps_output_for_the_session() {
        let (input, rx) = mpsc::channel(8);
        let finish = Arc::new(tokio::sync::Notify::new());
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let incoming = futures_util::stream::unfold(listener, |listener| async {
            let stream = listener.accept().await.map(|(stream, _)| stream);
            Some((stream, listener))
        });
        let server = tokio::spawn(
            tonic::transport::Server::builder()
                .add_service(RobonixSystemExecutorControlPlanServer::new(CancelControl {
                    input: Arc::new(Mutex::new(rx)),
                    reject: false,
                    completed: false,
                }))
                .add_service(RobonixSystemExecutorExecuteStreamServer::new(
                    DeferredExecution {
                        finish: Arc::clone(&finish),
                    },
                ))
                .serve_with_incoming(incoming),
        );
        let channel = Channel::from_shared(format!("http://{address}"))
            .unwrap()
            .connect()
            .await
            .unwrap();
        let mut client = RobonixSystemExecutorExecuteStreamClient::new(channel.clone());
        let (_, request) = mpsc::channel(1);
        let output = client
            .execute_stream(ReceiverStream::new(request))
            .await
            .unwrap()
            .into_inner();
        let mut live = StreamExecution {
            input: Some(input),
            output: Some(output),
            plan: Plan {
                plan_id: "partial".into(),
                ..Default::default()
            },
            ..Default::default()
        };

        live.cancel(
            &mut RobonixSystemExecutorControlPlanClient::new(channel),
            "",
        )
        .await
        .unwrap();

        assert!(!live.complete);
        assert!(!live.transport_failed);
        assert!(live.output.is_some());
        finish.notify_one();
        let event = tokio::time::timeout(std::time::Duration::from_secs(2), live.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        assert_eq!(event.plan_complete.unwrap().plan_id, "partial");
        server.abort();
    }

    /// A transport already removed during cleanup must report EOF to the forest driver.
    #[tokio::test]
    async fn removed_output_does_not_hang_forest_handoff() {
        let mut live = StreamExecution {
            plan: Plan {
                plan_id: "disconnected".into(),
                ..Default::default()
            },
            transport_failed: true,
            ..Default::default()
        };
        let result = tokio::time::timeout(std::time::Duration::from_millis(50), live.next())
            .await
            .unwrap()
            .unwrap();
        assert!(result.is_none());
    }
}
