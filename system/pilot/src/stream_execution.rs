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

pub(crate) struct SubmissionGuard<'a> {
    pub client: &'a mut RobonixSystemExecutorExecuteStreamClient<Channel>,
    pub forbidden: &'a HashSet<String>,
    pub revision: &'a AtomicU64,
    pub expected_revision: u64,
}

#[derive(Default)]
pub(crate) struct StreamExecution {
    input: Option<mpsc::Sender<ExecuteStreamInput>>,
    pub(crate) output: Option<Streaming<RtdlEvent>>,
    pub(crate) events: VecDeque<RtdlEvent>,
    pub(crate) plan: Plan,
    closed: Vec<bool>,
    pub(crate) complete: bool,
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
            self.send(vec![message], guard.client, guard.forbidden)
                .await?;
        }
        Ok(None)
    }

    /// Send accepted messages once and mirror only the actual submitted arena.
    pub(crate) async fn send(
        &mut self,
        messages: Vec<ExecuteStreamInput>,
        client: &mut RobonixSystemExecutorExecuteStreamClient<Channel>,
        forbidden: &HashSet<String>,
    ) -> Result<()> {
        for message in messages {
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
                continue;
            }
            if message.message_kind == EXECUTE_STREAM_NODE {
                let node = message.node.as_ref().expect("parser NODE");
                if node.call.is_some() {
                    let single = Plan {
                        nodes: vec![node.clone()],
                        ..Default::default()
                    };
                    ensure!(
                        crate::planner::plan_call_signatures(&single).is_disjoint(forbidden),
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

    /// Cancel only this generating plan and wait for Executor's active-table removal or its timeout.
    pub(crate) async fn cancel(
        &mut self,
        control: &mut RobonixSystemExecutorControlPlanClient<Channel>,
    ) -> Result<()> {
        let seal = self.seal().await;
        if self.plan.plan_id.is_empty() {
            return seal;
        }
        let response = tokio::time::timeout(
            std::time::Duration::from_secs(6),
            control.control_plan(ControlPlanRequest {
                action: "cancel".into(),
                plan_id: self.plan.plan_id.clone(),
                wait_ms: 5000,
            }),
        )
        .await??;
        let response = response.into_inner();
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
        self.output.take();
        if !self.complete {
            self.transport_failed = true;
        }
        if let Err(error) = seal {
            robonix_scribe::warn!("[pilot/stream] request closure failed during cancel: {error:#}");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
