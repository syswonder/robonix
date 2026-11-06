// SPDX-License-Identifier: MulanPSL-2.0
//! MCP communication and execution ownership in the consumer process.

use super::AtlasClient;
use crate::pb;
use anyhow::{Context, Result, bail, ensure};
use rmcp::ServiceExt;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;
use tokio::sync::{Mutex as AsyncMutex, OwnedSemaphorePermit, Semaphore};

type McpClient = rmcp::service::RunningService<rmcp::RoleClient, ()>;

/// Bounded execution ownership shared by all clones of an Atlas client.
pub(super) struct Channels {
    /// Executions remain available after the original caller has returned.
    live: Mutex<HashMap<String, Channel>>,
    /// Includes connections still being prepared, preventing concurrent overflow.
    capacity: Arc<Semaphore>,
    /// Companion calls remain possible when all business execution slots are occupied.
    companion_capacity: Arc<Semaphore>,
}

impl Default for Channels {
    fn default() -> Self {
        Self {
            live: Mutex::new(HashMap::new()),
            capacity: Arc::new(Semaphore::new(1022)),
            companion_capacity: Arc::new(Semaphore::new(2)),
        }
    }
}

/// One capability execution and its initialized communication resources.
#[derive(Clone)]
pub struct Channel {
    /// Atlas identity of the provider executing the capability.
    pub provider_id: String,
    /// Contract bound to this execution.
    pub contract_id: String,
    /// Communication transport selected when allocating this channel.
    pub transport: pb::Transport,
    /// Authoritative endpoint returned by Atlas.
    pub endpoint: String,
    /// Atlas allocation retained until execution termination is confirmed.
    pub channel_id: String,
    /// Transport metadata returned by Atlas.
    pub params: pb::TransportParams,
    /// Shared ownership prevents dropping a handle from releasing active work.
    execution: Arc<Execution>,
}

/// Communication and lifecycle state kept independently of an Executor plan.
struct Execution {
    /// Raw client avoids a strong reference cycle through the live-channel table.
    atlas: pb::atlas_client::AtlasClient<tonic::transport::Channel>,
    /// Registry to remove the execution after successful release.
    channels: Weak<Channels>,
    /// Identity used for independently allocated companion calls.
    consumer_id: String,
    /// Status and cancellation contracts, both present or both absent.
    companions: Option<(String, String)>,
    /// Initialized session; locked while invoking or shutting down communication.
    session: AsyncMutex<Option<McpClient>>,
    /// Concurrent cleanup callers observe the same completed release before retrying.
    close_lock: AsyncMutex<()>,
    /// Short-lived lock for transitions that must precede any network await.
    progress: Mutex<Progress>,
}

/// Locally known facts about execution completion and release.
struct Progress {
    /// A business call was submitted; the channel cannot be reused.
    started: bool,
    /// Confirmed start response ID; an empty string preserves legacy behavior.
    run_id: Option<String>,
    /// A valid synchronous response or asynchronous terminal state was received.
    terminal: bool,
    /// Atlas has acknowledged release of the allocation.
    closed: bool,
    /// Another close operation is coordinating termination or cleanup.
    closing: bool,
    /// Capacity is returned only after the allocation has been released.
    permit: Option<OwnedSemaphorePermit>,
}

impl AtlasClient {
    /// Prepare one MCP execution. Other transports are rejected before allocation.
    pub fn connect_channel<'a>(
        &'a self,
        consumer_id: &'a str,
        provider_id: &'a str,
        contract_id: &'a str,
        transport: pb::Transport,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<Channel>> + Send + 'a>> {
        Box::pin(async move {
            ensure!(
                transport == pb::Transport::Mcp,
                "connect_channel currently supports MCP only"
            );
            let atlas = self.clone();
            let (consumer, provider, contract) = (
                consumer_id.to_owned(),
                provider_id.to_owned(),
                contract_id.to_owned(),
            );
            // Acknowledgement also covers cancellation after send but before receipt.
            let (sender, receiver) = tokio::sync::oneshot::channel();
            let (acknowledge, received) = tokio::sync::oneshot::channel();
            tokio::spawn(async move {
                let result = atlas.prepare_channel(consumer, provider, contract).await;
                let cleanup = result.as_ref().ok().cloned();
                if sender.send(result).is_ok() && received.await.is_ok() {
                    return;
                }
                if let Some(channel) = cleanup
                    && let Err(error) = channel.close().await
                {
                    robonix_scribe::warn!(
                        "[atlas-client] abandoned setup cleanup failed: {error:#}"
                    );
                }
            });
            let result = receiver.await?;
            let _ = acknowledge.send(());
            result
        })
    }

    /// Recover an allocation retained after a timeout, error, or dropped caller.
    pub fn get_channel(&self, channel_id: &str) -> Option<Channel> {
        self.channels.live.lock().unwrap().get(channel_id).cloned()
    }

    async fn prepare_channel(
        mut self,
        consumer: String,
        provider: String,
        contract: String,
    ) -> Result<Channel> {
        let companion = matches!(contract.rsplit('/').next(), Some("status" | "cancel"));
        let permit = self.channels.capacity.clone()
            .try_acquire_owned()
            .or_else(|error| if companion {
                self.channels.companion_capacity.clone().try_acquire_owned()
            } else { Err(error) })
            .context("Atlas client channel capacity exhausted (two slots reserved for companions); close existing channels")?;
        let descriptor =
            tokio::time::timeout(Duration::from_secs(30), self.query_contract(&contract))
                .await
                .context("Atlas contract query timed out")??
                .with_context(|| format!("unknown contract: {contract}"))?;
        ensure!(
            descriptor.mode == "rpc",
            "MCP channel requires rpc mode: {contract}"
        );
        let companions = if matches!(contract.rsplit('/').next(), Some("status" | "cancel")) {
            None
        } else {
            let providers = tokio::time::timeout(
                Duration::from_secs(30),
                self.query_capabilities(&provider, "", pb::Transport::Mcp),
            )
            .await
            .context("Atlas capability query timed out")??;
            let available: Vec<_> = providers.iter().flat_map(|p| &p.capabilities).collect();
            let status = format!("{contract}/status");
            let cancel = format!("{contract}/cancel");
            match (
                available.iter().any(|c| c.contract_id == status),
                available.iter().any(|c| c.contract_id == cancel),
            ) {
                (false, false) => None,
                (true, true) => {
                    for cid in [&status, &cancel] {
                        let desc =
                            tokio::time::timeout(Duration::from_secs(30), self.query_contract(cid))
                                .await
                                .context("Atlas companion contract query timed out")??
                                .with_context(|| format!("unknown contract: {cid}"))?;
                        ensure!(desc.mode == "rpc", "MCP channel requires rpc mode: {cid}");
                    }
                    Some((status, cancel))
                }
                _ => bail!("async capability '{contract}' must declare both /status and /cancel"),
            }
        };
        let (channel_id, endpoint, params) = tokio::time::timeout(
            Duration::from_secs(30),
            self.allocate_channel(&consumer, &provider, &contract, pb::Transport::Mcp),
        )
        .await
        .context("ConnectCapability allocation outcome unconfirmed after timeout")?
        .context("ConnectCapability failed")?;
        let ch = Channel {
            provider_id: provider,
            contract_id: contract,
            transport: pb::Transport::Mcp,
            endpoint,
            channel_id,
            params,
            execution: Arc::new(Execution {
                atlas: self.inner.clone(),
                channels: Arc::downgrade(&self.channels),
                consumer_id: consumer,
                companions,
                session: AsyncMutex::new(None),
                close_lock: AsyncMutex::new(()),
                progress: Mutex::new(Progress {
                    started: false,
                    run_id: None,
                    terminal: false,
                    closed: false,
                    closing: false,
                    permit: Some(permit),
                }),
            }),
        };
        match tokio::time::timeout(Duration::from_secs(30), connect_mcp(&ch.endpoint)).await {
            Ok(Ok(session)) => *ch.execution.session.lock().await = Some(session),
            result => {
                let error = match result {
                    Ok(Err(error)) => error,
                    _ => anyhow::anyhow!("MCP initialization timed out"),
                };
                if let Err(cleanup) = ch.close().await {
                    self.channels
                        .live
                        .lock()
                        .unwrap()
                        .insert(ch.channel_id.clone(), ch.clone());
                    bail!("MCP setup failed: {error:#}; cleanup failed: {cleanup:#}");
                }
                return Err(error);
            }
        }
        self.channels
            .live
            .lock()
            .unwrap()
            .insert(ch.channel_id.clone(), ch.clone());
        robonix_scribe::debug!("[atlas-client] prepared channel {}", ch.channel_id);
        Ok(ch)
    }
}

impl Channel {
    pub fn is_async(&self) -> bool {
        self.execution.companions.is_some()
    }

    pub fn run_id(&self) -> Option<String> {
        self.execution.progress.lock().unwrap().run_id.clone()
    }

    pub fn is_terminal(&self) -> bool {
        self.execution.progress.lock().unwrap().terminal
    }

    pub fn is_closed(&self) -> bool {
        self.execution.progress.lock().unwrap().closed
    }

    /// Submit the single business request; dropping the waiter does not abandon tracking.
    pub async fn call(&self, request: Value) -> Result<Value> {
        ensure!(request.is_object(), "MCP request must be a JSON object");
        {
            let mut p = self.execution.progress.lock().unwrap();
            ensure!(
                !p.started && !p.closing && !p.closed,
                "a channel can execute only one capability call"
            );
            p.started = true;
        }
        let ch = self.clone();
        tokio::spawn(async move {
            let result = ch.invoke(request).await;
            match result {
                Ok(value) => {
                    let mut p = ch.execution.progress.lock().unwrap();
                    if ch.is_async() && value.get("accepted") == Some(&Value::Bool(false)) {
                        p.terminal = true;
                        return Ok(value);
                    }
                    let run_id = match value.get("run_id") {
                        None | Some(Value::Null) => String::new(),
                        Some(Value::String(s)) => s.clone(),
                        _ => return Err(ch.retained("MCP run_id must be a string")),
                    };
                    p.run_id = Some(run_id);
                    p.terminal = !ch.is_async();
                    Ok(value)
                }
                Err(error) => Err(ch.retained(&format!("MCP request failed: {error:#}"))),
            }
        })
        .await?
    }

    async fn invoke(&self, request: Value) -> Result<Value> {
        let session = self.execution.session.lock().await;
        let client = session.as_ref().context("MCP session unavailable")?;
        let name = self
            .contract_id
            .rsplit('/')
            .next()
            .unwrap_or(&self.contract_id);
        let result = client
            .call_tool(
                rmcp::model::CallToolRequestParams::new(name.to_owned())
                    .with_arguments(request.as_object().cloned().unwrap()),
            )
            .await
            .context("MCP protocol error")?;
        let text = result
            .content
            .iter()
            .filter_map(|c| match &c.raw {
                rmcp::model::RawContent::Text(t) => Some(t.text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        // Protocol completion is known even if the business payload cannot be decoded.
        if !self.is_async() {
            self.execution.progress.lock().unwrap().terminal = true;
        }
        ensure!(!result.is_error.unwrap_or(false), "MCP tool error: {text}");
        let value = match result.structured_content {
            Some(v) => v,
            None => serde_json::from_str(&text).context("MCP result decoding failed")?,
        };
        ensure!(value.is_object(), "MCP response must be a JSON object");
        Ok(value)
    }

    pub async fn status(&self) -> Result<Value> {
        self.companion(false).await
    }

    pub async fn cancel(&self) -> Result<Value> {
        self.companion(true).await
    }

    async fn companion(&self, cancel: bool) -> Result<Value> {
        let channel = self.clone();
        tokio::spawn(async move { channel.call_companion(cancel).await }).await?
    }

    async fn call_companion(&self, cancel: bool) -> Result<Value> {
        let run_id = {
            let p = self.execution.progress.lock().unwrap();
            ensure!(!p.closed, "channel already closed");
            p.run_id
                .clone()
                .ok_or_else(|| self.retained("no confirmed async execution"))?
        };
        let (status_id, cancel_id) = self
            .execution
            .companions
            .as_ref()
            .context("capability is synchronous")?;
        let cid = if cancel { cancel_id } else { status_id };
        let atlas = AtlasClient {
            inner: self.execution.atlas.clone(),
            channels: self
                .execution
                .channels
                .upgrade()
                .context("Atlas client no longer available")?,
        };
        let ch = atlas
            .connect_channel(
                &self.execution.consumer_id,
                &self.provider_id,
                cid,
                pb::Transport::Mcp,
            )
            .await?;
        let args = if run_id.is_empty() {
            json!({})
        } else {
            json!({"run_id": run_id})
        };
        let result = ch.call(args).await;
        let cleanup = ch.close().await;
        let value = result.map_err(|e| {
            self.retained(&format!("companion failed: {e:#}; cleanup: {cleanup:?}"))
        })?;
        cleanup?;
        if !cancel {
            let state = value
                .get("state")
                .and_then(Value::as_str)
                .context("status response missing state")?;
            let terminal = match state.to_ascii_uppercase().as_str() {
                "SUCCEEDED" | "FAILED" | "CANCELED" | "CANCELLED" | "TIMEOUT" => true,
                "PENDING" | "RUNNING" | "PAUSED" => false,
                _ => return Err(self.retained(&format!("unknown capability state: {state}"))),
            };
            let mut p = self.execution.progress.lock().unwrap();
            p.terminal |= terminal;
        }
        Ok(value)
    }

    /// Cancel active work and wait up to five seconds for confirmed termination.
    pub async fn close(&self) -> Result<()> {
        self.close_with_timeout(Duration::from_secs(5)).await
    }

    pub fn close_with_timeout(
        &self,
        timeout: Duration,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + '_>> {
        Box::pin(async move {
            {
                let mut p = self.execution.progress.lock().unwrap();
                if p.closed {
                    return Ok(());
                }
                p.closing = true;
            }
            let ch = self.clone();
            tokio::spawn(async move {
                let _closing = ch.execution.close_lock.lock().await;
                if ch.execution.progress.lock().unwrap().closed {
                    return Ok(());
                }
                let result = ch.finish_close(timeout).await;
                ch.execution.progress.lock().unwrap().closing = false;
                result.map_err(|e| ch.retained(&format!("close failed: {e:#}")))
            })
            .await?
        })
    }

    async fn finish_close(&self, timeout: Duration) -> Result<()> {
        let active = {
            let p = self.execution.progress.lock().unwrap();
            p.started && !p.terminal
        };
        if active {
            tokio::time::timeout(timeout, async {
                self.cancel().await?;
                loop {
                    self.status().await?;
                    if self.is_terminal() {
                        return Ok::<_, anyhow::Error>(());
                    }
                    tokio::time::sleep(Duration::from_secs(2)).await;
                }
            })
            .await
            .context("timed out waiting for execution termination")??;
        }
        {
            let mut session =
                tokio::time::timeout(Duration::from_secs(5), self.execution.session.lock())
                    .await
                    .context("MCP session still in use during cleanup")?;
            if let Some(client) = session.as_mut() {
                client
                    .close_with_timeout(Duration::from_secs(5))
                    .await
                    .context("MCP session cleanup failed")?
                    .context("MCP session cleanup timed out")?;
            }
            *session = None;
        }
        let mut atlas = self.execution.atlas.clone();
        tokio::time::timeout(
            Duration::from_secs(5),
            atlas.disconnect_capability(pb::DisconnectCapabilityRequest {
                channel_id: self.channel_id.clone(),
            }),
        )
        .await
        .context("Atlas disconnect timed out")??;
        let mut p = self.execution.progress.lock().unwrap();
        p.closed = true;
        p.permit.take();
        if let Some(channels) = self.execution.channels.upgrade() {
            channels.live.lock().unwrap().remove(&self.channel_id);
        }
        Ok(())
    }

    fn retained(&self, message: &str) -> anyhow::Error {
        anyhow::anyhow!(
            "{message}; channel {} retained; use get_channel() to recover",
            self.channel_id
        )
    }
}

async fn connect_mcp(endpoint: &str) -> Result<McpClient> {
    let base = if endpoint.starts_with("http://") || endpoint.starts_with("https://") {
        endpoint.to_owned()
    } else {
        format!("http://{endpoint}")
    };
    let uri = if base.contains("/mcp") {
        base
    } else {
        format!("{}/mcp", base.trim_end_matches('/'))
    };
    // Reinitialization must not replay a business request with uncertain effects.
    let config =
        rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig::with_uri(uri)
            .reinit_on_expired_session(false);
    let http = reqwest::Client::builder().no_proxy().build()?;
    ().serve(
        rmcp::transport::streamable_http_client::StreamableHttpClientTransport::with_client(
            http, config,
        ),
    )
    .await
    .context("MCP HTTP initialization failed")
}
