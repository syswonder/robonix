// SPDX-License-Identifier: MulanPSL-2.0
//! Consumer communication and execution ownership in the consumer process.

#[doc(hidden)]
pub mod grpc;

use super::AtlasClient;
use crate::pb;
use anyhow::{Context, Result, bail, ensure};
use rmcp::ServiceExt;
use serde_json::{Value, json};
use std::any::Any;
use std::collections::HashMap;
use std::future::Future;
use std::marker::PhantomData;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;
use tokio::sync::watch;
use tokio::sync::{Mutex as AsyncMutex, OwnedSemaphorePermit, Semaphore};

type McpClient = rmcp::service::RunningService<rmcp::RoleClient, ()>;

/// Compile-time message types and communication for one capability contract.
pub trait ChannelContract: Clone + Send + Sync + 'static {
    type Input: Send + 'static;
    type Output: Send + 'static;
    type StatusResponse: Send + 'static;
    type CancelResponse: Send + 'static;

    const TRANSPORT: pb::Transport;
    /// Contract ID fixed by the binding; empty when selected at connection time.
    const ID: &'static str = "";
    const MODE: &'static str = "rpc";
    const SERVICE: &'static str = "";
    const METHOD: &'static str = "";
    const IDL: &'static str = "";

    fn matches_contract(contract: &pb::ContractDescriptor) -> bool {
        Self::ID.is_empty() || contract.id == Self::ID
    }

    fn validate_companion(_contract: &pb::ContractDescriptor, _status: bool) -> Result<()> {
        bail!("channel binding has no status/cancel contracts; regenerate its bindings")
    }

    fn validate_input(_request: &Self::Input) -> Result<()> {
        Ok(())
    }

    fn invoke(
        channel: Channel<Self>,
        request: Self::Input,
    ) -> impl Future<Output = Result<Self::Output>> + Send;

    fn execution(_response: &Self::Output) -> Result<(bool, String)> {
        Ok((true, String::new()))
    }

    fn status(
        _channel: Channel<Self>,
    ) -> impl Future<Output = Result<Self::StatusResponse>> + Send {
        async { bail!("capability is synchronous") }
    }

    fn cancel(
        _channel: Channel<Self>,
    ) -> impl Future<Output = Result<Self::CancelResponse>> + Send {
        async { bail!("capability is synchronous") }
    }

    fn status_state(_response: &Self::StatusResponse) -> Result<&str> {
        bail!("capability is synchronous")
    }
}

pub fn validate_contract<C: ChannelContract>(contract: &pb::ContractDescriptor) -> Result<()> {
    ensure!(
        C::matches_contract(contract),
        "contract does not match channel binding: {}",
        contract.id
    );
    ensure!(
        contract.mode == C::MODE,
        "contract mode does not match channel binding"
    );
    if !C::IDL.is_empty() {
        ensure!(
            contract.io_srv_type == C::IDL || contract.io_msg_type == C::IDL,
            "contract IDL does not match channel binding"
        );
    }
    Ok(())
}

/// JSON request/response binding for runtime-selected MCP capabilities.
#[derive(Clone)]
pub struct Mcp;

impl ChannelContract for Mcp {
    type Input = Value;
    type Output = Value;
    type StatusResponse = Value;
    type CancelResponse = Value;
    const TRANSPORT: pb::Transport = pb::Transport::Mcp;

    fn validate_companion(contract: &pb::ContractDescriptor, _status: bool) -> Result<()> {
        validate_contract::<Self>(contract)
    }

    fn validate_input(request: &Value) -> Result<()> {
        ensure!(request.is_object(), "request must be a JSON object");
        Ok(())
    }

    async fn invoke(channel: Channel<Self>, request: Value) -> Result<Value> {
        let response = channel.invoke_mcp(request).await?;
        let (accepted, run_id) = Self::execution(&response)?;
        channel.complete(accepted, run_id);
        Ok(response)
    }

    fn execution(response: &Value) -> Result<(bool, String)> {
        let run_id = match response.get("run_id") {
            None | Some(Value::Null) => String::new(),
            Some(Value::String(id)) => id.clone(),
            _ => bail!("run_id must be a string"),
        };
        Ok((
            response.get("accepted") != Some(&Value::Bool(false)),
            run_id,
        ))
    }

    async fn status(channel: Channel<Self>) -> Result<Value> {
        channel
            .companion_call::<Self>(true, |run_id| Ok(companion_request(run_id)))
            .await
    }

    async fn cancel(channel: Channel<Self>) -> Result<Value> {
        channel
            .companion_call::<Self>(false, |run_id| Ok(companion_request(run_id)))
            .await
    }

    fn status_state(response: &Value) -> Result<&str> {
        response
            .get("state")
            .and_then(Value::as_str)
            .context("status response missing state")
    }
}

fn companion_request(run_id: String) -> Value {
    if run_id.is_empty() {
        json!({})
    } else {
        json!({"run_id": run_id})
    }
}

/// Bounded execution ownership shared by all clones of an Atlas client.
pub(super) struct Channels {
    /// Executions remain available after the original caller has returned.
    live: Mutex<HashMap<String, Box<dyn Any + Send + Sync>>>,
    /// Includes connections still being prepared, preventing concurrent overflow.
    capacity: Arc<Semaphore>,
    /// Companion calls remain possible when all business execution slots are occupied.
    companion_capacity: Arc<Semaphore>,
}

/// Allocations per Atlas client unless `ROBONIX_ATLAS_CHANNEL_LIMIT` overrides it.
const DEFAULT_CHANNEL_LIMIT: usize = 1024;
/// Part of the limit that only status/cancel companions may use.
const COMPANION_SLOTS: usize = 2;

fn channel_limit() -> usize {
    let Ok(raw) = std::env::var("ROBONIX_ATLAS_CHANNEL_LIMIT") else {
        return DEFAULT_CHANNEL_LIMIT;
    };
    match raw.trim().parse::<usize>() {
        Ok(limit) if limit > COMPANION_SLOTS => limit,
        _ => {
            robonix_scribe::warn!(
                "[atlas-client] ignoring ROBONIX_ATLAS_CHANNEL_LIMIT={raw:?}; \
                 expected an integer above {COMPANION_SLOTS}, using {DEFAULT_CHANNEL_LIMIT}"
            );
            DEFAULT_CHANNEL_LIMIT
        }
    }
}

impl Default for Channels {
    fn default() -> Self {
        Self {
            live: Mutex::new(HashMap::new()),
            capacity: Arc::new(Semaphore::new(channel_limit() - COMPANION_SLOTS)),
            companion_capacity: Arc::new(Semaphore::new(COMPANION_SLOTS)),
        }
    }
}

/// One capability execution and its initialized communication resources.
#[derive(Clone)]
pub struct Channel<C: ChannelContract = Mcp> {
    /// Compile-time contract binding; communication state is shared by clones.
    contract: PhantomData<C>,
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
    /// Generated gRPC binding, prepared before exposing the channel.
    grpc: AsyncMutex<Option<grpc::Binding>>,
    /// Close requests stop stream input and let output drain to confirmed completion.
    stop_stream: watch::Sender<bool>,
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
    /// Prepare one capability execution and its transport resources.
    pub fn connect_channel<'a, C: ChannelContract>(
        &'a self,
        consumer_id: &'a str,
        provider_id: &'a str,
        contract_id: &'a str,
        transport: pb::Transport,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<Channel<C>>> + Send + 'a>> {
        Box::pin(async move {
            ensure!(
                transport == C::TRANSPORT,
                "transport does not match channel binding"
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
                let result = atlas
                    .prepare_channel::<C>(consumer, provider, contract, transport)
                    .await;
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
    pub fn get_channel<C: ChannelContract>(&self, channel_id: &str) -> Result<Option<Channel<C>>> {
        self.channels
            .live
            .lock()
            .unwrap()
            .get(channel_id)
            .map(|value| {
                value
                    .downcast_ref::<Channel<C>>()
                    .cloned()
                    .context("channel binding does not match retained execution")
            })
            .transpose()
    }

    async fn prepare_channel<C: ChannelContract>(
        mut self,
        consumer: String,
        provider: String,
        contract: String,
        transport: pb::Transport,
    ) -> Result<Channel<C>> {
        let companion = matches!(contract.rsplit('/').next(), Some("status" | "cancel"));
        let permit = self.channels.capacity.clone()
            .try_acquire_owned()
            .or_else(|error| if companion {
                self.channels.companion_capacity.clone().try_acquire_owned()
            } else { Err(error) })
            .context("Atlas client channel capacity exhausted (two slots reserved for companions); close existing channels or raise ROBONIX_ATLAS_CHANNEL_LIMIT")?;
        let descriptor =
            tokio::time::timeout(Duration::from_secs(30), self.query_contract(&contract))
                .await
                .context("Atlas contract query timed out")??
                .with_context(|| format!("unknown contract: {contract}"))?;
        ensure!(
            descriptor.id == contract,
            "Atlas returned a different contract"
        );
        validate_contract::<C>(&descriptor)?;
        let companions = if descriptor.mode != "rpc"
            || matches!(contract.rsplit('/').next(), Some("status" | "cancel"))
        {
            None
        } else {
            let providers = tokio::time::timeout(
                Duration::from_secs(30),
                self.query_capabilities(&provider, "", pb::Transport::Unspecified),
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
                        ensure!(
                            available
                                .iter()
                                .any(|c| c.contract_id == *cid && c.transport == transport as i32),
                            "async companion {cid} must support {transport:?}"
                        );
                        let desc =
                            tokio::time::timeout(Duration::from_secs(30), self.query_contract(cid))
                                .await
                                .context("Atlas companion contract query timed out")??
                                .with_context(|| format!("unknown contract: {cid}"))?;
                        ensure!(desc.id == *cid, "Atlas returned a different contract");
                        C::validate_companion(&desc, cid == &status)?;
                    }
                    Some((status, cancel))
                }
                _ => bail!("async capability '{contract}' must declare both /status and /cancel"),
            }
        };
        let (channel_id, endpoint, params) = tokio::time::timeout(
            Duration::from_secs(30),
            self.allocate_channel(&consumer, &provider, &contract, transport),
        )
        .await
        .context("ConnectCapability allocation outcome unconfirmed after timeout")?
        .context("ConnectCapability failed")?;
        let ch = Channel {
            contract: PhantomData,
            provider_id: provider,
            contract_id: contract,
            transport,
            endpoint,
            channel_id,
            params,
            execution: Arc::new(Execution {
                atlas: self.inner.clone(),
                channels: Arc::downgrade(&self.channels),
                consumer_id: consumer,
                companions,
                session: AsyncMutex::new(None),
                grpc: AsyncMutex::new(None),
                stop_stream: watch::channel(false).0,
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
        let setup = async {
            if transport == pb::Transport::Mcp {
                *ch.execution.session.lock().await = Some(connect_mcp(&ch.endpoint).await?);
            } else {
                let binding =
                    grpc::Binding::connect::<C>(&ch.endpoint, &ch.params, &descriptor).await?;
                *ch.execution.grpc.lock().await = Some(binding);
            }
            Ok::<_, anyhow::Error>(())
        };
        if let Err(error) = tokio::time::timeout(Duration::from_secs(30), setup)
            .await
            .context("channel initialization timed out")
            .and_then(|r| r)
        {
            if let Err(cleanup) = ch.close().await {
                self.channels
                    .live
                    .lock()
                    .unwrap()
                    .insert(ch.channel_id.clone(), Box::new(ch.clone()));
                bail!("channel setup failed: {error:#}; cleanup failed: {cleanup:#}");
            }
            return Err(error);
        }
        self.channels
            .live
            .lock()
            .unwrap()
            .insert(ch.channel_id.clone(), Box::new(ch.clone()));
        robonix_scribe::debug!("[atlas-client] prepared channel {}", ch.channel_id);
        Ok(ch)
    }
}

impl<C: ChannelContract> Channel<C> {
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

    /// Submit one execution; dropping its waiter retains completion tracking.
    pub async fn call(&self, request: C::Input) -> Result<C::Output> {
        C::validate_input(&request)?;
        self.start()?;
        let channel = self.clone();
        tokio::spawn(async move {
            C::invoke(channel.clone(), request)
                .await
                .map_err(|error| channel.retained(&format!("request failed: {error:#}")))
        })
        .await?
    }

    fn start(&self) -> Result<()> {
        let mut p = self.execution.progress.lock().unwrap();
        ensure!(
            !p.started && !p.closing && !p.closed,
            "a channel can execute only one capability call"
        );
        p.started = true;
        Ok(())
    }

    fn complete(&self, accepted: bool, run_id: String) {
        let mut progress = self.execution.progress.lock().unwrap();
        if self.is_async() && !accepted {
            progress.terminal = true;
            return;
        }
        progress.run_id = Some(run_id);
        progress.terminal = !self.is_async();
    }

    async fn invoke_mcp(&self, request: Value) -> Result<Value> {
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
        let rejected = result.is_error.unwrap_or(false);
        // A tool error rejects the start before there is a run to cancel.
        if rejected || !self.is_async() {
            self.execution.progress.lock().unwrap().terminal = true;
        }
        ensure!(!rejected, "MCP tool error: {text}");
        let value = match result.structured_content {
            Some(v) => {
                // FastMCP may wrap the structured reply while leaving the text reply flat.
                if v.as_object().is_some_and(|object| object.len() == 1)
                    && let Some(inner) = v.get("result").filter(|inner| inner.is_object())
                    && let Ok(flat) = parse_tool_text(&text)
                    && &flat == inner
                {
                    flat
                } else {
                    v
                }
            }
            None => parse_tool_text(&text).context("MCP result decoding failed")?,
        };
        ensure!(value.is_object(), "MCP response must be a JSON object");
        Ok(value)
    }

    pub async fn status(&self) -> Result<C::StatusResponse> {
        let channel = self.clone();
        tokio::spawn(async move {
            let response = C::status(channel.clone()).await?;
            let state = C::status_state(&response)?;
            let terminal = match state.to_ascii_uppercase().as_str() {
                "SUCCEEDED" | "FAILED" | "CANCELED" | "CANCELLED" | "TIMEOUT" => true,
                "PENDING" | "RUNNING" | "PAUSED" => false,
                _ => return Err(channel.retained(&format!("unknown capability state: {state}"))),
            };
            channel.execution.progress.lock().unwrap().terminal |= terminal;
            Ok(response)
        })
        .await?
    }

    pub async fn cancel(&self) -> Result<C::CancelResponse> {
        let channel = self.clone();
        tokio::spawn(async move { C::cancel(channel).await }).await?
    }

    #[doc(hidden)]
    pub async fn companion_call<S: ChannelContract>(
        &self,
        status: bool,
        request: impl FnOnce(String) -> Result<S::Input> + Send,
    ) -> Result<S::Output> {
        let run_id = {
            let progress = self.execution.progress.lock().unwrap();
            ensure!(!progress.closed, "channel already closed");
            progress
                .run_id
                .clone()
                .ok_or_else(|| self.retained("no confirmed async execution"))?
        };
        let (status_id, cancel_id) = self
            .execution
            .companions
            .as_ref()
            .context("capability is synchronous")?;
        let cid = if status { status_id } else { cancel_id };
        let request = request(run_id)
            .map_err(|error| self.retained(&format!("invalid companion request: {error:#}")))?;
        let atlas = AtlasClient {
            inner: self.execution.atlas.clone(),
            channels: self
                .execution
                .channels
                .upgrade()
                .context("Atlas client no longer available")?,
        };
        let channel = atlas
            .connect_channel::<S>(
                &self.execution.consumer_id,
                &self.provider_id,
                cid,
                self.transport,
            )
            .await?;
        let result = channel.call(request).await;
        let cleanup = channel.close().await;
        let response = result.map_err(|error| {
            self.retained(&format!(
                "companion failed: {error:#}; cleanup: {cleanup:?}"
            ))
        })?;
        cleanup?;
        Ok(response)
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
            self.execution.stop_stream.send_replace(true);
            tokio::time::timeout(timeout, async {
                if self.is_async() {
                    self.cancel().await?;
                }
                loop {
                    if self.is_async() {
                        self.status().await?;
                    }
                    if self.is_terminal() {
                        return Ok::<_, anyhow::Error>(());
                    }
                    tokio::time::sleep(if self.is_async() {
                        Duration::from_secs(2)
                    } else {
                        Duration::from_millis(50)
                    })
                    .await;
                }
            })
            .await
            .context("timed out waiting for execution termination")??;
        }
        *self.execution.grpc.lock().await = None;
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

/// Parse an MCP tool's text reply. Python MCP servers write non-finite floats
/// (e.g. LaserScan ranges) as `NaN`/`Infinity`, which JSON lacks; read them as
/// null, as FastMCP's structured output does.
fn parse_tool_text(text: &str) -> serde_json::Result<Value> {
    let mut json = String::with_capacity(text.len());
    let (mut rest, mut in_string, mut escaped) = (text, false, false);
    while let Some(c) = rest.chars().next() {
        if !in_string
            && let Some(token) = ["-Infinity", "Infinity", "NaN"]
                .into_iter()
                .find(|token| rest.starts_with(token))
        {
            json.push_str("null");
            rest = &rest[token.len()..];
            continue;
        }
        if in_string {
            if escaped {
                escaped = false;
            } else if c == '\\' {
                escaped = true;
            } else if c == '"' {
                in_string = false;
            }
        } else if c == '"' {
            in_string = true;
        }
        json.push(c);
        rest = &rest[c.len_utf8()..];
    }
    serde_json::from_str(&json)
}

async fn connect_mcp(endpoint: &str) -> Result<McpClient> {
    let base = if endpoint.starts_with("http://") || endpoint.starts_with("https://") {
        endpoint.to_owned()
    } else {
        format!("http://{endpoint}")
    };
    let mut uri = reqwest::Url::parse(&base).context("invalid MCP endpoint")?;
    if uri.path() == "/" {
        uri.set_path("/mcp");
    }
    // Reinitialization must not replay a business request with uncertain effects.
    let config =
        rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig::with_uri(
            uri.as_str(),
        )
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tool_text_reads_non_finite_floats_as_null() {
        let value = parse_tool_text(
            r#"{"ranges": [0.5, Infinity, -Infinity, NaN], "note": "NaN \"Infinity\""}"#,
        )
        .unwrap();
        assert_eq!(
            value,
            json!({"ranges": [0.5, null, null, null], "note": "NaN \"Infinity\""})
        );
        assert!(parse_tool_text("{\"x\": Inf}").is_err());
    }
}
