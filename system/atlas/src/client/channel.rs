// SPDX-License-Identifier: MulanPSL-2.0
//! Consumer communication and execution ownership in the consumer process.

mod grpc;

use super::AtlasClient;
use crate::pb;
use anyhow::{Context, Result, bail, ensure};
use futures_util::{Stream, StreamExt};
use prost_reflect::{DescriptorPool, DynamicMessage};
use rmcp::ServiceExt;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;
use tokio::sync::{Mutex as AsyncMutex, OwnedSemaphorePermit, Semaphore};
use tokio::sync::{mpsc, watch};
use tokio_stream::wrappers::ReceiverStream;

type RequestStream = std::pin::Pin<Box<dyn Stream<Item = DynamicMessage> + Send>>;
type RequestError = Arc<Mutex<Option<anyhow::Error>>>;

type McpClient = rmcp::service::RunningService<rmcp::RoleClient, ()>;

/// Bounded execution ownership shared by all clones of an Atlas client.
pub(super) struct Channels {
    /// Executions remain available after the original caller has returned.
    live: Mutex<HashMap<String, Channel>>,
    /// Deployment descriptors shared by every clone of the client.
    descriptors: Mutex<DescriptorPool>,
    /// Includes connections still being prepared, preventing concurrent overflow.
    capacity: Arc<Semaphore>,
    /// Companion calls remain possible when all business execution slots are occupied.
    companion_capacity: Arc<Semaphore>,
}

impl Default for Channels {
    fn default() -> Self {
        Self {
            live: Mutex::new(HashMap::new()),
            descriptors: Mutex::new(DescriptorPool::new()),
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
    /// Register deployment-generated protobuf types without service-side reflection.
    pub fn add_descriptor_set(&self, bytes: &[u8]) -> Result<()> {
        let mut pool = self.channels.descriptors.lock().unwrap();
        let mut updated = pool.clone();
        updated.decode_file_descriptor_set(bytes)?;
        *pool = updated;
        Ok(())
    }

    /// Prepare one capability execution and its transport resources.
    pub fn connect_channel<'a>(
        &'a self,
        consumer_id: &'a str,
        provider_id: &'a str,
        contract_id: &'a str,
        transport: pb::Transport,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<Channel>> + Send + 'a>> {
        Box::pin(async move {
            ensure!(
                matches!(transport, pb::Transport::Mcp | pb::Transport::Grpc),
                "Rust execution channels support MCP and gRPC; use the Python client for ROS 2"
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
                    .prepare_channel(consumer, provider, contract, transport)
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
    pub fn get_channel(&self, channel_id: &str) -> Option<Channel> {
        self.channels.live.lock().unwrap().get(channel_id).cloned()
    }

    async fn prepare_channel(
        mut self,
        consumer: String,
        provider: String,
        contract: String,
        transport: pb::Transport,
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
            transport == pb::Transport::Grpc || descriptor.mode == "rpc",
            "MCP channel requires rpc mode: {contract}"
        );
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
                        ensure!(desc.mode == "rpc", "companion requires rpc mode: {cid}");
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
                let pool = self.channels.descriptors.lock().unwrap().clone();
                let binding =
                    grpc::Binding::connect(&ch.endpoint, &ch.params, &pool, &descriptor).await?;
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
                    .insert(ch.channel_id.clone(), ch.clone());
                bail!("channel setup failed: {error:#}; cleanup failed: {cleanup:#}");
            }
            return Err(error);
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
        ensure!(request.is_object(), "request must be a JSON object");
        let prepared = if self.transport == pb::Transport::Grpc {
            let binding = self.grpc_binding().await?;
            ensure!(
                !binding.method.is_client_streaming() && !binding.method.is_server_streaming(),
                "use the streaming method for this contract"
            );
            Some(binding.request(&request)?)
        } else {
            None
        };
        self.start()?;
        let ch = self.clone();
        tokio::spawn(async move {
            let result = ch.invoke(request, prepared).await;
            ch.complete(result)
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

    fn complete(&self, result: Result<Value>) -> Result<Value> {
        let value = result.map_err(|e| self.retained(&format!("request failed: {e:#}")))?;
        let mut p = self.execution.progress.lock().unwrap();
        if self.is_async() && value.get("accepted") == Some(&Value::Bool(false)) {
            p.terminal = true;
            return Ok(value);
        }
        let run_id = match value.get("run_id") {
            None | Some(Value::Null) => String::new(),
            Some(Value::String(s)) => s.clone(),
            _ => return Err(self.retained("run_id must be a string")),
        };
        p.run_id = Some(run_id);
        p.terminal = !self.is_async();
        Ok(value)
    }

    async fn grpc_binding(&self) -> Result<grpc::Binding> {
        self.execution
            .grpc
            .lock()
            .await
            .clone()
            .context("gRPC channel unavailable")
    }

    /// Start one request and yield its server response messages with bounded buffering.
    pub async fn server_stream(&self, request: Value) -> Result<ReceiverStream<Result<Value>>> {
        let binding = self.grpc_binding().await?;
        ensure!(
            !binding.method.is_client_streaming() && binding.method.is_server_streaming(),
            "contract is not a server stream"
        );
        let request = binding.request(&request)?;
        self.start()?;
        let (sender, receiver) = mpsc::channel(16);
        let ch = self.clone();
        tokio::spawn(async move {
            let result = async {
                let mut client = binding.client().await?;
                let response = client
                    .server_streaming(
                        tonic::Request::new(request),
                        binding.path.clone(),
                        binding.codec(),
                    )
                    .await?;
                ch.receive_stream(response.into_inner(), &sender).await
            }
            .await;
            if let Err(error) = result {
                let _ = sender
                    .send(Err(ch.retained(&format!("gRPC stream failed: {error:#}"))))
                    .await;
            }
        });
        Ok(ReceiverStream::new(receiver))
    }

    /// Stream request messages and receive the single final response.
    pub async fn client_stream<S>(&self, requests: S) -> Result<Value>
    where
        S: Stream<Item = Value> + Send + 'static,
    {
        let binding = self.grpc_binding().await?;
        ensure!(
            binding.method.is_client_streaming() && !binding.method.is_server_streaming(),
            "contract is not a client stream"
        );
        self.start()?;
        let ch = self.clone();
        tokio::spawn(async move {
            let result = async {
                let (requests, errors) = ch.grpc_requests(requests, binding.clone());
                let mut client = binding.client().await?;
                let response = client
                    .client_streaming(
                        tonic::Request::new(requests),
                        binding.path.clone(),
                        binding.codec(),
                    )
                    .await?;
                ch.execution.progress.lock().unwrap().terminal = true;
                if let Some(error) = errors.lock().unwrap().take() {
                    return Err(error);
                }
                grpc::response(&response.into_inner())
            }
            .await;
            ch.complete(result)
        })
        .await?
    }

    /// Exchange request and response streams without collecting either in memory.
    pub async fn bidirectional_stream<S>(
        &self,
        requests: S,
    ) -> Result<ReceiverStream<Result<Value>>>
    where
        S: Stream<Item = Value> + Send + 'static,
    {
        let binding = self.grpc_binding().await?;
        ensure!(
            binding.method.is_client_streaming() && binding.method.is_server_streaming(),
            "contract is not a bidirectional stream"
        );
        self.start()?;
        let (sender, receiver) = mpsc::channel(16);
        let ch = self.clone();
        tokio::spawn(async move {
            let result = async {
                let (requests, errors) = ch.grpc_requests(requests, binding.clone());
                let mut client = binding.client().await?;
                let response = client
                    .streaming(
                        tonic::Request::new(requests),
                        binding.path.clone(),
                        binding.codec(),
                    )
                    .await?;
                ch.receive_stream(response.into_inner(), &sender).await?;
                if let Some(error) = errors.lock().unwrap().take() {
                    return Err(error);
                }
                Ok(())
            }
            .await;
            if let Err(error) = result {
                let _ = sender
                    .send(Err(ch.retained(&format!("gRPC stream failed: {error:#}"))))
                    .await;
            }
        });
        Ok(ReceiverStream::new(receiver))
    }

    fn grpc_requests<S>(&self, requests: S, binding: grpc::Binding) -> (RequestStream, RequestError)
    where
        S: Stream<Item = Value> + Send + 'static,
    {
        let errors = Arc::new(Mutex::new(None));
        let stop = self.execution.stop_stream.subscribe();
        let errors_for_stream = errors.clone();
        let input = futures_util::stream::unfold(Box::pin(requests), move |mut input| {
            let binding = binding.clone();
            let errors = errors_for_stream.clone();
            let mut stop = stop.clone();
            async move {
                if *stop.borrow() {
                    return None;
                }
                let value = tokio::select! {
                    _ = stop.changed() => return None,
                    value = input.next() => value?,
                };
                match binding.request(&value) {
                    Ok(message) => Some((message, input)),
                    Err(error) => {
                        *errors.lock().unwrap() = Some(error);
                        None
                    }
                }
            }
        });
        (Box::pin(input), errors)
    }

    async fn receive_stream(
        &self,
        mut stream: tonic::Streaming<DynamicMessage>,
        sender: &mpsc::Sender<Result<Value>>,
    ) -> Result<()> {
        let mut stop = self.execution.stop_stream.subscribe();
        while let Some(message) = stream.message().await? {
            let value = grpc::response(&message)?;
            if !*stop.borrow() && !sender.is_closed() {
                tokio::select! {
                    _ = stop.changed() => {},
                    _ = sender.send(Ok(value)) => {},
                }
            }
        }
        self.execution.progress.lock().unwrap().terminal = true;
        Ok(())
    }

    async fn invoke(&self, request: Value, prepared: Option<DynamicMessage>) -> Result<Value> {
        if self.transport == pb::Transport::Grpc {
            let binding = self.grpc_binding().await?;
            let mut client = binding.client().await?;
            let response = client
                .unary(
                    tonic::Request::new(prepared.expect("gRPC request prepared before start")),
                    binding.path.clone(),
                    binding.codec(),
                )
                .await?;
            if !self.is_async() {
                self.execution.progress.lock().unwrap().terminal = true;
            }
            return grpc::response(&response.into_inner());
        }

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
                self.transport,
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
