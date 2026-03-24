// SPDX-License-Identifier: MulanPSL-2.0

use anyhow::{Context, Result, bail};
use libloading::Library;
use rclrs::{
    Client, Context as RosContext, CreateBasicExecutor, Executor, Node, Service, SpinOptions,
};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration as StdDuration, Instant};

pub type QueryClientHandle<Svc> = Client<Svc>;
pub type QueryServerHandle<Svc> = Service<Svc>;

pub struct RuntimeNode {
    _context: RosContext,
    executor: Mutex<Option<Executor>>,
    node: Node,
}

impl RuntimeNode {
    pub fn new(node_name: &str) -> Result<Arc<Self>> {
        let context = RosContext::default_from_env().context("failed to create ROS2 context")?;
        let executor = context.create_basic_executor();
        let node = executor
            .create_node(node_name)
            .with_context(|| format!("failed to create ROS2 node '{node_name}'"))?;
        Ok(Arc::new(Self {
            _context: context,
            executor: Mutex::new(Some(executor)),
            node,
        }))
    }

    pub fn node(&self) -> &Node {
        &self.node
    }

    pub fn start_background_spin(self: &Arc<Self>) -> Result<()> {
        let executor = self
            .executor
            .lock()
            .map_err(|_| anyhow::anyhow!("executor mutex poisoned"))?
            .take()
            .context("executor already started")?;
        std::thread::Builder::new()
            .name(format!("rclrs-{}", self.node.name()))
            .spawn(move || {
                let mut executor = executor;
                let _ = executor.spin(SpinOptions::default());
            })
            .context("failed to spawn ROS2 executor thread")?;
        Ok(())
    }
}

pub struct RuntimeClient {
    inner: crate::meta_runtime::pb::robonix_runtime_client::RobonixRuntimeClient<
        tonic::transport::Channel,
    >,
}

impl RuntimeClient {
    pub async fn connect(endpoint: &str) -> Result<Self> {
        let endpoint = normalize_grpc_endpoint(endpoint);
        let inner = crate::meta_runtime::pb::robonix_runtime_client::RobonixRuntimeClient::connect(
            endpoint.clone(),
        )
        .await
        .with_context(|| format!("failed to connect runtime gRPC endpoint '{endpoint}'"))?;
        Ok(Self { inner })
    }

    pub async fn connect_with_retry(
        endpoint: &str,
        attempts: usize,
        delay: StdDuration,
    ) -> Result<Self> {
        let mut last_error = None;
        for _ in 0..attempts {
            match Self::connect(endpoint).await {
                Ok(client) => return Ok(client),
                Err(err) => {
                    last_error = Some(err);
                    tokio::time::sleep(delay).await;
                }
            }
        }
        Err(last_error.unwrap_or_else(|| anyhow::anyhow!("runtime gRPC connection failed")))
    }

    pub async fn register_node(
        &mut self,
        node_id: &str,
        namespace: &str,
        kind: &str,
        skill_md: Option<&str>,
    ) -> Result<String> {
        let req = crate::meta_runtime::pb::RegisterNodeRequest {
            node_id: node_id.to_string(),
            namespace: namespace.to_string(),
            kind: kind.to_string(),
            skill_md: skill_md.unwrap_or_default().to_string(),
        };
        let resp = self
            .inner
            .register_node(req)
            .await
            .context("register_node RPC failed")?
            .into_inner();
        Ok(resp.node_id)
    }

    pub async fn declare_interface(
        &mut self,
        node_id: &str,
        name: &str,
        supported_transports: Vec<String>,
        metadata_json: &str,
    ) -> Result<()> {
        let req = crate::meta_runtime::pb::DeclareInterfaceRequest {
            node_id: node_id.to_string(),
            name: name.to_string(),
            supported_transports,
            metadata_json: metadata_json.to_string(),
        };
        self.inner
            .declare_interface(req)
            .await
            .context("declare_interface RPC failed")?;
        Ok(())
    }

    pub async fn negotiate_channel(
        &mut self,
        consumer_id: &str,
        provider_node_id: &str,
        interface_name: &str,
        transport: &str,
    ) -> Result<(String, String)> {
        let req = crate::meta_runtime::pb::NegotiateChannelRequest {
            consumer_id: consumer_id.to_string(),
            provider_node_id: provider_node_id.to_string(),
            interface_name: interface_name.to_string(),
            transport: transport.to_string(),
        };
        let resp = self
            .inner
            .negotiate_channel(req)
            .await
            .context("negotiate_channel RPC failed")?
            .into_inner();
        Ok((resp.channel_id, resp.endpoint))
    }
}

pub fn create_query_server<Svc>(
    node: &RuntimeNode,
    channel_name: &str,
) -> Result<QueryServerHandle<Svc>>
where
    Svc: rosidl_runtime_rs::Service,
{
    node.node()
        .create_service::<Svc, _>(channel_name, |_request: Svc::Request| {
            Svc::Response::default()
        })
        .with_context(|| format!("failed to create ROS query server '{channel_name}'"))
}

pub fn create_query_client<Svc>(
    node: &RuntimeNode,
    channel_name: &str,
) -> Result<QueryClientHandle<Svc>>
where
    Svc: rosidl_runtime_rs::Service,
{
    node.node()
        .create_client::<Svc>(channel_name)
        .with_context(|| format!("failed to create ROS query client '{channel_name}'"))
}

pub async fn wait_for_service<Svc>(
    client: &QueryClientHandle<Svc>,
    timeout: StdDuration,
) -> Result<()>
where
    Svc: rosidl_runtime_rs::Service,
{
    let deadline = Instant::now() + timeout;
    loop {
        if client
            .service_is_ready()
            .context("failed to check query service readiness")?
        {
            return Ok(());
        }
        if Instant::now() >= deadline {
            bail!("query service was not ready before timeout");
        }
        tokio::time::sleep(StdDuration::from_millis(100)).await;
    }
}

pub async fn call_query<Svc>(
    client: &QueryClientHandle<Svc>,
    request: Svc::Request,
    timeout: StdDuration,
) -> Result<Svc::Response>
where
    Svc: rosidl_runtime_rs::Service,
{
    let promise = client
        .call(request)
        .map_err(|err| anyhow::anyhow!("query transport error: {err:?}"))?;
    tokio::time::timeout(timeout, promise)
        .await
        .context("query timed out")?
        .map_err(|_| anyhow::anyhow!("query response promise dropped"))
}

pub fn load_symbol<T>(library_name: &str, symbol_name: &[u8]) -> T
where
    T: Copy,
{
    let library = shared_library(library_name);
    // SAFETY: The caller provides the exact symbol type expected by the generated code.
    let symbol = unsafe { library.get::<T>(symbol_name) }.unwrap_or_else(|err| {
        panic!(
            "failed to load symbol '{}' from '{}': {err}",
            String::from_utf8_lossy(symbol_name),
            library_name
        )
    });
    *symbol
}

fn shared_library(library_name: &str) -> &'static Library {
    static LIBRARIES: OnceLock<Mutex<HashMap<String, &'static Library>>> = OnceLock::new();
    let libraries = LIBRARIES.get_or_init(|| Mutex::new(HashMap::new()));
    let mut guard = libraries.lock().expect("library cache mutex poisoned");
    if let Some(existing) = guard.get(library_name) {
        return existing;
    }
    // SAFETY: Loading a shared library is required to access ROS2 type support symbols.
    let library = unsafe { Library::new(library_name) }
        .unwrap_or_else(|err| panic!("failed to open shared library '{}': {err}", library_name));
    let leaked = Box::leak(Box::new(library));
    guard.insert(library_name.to_string(), leaked);
    leaked
}

fn normalize_grpc_endpoint(endpoint: &str) -> String {
    if endpoint.starts_with("http://") || endpoint.starts_with("https://") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    }
}
