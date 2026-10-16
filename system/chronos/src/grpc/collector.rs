// SPDX-License-Identifier: MulanPSL-2.0

//! Owned ROS/query helper and bounded forwarding of clock pairs into the observation worker.

use std::process::Stdio;
use std::sync::{Arc, mpsc};
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use robonix_atlas::{client::AtlasClient, pb as atlas_pb};
use serde::Deserialize;
use tokio::io::{AsyncBufRead, AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::process::{Child, Command};
use tokio::sync::{oneshot, watch};
use tokio::task::JoinHandle;

use super::observation::{ObservationJob, ObservationWorker};
use super::pb::chronos::ObserveDeviceTimestampRequest;
use crate::{
    CapabilityTransportKind, ChronosConfig, DomainConfig, DomainStrategyConfig, QualityRuntime,
};

const IPC_LINE_LIMIT: usize = 4096;

/// Bound helper lines while reading, including unterminated output; preserve UTF-8 framing.
pub(super) async fn read_ipc_line(
    reader: &mut (impl AsyncBufRead + Unpin),
) -> Result<Option<String>> {
    let mut line = Vec::with_capacity(IPC_LINE_LIMIT + 1);
    let count = (&mut *reader)
        .take((IPC_LINE_LIMIT + 1) as u64)
        .read_until(b'\n', &mut line)
        .await?;
    if count == 0 {
        return Ok(None);
    }
    // A full-size CRLF frame needs one more delimiter byte, never more payload.
    if count == IPC_LINE_LIMIT + 1 && line.last() == Some(&b'\r') {
        let delimiter = reader
            .read_u8()
            .await
            .context("collector output exceeded IPC size limit")?;
        ensure!(
            delimiter == b'\n',
            "collector output exceeded IPC size limit"
        );
        line.pop();
    } else if line.last() == Some(&b'\n') {
        line.pop();
        if line.last() == Some(&b'\r') {
            line.pop();
        }
    }
    ensure!(
        line.len() <= IPC_LINE_LIMIT,
        "collector output exceeded IPC size limit"
    );
    Ok(Some(
        String::from_utf8(line).context("collector output is not UTF-8")?,
    ))
}

pub(super) struct Collector {
    stop: watch::Sender<bool>,
    task: JoinHandle<Result<()>>,
}

struct Subscription {
    stop: watch::Sender<bool>,
    task: JoinHandle<Result<()>>,
    edges: ChannelGuard,
}

/// Cancellation still schedules release of edges acquired during partial startup.
struct ChannelGuard {
    channels: Vec<String>,
    atlas: AtlasClient,
    owner: String,
    attempted: Vec<(String, String)>,
}

impl ChannelGuard {
    /// Recover edge IDs after an uncertain Connect reply, scoped to this collector's targets.
    async fn reconcile(&mut self) -> Result<()> {
        if self.attempted.is_empty() {
            return Ok(());
        }
        let response = tokio::time::timeout(
            Duration::from_secs(2),
            self.atlas
                .inner()
                .inspect_atlas(atlas_pb::InspectAtlasRequest {}),
        )
        .await
        .context("collector edge reconciliation timed out")??;
        let inspected: serde_json::Value = serde_json::from_str(&response.into_inner().json)?;
        let channels = inspected
            .get("channels")
            .and_then(serde_json::Value::as_object)
            .context("Atlas inspection has no channels object")?;
        for (id, channel) in channels {
            if channel["consumer_id"].as_str() == Some(&self.owner)
                && channel["transport"].as_str() == Some(atlas_pb::Transport::Ros2.as_str_name())
                && self.attempted.iter().any(|(provider, contract)| {
                    channel["provider_id"].as_str() == Some(provider)
                        && channel["contract_id"].as_str() == Some(contract)
                })
                && !self.channels.contains(id)
            {
                self.channels.push(id.clone());
            }
        }
        Ok(())
    }
}

impl Drop for ChannelGuard {
    /// Last-resort release after panic/drop; lifecycle cancellation normally awaits cleanup.
    fn drop(&mut self) {
        if self.channels.is_empty() {
            return;
        }
        let channels = std::mem::take(&mut self.channels);
        let mut atlas = self.atlas.clone();
        tokio::spawn(async move {
            if let Err(error) = disconnect(&mut atlas, &channels).await {
                robonix_scribe::error!(
                    "Chronos cancelled collector channel cleanup failed: {error:#}"
                );
            }
        });
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    domain_id: String,
    session_id: String,
    device_generation: u64,
    sequence_id: u64,
    device_timestamp_ns: i64,
    received_reference_ns: i64,
    received_steady_ns: u64,
    received_uncertainty_ns: u64,
}

impl Collector {
    /// Validate static settings, then wait for device activation without blocking Driver ACTIVATE.
    pub async fn start(
        config: &ChronosConfig,
        mut atlas: AtlasClient,
        owner: &str,
        runtime: Arc<QualityRuntime>,
        worker: &ObservationWorker,
        cancelled: watch::Receiver<bool>,
    ) -> Result<Self> {
        let contracts = validate_collection(config, &mut atlas).await?;
        let owner = owner.to_owned();
        let sender = worker.sender();
        let (stop, mut stopped) = watch::channel(false);
        let mut domains = tokio::task::JoinSet::new();
        for (domain_id, domain) in &config.chronos.core.domains {
            if !config.chronos.collects_domain(domain) {
                continue;
            }
            let domain_id = domain_id.clone();
            let domain = domain.clone();
            let atlas = atlas.clone();
            let owner = owner.clone();
            let mut stopped = stopped.clone();
            let contracts = contracts.clone();
            let runtime = runtime.clone();
            let observations = (sender.clone(), cancelled.clone());
            domains.spawn(async move {
                let result = async {
                    let Some(subscription) = Subscription::start(
                        &domain_id,
                        &domain,
                        atlas,
                        &owner,
                        stopped.clone(),
                        contracts,
                        runtime,
                        observations,
                    )
                    .await?
                    else {
                        return Ok(());
                    };
                    while !subscription.is_finished() && !*stopped.borrow() {
                        tokio::select! {
                            _ = stopped.changed() => break,
                            _ = tokio::time::sleep(Duration::from_millis(100)) => {}
                        }
                    }
                    subscription.stop().await
                }
                .await;
                result.with_context(|| format!("collect Chronos domain '{domain_id}'"))
            });
        }
        let failure_stop = stop.clone();
        let task = tokio::spawn(async move {
            let mut result = Ok(());
            while !domains.is_empty() {
                let next = tokio::select! {
                    next = domains.join_next() => next,
                    _ = stopped.changed(), if !*stopped.borrow() => {
                        continue;
                    }
                };
                if let Some(next) = next {
                    let domain = next
                        .context("join Chronos domain collector")
                        .and_then(|result| result);
                    if domain.is_err() {
                        failure_stop.send_replace(true);
                    }
                    result = super::service::combine_cleanup(result, domain);
                }
            }
            if let Err(error) = &result {
                robonix_scribe::error!("Chronos collection failed: {error:#}");
            }
            result
        });
        Ok(Self { stop, task })
    }

    pub fn is_finished(&self) -> bool {
        self.task.is_finished()
    }

    /// Cancel discovery or stop the owned subscription, and await all resource cleanup.
    pub async fn stop(self) -> Result<()> {
        self.stop.send_replace(true);
        self.task
            .await
            .context("join Chronos collection supervisor")?
    }
}

impl Subscription {
    /// Resolve one domain independently; rollback every opened Atlas edge on error.
    #[allow(clippy::too_many_arguments)]
    pub async fn start(
        domain_id: &str,
        domain: &DomainConfig,
        mut atlas: AtlasClient,
        owner: &str,
        mut stopped: watch::Receiver<bool>,
        contracts: Vec<atlas_pb::ContractDescriptor>,
        runtime: Arc<QualityRuntime>,
        observations: (mpsc::SyncSender<ObservationJob>, watch::Receiver<bool>),
    ) -> Result<Option<Self>> {
        let mut edges = ChannelGuard {
            channels: Vec::new(),
            atlas: atlas.clone(),
            owner: owner.into(),
            attempted: Vec::new(),
        };
        let result = async {
            let DomainStrategyConfig::AffineModel { observation, policy } = &domain.strategy else {
                anyhow::bail!("collection requires an affine domain");
            };
            let plan = if let Some(query) = &observation.query {
                for binding in &domain.capabilities {
                    if !wait_for_capability(&mut atlas, &binding.provider_id, &binding.contract_id,
                        binding.transport, domain_id, &mut stopped).await? {
                        return Ok(None);
                    }
                }
                robonix_scribe::info!("Chronos domain '{domain_id}' uses configured hardware clock query adapter '{}'", query.adapter);
                serde_json::json!({
                    "domain_id": domain_id, "query": {
                        "adapter": query.adapter, "options": query.options,
                        "tick_ns": query.tick_ns, "timeout_ns": query.timeout_ns,
                    },
                    "sampling_interval_ns": observation.sampling_interval_ns,
                    "wrap_period_ns": domain.timestamp.wrap_period_ns,
                    "max_drift_ppm": policy.max_drift_ppm,
                })
            } else {
                let binding = &domain.capabilities[0];
                let contract = contracts.iter().find(|contract| contract.id == binding.contract_id)
                    .with_context(|| format!("missing contract '{}'", binding.contract_id))?;
                edges.attempted.push((binding.provider_id.clone(), binding.contract_id.clone()));
                let (channel, topic, params, time) = loop {
                    if !wait_for_capability(&mut atlas, &binding.provider_id, &binding.contract_id,
                        binding.transport, domain_id, &mut stopped).await? {
                        return Ok(None);
                    }
                    let connected = tokio::time::timeout(Duration::from_secs(2), atlas.connect_capability_with_time(owner, &binding.provider_id, &binding.contract_id, atlas_pb::Transport::Ros2)).await.context("collector capability connection timed out")?;
                    match connected {
                        Ok(connected) => break connected,
                        Err(error) if error.downcast_ref::<tonic::Status>().is_some_and(|status| status.code() == tonic::Code::NotFound) => {
                            robonix_scribe::info!("Chronos source '{}' disappeared during connection; retrying discovery", binding.provider_id);
                        }
                        Err(error) => return Err(error),
                    }
                };
                edges.channels.push(channel);
                if let Some(time) = time {
                    ensure!(time.domain_id == domain_id,
                        "capability time domain differs from configured domain '{domain_id}'");
                }
                let qos = match params.kind {
                    Some(atlas_pb::transport_params::Kind::Ros2(params)) if !params.qos_profile.is_empty() => params.qos_profile,
                    _ => "best_effort".into(),
                };
                serde_json::json!({
                    "domain_id": domain_id, "topic": topic, "msg_type": contract.io_msg_type,
                    "qos": qos, "timestamp_field": binding.timestamp_field,
                    "session_id_field": binding.session_id_field,
                    "device_generation_field": binding.device_generation_field,
                    "sequence_id_field": binding.sequence_id_field,
                    "wrap_period_ns": domain.timestamp.wrap_period_ns,
                    "max_drift_ppm": policy.max_drift_ppm,
                })
            };
            if *stopped.borrow() {
                return Ok(None);
            }
            let (mut command, profile) = collector_command(observation.query.is_none())?;
            let scripts = tempfile::tempdir().context("create Chronos collector scripts")?;
            std::fs::write(scripts.path().join("collector.py"), include_bytes!("../../collector.py"))?;
            std::fs::write(scripts.path().join("chronos_query.py"), include_bytes!("../../chronos_query.py"))?;
            let mut child = command
                .arg("-u").arg(scripts.path().join("collector.py"))
                .env("RCUTILS_LOGGING_USE_STDOUT", "0")
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(Stdio::inherit())
                .kill_on_drop(true)
                .spawn()
                .context("start Chronos collector (python3 with configured adapters or rclpy)")?;
            let mut stdin = child.stdin.take().context("collector stdin")?;
            let mut stdout = BufReader::new(child.stdout.take().context("collector stdout")?);
            let startup = async {
                stdin
                    .write_all(format!("{}\n", serde_json::to_string(&[plan])?).as_bytes())
                    .await?;
                let ready = tokio::time::timeout(Duration::from_secs(5), read_ipc_line(&mut stdout))
                    .await
                    .context("Chronos collector subscription startup timed out")??;
                ensure!(
                    ready.as_deref() == Some("{\"ready\": true}"),
                    "Chronos collector failed before subscriptions were ready"
                );
                Ok::<_, anyhow::Error>(())
            };
            let startup = tokio::select! {
                result = startup => Some(result),
                _ = stopped.changed() => None,
            };
            match startup {
                Some(Ok(())) => {}
                None => {
                    stop_child(&mut child).await?;
                    return Ok(None);
                }
                Some(Err(error)) => {
                    if let Err(cleanup) = stop_child(&mut child).await {
                        return Err(error).context(format!(
                            "collector child startup cleanup also failed: {cleanup:#}"
                        ));
                    }
                    return Err(error);
                }
            }
            let (stop, mut stopped) = watch::channel(false);
            let task = tokio::spawn(async move {
                // Retain the XML until the owned helper is reaped, including on failure.
                let _profile = profile;
                let _scripts = scripts;
                let (sender, cancelled) = observations;
                let forwarding = async {
                    while let Some(line) = read_ipc_line(&mut stdout).await? {
                        let input: Input =
                            serde_json::from_str(&line).context("decode collector observation")?;
                        let result = forward(&runtime, &sender, &cancelled, input).await;
                        match result {
                            Ok(reply) if !reply.accepted => robonix_scribe::debug!(
                                "Chronos collector observation rejected: {}",
                                reply.reason
                            ),
                            Ok(_) => {}
                            Err(error)
                                if matches!(
                                    error.code(),
                                    tonic::Code::Unavailable
                                        | tonic::Code::ResourceExhausted
                                        | tonic::Code::DeadlineExceeded
                                        | tonic::Code::FailedPrecondition
                                ) =>
                            {
                                robonix_scribe::warn!(
                                    "Chronos collector dropped observation: {error}"
                                )
                            }
                            Err(error) => {
                                return Err(error).context("Chronos collector input rejected");
                            }
                        }
                        stdin.write_all(b"\n").await?;
                    }
                    anyhow::bail!("Chronos collector exited unexpectedly")
                };
                let result = tokio::select! {
                    result = forwarding => result,
                    _ = stopped.changed() => Ok(()),
                };
                let cleanup = stop_child(&mut child).await;
                super::service::combine_cleanup(result, cleanup)
            });
            Ok::<_, anyhow::Error>(Some((stop, task)))
        }
        .await;
        match result {
            Ok(Some((stop, task))) => Ok(Some(Self { stop, task, edges })),
            Ok(None) => {
                disconnect(&mut atlas, &edges.channels).await?;
                edges.channels.clear();
                Ok(None)
            }
            Err(error) => {
                let reconciliation = edges.reconcile().await;
                let cleanup = disconnect(&mut atlas, &edges.channels).await;
                if cleanup.is_ok() {
                    edges.channels.clear();
                }
                if let Err(failure) = super::service::combine_cleanup(reconciliation, cleanup) {
                    return Err(error)
                        .context(format!("collector startup cleanup failed: {failure:#}"));
                }
                Err(error)
            }
        }
    }

    pub fn is_finished(&self) -> bool {
        self.task.is_finished()
    }

    /// Stop and reap the child before releasing every Atlas consumer edge.
    pub async fn stop(mut self) -> Result<()> {
        self.stop.send_replace(true);
        let task = self
            .task
            .await
            .context("join Chronos collector")
            .and_then(|result| result);
        let edges = disconnect(&mut self.edges.atlas, &self.edges.channels).await;
        if edges.is_ok() {
            self.edges.channels.clear();
        }
        match (task, edges) {
            (Err(task), Err(edges)) => {
                Err(task.context(format!("collector channel cleanup also failed: {edges:#}")))
            }
            (task, edges) => task.and(edges),
        }
    }
}

/// Admit only fresh local clock pairs; fitting keeps the callback time and existing identities.
async fn forward(
    runtime: &QualityRuntime,
    sender: &mpsc::SyncSender<ObservationJob>,
    cancelled: &watch::Receiver<bool>,
    input: Input,
) -> Result<super::pb::chronos::ObserveDeviceTimestampResponse, tonic::Status> {
    if *cancelled.borrow() {
        return Err(tonic::Status::unavailable("Chronos is inactive"));
    }
    let request = ObserveDeviceTimestampRequest {
        domain_id: input.domain_id,
        session_id: input.session_id,
        device_generation: input.device_generation,
        sequence_id: input.sequence_id,
        device_timestamp_ns: input.device_timestamp_ns,
    };
    super::observation::validate(&request)?;
    let snapshot = runtime
        .try_collector_snapshot(
            input.received_reference_ns,
            input.received_steady_ns,
            input.received_uncertainty_ns,
        )
        .map_err(|error| tonic::Status::failed_precondition(error.to_string()))?
        .ok_or_else(|| {
            tonic::Status::resource_exhausted(
                "Chronos clock owner is busy; submit a fresh observation",
            )
        })?;
    let (reply, receiver) = oneshot::channel();
    sender
        .try_send(ObservationJob {
            request,
            snapshot,
            cancelled: cancelled.clone(),
            reply,
        })
        .map_err(|error| match error {
            mpsc::TrySendError::Full(_) => {
                tonic::Status::resource_exhausted("Chronos observation queue is full")
            }
            mpsc::TrySendError::Disconnected(_) => {
                tonic::Status::unavailable("Chronos observation worker stopped")
            }
        })?;
    tokio::time::timeout(Duration::from_millis(100), receiver)
        .await
        .map_err(|_| tonic::Status::deadline_exceeded("Chronos collector observation timed out"))?
        .map_err(|_| tonic::Status::unavailable("Chronos observation worker stopped"))?
}

/// Configure only the helper's default Fast DDS transport; retain caller overrides.
/// The caller owns the temporary XML until the child has been reaped.
fn collector_command(ros: bool) -> Result<(Command, Option<tempfile::NamedTempFile>)> {
    use std::io::Write;

    let mut command = Command::new("python3");
    if !ros {
        return Ok((command, None));
    }
    let middleware = std::env::var_os("RMW_IMPLEMENTATION");
    let custom_transport = [
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "FASTDDS_DEFAULT_PROFILES_FILE",
        "FASTDDS_BUILTIN_TRANSPORTS",
    ]
    .iter()
    .any(|key| std::env::var_os(key).is_some_and(|value| !value.is_empty()));
    if middleware.is_some_and(|value| value != "rmw_fastrtps_cpp") || custom_transport {
        return Ok((command, None));
    }
    let mut profile = tempfile::NamedTempFile::new().context("create collector DDS profile")?;
    profile
        .write_all(include_bytes!("../../collector-fastdds.xml"))
        .context("write collector DDS profile")?;
    command.env("FASTRTPS_DEFAULT_PROFILES_FILE", profile.path());
    robonix_scribe::info!(
        "Chronos collector requests Fast DDS UDP with 4 MiB send/receive buffers"
    );
    Ok((command, Some(profile)))
}

/// Fail invalid configurations before background discovery can wait for absent devices.
async fn validate_collection(
    config: &ChronosConfig,
    atlas: &mut AtlasClient,
) -> Result<Vec<atlas_pb::ContractDescriptor>> {
    let domains: Vec<_> = config
        .chronos
        .core
        .domains
        .iter()
        .filter(|(_, domain)| config.chronos.collects_domain(domain))
        .collect();
    let needs_contracts = domains
        .iter()
        .any(|(_, domain)| !domain.capabilities.is_empty());
    let contracts = if needs_contracts {
        tokio::time::timeout(Duration::from_secs(2), atlas.list_contracts(""))
            .await
            .context("collector contract discovery timed out")??
    } else {
        Vec::new()
    };
    for (domain_id, domain) in domains {
        let DomainStrategyConfig::AffineModel { observation, .. } = &domain.strategy else {
            unreachable!("collects_domain selects affine domains");
        };
        if observation.query.is_none() {
            ensure!(
                domain.capabilities.len() == 1,
                "ROS collection requires exactly one capability for domain '{domain_id}'"
            );
        }
        for binding in &domain.capabilities {
            let contract = contracts
                .iter()
                .find(|contract| contract.id == binding.contract_id)
                .with_context(|| format!("missing contract '{}'", binding.contract_id))?;
            if observation.query.is_none() {
                ensure!(
                    binding.transport == CapabilityTransportKind::Ros2
                        && contract.mode == "topic_out"
                        && !contract.io_msg_type.is_empty(),
                    "ROS collection requires a ros2 topic_out message contract"
                );
            }
        }
    }
    Ok(contracts)
}

/// Wait for an ACTIVE declaration with cancellable polling; discovery opens no Atlas edges.
async fn wait_for_capability(
    atlas: &mut AtlasClient,
    provider: &str,
    contract: &str,
    transport: CapabilityTransportKind,
    domain_id: &str,
    stopped: &mut watch::Receiver<bool>,
) -> Result<bool> {
    let transport = match transport {
        CapabilityTransportKind::Ros2 => atlas_pb::Transport::Ros2,
        CapabilityTransportKind::Grpc => atlas_pb::Transport::Grpc,
    };
    let mut last_report = None;
    while !*stopped.borrow() {
        let discovered = tokio::time::timeout(
            Duration::from_secs(2),
            atlas.query_capabilities(provider, contract, transport),
        )
        .await;
        if *stopped.borrow() {
            return Ok(false);
        }
        match discovered {
            Ok(Ok(providers))
                if providers.iter().any(|provider| {
                    provider.state == atlas_pb::LifecycleState::StateActive as i32
                        && provider.capabilities.iter().any(|capability| {
                            capability.contract_id == contract
                                && capability.transport == transport as i32
                        })
                }) =>
            {
                for capability in providers
                    .iter()
                    .filter(|provider| {
                        provider.state == atlas_pb::LifecycleState::StateActive as i32
                    })
                    .flat_map(|provider| &provider.capabilities)
                    .filter(|capability| {
                        capability.contract_id == contract
                            && capability.transport == transport as i32
                    })
                {
                    if let Some(time) = &capability.time_descriptor {
                        ensure!(
                            time.domain_id == domain_id,
                            "capability time domain differs from configured domain '{domain_id}'"
                        );
                    }
                }
                return Ok(true);
            }
            Ok(Err(error))
                if !error.downcast_ref::<tonic::Status>().is_some_and(|status| {
                    matches!(
                        status.code(),
                        tonic::Code::Unavailable
                            | tonic::Code::DeadlineExceeded
                            | tonic::Code::Cancelled
                    )
                }) =>
            {
                return Err(error).context("Chronos collector discovery failed");
            }
            result => {
                if last_report.is_none_or(|last: tokio::time::Instant| {
                    last.elapsed() >= Duration::from_secs(10)
                }) {
                    match result {
                        Ok(Ok(_)) => robonix_scribe::info!(
                            "Chronos collection waiting for ACTIVE provider '{provider}' capability '{contract}'"
                        ),
                        result => robonix_scribe::warn!(
                            "Chronos collection discovery unavailable for '{provider}' '{contract}': {result:?}"
                        ),
                    }
                    last_report = Some(tokio::time::Instant::now());
                }
            }
        }
        tokio::select! {
            _ = stopped.changed() => return Ok(false),
            _ = tokio::time::sleep(Duration::from_millis(250)) => {}
        }
    }
    Ok(false)
}

/// Reap even an already-exited child; kill is used because ROS shutdown may hang.
async fn stop_child(child: &mut Child) -> Result<()> {
    if child.try_wait()?.is_none() {
        child.start_kill()?;
    }
    child.wait().await.context("reap Chronos collector")?;
    Ok(())
}

/// Try every disconnect with a deadline and retain failures instead of swallowing them.
async fn disconnect(atlas: &mut AtlasClient, channels: &[String]) -> Result<()> {
    let mut errors = Vec::new();
    let mut pending = tokio::task::JoinSet::new();
    for channel in channels {
        let channel = channel.clone();
        let mut atlas = atlas.clone();
        pending.spawn(async move {
            match tokio::time::timeout(
                Duration::from_secs(1),
                atlas.disconnect_capability(&channel),
            )
            .await
            {
                Ok(Ok(_)) => Ok(()),
                result => Err(anyhow::anyhow!(
                    "disconnect collector channel '{channel}': {result:?}"
                )),
            }
        });
    }
    while let Some(result) = pending.join_next().await {
        match result {
            Ok(Ok(_)) => {}
            result => errors.push(format!("collector channel cleanup: {result:?}")),
        }
    }
    ensure!(errors.is_empty(), "{}", errors.join("; "));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[tokio::test]
    /// Legal handshake and boundary-size payloads remain separate despite fragmented reads.
    async fn ipc_lines_preserve_framing_at_the_limit() {
        let payload = "x".repeat(IPC_LINE_LIMIT);
        let bytes = format!("{{\"ready\": true}}\r\n{payload}\n{payload}\r\nlast").into_bytes();
        let mut reader = BufReader::with_capacity(7, std::io::Cursor::new(bytes));
        assert_eq!(
            read_ipc_line(&mut reader).await.unwrap().unwrap(),
            "{\"ready\": true}"
        );
        assert_eq!(read_ipc_line(&mut reader).await.unwrap().unwrap(), payload);
        assert_eq!(read_ipc_line(&mut reader).await.unwrap().unwrap(), payload);
        assert_eq!(read_ipc_line(&mut reader).await.unwrap().unwrap(), "last");
        assert!(read_ipc_line(&mut reader).await.unwrap().is_none());
    }

    #[tokio::test]
    /// Oversized output is rejected without waiting for a delimiter or helper exit.
    async fn ipc_rejects_unterminated_output_before_eof() {
        let (mut output, input) = tokio::io::duplex(IPC_LINE_LIMIT * 2);
        output
            .write_all(&vec![b'x'; IPC_LINE_LIMIT + 1])
            .await
            .unwrap();
        let mut reader = BufReader::with_capacity(7, input);
        let error = tokio::time::timeout(Duration::from_secs(1), read_ipc_line(&mut reader))
            .await
            .expect("oversized output must fail while the helper still holds stdout open")
            .unwrap_err();
        assert!(error.to_string().contains("IPC size limit"));
        drop(output);
    }

    #[tokio::test]
    /// The real bounded worker fits helper receive pairs, refuses stale input and honors cancellation.
    async fn forwarding_preserves_callback_time() {
        let runtime = Arc::new(
            QualityRuntime::start(crate::Chronos::new(super::super::tests::test_config()).unwrap())
                .unwrap(),
        );
        let worker = ObservationWorker::start(runtime.clone()).unwrap();
        let (cancel, cancelled) = watch::channel(false);
        let mut latest = (0, 0);
        for sequence in 1..=2 {
            let pair = runtime.snapshot().unwrap();
            let reference = pair.reference().timestamp_ns - 30_000_000;
            let steady = pair.steady().timestamp_ns - 30_000_000;
            let reply = forward(
                &runtime,
                &worker.sender(),
                &cancelled,
                Input {
                    domain_id: "device".into(),
                    session_id: "camera".into(),
                    device_generation: 1,
                    sequence_id: sequence,
                    device_timestamp_ns: steady as i64,
                    received_reference_ns: reference,
                    received_steady_ns: steady,
                    received_uncertainty_ns: pair.reference().uncertainty_ns,
                },
            )
            .await
            .unwrap();
            assert!(reply.accepted, "{}", reply.reason);
            latest = (reference, steady);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let model = runtime.model("device").unwrap().unwrap();
        assert_eq!(model.last_observation_steady_ns(), latest.1);
        assert!(
            (model
                .convert(latest.1 as i64)
                .unwrap()
                .reference_timestamp_ns
                - latest.0)
                .abs()
                <= 1
        );
        let stale = || Input {
            domain_id: "device".into(),
            session_id: "camera".into(),
            device_generation: 1,
            sequence_id: 3,
            device_timestamp_ns: latest.1 as i64 + 1,
            received_reference_ns: latest.0 - 200_000_000,
            received_steady_ns: latest.1 - 200_000_000,
            received_uncertainty_ns: 1,
        };
        assert_eq!(
            forward(&runtime, &worker.sender(), &cancelled, stale())
                .await
                .unwrap_err()
                .code(),
            tonic::Code::FailedPrecondition
        );
        cancel.send_replace(true);
        assert_eq!(
            forward(&runtime, &worker.sender(), &cancelled, stale())
                .await
                .unwrap_err()
                .code(),
            tonic::Code::Unavailable
        );
        assert_eq!(
            runtime
                .model("device")
                .unwrap()
                .unwrap()
                .last_observation_steady_ns(),
            latest.1
        );
        worker.stop().unwrap();
    }

    #[tokio::test]
    #[ignore = "requires python3 with sourced ROS, rclpy and sensor_msgs"]
    /// Cancellation reaps a ready ROS helper and releases its edge while another domain is absent.
    async fn waiting_for_second_domain_releases_partial_edges() {
        let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = reservation.local_addr().unwrap();
        drop(reservation);
        let contracts =
            robonix_atlas::contract_registry::ContractRegistry::load_from_capabilities_dir(
                &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../capabilities"),
            )
            .unwrap();
        let registry = Arc::new(robonix_atlas::service::AtlasRegistry::with_contracts(
            contracts,
        ));
        let server = tokio::spawn(robonix_atlas::service::serve_atlas(
            registry.clone(),
            address,
        ));
        let mut atlas =
            AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
                .await
                .unwrap();
        atlas
            .register_service("collector", "robonix/system/chronos", "")
            .await
            .unwrap();
        atlas
            .register_primitive("lidar", "robonix/primitive/lidar", "")
            .await
            .unwrap();
        atlas
            .declare_capability(
                "lidar",
                "robonix/lifecycle/driver",
                atlas_pb::Transport::Grpc,
                "127.0.0.1:1",
                robonix_atlas::client::grpc_params(
                    "capabilities/lifecycle/driver.v1.toml",
                    "robonix.contracts.RobonixLifecycleDriver",
                    "/robonix.contracts.RobonixLifecycleDriver/Driver",
                ),
            )
            .await
            .unwrap();
        atlas
            .declare_capability(
                "lidar",
                "robonix/primitive/lidar/lidar",
                atlas_pb::Transport::Ros2,
                "/lidar",
                atlas_pb::TransportParams {
                    kind: Some(atlas_pb::transport_params::Kind::Ros2(
                        atlas_pb::Ros2Params {
                            qos_profile: "reliable".into(),
                        },
                    )),
                },
            )
            .await
            .unwrap();
        atlas
            .set_lifecycle_state("lidar", atlas_pb::LifecycleState::StateInactive, "")
            .await
            .unwrap();
        atlas
            .set_lifecycle_state("lidar", atlas_pb::LifecycleState::StateActive, "")
            .await
            .unwrap();
        let mut config = super::super::tests::test_config();
        config.chronos.collect_observations = true;
        config.chronos.collect_ros_observations = true;
        let mut late = config.chronos.core.domains["device"].clone();
        late.capabilities[0].provider_id = "missing-lidar".into();
        config.chronos.core.domains.insert("late".into(), late);
        let runtime =
            Arc::new(QualityRuntime::start(crate::Chronos::new(config.clone()).unwrap()).unwrap());
        let worker = ObservationWorker::start(runtime.clone()).unwrap();
        let (_cancel, cancelled) = watch::channel(false);
        let collector = Collector::start(&config, atlas, "collector", runtime, &worker, cancelled)
            .await
            .unwrap();
        tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let inspected: serde_json::Value =
                    serde_json::from_str(&registry.inspect_json().await.unwrap()).unwrap();
                if inspected["channels"].as_object().unwrap().len() == 1 {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .unwrap();
        assert!(!collector.is_finished());
        tokio::time::timeout(Duration::from_secs(3), collector.stop())
            .await
            .unwrap()
            .unwrap();
        let inspected: serde_json::Value =
            serde_json::from_str(&registry.inspect_json().await.unwrap()).unwrap();
        assert!(inspected["channels"].as_object().unwrap().is_empty());
        server.abort();
    }

    #[tokio::test]
    /// A declared but inactive source cannot start collection; stopping discovery opens no edge.
    async fn discovery_waits_for_activation_and_cancels() {
        let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = reservation.local_addr().unwrap();
        drop(reservation);
        let registry = Arc::new(robonix_atlas::service::AtlasRegistry::default());
        let server = tokio::spawn(robonix_atlas::service::serve_atlas(
            registry.clone(),
            address,
        ));
        let mut atlas =
            AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
                .await
                .unwrap();
        atlas
            .register_primitive("camera", "robonix/primitive/camera", "")
            .await
            .unwrap();
        atlas
            .declare_capability(
                "camera",
                "robonix/primitive/camera/rgb",
                atlas_pb::Transport::Ros2,
                "/camera",
                atlas_pb::TransportParams {
                    kind: Some(atlas_pb::transport_params::Kind::Ros2(
                        atlas_pb::Ros2Params {
                            qos_profile: "reliable".into(),
                        },
                    )),
                },
            )
            .await
            .unwrap();
        let (stop, mut stopped) = watch::channel(false);
        let mut client = atlas.clone();
        let waiting = tokio::spawn(async move {
            wait_for_capability(
                &mut client,
                "camera",
                "robonix/primitive/camera/rgb",
                CapabilityTransportKind::Ros2,
                "camera_clock",
                &mut stopped,
            )
            .await
        });
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(!waiting.is_finished());
        stop.send_replace(true);
        assert!(
            !tokio::time::timeout(Duration::from_secs(1), waiting)
                .await
                .unwrap()
                .unwrap()
                .unwrap()
        );
        let inspected: serde_json::Value =
            serde_json::from_str(&registry.inspect_json().await.unwrap()).unwrap();
        assert!(inspected["channels"].as_object().unwrap().is_empty());
        server.abort();
    }

    #[tokio::test]
    /// A lost local edge ID is recovered from real Atlas data without touching other consumers.
    async fn reconciliation_finds_only_owned_ros_edges() {
        let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let address = reservation.local_addr().unwrap();
        drop(reservation);
        let registry = Arc::new(robonix_atlas::service::AtlasRegistry::default());
        let server = tokio::spawn(robonix_atlas::service::serve_atlas(registry, address));
        let mut atlas =
            AtlasClient::connect_with_retry(address.to_string(), 50, Duration::from_millis(10))
                .await
                .unwrap();
        for id in ["collector", "other"] {
            atlas
                .register_service(id, "robonix/system/chronos", "")
                .await
                .unwrap();
        }
        atlas
            .register_primitive("camera", "robonix/primitive/camera", "")
            .await
            .unwrap();
        atlas
            .declare_capability(
                "camera",
                "robonix/primitive/camera/rgb",
                atlas_pb::Transport::Ros2,
                "/camera",
                atlas_pb::TransportParams {
                    kind: Some(atlas_pb::transport_params::Kind::Ros2(
                        atlas_pb::Ros2Params {
                            qos_profile: "best_effort".into(),
                        },
                    )),
                },
            )
            .await
            .unwrap();
        let (own, _, _) = atlas
            .connect_capability(
                "collector",
                "camera",
                "robonix/primitive/camera/rgb",
                atlas_pb::Transport::Ros2,
            )
            .await
            .unwrap();
        let (other, _, _) = atlas
            .connect_capability(
                "other",
                "camera",
                "robonix/primitive/camera/rgb",
                atlas_pb::Transport::Ros2,
            )
            .await
            .unwrap();
        let mut edges = ChannelGuard {
            channels: Vec::new(),
            atlas: atlas.clone(),
            owner: "collector".into(),
            attempted: vec![("camera".into(), "robonix/primitive/camera/rgb".into())],
        };
        edges.reconcile().await.unwrap();
        assert_eq!(edges.channels, vec![own.clone()]);
        disconnect(&mut atlas, &edges.channels).await.unwrap();
        edges.channels.clear();
        assert!(!atlas.disconnect_capability(&own).await.unwrap());
        assert!(atlas.disconnect_capability(&other).await.unwrap());
        server.abort();
    }
}
