// SPDX-License-Identifier: MulanPSL-2.0
//! Shared Driver lifecycle for the Executor service.

use crate::active_plans::PlanRuntime;
use crate::pb::contracts::robonix_lifecycle_driver_client::RobonixLifecycleDriverClient;
use crate::pb::contracts::robonix_lifecycle_driver_server::RobonixLifecycleDriver;
use crate::pb::lifecycle::{DriverRequest, DriverResponse};
use anyhow::{Context, Result};
use robonix_atlas::client::AtlasClient;
use robonix_atlas::pb::LifecycleState;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, watch};
use tonic::{Request, Response, Status};

pub const SHARED_DRIVER_CONTRACT: &str = "robonix/lifecycle/driver";
const CMD_INIT: u32 = 0;
const CMD_ACTIVATE: u32 = 1;
const CMD_DEACTIVATE: u32 = 2;
const CMD_SHUTDOWN: u32 = 3;

/// Publish lifecycle transitions and request graceful gRPC shutdown.
#[derive(Clone)]
pub struct ExecutorLifecycleDriver {
    /// Registry connection for authoritative provider state.
    atlas: AtlasClient,
    /// Registered Executor provider identity.
    provider_id: String,
    /// Execution admission and active work shared with both Execute RPCs.
    runtime: PlanRuntime,
    /// Serialize lifecycle commands through their Atlas acknowledgement.
    transition_lock: Arc<Mutex<()>>,
    /// Retain shutdown requests until the server and heartbeat observe them.
    shutdown_tx: watch::Sender<bool>,
}

impl ExecutorLifecycleDriver {
    pub fn new(atlas: AtlasClient, provider_id: String, runtime: PlanRuntime) -> Self {
        let (shutdown_tx, _) = watch::channel(false);
        Self {
            atlas,
            provider_id,
            runtime,
            transition_lock: Arc::new(Mutex::new(())),
            shutdown_tx,
        }
    }

    pub fn subscribe_shutdown(&self) -> watch::Receiver<bool> {
        self.shutdown_tx.subscribe()
    }

    async fn transition(&self, command: u32) -> Result<&'static str> {
        let _transition = self.transition_lock.lock().await;
        let (state, label) = match command {
            CMD_INIT | CMD_DEACTIVATE => (LifecycleState::StateInactive, "inactive"),
            CMD_ACTIVATE => (LifecycleState::StateActive, "active"),
            CMD_SHUTDOWN => (LifecycleState::StateTerminated, "terminated"),
            _ => anyhow::bail!("unknown lifecycle command code {command}"),
        };
        if command != CMD_ACTIVATE {
            self.runtime
                .close_admission()
                .await
                .map_err(anyhow::Error::msg)?;
        }
        // Do not reopen admission if publishing a non-active state fails.
        self.atlas
            .clone()
            .set_lifecycle_state(&self.provider_id, state, "")
            .await
            .with_context(|| format!("publish lifecycle state for '{}'", self.provider_id))?;
        if command == CMD_ACTIVATE {
            self.runtime.open_admission().await;
        }
        if command == CMD_SHUTDOWN {
            self.shutdown_tx.send_replace(true);
        }
        Ok(label)
    }
}

#[tonic::async_trait]
impl RobonixLifecycleDriver for ExecutorLifecycleDriver {
    async fn driver(
        &self,
        request: Request<DriverRequest>,
    ) -> Result<Response<DriverResponse>, Status> {
        let response = match self.transition(request.into_inner().command).await {
            Ok(state) => DriverResponse {
                ok: true,
                state: state.into(),
                error: String::new(),
            },
            Err(error) => DriverResponse {
                ok: false,
                state: "error".into(),
                error: format!("{error:#}"),
            },
        };
        Ok(Response::new(response))
    }
}

/// Preserve a shutdown sent before a waiter starts polling.
pub async fn wait_for_shutdown(mut shutdown: watch::Receiver<bool>) {
    while !*shutdown.borrow_and_update() {
        if shutdown.changed().await.is_err() {
            break;
        }
    }
}

/// Use a dialable address of the same family when binding all interfaces.
pub fn advertised_endpoint(listen_addr: SocketAddr) -> String {
    let ip = match listen_addr.ip() {
        IpAddr::V4(ip) if ip.is_unspecified() => IpAddr::V4(Ipv4Addr::LOCALHOST),
        IpAddr::V6(ip) if ip.is_unspecified() => IpAddr::V6(Ipv6Addr::LOCALHOST),
        ip => ip,
    };
    SocketAddr::new(ip, listen_addr.port()).to_string()
}

/// The listener is already bound; self-dial proves the shared Driver is serving
/// before publishing the legal REGISTERED → INACTIVE → ACTIVE transitions.
pub async fn initialize_and_activate(endpoint: &str) -> Result<()> {
    tokio::time::timeout(Duration::from_secs(5), async {
        let mut client = RobonixLifecycleDriverClient::connect(format!("http://{endpoint}"))
            .await
            .context("connect startup lifecycle Driver")?;
        for command in [CMD_INIT, CMD_ACTIVATE] {
            let response = client
                .driver(DriverRequest {
                    command,
                    config_json: "{}".into(),
                })
                .await
                .context("call startup lifecycle Driver")?
                .into_inner();
            if !response.ok {
                anyhow::bail!("startup lifecycle Driver failed: {}", response.error);
            }
        }
        Ok(())
    })
    .await
    .context("startup lifecycle Driver timed out")?
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grpc_server::ExecutorServiceImpl;
    use crate::pb::contracts::robonix_lifecycle_driver_server::RobonixLifecycleDriverServer;
    use crate::pb::contracts::robonix_system_executor_execute_client::RobonixSystemExecutorExecuteClient;
    use crate::pb::contracts::robonix_system_executor_execute_server::RobonixSystemExecutorExecuteServer;
    use crate::pb::contracts::robonix_system_executor_execute_stream_client::RobonixSystemExecutorExecuteStreamClient;
    use crate::pb::contracts::robonix_system_executor_execute_stream_server::RobonixSystemExecutorExecuteStreamServer;
    use crate::pb::contracts::robonix_system_executor_get_health_client::RobonixSystemExecutorGetHealthClient;
    use crate::pb::contracts::robonix_system_executor_get_health_server::RobonixSystemExecutorGetHealthServer;
    use crate::pb::module_health::GetModuleHealthRequest;
    use crate::pb::pilot::{Plan, RtdlNode};
    use crate::verification::VerificationPolicy;
    use robonix_atlas::client::grpc_params;
    use robonix_atlas::pb::{Kind, Transport};
    use robonix_atlas::service::{AtlasRegistry, serve_atlas};

    #[test]
    fn wildcard_endpoints_keep_the_address_family() {
        for (listen, expected) in [
            ("0.0.0.0:51001", "127.0.0.1:51001"),
            ("[::]:51001", "[::1]:51001"),
            ("127.0.0.2:51001", "127.0.0.2:51001"),
        ] {
            assert_eq!(advertised_endpoint(listen.parse().unwrap()), expected);
        }
    }

    async fn assert_plan_admission(
        execute: &mut RobonixSystemExecutorExecuteClient<tonic::transport::Channel>,
        active: bool,
    ) {
        let result = execute
            .execute(Plan {
                plan_id: "admission-check".into(),
                nodes: vec![RtdlNode {
                    op_id: "root".into(),
                    description: "check lifecycle admission".into(),
                    ..Default::default()
                }],
                ..Default::default()
            })
            .await;
        if !active {
            assert!(
                result
                    .unwrap_err()
                    .message()
                    .contains("not accepting plans")
            );
            return;
        }
        let mut stream = result.unwrap().into_inner();
        let mut completed = false;
        while let Some(event) = stream.message().await.unwrap() {
            if let Some(complete) = event.plan_complete {
                assert!(!complete.any_failed);
                completed = true;
            }
        }
        assert!(completed, "admitted plan must report completion");
    }

    /// Exercise the generated Driver RPC against Atlas, including rejected
    /// transitions and shutdown acknowledgement before server exit.
    #[tokio::test]
    async fn driver_publishes_only_legal_transitions() {
        let atlas_port = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let atlas_addr = atlas_port.local_addr().unwrap();
        drop(atlas_port);
        let atlas_server =
            tokio::spawn(serve_atlas(Arc::new(AtlasRegistry::default()), atlas_addr));
        let mut atlas =
            AtlasClient::connect_with_retry(atlas_addr.to_string(), 50, Duration::from_millis(10))
                .await
                .unwrap();
        let provider_id = "executor-driver-test";
        atlas
            .register_service(provider_id, crate::config::EXECUTOR_NAMESPACE, "")
            .await
            .unwrap();
        let incoming =
            tonic::transport::server::TcpIncoming::bind("127.0.0.1:0".parse().unwrap()).unwrap();
        let endpoint = incoming.local_addr().unwrap().to_string();
        atlas
            .declare_capability(
                provider_id,
                SHARED_DRIVER_CONTRACT,
                Transport::Grpc,
                &endpoint,
                grpc_params(
                    "capabilities/lifecycle/driver.v1.toml",
                    "robonix.contracts.RobonixLifecycleDriver",
                    "/robonix.contracts.RobonixLifecycleDriver/Driver",
                ),
            )
            .await
            .unwrap();
        let svc = ExecutorServiceImpl::new(
            atlas.clone(),
            provider_id.into(),
            Arc::new(VerificationPolicy::new(false, vec![])),
        );
        let runtime = svc.runtime.clone();
        runtime.close_admission().await.unwrap();
        let driver =
            ExecutorLifecycleDriver::new(atlas.clone(), provider_id.into(), runtime.clone());
        let shutdown = driver.subscribe_shutdown();
        let server = tokio::spawn(
            tonic::transport::Server::builder()
                .add_service(RobonixLifecycleDriverServer::new(driver))
                .add_service(RobonixSystemExecutorExecuteServer::new(svc.clone()))
                .add_service(RobonixSystemExecutorExecuteStreamServer::new(svc.clone()))
                .add_service(RobonixSystemExecutorGetHealthServer::new(svc))
                .serve_with_incoming_shutdown(incoming, wait_for_shutdown(shutdown)),
        );
        let mut client = RobonixLifecycleDriverClient::connect(format!("http://{endpoint}"))
            .await
            .unwrap();
        let mut execute = RobonixSystemExecutorExecuteClient::connect(format!("http://{endpoint}"))
            .await
            .unwrap();
        let mut health =
            RobonixSystemExecutorGetHealthClient::connect(format!("http://{endpoint}"))
                .await
                .unwrap();
        for (command, expected_ok, expected_state, busy) in [
            (CMD_ACTIVATE, false, LifecycleState::StateRegistered, false),
            (CMD_INIT, true, LifecycleState::StateInactive, false),
            (CMD_ACTIVATE, true, LifecycleState::StateActive, false),
            (CMD_INIT, false, LifecycleState::StateActive, true),
            (CMD_DEACTIVATE, false, LifecycleState::StateActive, true),
            (CMD_SHUTDOWN, false, LifecycleState::StateActive, true),
            (99, false, LifecycleState::StateActive, false),
            (CMD_DEACTIVATE, true, LifecycleState::StateInactive, false),
            (CMD_ACTIVATE, true, LifecycleState::StateActive, false),
            (CMD_SHUTDOWN, true, LifecycleState::StateTerminated, false),
        ] {
            if busy {
                runtime.register_plan("busy").await.unwrap();
            }
            let idle_input = if matches!(command, CMD_DEACTIVATE | CMD_SHUTDOWN) && !busy {
                let mut stream_client =
                    RobonixSystemExecutorExecuteStreamClient::connect(format!("http://{endpoint}"))
                        .await
                        .unwrap();
                let (tx, rx) = tokio::sync::mpsc::channel(1);
                let request = tokio::spawn(async move {
                    stream_client
                        .execute_stream(tokio_stream::wrappers::ReceiverStream::new(rx))
                        .await
                });
                tokio::task::yield_now().await;
                Some((tx, request))
            } else {
                None
            };
            let response = client
                .driver(DriverRequest {
                    command,
                    config_json: "{}".into(),
                })
                .await
                .unwrap()
                .into_inner();
            assert_eq!(response.ok, expected_ok, "command {command}: {response:?}");
            assert_eq!(response.error.is_empty(), expected_ok);
            if let Some((_tx, request)) = idle_input {
                let error = tokio::time::timeout(Duration::from_secs(2), request)
                    .await
                    .unwrap()
                    .unwrap()
                    .unwrap_err();
                assert!(
                    error.message().contains("not accepting plans")
                        || (command == CMD_SHUTDOWN && error.code() == tonic::Code::Unavailable)
                );
            }
            if busy {
                assert!(response.error.contains("active plans"));
                runtime.complete_plan("busy").await;
            }
            let provider = atlas
                .query(Kind::Service, provider_id, "", "", Transport::Unspecified)
                .await
                .unwrap()
                .pop()
                .unwrap();
            assert_eq!(provider.state, expected_state as i32, "command {command}");
            if expected_state != LifecycleState::StateTerminated {
                let active = expected_state == LifecycleState::StateActive;
                let module = health
                    .get_module_health(GetModuleHealthRequest {})
                    .await
                    .unwrap()
                    .into_inner()
                    .report
                    .unwrap()
                    .module
                    .unwrap();
                assert_eq!(module.state, if active { "active" } else { "inactive" });
                assert_plan_admission(&mut execute, active).await;
            }
        }
        drop(client);
        drop(execute);
        drop(health);
        tokio::time::timeout(Duration::from_secs(2), server)
            .await
            .expect("Driver shutdown must stop the server")
            .unwrap()
            .unwrap();
        atlas_server.abort();
    }
}
