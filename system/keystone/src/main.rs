// SPDX-License-Identifier: MulanPSL-2.0

mod config;
mod service;

use std::net::{IpAddr, SocketAddr};
use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result};
use base64::Engine;
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use clap::Parser;
use rand::RngCore;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_keystone::KeystoneStore;
use robonix_keystone::pb::contracts::{
    robonix_lifecycle_driver_client::RobonixLifecycleDriverClient,
    robonix_lifecycle_driver_server::{RobonixLifecycleDriver, RobonixLifecycleDriverServer},
    robonix_system_keystone_admin_delete_user_server::RobonixSystemKeystoneAdminDeleteUserServer,
    robonix_system_keystone_admin_reset_voiceprint_server::RobonixSystemKeystoneAdminResetVoiceprintServer,
    robonix_system_keystone_admin_update_user_server::RobonixSystemKeystoneAdminUpdateUserServer,
    robonix_system_keystone_change_password_server::RobonixSystemKeystoneChangePasswordServer,
    robonix_system_keystone_get_profile_server::RobonixSystemKeystoneGetProfileServer,
    robonix_system_keystone_get_system_config_server::RobonixSystemKeystoneGetSystemConfigServer,
    robonix_system_keystone_get_voiceprint_preview_server::RobonixSystemKeystoneGetVoiceprintPreviewServer,
    robonix_system_keystone_list_users_server::RobonixSystemKeystoneListUsersServer,
    robonix_system_keystone_login_server::RobonixSystemKeystoneLoginServer,
    robonix_system_keystone_logout_server::RobonixSystemKeystoneLogoutServer,
    robonix_system_keystone_register_server::RobonixSystemKeystoneRegisterServer,
    robonix_system_keystone_replace_voiceprint_server::RobonixSystemKeystoneReplaceVoiceprintServer,
    robonix_system_keystone_unbind_voiceprint_server::RobonixSystemKeystoneUnbindVoiceprintServer,
    robonix_system_keystone_update_profile_server::RobonixSystemKeystoneUpdateProfileServer,
    robonix_system_keystone_update_system_config_server::RobonixSystemKeystoneUpdateSystemConfigServer,
    robonix_system_keystone_verify_voice_server::RobonixSystemKeystoneVerifyVoiceServer,
};
use robonix_keystone::pb::lifecycle::{DriverRequest, DriverResponse};
use robonix_scribe::{info, warn};
use service::KeystoneService;
use tonic::{Request, Response, Status};

use crate::config::{Args, KEYSTONE_NAMESPACE, KeystoneConfig};

const SHARED_DRIVER_CONTRACT: &str = "robonix/lifecycle/driver";
const CMD_INIT: u32 = 0;
const CMD_ACTIVATE: u32 = 1;
const CMD_DEACTIVATE: u32 = 2;
const CMD_SHUTDOWN: u32 = 3;

#[derive(Clone)]
struct SystemLifecycleDriver {
    atlas: AtlasClient,
    provider_id: String,
    shutdown_tx: tokio::sync::watch::Sender<bool>,
}

impl SystemLifecycleDriver {
    fn new(atlas: AtlasClient, provider_id: String) -> Self {
        let (shutdown_tx, _) = tokio::sync::watch::channel(false);
        Self {
            atlas,
            provider_id,
            shutdown_tx,
        }
    }

    async fn transition(&self, command: u32) -> Result<&'static str> {
        let (state, label) = lifecycle_target(command)
            .ok_or_else(|| anyhow::anyhow!("unknown lifecycle command code {command}"))?;
        let mut atlas = self.atlas.clone();
        atlas
            .set_lifecycle_state(&self.provider_id, state, "")
            .await
            .with_context(|| format!("publish lifecycle state for '{}'", self.provider_id))?;
        if command == CMD_SHUTDOWN {
            self.shutdown_tx.send_replace(true);
        }
        Ok(label)
    }

    fn subscribe_shutdown(&self) -> tokio::sync::watch::Receiver<bool> {
        self.shutdown_tx.subscribe()
    }
}

#[tonic::async_trait]
impl RobonixLifecycleDriver for SystemLifecycleDriver {
    async fn driver(
        &self,
        request: Request<DriverRequest>,
    ) -> std::result::Result<Response<DriverResponse>, Status> {
        let response = match self.transition(request.into_inner().command).await {
            Ok(state) => DriverResponse {
                ok: true,
                state: state.to_string(),
                error: String::new(),
            },
            Err(error) => DriverResponse {
                ok: false,
                state: "error".to_string(),
                error: format!("{error:#}"),
            },
        };
        Ok(Response::new(response))
    }
}

fn lifecycle_target(command: u32) -> Option<(atlas_pb::LifecycleState, &'static str)> {
    match command {
        CMD_INIT => Some((atlas_pb::LifecycleState::StateInactive, "inactive")),
        CMD_ACTIVATE => Some((atlas_pb::LifecycleState::StateActive, "active")),
        CMD_DEACTIVATE => Some((atlas_pb::LifecycleState::StateInactive, "inactive")),
        CMD_SHUTDOWN => Some((atlas_pb::LifecycleState::StateTerminated, "terminated")),
        _ => None,
    }
}

async fn wait_for_driver_shutdown(mut shutdown: tokio::sync::watch::Receiver<bool>) {
    if *shutdown.borrow() {
        return;
    }
    while shutdown.changed().await.is_ok() {
        if *shutdown.borrow() {
            return;
        }
    }
}

fn startup_driver_endpoint(listen_addr: SocketAddr) -> String {
    let ip = match listen_addr.ip() {
        IpAddr::V4(ip) if ip.is_unspecified() => IpAddr::V4(std::net::Ipv4Addr::LOCALHOST),
        IpAddr::V6(ip) if ip.is_unspecified() => IpAddr::V6(std::net::Ipv6Addr::LOCALHOST),
        ip => ip,
    };
    SocketAddr::new(ip, listen_addr.port()).to_string()
}

async fn connect_startup_driver(
    endpoint: &str,
) -> Result<RobonixLifecycleDriverClient<tonic::transport::Channel>> {
    let endpoint = if endpoint.starts_with("http") {
        endpoint.to_string()
    } else {
        format!("http://{endpoint}")
    };
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        match RobonixLifecycleDriverClient::connect(endpoint.clone()).await {
            Ok(client) => return Ok(client),
            Err(_) if tokio::time::Instant::now() < deadline => {
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
            Err(error) => return Err(error).context("connect startup lifecycle Driver"),
        }
    }
}

async fn call_startup_driver(
    client: &mut RobonixLifecycleDriverClient<tonic::transport::Channel>,
    command: u32,
) -> Result<String> {
    let response = client
        .driver(DriverRequest {
            command,
            config_json: "{}".to_string(),
        })
        .await
        .context("call startup lifecycle Driver")?
        .into_inner();
    if !response.ok {
        anyhow::bail!("startup lifecycle Driver failed: {}", response.error);
    }
    Ok(response.state)
}

struct CapabilityBinding {
    contract_id: &'static str,
    contract_toml: &'static str,
    service: &'static str,
    method: &'static str,
}

macro_rules! capability {
    ($name:literal, $suffix:literal) => {
        CapabilityBinding {
            contract_id: concat!("robonix/system/keystone/", $name),
            contract_toml: concat!("capabilities/system/keystone/", $name, ".v1.toml"),
            service: concat!("robonix.contracts.RobonixSystemKeystone", $suffix),
            method: concat!(
                "/robonix.contracts.RobonixSystemKeystone",
                $suffix,
                "/",
                $suffix
            ),
        }
    };
}

const CAPABILITIES: &[CapabilityBinding] = &[
    capability!("register", "Register"),
    capability!("login", "Login"),
    capability!("logout", "Logout"),
    capability!("get_profile", "GetProfile"),
    capability!("get_voiceprint_preview", "GetVoiceprintPreview"),
    capability!("update_profile", "UpdateProfile"),
    capability!("change_password", "ChangePassword"),
    capability!("list_users", "ListUsers"),
    capability!("admin_update_user", "AdminUpdateUser"),
    capability!("admin_delete_user", "AdminDeleteUser"),
    capability!("replace_voiceprint", "ReplaceVoiceprint"),
    capability!("unbind_voiceprint", "UnbindVoiceprint"),
    capability!("admin_reset_voiceprint", "AdminResetVoiceprint"),
    capability!("verify_voice", "VerifyVoice"),
    capability!("get_system_config", "GetSystemConfig"),
    capability!("update_system_config", "UpdateSystemConfig"),
];

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    robonix_scribe::init_from_config("keystone", args.config_json.as_deref());
    let config = KeystoneConfig::resolve(args)?;

    ensure_parent(&config.database)?;
    let store = KeystoneStore::open(&config.database)
        .with_context(|| format!("open Keystone database '{}'", config.database.display()))?;
    bootstrap_admin(&store, &config)?;

    let listen: SocketAddr = config
        .listen
        .parse()
        .with_context(|| format!("invalid Keystone listen address '{}'", config.listen))?;
    let advertised = startup_driver_endpoint(listen);

    let mut atlas =
        AtlasClient::connect_with_retry(&config.atlas_endpoint, 10, Duration::from_secs(2))
            .await
            .context("connect Keystone to Atlas")?;
    atlas
        .register_service(&config.provider_id, KEYSTONE_NAMESPACE, "")
        .await
        .context("register Keystone")?;
    atlas
        .declare_capability(
            &config.provider_id,
            SHARED_DRIVER_CONTRACT,
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/lifecycle/driver.v1.toml",
                "robonix.contracts.RobonixLifecycleDriver",
                "/robonix.contracts.RobonixLifecycleDriver/Driver",
            ),
        )
        .await
        .context("declare Keystone shared lifecycle Driver")?;
    let lifecycle = SystemLifecycleDriver::new(atlas.clone(), config.provider_id.clone());
    for capability in CAPABILITIES {
        atlas
            .declare_capability(
                &config.provider_id,
                capability.contract_id,
                atlas_pb::Transport::Grpc,
                &advertised,
                atlas_client::grpc_params(
                    capability.contract_toml,
                    capability.service,
                    capability.method,
                ),
            )
            .await
            .with_context(|| format!("declare '{}'", capability.contract_id))?;
    }
    let service_atlas = std::sync::Arc::new(tokio::sync::Mutex::new(atlas.clone()));
    let service = std::sync::Arc::new(KeystoneService::new(store, service_atlas));
    let server_shutdown = lifecycle.subscribe_shutdown();
    let server_lifecycle = lifecycle.clone();
    let mut server_task = tokio::spawn(async move {
        tonic::transport::Server::builder()
            .add_service(RobonixLifecycleDriverServer::new(server_lifecycle))
            .add_service(RobonixSystemKeystoneRegisterServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneLoginServer::from_arc(service.clone()))
            .add_service(RobonixSystemKeystoneLogoutServer::from_arc(service.clone()))
            .add_service(RobonixSystemKeystoneGetProfileServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneGetVoiceprintPreviewServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneUpdateProfileServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneChangePasswordServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneListUsersServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneAdminUpdateUserServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneAdminDeleteUserServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneReplaceVoiceprintServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneUnbindVoiceprintServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneAdminResetVoiceprintServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneVerifyVoiceServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneGetSystemConfigServer::from_arc(
                service.clone(),
            ))
            .add_service(RobonixSystemKeystoneUpdateSystemConfigServer::from_arc(
                service,
            ))
            .serve_with_shutdown(listen, wait_for_driver_shutdown(server_shutdown))
            .await
    });

    let mut startup_driver = tokio::select! {
        client = connect_startup_driver(&advertised) => client?,
        result = &mut server_task => {
            result.context("join Keystone gRPC server")?
                .context("Keystone gRPC server failed before readiness")?;
            anyhow::bail!("Keystone gRPC server stopped before readiness");
        }
    };
    call_startup_driver(&mut startup_driver, CMD_INIT)
        .await
        .context("initialize Keystone lifecycle")?;
    call_startup_driver(&mut startup_driver, CMD_ACTIVATE)
        .await
        .context("activate Keystone lifecycle")?;
    drop(startup_driver);

    spawn_heartbeat(atlas, config.provider_id.clone());
    info!(
        "Keystone ready on {listen}; database={}",
        config.database.display()
    );
    server_task
        .await
        .context("join Keystone gRPC server")?
        .context(format!("Keystone gRPC server failed at {advertised}"))
}

/// Create the first administrator without embedding a reusable password in source.
fn bootstrap_admin(store: &KeystoneStore, config: &KeystoneConfig) -> Result<()> {
    let (password, generated) = match config.bootstrap_admin_password.clone() {
        Some(password) if !password.is_empty() => (password, false),
        _ => (random_bootstrap_password(), true),
    };
    let created = store.bootstrap_admin(
        &config.bootstrap_admin_username,
        &config.bootstrap_admin_display_name,
        &config.bootstrap_admin_email,
        &password,
        true,
    )?;
    if created.is_some() && generated {
        write_bootstrap_credentials(
            &config.bootstrap_credentials_file,
            &config.bootstrap_admin_username,
            &password,
        )?;
        warn!(
            "first administrator created; one-time credentials written to '{}'",
            config.bootstrap_credentials_file.display()
        );
    } else if created.is_some() {
        info!("first administrator created from explicit bootstrap credentials");
    }
    Ok(())
}

fn random_bootstrap_password() -> String {
    let mut bytes = [0_u8; 24];
    rand::rng().fill_bytes(&mut bytes);
    URL_SAFE_NO_PAD.encode(bytes)
}

fn ensure_parent(path: &Path) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("create data directory '{}'", parent.display()))?;
    }
    Ok(())
}

fn write_bootstrap_credentials(path: &Path, username: &str, password: &str) -> Result<()> {
    ensure_parent(path)?;
    let contents =
        format!("username={username}\npassword={password}\nchange_password_required=true\n");
    std::fs::write(path, contents)
        .with_context(|| format!("write bootstrap credentials '{}'", path.display()))?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))
            .with_context(|| format!("protect bootstrap credentials '{}'", path.display()))?;
    }
    Ok(())
}

fn spawn_heartbeat(mut atlas: AtlasClient, provider_id: String) {
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(20));
        interval.tick().await;
        loop {
            interval.tick().await;
            if let Err(error) = atlas.heartbeat(&provider_id).await {
                warn!("Keystone heartbeat failed: {error:#}");
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_bootstrap_credentials_are_private() {
        let directory = tempfile::tempdir().expect("tempdir");
        let path = directory.path().join("bootstrap.txt");
        write_bootstrap_credentials(&path, "admin", "one-time-secret").expect("write");
        let contents = std::fs::read_to_string(&path).expect("read");
        assert!(contents.contains("username=admin"));
        assert!(contents.contains("password=one-time-secret"));
        assert!(contents.contains("change_password_required=true"));
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(
                std::fs::metadata(&path)
                    .expect("metadata")
                    .permissions()
                    .mode()
                    & 0o777,
                0o600
            );
        }
    }

    #[test]
    fn generated_bootstrap_password_is_not_reused() {
        let first = random_bootstrap_password();
        let second = random_bootstrap_password();
        assert_ne!(first, second);
        assert!(first.len() >= 32);
        assert!(second.len() >= 32);
    }
}
