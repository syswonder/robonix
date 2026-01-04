// SPDX-License-Identifier: MulanPSL-2.0
// Daemon Module
//
// Daemon process management for robonix-cli

use crate::daemon_client::{DaemonCommand, DaemonResponse, ProcessStatus};
use crate::daemon_ros2::DaemonRos2Clients;
use crate::process::ProcessManager;
use anyhow::{Context, Result};
use dirs;
use log::{error, info, warn};
use robonix_core::ros_idl::primitive::RegisterPrimitiveRequest;
use robonix_core::ros_idl::service_registry::RegisterServiceRequest;
use robonix_core::ros_idl::skill::RegisterSkillRequest;
use robonix_core::ros_idl::task::{SubmitTaskRequest, TaskDataRequest};
use std::path::PathBuf;
use std::sync::Arc;
use tokio::fs;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{UnixListener, UnixStream};
use tokio::signal;

pub struct Daemon {
    process_manager: Arc<ProcessManager>,
    ros2_clients: Arc<DaemonRos2Clients>,
    socket_path: PathBuf,
}

impl Daemon {
    pub async fn new(config: crate::Config) -> Result<Self> {
        let log_dir = config.package_storage_path.join("logs");
        let process_manager = Arc::new(ProcessManager::new(log_dir)?);

        // Initialize ROS2 clients (persistent node)
        info!("Initializing ROS2 node and service clients...");
        let ros2_clients = Arc::new(DaemonRos2Clients::new().await?);
        info!("ROS2 node and service clients initialized");

        let home_dir = dirs::home_dir().context("Failed to get home directory")?;
        let socket_dir = home_dir.join(".robonix");
        std::fs::create_dir_all(&socket_dir).with_context(|| {
            format!(
                "Failed to create socket directory: {}",
                socket_dir.display()
            )
        })?;
        let socket_path = socket_dir.join("daemon.sock");

        // Remove old socket if exists
        if socket_path.exists() {
            fs::remove_file(&socket_path).await?;
        }

        Ok(Self {
            process_manager,
            ros2_clients,
            socket_path,
        })
    }

    pub async fn run(&self) -> Result<()> {
        info!("Starting robonix daemon...");

        // Create Unix socket listener
        let listener = UnixListener::bind(&self.socket_path)
            .with_context(|| format!("Failed to bind socket: {}", self.socket_path.display()))?;

        info!("Daemon listening on {}", self.socket_path.display());

        // Spawn task to handle shutdown signal
        let socket_path_clone = self.socket_path.clone();
        tokio::spawn(async move {
            signal::ctrl_c().await.ok();
            info!("Received shutdown signal, cleaning up...");
            let _ = fs::remove_file(&socket_path_clone).await;
            std::process::exit(0);
        });

        // Main loop: accept connections and handle commands
        loop {
            match listener.accept().await {
                Ok((stream, _)) => {
                    let process_manager = self.process_manager.clone();
                    let ros2_clients = self.ros2_clients.clone();
                    tokio::spawn(async move {
                        if let Err(e) =
                            Self::handle_client(stream, process_manager, ros2_clients).await
                        {
                            error!("Error handling client: {}", e);
                        }
                    });
                }
                Err(e) => {
                    error!("Failed to accept connection: {}", e);
                }
            }
        }
    }

    async fn handle_client(
        mut stream: UnixStream,
        process_manager: Arc<ProcessManager>,
        ros2_clients: Arc<DaemonRos2Clients>,
    ) -> Result<()> {
        // Read command length
        let command_len = match stream.read_u32_le().await {
            Ok(len) => len,
            Err(e) => {
                warn!("Failed to read command length: {}", e);
                return Ok(());
            }
        };

        // Read command
        let mut command_bytes = vec![0u8; command_len as usize];
        stream.read_exact(&mut command_bytes).await?;

        // Deserialize command
        let command: DaemonCommand = match serde_json::from_slice(&command_bytes) {
            Ok(cmd) => cmd,
            Err(e) => {
                let response = DaemonResponse::Error(format!("Invalid command: {}", e));
                Self::send_response(&mut stream, response).await?;
                return Ok(());
            }
        };

        // Handle command
        let response = match command {
            DaemonCommand::Start {
                package_name,
                std_name,
                package_type,
                package_path,
                start_script,
                robonix_sdk_path,
            } => {
                // Check if already running
                if process_manager.is_running(&std_name, &package_type) {
                    DaemonResponse::Error(format!(
                        "{} {} is already running",
                        package_type, std_name
                    ))
                } else {
                    match process_manager
                        .start_process(
                            &package_name,
                            &std_name,
                            &package_type,
                            &package_path,
                            &start_script,
                            robonix_sdk_path.as_ref(),
                        )
                        .await
                    {
                        Ok(result) => DaemonResponse::OkWithDetails {
                            message: format!("Started {} {}", package_type, std_name),
                            pid: result.pid,
                            pgid: result.pgid,
                            pids: result.pids,
                        },
                        Err(e) => DaemonResponse::Error(format!("Failed to start: {}", e)),
                    }
                }
            }
            DaemonCommand::Stop {
                std_name,
                package_type,
            } => match process_manager.stop_process(&std_name, &package_type).await {
                Ok(result) => DaemonResponse::OkWithDetails {
                    message: format!("Stopped {} {}", package_type, std_name),
                    pid: result.pid,
                    pgid: result.pgid,
                    pids: result.pids,
                },
                Err(e) => DaemonResponse::Error(format!("Failed to stop: {}", e)),
            },
            DaemonCommand::Status => {
                let processes = process_manager.get_running_processes();
                let status: Vec<ProcessStatus> = processes
                    .into_iter()
                    .map(|proc| ProcessStatus {
                        package_name: proc.package_name,
                        std_name: proc.std_name,
                        package_type: proc.package_type,
                        pid: proc.pid,
                        log_file: proc.log_file,
                    })
                    .collect();
                DaemonResponse::Status(status)
            }
            DaemonCommand::Ping => DaemonResponse::Ok("pong".to_string()),
            DaemonCommand::CallRegisterPrimitive { request } => {
                match serde_json::from_str::<RegisterPrimitiveRequest>(&request) {
                    Ok(req) => match ros2_clients.call_register_primitive(req).await {
                        Ok(resp) => DaemonResponse::RegisterPrimitiveResponse {
                            response: serde_json::to_string(&resp)?,
                        },
                        Err(e) => DaemonResponse::Error(format!("Service call failed: {}", e)),
                    },
                    Err(e) => DaemonResponse::Error(format!("Invalid request: {}", e)),
                }
            }
            DaemonCommand::CallRegisterService { request } => {
                match serde_json::from_str::<RegisterServiceRequest>(&request) {
                    Ok(req) => match ros2_clients.call_register_service(req).await {
                        Ok(resp) => DaemonResponse::RegisterServiceResponse {
                            response: serde_json::to_string(&resp)?,
                        },
                        Err(e) => DaemonResponse::Error(format!("Service call failed: {}", e)),
                    },
                    Err(e) => DaemonResponse::Error(format!("Invalid request: {}", e)),
                }
            }
            DaemonCommand::CallRegisterSkill { request } => {
                match serde_json::from_str::<RegisterSkillRequest>(&request) {
                    Ok(req) => match ros2_clients.call_register_skill(req).await {
                        Ok(resp) => DaemonResponse::RegisterSkillResponse {
                            response: serde_json::to_string(&resp)?,
                        },
                        Err(e) => DaemonResponse::Error(format!("Service call failed: {}", e)),
                    },
                    Err(e) => DaemonResponse::Error(format!("Invalid request: {}", e)),
                }
            }
            DaemonCommand::CallSubmitTask { request } => {
                match serde_json::from_str::<SubmitTaskRequest>(&request) {
                    Ok(req) => match ros2_clients.call_submit_task(req).await {
                        Ok(resp) => DaemonResponse::SubmitTaskResponse {
                            response: serde_json::to_string(&resp)?,
                        },
                        Err(e) => DaemonResponse::Error(format!("Service call failed: {}", e)),
                    },
                    Err(e) => DaemonResponse::Error(format!("Invalid request: {}", e)),
                }
            }
            DaemonCommand::CallTaskData { request } => {
                match serde_json::from_str::<TaskDataRequest>(&request) {
                    Ok(req) => match ros2_clients.call_task_data(req).await {
                        Ok(resp) => DaemonResponse::TaskDataResponse {
                            response: serde_json::to_string(&resp)?,
                        },
                        Err(e) => DaemonResponse::Error(format!("Service call failed: {}", e)),
                    },
                    Err(e) => DaemonResponse::Error(format!("Invalid request: {}", e)),
                }
            }
        };

        // Send response
        Self::send_response(&mut stream, response).await?;
        Ok(())
    }

    async fn send_response(stream: &mut UnixStream, response: DaemonResponse) -> Result<()> {
        let response_json = serde_json::to_string(&response)?;
        let response_bytes = response_json.as_bytes();
        let len = response_bytes.len() as u32;

        // Send length prefix (little-endian)
        stream.write_u32_le(len).await?;
        // Send response
        stream.write_all(response_bytes).await?;
        stream.flush().await?;
        Ok(())
    }
}
