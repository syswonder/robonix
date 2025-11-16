use anyhow::{Context, Result};
use crate::process::ProcessManager;
use crate::daemon_client::{DaemonCommand, DaemonResponse, ProcessStatus};
use std::path::PathBuf;
use std::sync::Arc;
use tokio::fs;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{UnixListener, UnixStream};
use tokio::signal;
use tracing::{error, info, warn};

pub struct Daemon {
    process_manager: Arc<ProcessManager>,
    socket_path: PathBuf,
}

impl Daemon {
    pub async fn new(config: crate::Config) -> Result<Self> {
        let log_dir = config.package_storage_path.join("logs");
        let process_manager = Arc::new(ProcessManager::new(log_dir)?);

        let socket_dir = PathBuf::from("/var/run/robonix");
        std::fs::create_dir_all(&socket_dir)
            .with_context(|| format!("Failed to create socket directory: {}", socket_dir.display()))?;
        let socket_path = socket_dir.join("daemon.sock");

        // Remove old socket if exists
        if socket_path.exists() {
            fs::remove_file(&socket_path).await?;
        }

        Ok(Self {
            process_manager,
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
                    tokio::spawn(async move {
                        if let Err(e) = Self::handle_client(stream, process_manager).await {
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

    async fn handle_client(mut stream: UnixStream, process_manager: Arc<ProcessManager>) -> Result<()> {
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
                robonix_msg_path,
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
                            robonix_msg_path.as_ref(),
                        )
                        .await
                    {
                        Ok(_) => DaemonResponse::Ok(format!(
                            "Started {} {}",
                            package_type, std_name
                        )),
                        Err(e) => DaemonResponse::Error(format!("Failed to start: {}", e)),
                    }
                }
            }
            DaemonCommand::Stop {
                std_name,
                package_type,
            } => {
                match process_manager
                    .stop_process(&std_name, &package_type)
                    .await
                {
                    Ok(_) => DaemonResponse::Ok(format!(
                        "Stopped {} {}",
                        package_type, std_name
                    )),
                    Err(e) => DaemonResponse::Error(format!("Failed to stop: {}", e)),
                }
            }
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

