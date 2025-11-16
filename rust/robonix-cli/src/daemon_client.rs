use anyhow::{Context, Result};
use dirs;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;

#[derive(Debug, Serialize, Deserialize)]
pub enum DaemonCommand {
    Start {
        package_name: String,
        std_name: String,
        package_type: String,
        package_path: PathBuf,
        start_script: String,
        robonix_msg_path: Option<PathBuf>,
    },
    Stop {
        std_name: String,
        package_type: String,
    },
    Status,
    Ping,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum DaemonResponse {
    Ok(String),
    Error(String),
    Status(Vec<ProcessStatus>),
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ProcessStatus {
    pub package_name: String,
    pub std_name: String,
    pub package_type: String,
    pub pid: u32,
    pub log_file: PathBuf,
}

pub struct DaemonClient {
    socket_path: PathBuf,
}

impl DaemonClient {
    pub fn new() -> Result<Self> {
        let home_dir = dirs::home_dir().context("Failed to get home directory")?;
        let socket_dir = home_dir.join(".robonix");
        std::fs::create_dir_all(&socket_dir)
            .with_context(|| format!("Failed to create socket directory: {}", socket_dir.display()))?;
        let socket_path = socket_dir.join("daemon.sock");
        Ok(Self { socket_path })
    }

    pub async fn send_command(&self, command: DaemonCommand) -> Result<DaemonResponse> {
        // Check if daemon is running
        if !self.is_daemon_running().await {
            anyhow::bail!("Daemon is not running. Please start it with 'rbnx-daemon'");
        }

        // Connect to daemon socket
        let mut stream = UnixStream::connect(&self.socket_path)
            .await
            .with_context(|| format!("Failed to connect to daemon socket: {}", self.socket_path.display()))?;

        // Serialize and send command
        let command_json = serde_json::to_string(&command)?;
        let command_bytes = command_json.as_bytes();
        let len = command_bytes.len() as u32;
        
        // Send length prefix (little-endian)
        stream.write_u32_le(len).await?;
        // Send command
        stream.write_all(command_bytes).await?;
        stream.flush().await?;

        // Read response length
        let response_len = stream.read_u32_le().await?;
        let mut response_bytes = vec![0u8; response_len as usize];
        stream.read_exact(&mut response_bytes).await?;

        // Deserialize response
        let response: DaemonResponse = serde_json::from_slice(&response_bytes)?;
        Ok(response)
    }

    pub async fn is_daemon_running(&self) -> bool {
        // Check if socket file exists and is accessible
        if !self.socket_path.exists() {
            return false;
        }
        
        // Try to connect and ping
        match UnixStream::connect(&self.socket_path).await {
            Ok(_) => true,
            Err(_) => false,
        }
    }

    pub async fn ensure_daemon_running(&self) -> Result<()> {
        if !self.is_daemon_running().await {
            // Try to start daemon
            tracing::info!("Daemon not running, attempting to start...");
            let daemon_path = std::env::current_exe()?
                .parent()
                .ok_or_else(|| anyhow::anyhow!("Failed to get executable directory"))?
                .join("rbnx-daemon");
            
            if !daemon_path.exists() {
                anyhow::bail!(
                    "Daemon executable not found at {}. Please build the project first.",
                    daemon_path.display()
                );
            }

            // Start daemon in background
            #[cfg(unix)]
            {
                use std::process::Stdio;
                std::process::Command::new(&daemon_path)
                    .stdout(Stdio::null())
                    .stderr(Stdio::null())
                    .spawn()
                    .with_context(|| format!("Failed to start daemon: {}", daemon_path.display()))?;
            }

            // Wait a bit for daemon to start
            tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

            // Check if daemon is now running
            if !self.is_daemon_running().await {
                anyhow::bail!("Failed to start daemon. Please start it manually with 'rbnx-daemon'");
            }
        }
        Ok(())
    }
}

