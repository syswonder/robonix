// SPDX-License-Identifier: MulanPSL-2.0
// Daemon Client Module
//
// Client for communicating with robonix daemon via Unix domain socket

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
        robonix_sdk_path: Option<PathBuf>,
    },
    Stop {
        std_name: String,
        package_type: String,
    },
    Status,
    Ping,
    // Core service calls
    CallRegisterPrimitive {
        request: String, // JSON serialized RegisterPrimitiveRequest
    },
    CallRegisterService {
        request: String, // JSON serialized RegisterServiceRequest
    },
    CallRegisterSkill {
        request: String, // JSON serialized RegisterSkillRequest
    },
    CallSubmitTask {
        request: String, // JSON serialized SubmitTaskRequest
    },
    CallTaskData {
        request: String, // JSON serialized TaskDataRequest
    },
}

#[derive(Debug, Serialize, Deserialize)]
pub enum DaemonResponse {
    Ok(String),
    OkWithDetails {
        message: String,
        pid: u32,
        pgid: Option<u32>,
        pids: Option<Vec<u32>>,
    },
    Error(String),
    Status(Vec<ProcessStatus>),
    // Core service responses
    RegisterPrimitiveResponse {
        response: String, // JSON serialized RegisterPrimitiveResponse
    },
    RegisterServiceResponse {
        response: String, // JSON serialized RegisterServiceResponse
    },
    RegisterSkillResponse {
        response: String, // JSON serialized RegisterSkillResponse
    },
    SubmitTaskResponse {
        response: String, // JSON serialized SubmitTaskResponse
    },
    TaskDataResponse {
        response: String, // JSON serialized TaskDataResponse
    },
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
        std::fs::create_dir_all(&socket_dir).with_context(|| {
            format!(
                "Failed to create socket directory: {}",
                socket_dir.display()
            )
        })?;
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
            .with_context(|| {
                format!(
                    "Failed to connect to daemon socket: {}",
                    self.socket_path.display()
                )
            })?;

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

    pub async fn get_daemon_pid(&self) -> Result<Option<u32>> {
        #[cfg(unix)]
        {
            // Try to find daemon process by checking socket file's owner or by name
            let output = std::process::Command::new("pgrep")
                .arg("-f")
                .arg("rbnx-daemon")
                .output();

            match output {
                Ok(output) if output.status.success() => {
                    let pid_str_owned = String::from_utf8_lossy(&output.stdout).to_string();
                    let pid_str = pid_str_owned.trim();
                    if let Ok(pid) = pid_str.parse::<u32>() {
                        return Ok(Some(pid));
                    }
                }
                _ => {}
            }

            // Alternative: try to get PID from socket file using lsof
            // This is less reliable but can work if pgrep fails
            let socket_path_str = self.socket_path.to_string_lossy().to_string();
            let output = std::process::Command::new("lsof")
                .arg("-t")
                .arg(&socket_path_str)
                .output();

            if let Ok(output) = output {
                if output.status.success() {
                    let pid_str_owned = String::from_utf8_lossy(&output.stdout).to_string();
                    let pid_str = pid_str_owned.trim();
                    if let Ok(pid) = pid_str.parse::<u32>() {
                        return Ok(Some(pid));
                    }
                }
            }
        }

        Ok(None)
    }

    pub async fn ensure_daemon_running(&self) -> Result<()> {
        if !self.is_daemon_running().await {
            // Try to start daemon
            log::info!("Daemon not running, attempting to start...");
            let daemon_path = Self::find_daemon_executable()?;
            log::info!("Found daemon executable at: {}", daemon_path.display());

            // Start daemon in background
            #[cfg(unix)]
            {
                use std::process::Stdio;

                // Spawn daemon process with null stdio
                // The daemon will run in the background
                let mut child = std::process::Command::new(&daemon_path)
                    .stdout(Stdio::null())
                    .stderr(Stdio::null())
                    .stdin(Stdio::null())
                    .spawn()
                    .with_context(|| {
                        format!("Failed to spawn daemon process: {}", daemon_path.display())
                    })?;

                // Check if process started successfully
                // If it exits immediately, there was an error
                tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

                // Try to wait for the process with a timeout to see if it exited immediately
                // Note: This is a best-effort check - the process might have already started successfully
                if let Ok(Some(status)) = child.try_wait() {
                    if !status.success() {
                        anyhow::bail!(
                            "Daemon process exited immediately with status: {:?}. \
                            The daemon may have encountered an error during startup. \
                            Try running 'rbnx-daemon' manually to see error messages.",
                            status
                        );
                    }
                }

                // Detach the process so it can run independently
                drop(child);

                // Wait a bit for daemon to start and create socket
                tokio::time::sleep(tokio::time::Duration::from_millis(1500)).await;
            }

            // Check if daemon is now running
            if !self.is_daemon_running().await {
                // Try to get more info - check if socket directory exists
                let home_dir = dirs::home_dir().context("Failed to get home directory")?;
                let socket_dir = home_dir.join(".robonix");
                let socket_path = socket_dir.join("daemon.sock");

                if !socket_dir.exists() {
                    anyhow::bail!(
                        "Failed to start daemon. Socket directory does not exist: {}. \
                        The daemon may have failed during initialization. \
                        Try running 'rbnx-daemon' manually to see error messages.",
                        socket_dir.display()
                    );
                }

                if !socket_path.exists() {
                    anyhow::bail!(
                        "Failed to start daemon. Socket file was not created at: {}. \
                        The daemon may have crashed during startup. \
                        Try running 'rbnx-daemon' manually to see error messages.",
                        socket_path.display()
                    );
                }

                anyhow::bail!(
                    "Failed to start daemon. Socket exists but daemon is not responding. \
                    Try running 'rbnx-daemon' manually to see error messages."
                );
            }
        }
        Ok(())
    }

    fn find_daemon_executable() -> Result<PathBuf> {
        // Try 1: Same directory as current executable
        if let Ok(current_exe) = std::env::current_exe() {
            if let Some(parent) = current_exe.parent() {
                let daemon_path = parent.join("rbnx-daemon");
                if daemon_path.exists() {
                    return Ok(daemon_path);
                }
            }
        }

        // Try 2: In PATH
        if let Ok(path) = std::env::var("PATH") {
            for dir in path.split(':') {
                let daemon_path = PathBuf::from(dir).join("rbnx-daemon");
                if daemon_path.exists() {
                    return Ok(daemon_path);
                }
            }
        }

        // Try 3: Common installation paths
        let common_paths = vec![
            PathBuf::from("/usr/local/bin/rbnx-daemon"),
            PathBuf::from("/usr/bin/rbnx-daemon"),
            PathBuf::from("/opt/robonix/bin/rbnx-daemon"),
        ];

        for path in common_paths {
            if path.exists() {
                return Ok(path);
            }
        }

        // Try 4: Cargo build directory (for development)
        // When running with cargo run, current_exe might be the cargo binary,
        // so we also check common cargo build directories
        if let Ok(current_exe) = std::env::current_exe() {
            // Check if we're in a target directory structure
            let exe_path = current_exe.to_string_lossy();
            if exe_path.contains("/target/debug/") || exe_path.contains("/target/release/") {
                // Extract the target directory path
                if let Some(target_idx) = exe_path.find("/target/") {
                    let base_path = &exe_path[..target_idx + 8]; // +8 for "/target/"
                    let profile = if exe_path.contains("/target/debug/") {
                        "debug"
                    } else {
                        "release"
                    };
                    let daemon_path =
                        PathBuf::from(format!("{}{}/rbnx-daemon", base_path, profile));
                    if daemon_path.exists() {
                        return Ok(daemon_path);
                    }
                }
            }
            // Also check parent directory if it's a target directory
            if let Some(parent) = current_exe.parent() {
                let parent_str = parent.to_string_lossy();
                if parent_str.ends_with("/debug") || parent_str.ends_with("/release") {
                    let daemon_path = parent.join("rbnx-daemon");
                    if daemon_path.exists() {
                        return Ok(daemon_path);
                    }
                }
            }
        }

        // Try 5: Check current working directory for target folder (for cargo run)
        if let Ok(cwd) = std::env::current_dir() {
            for profile in &["debug", "release"] {
                let daemon_path = cwd.join("target").join(profile).join("rbnx-daemon");
                if daemon_path.exists() {
                    return Ok(daemon_path);
                }
            }
        }

        anyhow::bail!(
            "Daemon executable (rbnx-daemon) not found. Please build the project with 'cargo build' or install it."
        )
    }
}
