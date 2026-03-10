// SPDX-License-Identifier: MulanPSL-2.0
// Daemon Module
//
// Daemon process management for robonix-cli

use crate::daemon_client::{DaemonCommand, DaemonResponse, ProcessStatus};
use crate::daemon_ros2::DaemonRos2Clients;
use crate::database::PackageDatabase;
use crate::process::ProcessManager;
use crate::recipe_state::RecipeState;
use anyhow::{Context, Result};
use dirs;
use log::{error, info, trace, warn};
use robonix_server::ros_idl::primitive::RegisterPrimitiveRequest;
use robonix_server::ros_idl::service_registry::RegisterServiceRequest;
use robonix_server::ros_idl::skill::RegisterSkillRequest;
use robonix_server::ros_idl::task::{CancelTaskRequest, SubmitTaskRequest, TaskDataRequest};
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
    /// Core web HTTP base URL for pushing capability logs and node status
    core_http_url: Option<String>,
    /// Node ID (e.g. hostname) for this CLI
    node_id: String,
    /// Log directory (package_storage_path/logs) for reading capability log files
    log_dir: PathBuf,
    /// Package storage path for loading database and recipe state (node status)
    package_storage_path: PathBuf,
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

        let log_dir = config.package_storage_path.join("logs");
        let node_id = config.effective_node_id();
        let package_storage_path = config.package_storage_path.clone();

        Ok(Self {
            process_manager,
            ros2_clients,
            socket_path,
            core_http_url: config.core_http_url.clone(),
            node_id,
            log_dir,
            package_storage_path,
        })
    }

    pub async fn run(&self) -> Result<()> {
        info!("Starting robonix daemon...");

        // First: get core's listening IPs via ROS2 (daemon discovers core's network addresses)
        {
            let ros2 = self.ros2_clients.clone();
            tokio::spawn(async move {
                tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                match ros2.call_get_listening_ips().await {
                    Ok(ips) if !ips.is_empty() => {
                        info!("Core listening IPs (from ROS2): {}", ips.join(", "));
                    }
                    Ok(_) => {
                        trace!("Core get_listening_ips returned no IPs");
                    }
                    Err(e) => {
                        trace!("Could not get core listening IPs via ROS2: {}", e);
                    }
                }
            });
        }

        // Spawn background tasks when core_http_url is set: log push and node status
        if let Some(ref base_url) = self.core_http_url {
            let url = base_url.trim_end_matches('/').to_string();
            let node_id = self.node_id.clone();
            let log_dir = self.log_dir.clone();
            tokio::spawn(async move {
                Self::log_push_loop(url, node_id, log_dir).await;
            });
            info!("Log push to core enabled (core_http_url set)");

            let url_status = base_url.trim_end_matches('/').to_string();
            let node_id_status = self.node_id.clone();
            let process_manager = self.process_manager.clone();
            let package_storage_path = self.package_storage_path.clone();
            tokio::spawn(async move {
                Self::node_status_loop(
                    url_status,
                    node_id_status,
                    process_manager,
                    package_storage_path,
                )
                .await;
            });
            info!("Node status reporting to core enabled");
        }

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
            DaemonCommand::CallCancelTask { request } => {
                match serde_json::from_str::<CancelTaskRequest>(&request) {
                    Ok(req) => match ros2_clients.call_cancel_task(req).await {
                        Ok(resp) => DaemonResponse::CancelTaskResponse {
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

    /// Background loop: when core has opened logs for this node, push local log file content.
    async fn log_push_loop(base_url: String, node_id: String, log_dir: PathBuf) {
        let client = match reqwest::Client::builder()
            .timeout(std::time::Duration::from_secs(10))
            .build()
        {
            Ok(c) => c,
            Err(e) => {
                warn!("log push: failed to create HTTP client: {}", e);
                return;
            }
        };
        let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(2));
        loop {
            interval.tick().await;
            let subs_url = format!(
                "{}/api/log-subscriptions?node_id={}",
                base_url,
                urlencoding::encode(&node_id)
            );
            let resp = match client.get(&subs_url).send().await {
                Ok(r) => r,
                Err(e) => {
                    trace!("log push: GET subscriptions failed: {}", e);
                    continue;
                }
            };
            let capability_keys: Vec<String> = match resp.json().await {
                Ok(k) => k,
                Err(_) => continue,
            };
            for capability_key in capability_keys {
                // capability_key format: "provider/name" (e.g. demo_service_provider/srv::semantic_map)
                let (provider, name) = match capability_key.split_once('/') {
                    Some(p) => p,
                    None => continue,
                };
                let clean_name = name.replace("::", "_").replace('.', "_");
                let log_filename = format!("{}_{}.log", provider, clean_name);
                let log_path = log_dir.join(&log_filename);
                let content = match fs::read_to_string(&log_path).await {
                    Ok(c) => c,
                    Err(_) => continue,
                };
                let post_url = format!("{}/api/node-log", base_url);
                let body = serde_json::json!({
                    "node_id": node_id,
                    "capability_key": capability_key,
                    "content": content
                });
                if client.post(&post_url).json(&body).send().await.is_err() {
                    trace!("log push: POST node-log failed for {}", capability_key);
                }
            }
        }
    }

    /// Collect OS info: /etc/os-release key fields + kernel, arch, hostname.
    fn collect_os_version() -> Option<String> {
        let mut lines: Vec<String> = Vec::new();

        if let Ok(content) = std::fs::read_to_string("/etc/os-release") {
            let mut pretty = None;
            let mut name = None;
            let mut version = None;
            let mut version_id = None;
            let mut id_like = None;
            for line in content.lines() {
                if line.starts_with("PRETTY_NAME=") {
                    pretty = line
                        .strip_prefix("PRETTY_NAME=")
                        .map(|s| s.trim_matches('"').to_string());
                } else if line.starts_with("NAME=") && name.is_none() {
                    name = line
                        .strip_prefix("NAME=")
                        .map(|s| s.trim_matches('"').to_string());
                } else if line.starts_with("VERSION=") {
                    version = line
                        .strip_prefix("VERSION=")
                        .map(|s| s.trim_matches('"').to_string());
                } else if line.starts_with("VERSION_ID=") {
                    version_id = line
                        .strip_prefix("VERSION_ID=")
                        .map(|s| s.trim_matches('"').to_string());
                } else if line.starts_with("ID_LIKE=") {
                    id_like = line
                        .strip_prefix("ID_LIKE=")
                        .map(|s| s.trim_matches('"').to_string());
                }
            }
            if let Some(p) = pretty {
                lines.push(p);
            } else if let (Some(ref n), Some(v)) =
                (name.as_ref(), version.as_ref().or(version_id.as_ref()))
            {
                lines.push(format!("{} {}", n, v));
            } else if let Some(n) = name {
                lines.push(n);
            }
            if let Some(ref id) = version_id {
                lines.push(format!("Version: {}", id));
            }
            if let Some(like) = id_like {
                lines.push(format!("ID_LIKE: {}", like));
            }
        }

        if let Ok(out) = std::process::Command::new("uname").args(["-r"]).output() {
            if out.status.success() {
                let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !s.is_empty() {
                    lines.push(format!("Kernel: {}", s));
                }
            }
        }
        if let Ok(out) = std::process::Command::new("uname").args(["-m"]).output() {
            if out.status.success() {
                let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !s.is_empty() {
                    lines.push(format!("Arch: {}", s));
                }
            }
        }
        if let Ok(h) = hostname::get() {
            lines.push(format!("Hostname: {}", h.to_string_lossy()));
        }

        if lines.is_empty() {
            None
        } else {
            Some(lines.join("\n"))
        }
    }

    /// Collect CPU info: prefer lscpu (full output), else /proc/cpuinfo model + cores.
    fn collect_cpu_info() -> Option<String> {
        if let Ok(out) = std::process::Command::new("lscpu").output() {
            if out.status.success() {
                let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !s.is_empty() {
                    return Some(s);
                }
            }
        }

        let content = std::fs::read_to_string("/proc/cpuinfo").ok()?;
        let mut model = None;
        let mut n_processor = 0u32;
        for line in content.lines() {
            if line.starts_with("model name") {
                model = line.split(':').nth(1).map(|v| v.trim().to_string());
            } else if line.starts_with("processor") {
                n_processor += 1;
            }
        }
        match (model, n_processor) {
            (Some(m), n) if n > 0 => Some(format!("{}\nProcessors: {}", m, n)),
            (Some(m), _) => Some(m),
            _ => None,
        }
    }

    /// Collect memory info: /proc/meminfo or free -h (for non-collapsible display).
    fn collect_memory_info() -> Option<String> {
        if let Ok(content) = std::fs::read_to_string("/proc/meminfo") {
            let mut mem_total = None;
            let mut mem_avail = None;
            let mut mem_free = None;
            for line in content.lines() {
                if line.starts_with("MemTotal:") {
                    mem_total = line.split_whitespace().nth(1).map(|s| s.to_string());
                } else if line.starts_with("MemAvailable:") {
                    mem_avail = line.split_whitespace().nth(1).map(|s| s.to_string());
                } else if line.starts_with("MemFree:") {
                    mem_free = line.split_whitespace().nth(1).map(|s| s.to_string());
                }
            }
            let kb_to_gb = |s: Option<String>| {
                s.and_then(|v| v.parse::<u64>().ok())
                    .map(|kb| format!("{:.2} GiB", kb as f64 / 1_048_576.0))
            };
            if mem_total.is_some() || mem_avail.is_some() || mem_free.is_some() {
                let mut mem_lines = vec!["Memory:".to_string()];
                if let Some(t) = mem_total {
                    if let Some(g) = kb_to_gb(Some(t)) {
                        mem_lines.push(format!("  MemTotal:    {}", g));
                    }
                }
                if let Some(a) = mem_avail {
                    if let Some(g) = kb_to_gb(Some(a)) {
                        mem_lines.push(format!("  MemAvailable: {}", g));
                    }
                }
                if let Some(f) = mem_free {
                    if let Some(g) = kb_to_gb(Some(f)) {
                        mem_lines.push(format!("  MemFree:    {}", g));
                    }
                }
                return Some(mem_lines.join("\n"));
            }
        }
        if let Ok(out) = std::process::Command::new("free").arg("-h").output() {
            if out.status.success() {
                let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !s.is_empty() {
                    return Some(s);
                }
            }
        }
        None
    }

    /// Collect disk info: df -h (for non-collapsible display).
    fn collect_disk_info() -> Option<String> {
        if let Ok(out) = std::process::Command::new("df").args(["-h"]).output() {
            if out.status.success() {
                let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !s.is_empty() {
                    return Some(s);
                }
            }
        }
        None
    }

    /// Collect hardware: PCI, USB, lshw (collapsible block in UI).
    fn collect_hw_info() -> Option<String> {
        let mut sections: Vec<String> = Vec::new();

        // PCI: lspci (full)
        if let Ok(out) = std::process::Command::new("lspci").output() {
            if out.status.success() {
                let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !s.is_empty() {
                    sections.push(format!("PCI (lspci):\n{}", s));
                }
            }
        }

        // USB: lsusb (full)
        if let Ok(out) = std::process::Command::new("lsusb").output() {
            if out.status.success() {
                let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !s.is_empty() {
                    sections.push(format!("USB (lsusb):\n{}", s));
                }
            }
        }

        // lshw -short (full)
        if let Ok(output) = std::process::Command::new("lshw").args(["-short"]).output() {
            if output.status.success() {
                let s = String::from_utf8_lossy(&output.stdout).trim().to_string();
                if !s.is_empty() {
                    sections.push(format!("Hardware (lshw -short):\n{}", s));
                }
            }
        }

        if sections.is_empty() {
            None
        } else {
            Some(sections.join("\n\n"))
        }
    }

    /// Background loop: periodically report node status (machine info + capability status) to core.
    async fn node_status_loop(
        base_url: String,
        node_id: String,
        process_manager: Arc<ProcessManager>,
        package_storage_path: PathBuf,
    ) {
        let client = match reqwest::Client::builder()
            .timeout(std::time::Duration::from_secs(10))
            .build()
        {
            Ok(c) => c,
            Err(e) => {
                warn!("node status: failed to create HTTP client: {}", e);
                return;
            }
        };
        let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(15));
        loop {
            interval.tick().await;

            let machine_info = serde_json::json!({
                "os_version": Self::collect_os_version(),
                "cpu_info": Self::collect_cpu_info(),
                "memory_info": Self::collect_memory_info(),
                "disk_info": Self::collect_disk_info(),
                "hw_info": Self::collect_hw_info(),
            });

            let active_recipe = RecipeState::load(&package_storage_path)
                .ok()
                .flatten()
                .map(|s| s.recipe.name.clone());
            let packages: Vec<serde_json::Value> = PackageDatabase::load(&package_storage_path)
                .ok()
                .map(|db| {
                    db.list_packages()
                        .into_iter()
                        .map(|p| serde_json::json!({ "name": p.name, "version": p.version }))
                        .collect()
                })
                .unwrap_or_default();
            let running: Vec<serde_json::Value> = process_manager
                .get_running_processes()
                .into_iter()
                .map(|p| {
                    serde_json::json!({
                        "package_name": p.package_name,
                        "std_name": p.std_name,
                        "package_type": p.package_type,
                        "pid": p.pid,
                    })
                })
                .collect();

            let capability_status = serde_json::json!({
                "active_recipe": active_recipe,
                "packages": packages,
                "running": running,
            });

            let body = serde_json::json!({
                "node_id": node_id,
                "machine_info": machine_info,
                "capability_status": capability_status,
            });

            let post_url = format!("{}/api/node-status", base_url);
            if client.post(&post_url).json(&body).send().await.is_err() {
                trace!("node status: POST failed");
            }
        }
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
