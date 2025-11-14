use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use tokio::fs::OpenOptions;
use tokio::io::{AsyncWriteExt, BufWriter};
use tokio::process::Command;

/// Information about a running process for a capability or skill
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessInfo {
    pub package_name: String,
    pub std_name: String,
    pub package_type: String, // "cap" or "skl"
    pub pid: u32,
    pub log_file: PathBuf,
    pub hostname: String,
}

/// Manager for processes running capabilities and skills
pub struct ProcessManager {
    processes: Arc<Mutex<HashMap<String, ProcessInfo>>>, // key: "{package_type}::{std_name}"
    log_dir: PathBuf,
    state_file: PathBuf,
    hostname: String,
}

impl ProcessManager {
    pub fn new(log_dir: PathBuf) -> Result<Self> {
        // Ensure log directory exists
        std::fs::create_dir_all(&log_dir)
            .with_context(|| format!("Failed to create log directory: {}", log_dir.display()))?;

        // Get hostname
        let hostname = hostname::get()
            .context("Failed to get hostname")?
            .to_string_lossy()
            .to_string();

        // State file in /var/lib/robonix/processes.json
        let state_dir = PathBuf::from("/var/lib/robonix");
        std::fs::create_dir_all(&state_dir).with_context(|| {
            format!("Failed to create state directory: {}", state_dir.display())
        })?;
        let state_file = state_dir.join("processes.json");

        let mut manager = Self {
            processes: Arc::new(Mutex::new(HashMap::new())),
            log_dir,
            state_file,
            hostname,
        };

        // Load existing processes from state file
        manager.load_state()?;

        Ok(manager)
    }

    /// Load process state from persistent storage
    fn load_state(&mut self) -> Result<()> {
        if !self.state_file.exists() {
            return Ok(());
        }

        let content = std::fs::read_to_string(&self.state_file)
            .with_context(|| format!("Failed to read state file: {}", self.state_file.display()))?;

        let processes: Vec<ProcessInfo> = serde_json::from_str(&content).with_context(|| {
            format!("Failed to parse state file: {}", self.state_file.display())
        })?;

        // Verify processes are still running and filter out dead ones
        let mut valid_processes = HashMap::new();
        let original_count = processes.len();
        for process_info in processes {
            // Check if process is still running
            if Self::is_process_running(process_info.pid) {
                let key = format!("{}::{}", process_info.package_type, process_info.std_name);
                valid_processes.insert(key, process_info);
            } else {
                tracing::info!(
                    "Process {} (PID: {}) is no longer running, removing from state",
                    process_info.std_name,
                    process_info.pid
                );
            }
        }

        // Update state file with only valid processes
        if valid_processes.len() != original_count {
            self.save_state_internal(&valid_processes)?;
        }

        *self.processes.lock().unwrap() = valid_processes;

        Ok(())
    }

    /// Check if a process with given PID is still running
    fn is_process_running(pid: u32) -> bool {
        #[cfg(unix)]
        {
            use nix::sys::signal::kill;
            use nix::unistd::Pid;
            let pid = Pid::from_raw(pid as i32);
            // Send signal 0 to check if process exists
            kill(pid, None).is_ok()
        }
        #[cfg(not(unix))]
        {
            // On Windows, check process existence differently
            std::process::Command::new("tasklist")
                .args(&["/FI", &format!("PID eq {}", pid)])
                .output()
                .map(|output| {
                    let stdout = String::from_utf8_lossy(&output.stdout);
                    stdout.contains(&pid.to_string())
                })
                .unwrap_or(false)
        }
    }

    /// Save process state to persistent storage
    fn save_state(&self) -> Result<()> {
        let processes = self.processes.lock().unwrap();
        self.save_state_internal(&processes)
    }

    fn save_state_internal(&self, processes: &HashMap<String, ProcessInfo>) -> Result<()> {
        let processes_vec: Vec<&ProcessInfo> = processes.values().collect();
        let content = serde_json::to_string_pretty(&processes_vec)
            .context("Failed to serialize process state")?;

        std::fs::write(&self.state_file, content).with_context(|| {
            format!("Failed to write state file: {}", self.state_file.display())
        })?;

        Ok(())
    }

    pub fn get_hostname(&self) -> &str {
        &self.hostname
    }

    /// Start a process for a capability or skill
    pub async fn start_process(
        &self,
        package_name: &str,
        std_name: &str,
        package_type: &str,
        package_path: &Path,
        start_script: &str,
    ) -> Result<()> {
        let key = format!("{}::{}", package_type, std_name);

        // Check if already running
        {
            let processes = self.processes.lock().unwrap();
            if processes.contains_key(&key) {
                tracing::warn!("Process for {} already running, skipping", key);
                return Ok(());
            }
        }

        // Resolve script path
        let script_path = package_path.join(start_script);
        if !script_path.exists() {
            anyhow::bail!("Start script not found: {}", script_path.display());
        }

        // Make script executable
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mut perms = std::fs::metadata(&script_path)?.permissions();
            perms.set_mode(0o755);
            std::fs::set_permissions(&script_path, perms)?;
        }

        // Create log file path
        let log_filename = format!(
            "{}_{}_{}.log",
            package_name,
            package_type,
            std_name.replace("::", "_")
        );
        let log_file = self.log_dir.join(&log_filename);

        // Open log file for writing
        let log_file_handle = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&log_file)
            .await
            .with_context(|| format!("Failed to open log file: {}", log_file.display()))?;

        let mut log_writer = BufWriter::new(log_file_handle);

        // Write header to log
        let header = format!(
            "=== Started {} {} at {} ===\n",
            package_type,
            std_name,
            chrono::Utc::now().to_rfc3339()
        );
        log_writer.write_all(header.as_bytes()).await?;
        log_writer.flush().await?;

        // Start the process
        tracing::info!(
            "Starting process: {} (script: {})",
            key,
            script_path.display()
        );

        // Use tokio::process::Command for async I/O
        let mut cmd = Command::new(&script_path);
        cmd.current_dir(package_path);
        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());

        let mut child = cmd
            .spawn()
            .with_context(|| format!("Failed to start script: {}", script_path.display()))?;

        // Spawn task to capture output and write to log
        let log_file_clone = log_file.clone();
        let stdout = child.stdout.take();
        let stderr = child.stderr.take();

        if let Some(mut stdout) = stdout {
            let log_file_clone = log_file_clone.clone();
            tokio::spawn(async move {
                if let Ok(mut file) = OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&log_file_clone)
                    .await
                {
                    use tokio::io::AsyncReadExt;
                    let mut buffer = vec![0u8; 1024];
                    loop {
                        match stdout.read(&mut buffer).await {
                            Ok(0) => break, // EOF
                            Ok(n) => {
                                let _ = file.write_all(&buffer[..n]).await;
                                let _ = file.flush().await;
                            }
                            Err(_) => break,
                        }
                    }
                }
            });
        }

        if let Some(mut stderr) = stderr {
            let log_file_clone = log_file_clone.clone();
            tokio::spawn(async move {
                if let Ok(mut file) = OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&log_file_clone)
                    .await
                {
                    use tokio::io::AsyncReadExt;
                    let mut buffer = vec![0u8; 1024];
                    loop {
                        match stderr.read(&mut buffer).await {
                            Ok(0) => break, // EOF
                            Ok(n) => {
                                let _ = file.write_all(format!("[STDERR] ").as_bytes()).await;
                                let _ = file.write_all(&buffer[..n]).await;
                                let _ = file.flush().await;
                            }
                            Err(_) => break,
                        }
                    }
                }
            });
        }

        // Wait a bit to check if process started successfully
        tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

        // Check if process is still running
        match child.try_wait() {
            Ok(Some(status)) => {
                anyhow::bail!(
                    "Process exited immediately with status: {:?}. Check log: {}",
                    status,
                    log_file.display()
                );
            }
            Ok(None) => {
                // Process is still running, good
                tracing::info!("Process started successfully: {}", key);
            }
            Err(e) => {
                anyhow::bail!("Failed to check process status: {}", e);
            }
        }

        // Get PID and detach the child (let it run independently)
        let pid = child
            .id()
            .ok_or_else(|| anyhow::anyhow!("Failed to get process ID"))?;

        // Detach the child - it will continue running even if we drop the handle
        // We'll manage it by PID
        drop(child);

        // Store process info
        let process_info = ProcessInfo {
            package_name: package_name.to_string(),
            std_name: std_name.to_string(),
            package_type: package_type.to_string(),
            pid,
            log_file,
            hostname: self.hostname.clone(),
        };

        {
            let mut processes = self.processes.lock().unwrap();
            processes.insert(key, process_info.clone());
        }

        // Save state to persistent storage
        self.save_state()?;

        Ok(())
    }

    /// Get all running processes
    pub fn get_running_processes(&self) -> Vec<ProcessInfo> {
        let processes = self.processes.lock().unwrap();
        processes.values().cloned().collect()
    }

    /// Check if a process is running
    pub fn is_running(&self, std_name: &str, package_type: &str) -> bool {
        let key = format!("{}::{}", package_type, std_name);

        // Check in memory first
        let process_info = {
            let processes = self.processes.lock().unwrap();
            processes.get(&key).cloned()
        };

        if let Some(process_info) = process_info {
            // Verify the process is actually still running
            if Self::is_process_running(process_info.pid) {
                return true;
            } else {
                // Process is dead, remove it from state
                let mut processes = self.processes.lock().unwrap();
                processes.remove(&key);
                drop(processes);
                let _ = self.save_state();
                return false;
            }
        }

        false
    }

    /// Stop a specific process
    pub async fn stop_process(&self, std_name: &str, package_type: &str) -> Result<()> {
        let key = format!("{}::{}", package_type, std_name);
        
        // Get and remove process info, then drop the lock
        let process_info = {
            let mut processes = self.processes.lock().unwrap();
            processes.remove(&key)
        };

        if let Some(process_info) = process_info {
            tracing::info!("Stopping process: {} (PID: {})", key, process_info.pid);

            // Kill the process by PID
            #[cfg(unix)]
            {
                use nix::sys::signal::{kill, Signal};
                use nix::unistd::Pid;
                let pid = Pid::from_raw(process_info.pid as i32);
                if let Err(e) = kill(pid, Signal::SIGTERM) {
                    tracing::warn!(
                        "Failed to send SIGTERM to process {}: {:?}",
                        process_info.pid,
                        e
                    );
                    // Try SIGKILL as fallback
                    let _ = kill(pid, Signal::SIGKILL);
                }
            }
            #[cfg(not(unix))]
            {
                // On Windows, use taskkill or similar
                let _ = Command::new("taskkill")
                    .args(&["/PID", &process_info.pid.to_string(), "/F"])
                    .output()
                    .await;
            }

            // Append stop message to log
            if let Ok(mut file) = OpenOptions::new()
                .create(true)
                .append(true)
                .open(&process_info.log_file)
                .await
            {
                let stop_msg = format!(
                    "\n=== Stopped {} {} at {} ===\n",
                    package_type,
                    std_name,
                    chrono::Utc::now().to_rfc3339()
                );
                let _ = file.write_all(stop_msg.as_bytes()).await;
            }

            tracing::info!("Process stopped: {}", key);

            // Save state to persistent storage (lock is already dropped, so this is safe)
            self.save_state()?;
        } else {
            tracing::warn!("Process not found: {}", key);
        }

        Ok(())
    }

    /// Stop all processes
    pub async fn stop_all(&self) -> Result<()> {
        let processes = {
            let processes = self.processes.lock().unwrap();
            let keys: Vec<String> = processes.keys().cloned().collect();
            keys
        };

        for key in processes {
            let parts: Vec<&str> = key.split("::").collect();
            if parts.len() == 2 {
                let package_type = parts[0];
                let std_name = parts[1];
                let _ = self.stop_process(std_name, package_type).await;
            }
        }

        Ok(())
    }
}

// Note: We intentionally do NOT implement Drop for ProcessManager.
// Processes are started as daemon processes and should continue running
// even after the CLI exits. They can be stopped explicitly using the
// unregister command or stop_process/stop_all methods.
