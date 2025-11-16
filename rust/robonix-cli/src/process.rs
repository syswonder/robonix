use anyhow::{Context, Result};
use dirs;
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

/// Process start result with group information
#[derive(Debug, Clone)]
pub struct ProcessStartResult {
    pub pid: u32,
    pub pgid: Option<u32>,
    pub pids: Option<Vec<u32>>,
}

/// Process stop result with group information
#[derive(Debug, Clone)]
pub struct ProcessStopResult {
    pub pid: u32,
    pub pgid: Option<u32>,
    pub pids: Option<Vec<u32>>,
}

/// Process tree node structure
#[derive(Debug, Clone)]
pub struct ProcessTreeNode {
    pub pid: u32,
    pub cmd: String,
    pub children: Vec<ProcessTreeNode>,
}

impl ProcessTreeNode {
    /// Format tree as string with indentation
    pub fn format_tree(&self, prefix: &str, is_last: bool) -> String {
        let connector = if is_last { "└── " } else { "├── " };
        let mut result = format!("{}{}PID {}: {}\n", prefix, connector, self.pid, self.cmd);
        
        let new_prefix = if is_last {
            format!("{}    ", prefix)
        } else {
            format!("{}│   ", prefix)
        };
        
        let child_count = self.children.len();
        for (idx, child) in self.children.iter().enumerate() {
            let is_last_child = idx == child_count - 1;
            result.push_str(&child.format_tree(&new_prefix, is_last_child));
        }
        
        result
    }
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

        // State file in ~/.robonix/processes.json
        let home_dir = dirs::home_dir().context("Failed to get home directory")?;
        let state_dir = home_dir.join(".robonix");
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
        robonix_msg_path: Option<&PathBuf>,
    ) -> Result<ProcessStartResult> {
        let key = format!("{}::{}", package_type, std_name);

        // Check if already running
        {
            let processes = self.processes.lock().unwrap();
            if let Some(existing) = processes.get(&key) {
                tracing::warn!("Process for {} already running, skipping", key);
                // Return existing process info
                #[cfg(unix)]
                let (pgid, pids) = {
                    if let Ok(pgid) = Self::get_process_group_id(existing.pid) {
                        let pids = Self::get_processes_in_group(pgid).ok();
                        (Some(pgid), pids)
                    } else {
                        (None, None)
                    }
                };
                #[cfg(not(unix))]
                let (pgid, pids) = (None, None);
                return Ok(ProcessStartResult {
                    pid: existing.pid,
                    pgid,
                    pids,
                });
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
        
        // Set PYTHONUNBUFFERED=1 to disable Python output buffering
        // This ensures logs are written immediately
        cmd.env("PYTHONUNBUFFERED", "1");
        
        // Set ROBONIX_MSG_PATH from config or environment variable
        if std::env::var("ROBONIX_MSG_PATH").is_err() {
            if let Some(config_path) = robonix_msg_path {
                cmd.env("ROBONIX_MSG_PATH", config_path);
            }
        }

        // Create process in new process group for isolation
        #[cfg(unix)]
        {
            use nix::unistd::setsid;
            // Use pre_exec to create new session/process group before exec
            unsafe {
                cmd.pre_exec(|| {
                    // Create new session (which also creates new process group)
                    // This ensures all child processes are in the same group
                    setsid().map_err(|e| {
                        // Convert nix error to io::Error
                        std::io::Error::from_raw_os_error(e as i32)
                    })?;
                    Ok(())
                });
            }
        }

        let mut child = cmd
            .spawn()
            .with_context(|| format!("Failed to start script: {}", script_path.display()))?;

        // Spawn tasks to capture output and write to log
        // Use separate file handles for stdout and stderr to avoid synchronization issues
        let log_file_stdout = log_file.clone();
        let log_file_stderr = log_file.clone();
        let stdout = child.stdout.take();
        let stderr = child.stderr.take();

        // Capture stdout (line-by-line for better reliability)
        if let Some(stdout) = stdout {
            tokio::spawn(async move {
                if let Ok(mut file) = OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&log_file_stdout)
                    .await
                {
                    use tokio::io::{AsyncBufReadExt, BufReader};
                    let reader = BufReader::new(stdout);
                    let mut lines = reader.lines();
                    
                    while let Ok(Some(line)) = lines.next_line().await {
                        if let Err(e) = file.write_all(line.as_bytes()).await {
                            tracing::error!("Failed to write stdout to log: {}", e);
                            break;
                        }
                        if let Err(e) = file.write_all(b"\n").await {
                            tracing::error!("Failed to write newline: {}", e);
                            break;
                        }
                        // Flush after each line to ensure immediate persistence
                        if let Err(e) = file.flush().await {
                            tracing::error!("Failed to flush stdout log: {}", e);
                            break;
                        }
                    }
                }
            });
        }

        // Capture stderr with prefix (line-by-line for proper prefix placement)
        if let Some(stderr) = stderr {
            tokio::spawn(async move {
                if let Ok(mut file) = OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&log_file_stderr)
                    .await
                {
                    use tokio::io::{AsyncBufReadExt, BufReader};
                    let reader = BufReader::new(stderr);
                    let mut lines = reader.lines();
                    let mut is_new_line = true;
                    
                    while let Ok(Some(line)) = lines.next_line().await {
                        // Add prefix only at the start of each line
                        if is_new_line {
                            if let Err(e) = file.write_all(b"[STDERR] ").await {
                                tracing::error!("Failed to write stderr prefix: {}", e);
                                break;
                            }
                        }
                        if let Err(e) = file.write_all(line.as_bytes()).await {
                            tracing::error!("Failed to write stderr to log: {}", e);
                            break;
                        }
                        if let Err(e) = file.write_all(b"\n").await {
                            tracing::error!("Failed to write newline: {}", e);
                            break;
                        }
                        if let Err(e) = file.flush().await {
                            tracing::error!("Failed to flush stderr log: {}", e);
                            break;
                        }
                        is_new_line = true;
                    }
                }
            });
        }

        // Wait a bit to check if process started successfully and let initial output be captured
        tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;

        // Check if process is still running
        match child.try_wait() {
            Ok(Some(status)) => {
                // Process exited, wait a bit more for log capture to finish
                tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
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

        // Get process group information for return value
        #[cfg(unix)]
        let (pgid, pids) = {
            if let Ok(pgid) = Self::get_process_group_id(pid) {
                let pids = Self::get_processes_in_group(pgid).ok();
                (Some(pgid), pids)
            } else {
                (None, None)
            }
        };
        #[cfg(not(unix))]
        let (pgid, pids) = (None, None);

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

        Ok(ProcessStartResult { pid, pgid, pids })
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

    /// Kill a process group (more efficient than killing individual processes)
    #[cfg(unix)]
    fn kill_process_tree(&self, pid: u32) -> Result<()> {
        use nix::sys::signal::{killpg, kill, Signal};
        use nix::unistd::Pid;
        use std::io::{BufRead, BufReader};
        use std::process::Command as SyncCommand;

        let pid_obj = Pid::from_raw(pid as i32);
        
        // Get the actual process group ID from the process
        // If we used setsid, the PGID should equal the PID, but let's verify
        let pgid = match Self::get_process_group_id(pid) {
            Ok(gid) => {
                let pgid_obj = Pid::from_raw(gid as i32);
                // List all processes in the group before killing
                if let Ok(pids) = Self::get_processes_in_group(gid) {
                    tracing::info!("Stopping process group {} (root PID: {}): found {} processes: {:?}", gid, pid, pids.len(), pids);
                } else {
                    tracing::info!("Stopping process group {} (root PID: {})", gid, pid);
                }
                pgid_obj
            }
            Err(_) => {
                // Fallback: assume PGID equals PID (true if we used setsid)
                tracing::warn!("Could not get process group ID for PID {}, assuming PGID=PID", pid);
                tracing::info!("Stopping process (PID: {}, assumed PGID: {})", pid, pid);
                pid_obj
            }
        };
        
        // First, send SIGTERM to the entire process group
        if let Err(e) = killpg(pgid, Signal::SIGTERM) {
            tracing::warn!("Failed to send SIGTERM to process group {}: {:?}", pgid, e);
            // Fallback: try killing the process directly
            let _ = kill(pid_obj, Signal::SIGTERM);
        }
        
        // Wait a bit for processes to terminate gracefully
        std::thread::sleep(std::time::Duration::from_secs(2));
        
        // Check if process group still has any processes alive
        // Use pgrep to find processes in the group
        if let Ok(output) = SyncCommand::new("pgrep")
            .arg("-g")
            .arg(pgid.as_raw().to_string())
            .output()
        {
            if output.status.success() {
                let reader = BufReader::new(&*output.stdout);
                let mut still_alive = Vec::new();
                for line in reader.lines() {
                    if let Ok(pid_str) = line {
                        if let Ok(proc_pid) = pid_str.parse::<u32>() {
                            still_alive.push(proc_pid);
                        }
                    }
                }
                
                if !still_alive.is_empty() {
                    tracing::info!("Process group still has {} processes alive, sending SIGKILL", still_alive.len());
                    // Send SIGKILL to the process group
                    let _ = killpg(pgid, Signal::SIGKILL);
                    // Also kill individual processes as fallback
                    for proc_pid in still_alive {
                        let proc_pid_obj = Pid::from_raw(proc_pid as i32);
                        let _ = kill(proc_pid_obj, Signal::SIGKILL);
                    }
                }
            }
        } else {
            // Fallback: try to kill the process directly if pgrep fails
            // Check if process still exists
            if let Ok(output) = SyncCommand::new("ps")
                .arg("-p")
                .arg(pid.to_string())
                .output()
            {
                if output.status.success() {
                    let _ = kill(pid_obj, Signal::SIGKILL);
                }
            }
        }
        
        Ok(())
    }
    
    /// Get process group ID for a given PID
    #[cfg(unix)]
    fn get_process_group_id(pid: u32) -> Result<u32> {
        use std::process::Command as SyncCommand;
        
        // Use ps to get the process group ID
        if let Ok(output) = SyncCommand::new("ps")
            .arg("-o")
            .arg("pgid=")
            .arg("-p")
            .arg(pid.to_string())
            .output()
        {
            if output.status.success() {
                let pgid_str = String::from_utf8_lossy(&output.stdout).trim().to_string();
                if let Ok(pgid) = pgid_str.parse::<u32>() {
                    return Ok(pgid);
                }
            }
        }
        
        anyhow::bail!("Failed to get process group ID for PID {}", pid)
    }
    
    /// Get all process IDs in a process group
    #[cfg(unix)]
    fn get_processes_in_group(pgid: u32) -> Result<Vec<u32>> {
        use std::io::{BufRead, BufReader};
        use std::process::Command as SyncCommand;
        use std::process::Stdio;
        
        let mut pids = Vec::new();
        
        // Use pgrep to find all processes in the group
        if let Ok(output) = SyncCommand::new("pgrep")
            .arg("-g")
            .arg(pgid.to_string())
            .stdout(Stdio::piped())
            .output()
        {
            if output.status.success() {
                let reader = BufReader::new(&*output.stdout);
                for line in reader.lines() {
                    if let Ok(pid_str) = line {
                        if let Ok(pid) = pid_str.parse::<u32>() {
                            pids.push(pid);
                        }
                    }
                }
            }
        }
        
        Ok(pids)
    }
    
    /// Get process tree structure (parent-child relationships)
    #[cfg(unix)]
    pub fn get_process_tree(pid: u32) -> Result<ProcessTreeNode> {
        use std::collections::HashMap;
        use std::io::{BufRead, BufReader};
        use std::process::Command as SyncCommand;
        use std::process::Stdio;
        
        #[derive(Debug, Clone)]
        struct ProcessInfo {
            pid: u32,
            ppid: u32,
            cmd: String,
        }
        
        // Get all processes with their parent PIDs
        let mut processes: HashMap<u32, ProcessInfo> = HashMap::new();
        
        // Use ps to get process tree with full command
        // Use -ww to get full command line (no width limit)
        if let Ok(output) = SyncCommand::new("ps")
            .arg("-eo")
            .arg("pid,ppid,args")
            .arg("--no-headers")
            .stdout(Stdio::piped())
            .output()
        {
            if output.status.success() {
                let reader = BufReader::new(&*output.stdout);
                for line in reader.lines() {
                    if let Ok(line_str) = line {
                        // Parse line: PID PPID COMMAND...
                        // We need to handle spaces in command, so split by first two numbers
                        let trimmed = line_str.trim();
                        if let Some(first_space) = trimmed.find(' ') {
                            if let Ok(proc_pid) = trimmed[..first_space].parse::<u32>() {
                                let rest = &trimmed[first_space + 1..].trim_start();
                                if let Some(second_space) = rest.find(' ') {
                                    if let Ok(proc_ppid) = rest[..second_space].parse::<u32>() {
                                        let cmd = rest[second_space + 1..].trim().to_string();
                                        // Truncate long commands for display
                                        let cmd_display = if cmd.len() > 60 {
                                            format!("{}...", &cmd[..57])
                                        } else {
                                            cmd
                                        };
                                        processes.insert(proc_pid, ProcessInfo {
                                            pid: proc_pid,
                                            ppid: proc_ppid,
                                            cmd: cmd_display,
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Check if root process exists
        if !processes.contains_key(&pid) {
            anyhow::bail!("Process {} not found in process list", pid);
        }
        
        // Build tree starting from root PID
        fn build_tree(pid: u32, processes: &HashMap<u32, ProcessInfo>) -> ProcessTreeNode {
            let proc_info = processes.get(&pid);
            let cmd = proc_info.map(|p| p.cmd.clone()).unwrap_or_else(|| format!("[PID {}]", pid));
            
            // Find all children
            let children: Vec<ProcessTreeNode> = processes
                .values()
                .filter(|p| p.ppid == pid)
                .map(|p| build_tree(p.pid, processes))
                .collect();
            
            ProcessTreeNode {
                pid,
                cmd,
                children,
            }
        }
        
        Ok(build_tree(pid, &processes))
    }
    
    #[cfg(not(unix))]
    pub fn get_process_tree(pid: u32) -> Result<ProcessTreeNode> {
        // On Windows, return simple structure
        Ok(ProcessTreeNode {
            pid,
            cmd: format!("[PID {}]", pid),
            children: Vec::new(),
        })
    }
    
    #[cfg(not(unix))]
    fn kill_process_tree(&self, pid: u32) -> Result<()> {
        // On Windows, use taskkill with /T flag to kill process tree
        use std::process::Command as SyncCommand;
        let _ = SyncCommand::new("taskkill")
            .args(&["/PID", &pid.to_string(), "/T", "/F"])
            .output();
        Ok(())
    }

    /// Stop a specific process
    pub async fn stop_process(&self, std_name: &str, package_type: &str) -> Result<ProcessStopResult> {
        let key = format!("{}::{}", package_type, std_name);
        
        // Get and remove process info, then drop the lock
        let process_info = {
            let mut processes = self.processes.lock().unwrap();
            processes.remove(&key)
        };

        if let Some(process_info) = process_info {
            tracing::info!("Stopping process: {} (PID: {})", key, process_info.pid);

            // Get process group information before killing
            #[cfg(unix)]
            let (pgid, pids) = {
                if let Ok(pgid) = Self::get_process_group_id(process_info.pid) {
                    let pids = Self::get_processes_in_group(pgid).ok();
                    (Some(pgid), pids)
                } else {
                    (None, None)
                }
            };
            #[cfg(not(unix))]
            let (pgid, pids) = (None, None);

            // Kill the process tree (parent + all children)
            if let Err(e) = self.kill_process_tree(process_info.pid) {
                tracing::warn!(
                    "Failed to kill process tree for PID {}: {:?}",
                    process_info.pid,
                    e
                );
                // Fallback: try to kill just the main process
                #[cfg(unix)]
                {
                    use nix::sys::signal::{kill, Signal};
                    use nix::unistd::Pid;
                    let pid = Pid::from_raw(process_info.pid as i32);
                    let _ = kill(pid, Signal::SIGTERM);
                    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
                    let _ = kill(pid, Signal::SIGKILL);
                }
                #[cfg(not(unix))]
                {
                    let _ = Command::new("taskkill")
                        .args(&["/PID", &process_info.pid.to_string(), "/F"])
                        .output()
                        .await;
                }
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
            
            Ok(ProcessStopResult {
                pid: process_info.pid,
                pgid,
                pids,
            })
        } else {
            tracing::warn!("Process not found: {}", key);
            anyhow::bail!("Process not found: {}", key)
        }
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
