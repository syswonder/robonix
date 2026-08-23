// SPDX-License-Identifier: MulanPSL-2.0
// Process Module
//
// Process management for robonix-cli (start/stop/monitor processes)

use anyhow::{Context, Result};
use robonix_scribe::{info, warn};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use tokio::fs::OpenOptions;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt};
use tokio::process::Command;

/// Information about a running process for a capability or skill
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessInfo {
    pub package_name: String,
    pub std_name: String,
    pub package_type: String, // "package" today; reserved for future kind tags
    pub pid: u32,
    pub log_file: PathBuf,
    pub hostname: String,
    #[serde(default)]
    pub package_path: Option<PathBuf>,
    #[serde(default)]
    pub command: String,
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

/// Resolve the per-deploy process-state path from the log directory.
///
/// Two layouts are recognised:
///
///   * `<…>/rbnx-boot/logs/`  →  `<…>/rbnx-boot/processes.json` (sits
///     next to `rbnx-boot/state.json` written by `rbnx boot`/`shutdown`)
///   * anything else          →  `<log_dir>/../processes.json`
///     (covers `rbnx start --standalone` on a package whose default
///     `rbnx-build/logs/` is the log root)
///
/// The result is always inside the per-deploy tree — never `~/.robonix/`,
/// which is reserved for cross-deploy user state.
fn derive_state_file_path(log_dir: &Path) -> PathBuf {
    if let Some(parent) = log_dir.parent()
        && parent.file_name().and_then(|s| s.to_str()) == Some("rbnx-boot")
    {
        return parent.join("processes.json");
    }
    match log_dir.parent() {
        Some(p) => p.join("processes.json"),
        None => log_dir.join("processes.json"),
    }
}

/// Manager for processes running capabilities and skills
pub struct ProcessManager {
    processes: Arc<Mutex<HashMap<String, ProcessInfo>>>, // key: "{package_type}::{std_name}"
    /// Per-package log directory for Scribe structured logs.
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

        // State file lives in the per-deploy root, NOT ~/.robonix/.
        //
        // ~/.robonix/ is for cross-deploy user state (config.yaml, chat.yaml,
        // voiceprint enrollments, installed package db). The package runtime
        // record belongs to ONE deploy and dies with it, so it must live
        // inside the deploy tree — the same place `rbnx shutdown` reads
        // `<manifest-dir>/rbnx-boot/state.json` from — and disappear on
        // shutdown. A `~/.robonix/processes.json` from a half-killed boot
        // would otherwise leak across deploys and confuse `rbnx start` on
        // unrelated packages.
        //
        // The deploy root is anchored to the log directory (which
        // `rbnx start` defaults to `<pkg>/rbnx-build/logs`, and
        // `rbnx boot` overrides to `<manifest-dir>/rbnx-boot/logs`).
        // State file is the sibling `processes.json` in the same dir —
        // or, when a deploy root is unambiguous (`<…>/rbnx-boot/logs`),
        // inside the `rbnx-boot/` parent so it sits next to
        // `rbnx-boot/state.json` and `rbnx-boot/logs/`.
        let state_file = derive_state_file_path(&log_dir);

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

        if content.trim().is_empty() {
            warn!(
                "Process state file {} is empty; resetting stale runtime state",
                self.state_file.display()
            );
            self.save_state_internal(&HashMap::new())?;
            return Ok(());
        }

        let processes: Vec<ProcessInfo> = match serde_json::from_str(&content) {
            Ok(processes) => processes,
            Err(err) => {
                warn!(
                    "Process state file {} is invalid ({err}); resetting stale runtime state",
                    self.state_file.display()
                );
                self.save_state_internal(&HashMap::new())?;
                return Ok(());
            }
        };

        // Verify processes are still running and filter out dead ones
        let mut valid_processes = HashMap::new();
        let original_count = processes.len();
        for process_info in processes {
            // Check if process is still live. `kill(pid, 0)` alone accepts
            // zombies and PID reuse; validate procfs too when available.
            if Self::is_process_record_live(&process_info) {
                let key = format!("{}::{}", process_info.package_type, process_info.std_name);
                valid_processes.insert(key, process_info);
            } else {
                info!(
                    "Process {} (PID: {}) is no longer running, removing from state",
                    process_info.std_name, process_info.pid
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

    /// Check if a persisted process record still points at a live process.
    fn is_process_record_live(process_info: &ProcessInfo) -> bool {
        if !Self::is_process_running(process_info.pid) {
            return false;
        }
        #[cfg(unix)]
        {
            if Self::is_process_zombie(process_info.pid) {
                return false;
            }
            let Some(cmdline) = Self::process_cmdline(process_info.pid) else {
                return false;
            };
            if cmdline.trim().is_empty() || cmdline.contains("<defunct>") {
                return false;
            }
            // Old state files did not store command/cwd, which made PID reuse
            // indistinguishable from a real still-running package. Treat them
            // as stale after this schema upgrade; new records below carry both.
            if process_info.command.trim().is_empty() || !cmdline.contains(&process_info.command) {
                return false;
            }
            let Some(expected_cwd) = process_info.package_path.as_ref() else {
                return false;
            };
            if let Ok(actual_cwd) = std::fs::read_link(format!("/proc/{}/cwd", process_info.pid)) {
                if actual_cwd != *expected_cwd {
                    return false;
                }
            } else {
                return false;
            }
        }
        true
    }

    #[cfg(unix)]
    fn is_process_zombie(pid: u32) -> bool {
        let path = format!("/proc/{pid}/status");
        let Ok(status) = std::fs::read_to_string(path) else {
            return true;
        };
        status
            .lines()
            .find(|line| line.starts_with("State:"))
            .map(|line| line.contains(" Z") || line.contains(" X"))
            .unwrap_or(true)
    }

    #[cfg(unix)]
    fn process_cmdline(pid: u32) -> Option<String> {
        let path = format!("/proc/{pid}/cmdline");
        let raw = std::fs::read(path).ok()?;
        Some(
            String::from_utf8_lossy(&raw)
                .replace('\0', " ")
                .trim()
                .to_string(),
        )
    }

    /// Check if a process with given PID exists. This is only the first
    /// liveness probe; callers that use persisted state should prefer
    /// `is_process_record_live`.
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

    /// Start a process; blocks until it exits.  Stdout / stderr are captured
    /// line-by-line and written to Scribe structured logs under
    /// `$SCRIBE_LOG_DIR/{tag}.log` (tag = `std_name`, e.g.
    /// `"com.robonix.service.mapping"`).  Lines are NOT forwarded to the
    /// terminal — `rbnx boot` owns the display and raw package output would
    /// drown out the boot progress lines.  Use `rbnx logs -f` for live
    /// per-package log tailing.
    pub async fn start_process(
        &self,
        _package_name: &str,
        std_name: &str,
        package_type: &str,
        package_path: &Path,
        start_script: &str,
    ) -> Result<ProcessStartResult> {
        let key = format!("{}::{}", package_type, std_name);
        {
            let processes = self.processes.lock().unwrap();
            if let Some(existing) = processes.get(&key) {
                warn!("Process for {} already running, skipping", key);
                #[cfg(unix)]
                let (pgid, pids) = {
                    if let Ok(pgid) = Self::get_process_group_id(existing.pid) {
                        (Some(pgid), Self::get_processes_in_group(pgid).ok())
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

        let start_script = start_script.trim();
        if start_script.is_empty() {
            anyhow::bail!("start_script is empty");
        }

        // Use `bash -c` (not `-lc`) so we inherit the parent environment (PATH, conda, etc.).
        // A login shell can reset PATH via /etc/profile and pick a different `python3` than the caller.
        #[cfg(unix)]
        let mut cmd = Command::new("bash");
        #[cfg(unix)]
        cmd.arg("-c").arg(start_script);
        #[cfg(not(unix))]
        let mut cmd = Command::new("sh");
        #[cfg(not(unix))]
        cmd.arg("-c").arg(start_script);

        cmd.current_dir(package_path)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .env("PYTHONUNBUFFERED", "1")
            .env("SCRIBE_LOG_DIR", &self.log_dir);
        #[cfg(unix)]
        if std::env::var_os("RBNX_DEPLOY_MANAGED").is_none() {
            // A standalone `rbnx start` owns a group for its package. When
            // boot spawned this wrapper, preserve boot's PGID so one teardown
            // reaches the wrapper and its actual package process together.
            cmd.process_group(0);
        }

        let mut child = cmd
            .spawn()
            .with_context(|| format!("Failed to start: {}", start_script))?;
        let pid = child
            .id()
            .ok_or_else(|| anyhow::anyhow!("Failed to get process ID"))?;
        info!("Running {} (PID {})", key, pid);
        let log_file = self.log_dir.join(format!("{std_name}.log"));
        {
            let mut processes = self.processes.lock().unwrap();
            processes.insert(
                key.clone(),
                ProcessInfo {
                    package_name: _package_name.to_string(),
                    std_name: std_name.to_string(),
                    package_type: package_type.to_string(),
                    pid,
                    log_file: log_file.clone(),
                    hostname: self.hostname.clone(),
                    package_path: Some(package_path.to_path_buf()),
                    command: start_script.to_string(),
                },
            );
        }
        self.save_state()?;

        // Pipe stdout / stderr through Scribe so structured logs land in
        // $SCRIBE_LOG_DIR/{tag}.log.  Do NOT forward to the terminal —
        // `rbnx boot` owns the display and raw package output would
        // drown out the boot progress lines.
        let stdout = child.stdout.take().expect("stdout not piped");
        let stderr = child.stderr.take().expect("stderr not piped");
        // Scribe tag + log-file stem = `std_name` (the provider_id) verbatim,
        // matching `deploy::log_path`. No name mangling.
        let tag = std_name.to_string();

        let stdout_task = tokio::spawn(async move {
            let reader = tokio::io::BufReader::new(stdout);
            let mut lines = reader.lines();
            while let Ok(Some(line)) = lines.next_line().await {
                robonix_scribe::ingest(&tag, &line);
            }
        });

        let tag2 = std_name.to_string();
        let stderr_task = tokio::spawn(async move {
            let reader = tokio::io::BufReader::new(stderr);
            let mut lines = reader.lines();
            while let Ok(Some(line)) = lines.next_line().await {
                // Many programs (Python in particular) write INFO and
                // WARNING messages to stderr — use `info` rather than
                // `warn` so the level in the file doesn't misrepresent
                // the actual severity.
                robonix_scribe::ingest(&tag2, &line);
            }
        });

        let status = child
            .wait()
            .await
            .with_context(|| "Failed to wait for process")?;

        // Drain remaining pipe output before returning.
        let _ = tokio::join!(stdout_task, stderr_task);

        {
            let mut processes = self.processes.lock().unwrap();
            processes.remove(&key);
        }
        self.save_state()?;

        if !status.success() {
            anyhow::bail!("{}: process exited with {}", std_name, status);
        }

        #[cfg(unix)]
        let (pgid, pids) = {
            if let Ok(pgid) = Self::get_process_group_id(pid) {
                (Some(pgid), Self::get_processes_in_group(pgid).ok())
            } else {
                (None, None)
            }
        };
        #[cfg(not(unix))]
        let (pgid, pids) = (None, None);

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
            if Self::is_process_record_live(&process_info) {
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

    /// Return whether `start_process` has inserted the in-memory runtime
    /// record, without re-validating the child's current command line.
    ///
    /// This is a narrow spawn-synchronization primitive for callers that run
    /// lifecycle supervision alongside `start_process`. A valid package start
    /// body may `exec` into Docker or another launcher immediately, so the
    /// stricter persisted-record identity check used by [`Self::is_running`]
    /// is intentionally not appropriate during this short hand-off window.
    pub fn has_process_record(&self, std_name: &str, package_type: &str) -> bool {
        let key = format!("{}::{}", package_type, std_name);
        self.processes.lock().unwrap().contains_key(&key)
    }

    /// Kill a process group (more efficient than killing individual processes)
    #[cfg(unix)]
    fn kill_process_tree(&self, pid: u32) -> Result<()> {
        use nix::sys::signal::{Signal, kill, killpg};
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
                    info!(
                        "Stopping process group {} (root PID: {}): found {} processes: {:?}",
                        gid,
                        pid,
                        pids.len(),
                        pids
                    );
                } else {
                    info!("Stopping process group {} (root PID: {})", gid, pid);
                }
                pgid_obj
            }
            Err(_) => {
                // Fallback: assume PGID equals PID (true if we used setsid)
                warn!(
                    "Could not get process group ID for PID {}, assuming PGID=PID",
                    pid
                );
                info!("Stopping process (PID: {}, assumed PGID: {})", pid, pid);
                pid_obj
            }
        };

        // First, send SIGTERM to the entire process group
        if let Err(e) = killpg(pgid, Signal::SIGTERM) {
            warn!("Failed to send SIGTERM to process group {}: {:?}", pgid, e);
            // Fallback: try killing the process directly
            let _ = kill(pid_obj, Signal::SIGTERM);
        }

        // Wait for processes to terminate gracefully, but check periodically
        let max_wait = std::time::Duration::from_secs(1);
        let check_interval = std::time::Duration::from_millis(100);
        let start_time = std::time::Instant::now();

        while start_time.elapsed() < max_wait {
            // Check if process group still has any processes alive
            if let Ok(output) = SyncCommand::new("pgrep")
                .arg("-g")
                .arg(pgid.as_raw().to_string())
                .output()
                && !output.status.success()
            {
                // No processes in group, they've all terminated
                break;
            }
            std::thread::sleep(check_interval);
        }

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
                    if let Ok(pid_str) = line
                        && let Ok(proc_pid) = pid_str.parse::<u32>()
                    {
                        still_alive.push(proc_pid);
                    }
                }

                if !still_alive.is_empty() {
                    info!(
                        "Process group still has {} processes alive, sending SIGKILL",
                        still_alive.len()
                    );
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
                && output.status.success()
            {
                let _ = kill(pid_obj, Signal::SIGKILL);
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
            && output.status.success()
        {
            let pgid_str = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if let Ok(pgid) = pgid_str.parse::<u32>() {
                return Ok(pgid);
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
            && output.status.success()
        {
            let reader = BufReader::new(&*output.stdout);
            for line in reader.lines() {
                if let Ok(pid_str) = line
                    && let Ok(pid) = pid_str.parse::<u32>()
                {
                    pids.push(pid);
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
            && output.status.success()
        {
            let reader = BufReader::new(&*output.stdout);
            for line_str in reader.lines().map_while(Result::ok) {
                // Parse line: PID PPID COMMAND...
                // We need to handle spaces in command, so split by first two numbers
                let trimmed = line_str.trim();
                if let Some(first_space) = trimmed.find(' ')
                    && let Ok(proc_pid) = trimmed[..first_space].parse::<u32>()
                {
                    let rest = &trimmed[first_space + 1..].trim_start();
                    if let Some(second_space) = rest.find(' ')
                        && let Ok(proc_ppid) = rest[..second_space].parse::<u32>()
                    {
                        let cmd = rest[second_space + 1..].trim().to_string();
                        // Truncate long commands for display
                        let cmd_display = if cmd.len() > 60 {
                            format!("{}...", &cmd[..57])
                        } else {
                            cmd
                        };
                        processes.insert(
                            proc_pid,
                            ProcessInfo {
                                pid: proc_pid,
                                ppid: proc_ppid,
                                cmd: cmd_display,
                            },
                        );
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
            let cmd = proc_info
                .map(|p| p.cmd.clone())
                .unwrap_or_else(|| format!("[PID {}]", pid));

            // Find all children
            let children: Vec<ProcessTreeNode> = processes
                .values()
                .filter(|p| p.ppid == pid)
                .map(|p| build_tree(p.pid, processes))
                .collect();

            ProcessTreeNode { pid, cmd, children }
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
    pub async fn stop_process(
        &self,
        std_name: &str,
        package_type: &str,
    ) -> Result<ProcessStopResult> {
        let key = format!("{}::{}", package_type, std_name);

        // Get and remove process info, then drop the lock
        let process_info = {
            let mut processes = self.processes.lock().unwrap();
            processes.remove(&key)
        };

        if let Some(process_info) = process_info {
            info!("Stopping process: {} (PID: {})", key, process_info.pid);

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
                warn!(
                    "Failed to kill process tree for PID {}: {:?}",
                    process_info.pid, e
                );
                // Fallback: try to kill just the main process
                #[cfg(unix)]
                {
                    use nix::sys::signal::{Signal, kill};
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

            info!("Process stopped: {}", key);

            // Save state to persistent storage (lock is already dropped, so this is safe)
            self.save_state()?;

            Ok(ProcessStopResult {
                pid: process_info.pid,
                pgid,
                pids,
            })
        } else {
            warn!("Process not found: {}", key);
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

    /// Stop all processes for a given package name
    pub async fn stop_by_package(&self, package_name: &str) -> Result<Vec<String>> {
        let to_stop: Vec<(String, String)> = {
            let processes = self.processes.lock().unwrap();
            processes
                .values()
                .filter(|p| p.package_name == package_name)
                .map(|p| (p.std_name.clone(), p.package_type.clone()))
                .collect()
        };

        let mut stopped = Vec::new();
        for (std_name, package_type) in to_stop {
            if self.stop_process(&std_name, &package_type).await.is_ok() {
                stopped.push(std_name);
            }
        }
        Ok(stopped)
    }
}

// Note: We intentionally do NOT implement Drop for ProcessManager.
// Processes are started as daemon processes and should continue running
// even after the CLI exits. They can be stopped explicitly using the
// unregister command or stop_process/stop_all methods.

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn unique_temp_dir(label: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "robonix-process-test-{label}-{}-{}",
            std::process::id(),
            chrono::Utc::now().timestamp_nanos_opt().unwrap_or_default()
        ));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn empty_process_state_is_reset_instead_of_fatal() {
        let root = unique_temp_dir("empty");
        let log_dir = root.join("rbnx-boot").join("logs");
        fs::create_dir_all(&log_dir).unwrap();
        let state_file = root.join("rbnx-boot").join("processes.json");
        fs::write(&state_file, "").unwrap();

        let manager = ProcessManager::new(log_dir).unwrap();

        assert!(manager.processes.lock().unwrap().is_empty());
        assert_eq!(fs::read_to_string(&state_file).unwrap(), "[]");
        let _ = fs::remove_dir_all(root);
    }

    #[test]
    fn invalid_process_state_is_reset_instead_of_fatal() {
        let root = unique_temp_dir("invalid");
        let log_dir = root.join("rbnx-boot").join("logs");
        fs::create_dir_all(&log_dir).unwrap();
        let state_file = root.join("rbnx-boot").join("processes.json");
        fs::write(&state_file, "{").unwrap();

        let manager = ProcessManager::new(log_dir).unwrap();

        assert!(manager.processes.lock().unwrap().is_empty());
        assert_eq!(fs::read_to_string(&state_file).unwrap(), "[]");
        let _ = fs::remove_dir_all(root);
    }
}
