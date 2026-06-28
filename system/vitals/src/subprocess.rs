// SPDX-License-Identifier: MulanPSL-2.0
//
// subprocess — shared helper for Python script subprocess management.
//
// Both board (BoardCollector) and body (BodyCollector) use the same
// stdin/stdout JSON protocol to communicate with Python hardware bridges.
// When Soma is ready, SubprocessHandle is replaced with gRPC clients —
// the rest of the pipeline stays unchanged.

use anyhow::Context;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, Command, Stdio};

/// Manages a long-running Python subprocess that reads JSON commands from
/// stdin and writes JSON responses to stdout (one line per response).
pub struct SubprocessHandle {
    stdin: ChildStdin,
    reader: BufReader<std::process::ChildStdout>,
    #[allow(dead_code)]
    child: Child,
    label: String,
}

impl SubprocessHandle {
    /// Spawn a Python script with its stdin/stdout piped.
    #[allow(dead_code)]
    pub fn spawn(script: &str, python_bin: &str, label: &str) -> anyhow::Result<Self> {
        Self::spawn_with_args(script, python_bin, &[], label)
    }

    /// Spawn a Python script with extra CLI arguments (e.g. CAN port).
    pub fn spawn_with_args(
        script: &str,
        python_bin: &str,
        args: &[&str],
        label: &str,
    ) -> anyhow::Result<Self> {
        let mut cmd = Command::new(python_bin);
        cmd.arg(script);
        for a in args {
            cmd.arg(a);
        }
        let mut child = cmd
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .with_context(|| format!("spawn {label} script '{script}' with '{python_bin}'"))?;

        let stdin = child.stdin.take().context("capture script stdin")?;
        let stdout = child.stdout.take().context("capture script stdout")?;
        let reader = BufReader::new(stdout);

        Ok(Self {
            stdin,
            reader,
            child,
            label: label.to_string(),
        })
    }

    /// Send a `{"cmd":"collect"}` request and read one JSON line back.
    /// Returns the raw trimmed line, or `None` on any I/O error.
    pub fn collect_json(&mut self) -> Option<String> {
        let cmd = serde_json::json!({"cmd": "collect"});
        if let Err(e) = writeln!(self.stdin, "{cmd}") {
            log::warn!("[{}] write to script failed: {e:#}", self.label);
            return None;
        }
        if let Err(e) = self.stdin.flush() {
            log::warn!("[{}] flush stdin failed: {e:#}", self.label);
            return None;
        }
        let mut line = String::new();
        if let Err(e) = self.reader.read_line(&mut line) {
            log::warn!("[{}] read from script failed: {e:#}", self.label);
            return None;
        }
        let trimmed = line.trim().to_string();
        if trimmed.is_empty() {
            log::warn!("[{}] empty response from script", self.label);
            return None;
        }
        Some(trimmed)
    }

    /// Check whether the subprocess is still running.
    #[allow(dead_code)]
    pub fn is_alive(&mut self) -> bool {
        match self.child.try_wait() {
            Ok(None) => true,
            Ok(Some(status)) => {
                log::warn!("[{}] script exited with {status}", self.label);
                false
            }
            Err(e) => {
                log::warn!("[{}] try_wait failed: {e:#}", self.label);
                false
            }
        }
    }

    /// Replace the dead subprocess with a fresh one.
    #[allow(dead_code)]
    pub fn restart(&mut self, script: &str, python_bin: &str) -> anyhow::Result<()> {
        let mut child = Command::new(python_bin)
            .arg(script)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .with_context(|| format!("restart {} script '{script}'", self.label))?;

        self.stdin = child.stdin.take().context("capture script stdin")?;
        let stdout = child.stdout.take().context("capture script stdout")?;
        self.reader = BufReader::new(stdout);
        self.child = child;
        Ok(())
    }
}
