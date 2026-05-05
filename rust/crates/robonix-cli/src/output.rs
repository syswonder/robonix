// SPDX-License-Identifier: MulanPSL-2.0
// Output Module
//
// Output formatting and display utilities for robonix-cli

use colored::*;
use std::io::{self, Write};

/// Print a main action header (e.g., "Installing", "Registering")
pub fn action(action: &str, target: &str) {
    println!("{} {}", format!("[{}]", action).green().bold(), target);
}

/// Print a success completion message
pub fn success(message: &str) {
    println!("{} {}", "✓".green().bold(), message.green());
}

/// Print an info message
pub fn info(message: &str) {
    println!("{}", message);
}

/// Print a warning message
pub fn warning(message: &str) {
    println!("{} {}", "⚠".yellow().bold(), message.yellow());
}

/// Print an error message
pub fn error(message: &str) {
    eprintln!("{} {}", "✗".red().bold(), message.red());
}

/// Print a step message (like "Validating", "Processing", etc.)
pub fn step(action: &str, target: &str) {
    println!("  {} {}", format!("-> {}", action).cyan(), target);
}

/// Print a sub-step message (indented detail)
pub fn sub_step(message: &str) {
    println!("    {}", message);
}

/// Print a checkmark success message for individual items
pub fn check(message: &str) {
    println!("  {} {}", "✓".green(), message);
}

/// Print a cross error message for individual items
pub fn cross(message: &str) {
    eprintln!("  {} {}", "✗".red(), message);
}

/// Print a summary line
pub fn summary(message: &str) {
    println!("\n{}", message.dimmed());
}

// ── Boot-log helpers (init-system style) ────────────────────────────
//
// All boot/start lines route through these so the boot output reads
// like a systemd / SysV bring-up (`[ OK ] component  detail`) instead
// of the previous indented free-form `sub_step("[system] foo -> long
// path...")` lines that buried the salient info.

const W_BADGE: usize = 7; // "[ OK ] " etc.
const W_NAME: usize = 18;

/// `[ OK ] name  detail` — a component came up. The leading
/// `\r\x1b[K` clears any in-place spinner line that boot_progress
/// might have left, so the final result lands cleanly without a
/// trailing fragment of "registering... 4.2s".
pub fn boot_ok(name: &str, detail: &str) {
    println!(
        "\r\x1b[K{}  {:<width$}{}",
        format!("{:<bw$}", "[ OK ]".green().bold().to_string(), bw = W_BADGE),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// `[FAIL] name  detail` — a component failed to come up.
pub fn boot_fail(name: &str, detail: &str) {
    eprintln!(
        "\r\x1b[K{}  {:<width$}{}",
        format!("{:<bw$}", "[FAIL]".red().bold().to_string(), bw = W_BADGE),
        name,
        detail.red(),
        width = W_NAME,
    );
}

/// In-place spinner frame. Renders to stdout with `\r` (no newline),
/// flushes, returns. Caller is expected to overwrite or `boot_ok` /
/// `boot_fail` to finalize. Frames cycle through Braille dots — the
/// systemd-look spinner most users recognise from Linux init logs.
pub fn boot_progress(name: &str, detail: &str, frame: usize) {
    const GLYPHS: &[char] = &['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏'];
    let g = GLYPHS[frame % GLYPHS.len()];
    let badge = format!("[ {} ]", g);
    print!(
        "\r\x1b[K{}  {:<width$}{}",
        format!("{:<bw$}", badge.cyan().to_string(), bw = W_BADGE),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
    let _ = io::stdout().flush();
}

/// `[SKIP] name  detail` — declared in manifest but skipped (not
/// installed / disabled / out-of-scope on this host).
pub fn boot_skip(name: &str, detail: &str) {
    println!(
        "{}  {:<width$}{}",
        format!("{:<bw$}", "[SKIP]".yellow().to_string(), bw = W_BADGE),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// `[....] name  detail` — component is starting (use boot_ok / boot_fail later).
pub fn boot_wait(name: &str, detail: &str) {
    println!(
        "{}  {:<width$}{}",
        format!("{:<bw$}", "[....]".cyan().to_string(), bw = W_BADGE),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// `== section ==` — group header above a run of boot lines.
pub fn boot_section(label: &str) {
    println!("\n{} {} {}",
             "==".dimmed(),
             label.bold(),
             "==".dimmed());
}

/// Final boot summary line.
pub fn boot_summary(ok: usize, total: usize, hint: &str) {
    let badge = if ok == total {
        "✓".green().bold()
    } else {
        "⚠".yellow().bold()
    };
    println!("\n{} {}/{} components up — {}",
             badge, ok, total, hint.dimmed());
}

/// Spinner for animated progress indication
pub struct Spinner {
    message: String,
    frames: Vec<char>,
    handle: Option<tokio::task::JoinHandle<()>>,
}

impl Spinner {
    /// Create a new spinner with a message
    pub fn new(message: String) -> Self {
        Self {
            message,
            frames: vec!['|', '/', '-', '\\'],
            handle: None,
        }
    }

    /// Start the spinner animation (spawns background task)
    pub fn start(&mut self) {
        let message = self.message.clone();
        let mut frame = 0;
        let frames = self.frames.clone();

        let handle = tokio::spawn(async move {
            loop {
                let spinner_char = frames[frame % frames.len()];
                let line = format!("  {} {}", spinner_char, message);
                print!("\r{}", line);
                let _ = io::stdout().flush();
                frame += 1;
                tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
            }
        });

        self.handle = Some(handle);
    }

    /// Stop the spinner and show success message
    pub fn finish_success(&mut self, final_message: &str) {
        if let Some(handle) = self.handle.take() {
            handle.abort();
        }
        // Clear the line first (using ANSI escape code)
        print!("\r\x1b[K");
        let line = format!("  {} {}", "✓".green(), final_message.green());
        println!("{}", line);
        let _ = io::stdout().flush();
    }

    /// Stop the spinner and show error message
    pub fn finish_error(&mut self, final_message: &str) {
        if let Some(handle) = self.handle.take() {
            handle.abort();
        }
        // Clear the line first (using ANSI escape code)
        print!("\r\x1b[K");
        let line = format!("  {} {}", "✗".red(), final_message.red());
        println!("{}", line);
        let _ = io::stdout().flush();
    }
}

impl Drop for Spinner {
    fn drop(&mut self) {
        if let Some(handle) = self.handle.take() {
            handle.abort();
        }
    }
}
