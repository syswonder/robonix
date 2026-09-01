// SPDX-License-Identifier: MulanPSL-2.0
// Output Module
//
// Output formatting and display utilities for robonix-cli

use colored::*;
use std::io::{self, Write};
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

/// Monotonic origin for `[ssss.mmm]` boot-line timestamps. Initialised on
/// the first call to `boot_now()` (i.e. when the user actually starts a
/// boot run); subsequent calls measure from that origin so the log reads
/// like a kernel ring buffer / dmesg trace.
static BOOT_T0: OnceLock<Instant> = OnceLock::new();
static BOOT_VERBOSE: AtomicBool = AtomicBool::new(false);

pub fn set_boot_verbose(verbose: bool) {
    BOOT_VERBOSE.store(verbose, Ordering::Relaxed);
}

pub fn boot_verbose() -> bool {
    BOOT_VERBOSE.load(Ordering::Relaxed)
}

fn clear_progress_prefix() -> &'static str {
    if boot_verbose() { "" } else { "\r\x1b[K" }
}

/// Formatted `[ssss.mmm]` prefix relative to BOOT_T0. Width is fixed at
/// 12 chars (`[1234.567]`) — five-digit boots are unrealistic and we'd
/// rather wrap than re-jitter the column on long deploys.
fn boot_now() -> String {
    let t0 = BOOT_T0.get_or_init(Instant::now);
    let elapsed = t0.elapsed().as_secs_f64();
    format!("[{elapsed:>8.3}]")
}

/// Force-init BOOT_T0 to "now" so subsequent boot lines time from this
/// instant. Call once at the top of a deploy/boot run, before any
/// `boot_*` line, otherwise the first such call lazily wins.
pub fn boot_reset_clock() {
    let _ = BOOT_T0.set(Instant::now());
}

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

// ── Boot-log helpers (FreeBSD / dmesg style) ────────────────────────
//
// Each boot line carries a monotonic `[ssss.mmm]` timestamp prefix and
// a fixed-width status badge — same layout the BSD/Linux kernels print
// at boot, so a robonix bring-up reads like a real OS init log instead
// of a generic "deploying ..." trace.
//
// IMPORTANT: rust's `{:<N}` formatter pads to N BYTES, not visible
// chars. `colored` returns strings with ANSI escape sequences embedded
// (`"[ OK ]".green()` is ~13 bytes for 6 visible chars), so a naive
// `{:<7}` against a colored string produces no padding (already past
// 7 bytes). Every badge below is exactly 6 visible chars (`[ OK ]`,
// `[FAIL]`, `[SKIP]`, `[ →  ]`, `[ ⠙ ]`, …) so no badge-side
// alignment is needed; two spaces separate it from the name, and the
// padding lives between the (uncolored) name and the detail.

const W_NAME: usize = 18;

/// Print the ASCII banner at the very top of a boot run. Called once
/// from `rbnx boot` before any `[ ssss.mmm ]` line. Layout mimics the
/// kernel's "Linux version ..." splash: name + version + git sha,
/// builder, build time, compiler, target. All facts come from build.rs
/// via compile-time env vars; missing values gracefully render as
/// "unknown" so a tarball / sandboxed build still prints a banner.
pub fn boot_banner() {
    let version = env!("CARGO_PKG_VERSION");
    let sha = option_env!("ROBONIX_GIT_SHA").unwrap_or("dev");
    let builder = option_env!("ROBONIX_BUILDER").unwrap_or("unknown");
    let build_time = option_env!("ROBONIX_BUILD_TIME").unwrap_or("unknown");
    let rustc = option_env!("ROBONIX_RUSTC").unwrap_or("rustc unknown");
    let target = option_env!("ROBONIX_TARGET").unwrap_or("unknown");

    let lines = [
        "    ____        __                 _      ",
        "   / __ \\____  / /_  ____  ____  (_)  __ ",
        "  / /_/ / __ \\/ __ \\/ __ \\/ __ \\/ / |/_/",
        " / _, _/ /_/ / /_/ / /_/ / / / / />  <   ",
        "/_/ |_|\\____/_.___/\\____/_/ /_/_/_/|_|   ",
    ];
    println!();
    for line in &lines {
        println!("{}", line.cyan().bold());
    }
    println!("{}", "        Embodied AI Operating System".dimmed(),);
    println!();
    // Body block. Bullet-aligned, dimmed value column — readable but
    // visually distinct from the per-component boot lines below.
    let label_w = 9;
    let row = |k: &str, v: &str| {
        println!(
            "  {:label_w$} {}",
            format!("{k}:").bold(),
            v.dimmed(),
            label_w = label_w
        );
    };
    row("version", &format!("v{version} ({sha})"));
    row("built", &format!("{build_time} on {builder}"));
    row("compiler", rustc);
    row("target", target);
    println!();
}

/// Single-line "we are now booting X" announcement, the dmesg-style
/// banner that follows `boot_banner` and precedes the per-component
/// `[ ssss.mmm ]` log. Resets the boot clock so timestamps below count
/// from this point.
pub fn boot_start(deploy_name: &str, manifest_path: &str) {
    boot_reset_clock();
    println!(
        "{} booting {}",
        boot_now().cyan(),
        deploy_name.bold().green(),
    );
    println!("{} manifest {}", boot_now().cyan(), manifest_path.dimmed(),);
}

/// `[  ssss.mmm] [ OK ] name              detail` — component came up.
/// Leading `\r\x1b[K` clears any in-place spinner line so the final
/// result lands cleanly without a trailing fragment of "registering…".
pub fn boot_ok(name: &str, detail: &str) {
    println!(
        "{}{} {}  {:<width$}  {}",
        clear_progress_prefix(),
        boot_now().cyan(),
        "[ OK ]".green().bold(),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// `[  ssss.mmm] [FAIL] name              detail` — failed to come up.
pub fn boot_fail(name: &str, detail: &str) {
    eprintln!(
        "{}{} {}  {:<width$}  {}",
        clear_progress_prefix(),
        boot_now().cyan(),
        "[FAIL]".red().bold(),
        name,
        detail.red(),
        width = W_NAME,
    );
}

/// `[  ssss.mmm] [SKIP] name              detail` — declared in manifest
/// but skipped (not installed / disabled / out-of-scope on this host).
pub fn boot_skip(name: &str, detail: &str) {
    println!(
        "{}{} {}  {:<width$}  {}",
        clear_progress_prefix(),
        boot_now().cyan(),
        "[SKIP]".yellow(),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// `[  ssss.mmm] [ →  ] name              detail` — informational note
/// (cache hit, fetched a remote package, …). Neither up nor skipped.
pub fn boot_note(name: &str, detail: &str) {
    println!(
        "{}{} {}  {:<width$}  {}",
        clear_progress_prefix(),
        boot_now().cyan(),
        "[ →  ]".cyan(),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// `[  ssss.mmm] [ ↑ ] name              detail` — an update is available.
/// Leading `\r\x1b[K` clears the in-place `boot_progress` spinner so the
/// verdict lands cleanly. Yellow `↑` distinguishes "needs update" from the
/// green `[ OK ]` "up to date" verdict.
pub fn boot_update_avail(name: &str, detail: &str) {
    println!(
        "{}{} {}  {:<width$}  {}",
        clear_progress_prefix(),
        boot_now().cyan(),
        "[ ↑  ]".yellow().bold(),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// `[  ssss.mmm] [....] name              detail` — component is starting.
pub fn boot_wait(name: &str, detail: &str) {
    println!(
        "{}{} {}  {:<width$}  {}",
        clear_progress_prefix(),
        boot_now().cyan(),
        "[....]".cyan(),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
}

/// Group header above a run of boot lines. FreeBSD-rc-style ":: stage ::".
pub fn boot_section(label: &str) {
    println!(
        "{}\n{} {} {} {}",
        clear_progress_prefix(),
        boot_now().cyan(),
        "::".dimmed(),
        label.bold().yellow(),
        "::".dimmed(),
    );
}

/// In-place spinner frame. Renders to stdout with `\r` (no newline) and
/// flushes; caller overwrites with boot_ok / boot_fail to finalize.
/// Frames cycle through Braille dots — the systemd spinner most users
/// recognise from Linux init logs.
pub fn boot_progress(name: &str, detail: &str, frame: usize) {
    if boot_verbose() {
        return;
    }
    const GLYPHS: &[char] = &['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏'];
    let g = GLYPHS[frame % GLYPHS.len()];
    print!(
        "\r\x1b[K{} {}  {:<width$}  {}",
        boot_now().cyan(),
        format!("[ {g} ]").cyan(),
        name,
        detail.dimmed(),
        width = W_NAME,
    );
    let _ = io::stdout().flush();
}

/// Final boot summary line — total elapsed + component tally.
pub fn boot_summary(ok: usize, total: usize, hint: &str) {
    let elapsed = BOOT_T0
        .get()
        .map(|t| t.elapsed().as_secs_f64())
        .unwrap_or(0.0);
    let badge = if ok == total {
        "OK".green().bold()
    } else {
        "DEGRADED".yellow().bold()
    };
    println!();
    println!(
        "[ {badge} ] robonix up — {ok}/{total} components in {elapsed:.3}s   {}",
        hint.dimmed()
    );
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
