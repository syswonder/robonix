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
    println!("  {} {}", format!("→ {}", action).cyan(), target);
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
                io::stdout().flush().unwrap();
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
        print!("{}\n", line);
        io::stdout().flush().unwrap();
    }

    /// Stop the spinner and show error message
    pub fn finish_error(&mut self, final_message: &str) {
        if let Some(handle) = self.handle.take() {
            handle.abort();
        }
        // Clear the line first (using ANSI escape code)
        print!("\r\x1b[K");
        let line = format!("  {} {}", "✗".red(), final_message.red());
        print!("{}\n", line);
        io::stdout().flush().unwrap();
    }
}

impl Drop for Spinner {
    fn drop(&mut self) {
        if let Some(handle) = self.handle.take() {
            handle.abort();
        }
    }
}
