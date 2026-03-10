// SPDX-License-Identifier: MulanPSL-2.0
// Logging Module
//
// Logging initialization using env_logger with colored output

use ansi_term::{Colour, Style};
use env_logger::{Builder, Env, Target};
use std::io::Write;
use std::sync::{Arc, OnceLock};

use crate::web::{LogBuffer, LogEntry};

static LOG_BUFFER: OnceLock<Arc<LogBuffer>> = OnceLock::new();

pub fn init_logger() {
    init_logger_with_buffer(None);
}

pub fn init_logger_with_buffer(log_buffer: Option<Arc<LogBuffer>>) {
    if let Some(buffer) = log_buffer {
        let _ = LOG_BUFFER.set(buffer);
    }

    let env = Env::default()
        .filter_or("RUST_LOG", "robonix_server=info,rustdds=error")
        .write_style_or("RUST_LOG_STYLE", "auto");

    Builder::from_env(env)
        .target(Target::Stderr)
        .format(|buf, record| {
            static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
            let start = START.get_or_init(std::time::Instant::now);
            let elapsed = start.elapsed();
            let secs = elapsed.as_secs();
            let micros = elapsed.subsec_micros();

            let (level_char, level_color) = match record.level() {
                log::Level::Error => ('E', Colour::Red),
                log::Level::Warn => ('W', Colour::Yellow),
                log::Level::Info => ('I', Colour::Green),
                log::Level::Debug => ('D', Colour::Fixed(8)),
                log::Level::Trace => ('T', Colour::Purple),
            };

            let proc_name = std::env::current_exe()
                .ok()
                .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
                .unwrap_or_else(|| "robonix-server".to_string());

            let timestamp = format!("{}.{:06}", secs, micros);
            let proc_info = format!("{}[{}]", proc_name, std::process::id());

            let message = format!("{}", record.args());
            let painted_message = match record.level() {
                log::Level::Info => Style::new().paint(message.clone()),
                log::Level::Debug => Style::new().dimmed().paint(message.clone()),
                _ => level_color.paint(message.clone()),
            };

            if let Some(buffer) = LOG_BUFFER.get() {
                let level_str = match record.level() {
                    log::Level::Error => "ERROR",
                    log::Level::Warn => "WARN",
                    log::Level::Info => "INFO",
                    log::Level::Debug => "DEBUG",
                    log::Level::Trace => "TRACE",
                };
                buffer.add_log(LogEntry {
                    timestamp: timestamp.clone(),
                    level: level_str.to_string(),
                    message: message.clone(),
                });
            }

            write!(
                buf,
                "{} {}: {} {}\n",
                Style::new().dimmed().paint(&timestamp),
                Colour::Blue.paint(&proc_info),
                level_color.bold().paint(&level_char.to_string()),
                painted_message
            )
        })
        .init();
}
