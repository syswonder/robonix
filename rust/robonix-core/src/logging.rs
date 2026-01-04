// SPDX-License-Identifier: MulanPSL-2.0
// Logging Module
//
// Logging initialization using env_logger with colored output

use ansi_term::{Colour, Style};
use env_logger::{Builder, Env, Target};
use std::io::Write;

pub fn init_logger() {
    let env = Env::default()
        .filter_or("RUST_LOG", "robonix_core=info,rustdds=error")
        .write_style_or("RUST_LOG_STYLE", "auto");

    Builder::from_env(env)
        .target(Target::Stderr)
        .format(|buf, record| {
            // Get timestamp in Linux kernel style [seconds.microseconds]
            static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
            let start = START.get_or_init(std::time::Instant::now);
            let elapsed = start.elapsed();
            let secs = elapsed.as_secs();
            let micros = elapsed.subsec_micros();

            // Get log level with color
            let (level_char, level_color) = match record.level() {
                log::Level::Error => ('E', Colour::Red),
                log::Level::Warn => ('W', Colour::Yellow),
                log::Level::Info => ('I', Colour::Green),
                log::Level::Debug => ('D', Colour::Fixed(8)), // Gray (less prominent)
                log::Level::Trace => ('T', Colour::Purple),
            };

            // Get process name
            let proc_name = std::env::current_exe()
                .ok()
                .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
                .unwrap_or_else(|| "robonix-core".to_string());

            // Write Linux-style log entry with colors: timestamp procname[pid]: LEVEL message
            let timestamp = format!("{}.{:06}", secs, micros);
            let proc_info = format!("{}[{}]", proc_name, std::process::id());

            // Info messages are white (default), debug messages are dimmed gray, other levels use their level color
            let message = format!("{}", record.args());
            let painted_message = match record.level() {
                log::Level::Info => Style::new().paint(message), // White (default)
                log::Level::Debug => Style::new().dimmed().paint(message), // Dimmed gray (less prominent)
                _ => level_color.paint(message),
            };

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
