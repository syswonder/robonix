// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx logs` — read Scribe JSON-lines log files with tag / level
// filtering and optional follow mode.

use anyhow::Result;
use robonix_scribe::LogRecord;
use std::path::PathBuf;

/// Minimum level string → numeric floor for filtering.
fn level_floor(s: &str) -> u8 {
    match s.to_lowercase().as_str() {
        "debug" => 0,
        "info" => 1,
        "warn" => 2,
        "error" => 3,
        _ => 0, // unknown → show everything
    }
}

fn level_value(level: &robonix_scribe::Level) -> u8 {
    match level {
        robonix_scribe::Level::Debug => 0,
        robonix_scribe::Level::Info => 1,
        robonix_scribe::Level::Warn => 2,
        robonix_scribe::Level::Error => 3,
    }
}

/// Render one record as a logcat-style line (reuse Scribe's own formatter).
fn render_console(rec: &LogRecord) -> String {
    robonix_scribe::format::format_console(rec)
}

/// Read all `*.log` files in `dir`, parse every line as a LogRecord,
/// apply tag / level filters, sort by timestamp, and print.
fn read_all(dir: &PathBuf, tags: &[String], min_level: u8, raw_json: bool) -> Result<()> {
    let mut records: Vec<LogRecord> = Vec::new();

    let entries = std::fs::read_dir(dir)?;
    for entry in entries.flatten() {
        let p = entry.path();
        if p.extension().and_then(|s| s.to_str()) != Some("log") {
            continue;
        }
        let content = std::fs::read_to_string(&p)?;
        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            if let Ok(rec) = serde_json::from_str::<LogRecord>(line) {
                records.push(rec);
            }
        }
    }

    // Sort by timestamp.
    records.sort_by_key(|r| r.ts);

    // Filter and print.
    for rec in &records {
        // Tag filter (OR).
        if !tags.is_empty() && !tags.iter().any(|t| t == &rec.tag) {
            continue;
        }
        // Level filter.
        if level_value(&rec.level) < min_level {
            continue;
        }
        if raw_json {
            println!("{}", serde_json::to_string(rec)?);
        } else {
            print!("{}", render_console(rec));
        }
    }
    Ok(())
}

/// Follow mode: open all `*.log` files, seek to end, and tail new lines.
fn follow(dir: &PathBuf, tags: &[String], min_level: u8, raw_json: bool) -> Result<()> {
    use std::io::{BufRead, Seek, SeekFrom};

    // Collect initial file positions.
    let mut handles: Vec<(PathBuf, std::fs::File, u64)> = Vec::new();
    let entries = std::fs::read_dir(dir)?;
    for entry in entries.flatten() {
        let p = entry.path();
        if p.extension().and_then(|s| s.to_str()) != Some("log") {
            continue;
        }
        let mut f = std::fs::File::open(&p)?;
        let pos = f.seek(SeekFrom::End(0))?;
        handles.push((p, f, pos));
    }

    // Poll loop.
    loop {
        for (path, f, pos) in &mut handles {
            let current_len = f.metadata()?.len();
            if current_len > *pos {
                f.seek(SeekFrom::Start(*pos))?;
                let reader = std::io::BufReader::new(&*f);
                for line in reader.lines() {
                    let line = line?;
                    let line = line.trim();
                    if line.is_empty() {
                        continue;
                    }
                    if let Ok(rec) = serde_json::from_str::<LogRecord>(line) {
                        if !tags.is_empty() && !tags.iter().any(|t| t == &rec.tag) {
                            continue;
                        }
                        if level_value(&rec.level) < min_level {
                            continue;
                        }
                        if raw_json {
                            println!("{}", serde_json::to_string(&rec)?);
                        } else {
                            print!("{}", render_console(&rec));
                        }
                    }
                }
                // Update position (re-read metadata to avoid TOCTOU).
                *pos = std::fs::metadata(path)?.len();
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(250));
    }
}

pub async fn execute(
    log_dir: Option<PathBuf>,
    tags: Vec<String>,
    level: Option<String>,
    follow_mode: bool,
    raw_json: bool,
) -> Result<()> {
    let dir = log_dir.unwrap_or_else(|| {
        std::env::var("SCRIBE_LOG_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("./rbnx-boot/logs"))
    });

    if !dir.is_dir() {
        eprintln!(
            "rbnx logs: {} does not exist or is not a directory.",
            dir.display()
        );
        eprintln!("Set SCRIBE_LOG_DIR or run from a deploy directory.");
        std::process::exit(1);
    }

    let min_level = level.as_deref().map(level_floor).unwrap_or(0);

    if follow_mode {
        follow(&dir, &tags, min_level, raw_json)
    } else {
        read_all(&dir, &tags, min_level, raw_json)
    }
}
