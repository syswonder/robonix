//! Console formatter — logcat-style single-line human-readable output.
//!
//! Format: `MM-DD HH:MM:SS.sss  L tag       msg`
//!
//! Level is a single character (D/I/W/E).  Tag is padded to 24 characters
//! left-aligned.  Timestamp is local time derived from the nanosecond
//! UNIX-epoch `ts` field.

use crate::{Level, LogRecord};
use std::io::{self, IsTerminal, Write};

/// Tag display width in console output (left-aligned, space-padded).
const TAG_WIDTH: usize = 24;

/// Format a [`LogRecord`] as a logcat-style console line.
///
/// Returns a String ending in `\n`.
pub fn format_console(record: &LogRecord) -> String {
    let (month, day, hour, min, sec, ms) = decompose_ts(record.ts);
    let level_code = record.level.code();
    // Truncate overly long tags; pad short ones.
    let tag_display = if record.tag.len() > TAG_WIDTH {
        format!("{}…", &record.tag[..TAG_WIDTH - 1])
    } else {
        format!("{: <width$}", record.tag, width = TAG_WIDTH)
    };
    format!(
        "{:02}-{:02} {:02}:{:02}:{:02}.{:03}  {} {} {}\n",
        month, day, hour, min, sec, ms, level_code, tag_display, record.msg
    )
}

/// Write a [`LogRecord`] to stderr in console format.
///
/// Each call does a single `write_all`; the caller is responsible for
/// not interleaving partial writes across threads (the global `log()`
/// function synchronises via the file-sink mutex).
pub fn write_console(record: &LogRecord) -> io::Result<()> {
    let line = format_console(record);
    let mut stderr = io::stderr().lock();
    let use_color = stderr.is_terminal() && std::env::var_os("NO_COLOR").is_none();
    let color = match record.level {
        Level::Warn => Some("\x1b[33m"),
        Level::Error => Some("\x1b[31m"),
        _ => None,
    };
    if use_color && let Some(color) = color {
        stderr.write_all(color.as_bytes())?;
        stderr.write_all(line.as_bytes())?;
        stderr.write_all(b"\x1b[0m")
    } else {
        stderr.write_all(line.as_bytes())
    }
}

/// Decompose a nanosecond UNIX timestamp into local-time calendar fields
/// and milliseconds.
fn decompose_ts(ts_ns: u64) -> (u32, u32, u32, u32, u32, u32) {
    // Convert ns → s + residual ns → ms
    let secs = (ts_ns / 1_000_000_000) as i64;
    let nanos_rem = (ts_ns % 1_000_000_000) as u32;
    let ms = nanos_rem / 1_000_000;

    // Use libc localtime_r for thread-safe local-time decomposition.
    // Fall back to UTC if we can't determine local time.
    #[cfg(unix)]
    {
        let local = localtime(secs);
        let (year, month, day, hour, min, sec) = local;
        // year is years since 1900; month is 0-11
        let _ = year;
        (month + 1, day, hour, min, sec, ms)
    }
    #[cfg(not(unix))]
    {
        // UTC fallback on non-Unix
        let total_secs = secs;
        let day_secs = total_secs % 86400;
        let hour = (day_secs / 3600) as u32;
        let min = ((day_secs % 3600) / 60) as u32;
        let sec = (day_secs % 60) as u32;
        let month = 1;
        let day = 1;
        (month, day, hour, min, sec, ms)
    }
}

#[cfg(unix)]
fn localtime(secs: i64) -> (i32, u32, u32, u32, u32, u32) {
    use std::mem::MaybeUninit;
    let ts = libc::time_t::try_from(secs).unwrap_or(0);
    let mut tm: MaybeUninit<libc::tm> = MaybeUninit::uninit();
    // SAFETY: localtime_r writes to the caller-provided tm; no aliasing.
    let tm_ptr = unsafe {
        libc::localtime_r(&ts, tm.as_mut_ptr());
        tm.assume_init()
    };
    (
        tm_ptr.tm_year,
        tm_ptr.tm_mon as u32,
        tm_ptr.tm_mday as u32,
        tm_ptr.tm_hour as u32,
        tm_ptr.tm_min as u32,
        tm_ptr.tm_sec as u32,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Level;

    #[test]
    fn format_includes_level_code_and_tag() {
        let rec = LogRecord {
            ts: 0,
            level: Level::Warn,
            tag: "atlas".into(),
            msg: "test message".into(),
        };
        let line = format_console(&rec);
        assert!(line.contains(" W "), "expected W level code, got: {line}");
        assert!(line.contains("atlas"), "expected tag, got: {line}");
        assert!(line.contains("test message"), "expected msg, got: {line}");
        assert!(line.ends_with('\n'), "should end with newline");
    }

    #[test]
    fn tag_padded_to_fixed_width() {
        let rec_short = LogRecord {
            ts: 0,
            level: Level::Info,
            tag: "a".into(),
            msg: "msg".into(),
        };
        let rec_long = LogRecord {
            ts: 0,
            level: Level::Info,
            tag: "this_tag_is_way_too_long_for_display".into(),
            msg: "msg".into(),
        };
        let short = format_console(&rec_short);
        let long = format_console(&rec_long);
        // Short tag padded to TAG_WIDTH
        // Short tag padded to TAG_WIDTH: expect N spaces after "a"
        let after_level: Vec<&str> = short.splitn(2, " I ").collect();
        assert_eq!(after_level.len(), 2);
        let rest = after_level[1]; // "a                        msg"
        assert!(
            rest.starts_with(&format!("{: <width$} ", "a", width = TAG_WIDTH)),
            "tag 'a' should be padded to {TAG_WIDTH} chars, got rest: {rest:?}"
        );
        // Long tag should be truncated with …
        assert!(
            long.contains('…'),
            "long tag should be truncated with '…', got: {long}"
        );
    }

    #[test]
    fn level_codes_in_output() {
        for level in [Level::Debug, Level::Info, Level::Warn, Level::Error] {
            let rec = LogRecord {
                ts: 0,
                level,
                tag: "test".into(),
                msg: String::new(),
            };
            let line = format_console(&rec);
            let expected_code = format!(" {} ", level.code());
            assert!(
                line.contains(&expected_code),
                "level {level:?}: expected '{expected_code}' in '{line}'"
            );
        }
    }

    #[test]
    fn write_console_does_not_panic() {
        let rec = LogRecord {
            ts: 1765432100123456789,
            level: Level::Info,
            tag: "test".into(),
            msg: "hello".into(),
        };
        // Just verify it doesn't panic; stderr is always writable.
        write_console(&rec).unwrap();
    }
}
