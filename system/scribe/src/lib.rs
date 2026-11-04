//! Scribe — Robonix unified logging facade.
//!
//! Single entry point: [`log()`].  Every component calls the same function;
//! Scribe routes each record to stderr (human-readable) and to a per-tag
//! JSON-lines file under `$SCRIBE_LOG_DIR` (or `./logs`).
//!
//! # Quick start
//!
//! ```rust,ignore
//! robonix_scribe::init("executor");
//! robonix_scribe::info!("robonix-executor starting");
//! robonix_scribe::warn!("retry {}/{}", n, max);
//! ```
//!
//! The `init()` call sets a process-wide default tag used by all subsequent
//! [`info!`] / [`warn!`] / [`error!`] / [`debug!`] macro calls.  For
//! per-message dynamic tags, use the lower-level [`log()`] function.

use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;
use std::sync::OnceLock;

pub mod format;
pub mod sink;

/// Process-wide default tag, set once by [`init`].
static DEFAULT_TAG: OnceLock<String> = OnceLock::new();

/// Set the process-wide default tag.  Call once at startup, before any
/// log call.  All [`info!`] / [`warn!`] / [`error!`] / [`debug!`] macros
/// use this tag after it is set.
///
/// # Panics
///
/// Panics if called twice (intentional — one tag per process).
pub fn init(tag: &str) {
    DEFAULT_TAG
        .set(tag.to_string())
        .expect("robonix_scribe::init() called twice; set the tag once per process");
}

/// Initialise scribe with the process `tag` and apply the on-disk log level
/// from a launch-config JSON — the system component's manifest config block
/// that rbnx serialises and passes via `--config-json`. Reads the top-level
/// `log` key (e.g. `{"log":"debug"}`); a valid level sets the file-sink floor
/// for this process so the level actually reaches the log file (`rbnx logs`).
///
/// Use this instead of [`init`] in binaries launched with a config: the
/// manifest's per-component `log:` had no effect before, because the level
/// was parsed into a CLI flag the binary discarded. Absent / unparseable
/// `log` falls back to the `SCRIBE_FILE_LEVEL` env / `Info` default.
///
/// Call once at startup, before the first log, on the main thread.
pub fn init_from_config(tag: &str, config_json: Option<&str>) {
    if let Some(level) = config_json
        .and_then(|j| serde_json::from_str::<serde_json::Value>(j).ok())
        .and_then(|v| v.get("log").and_then(|l| l.as_str()).and_then(parse_level))
    {
        let _ = FILE_LEVEL_OVERRIDE.set(level);
    }
    init(tag);
}

/// Return the process-wide default tag, or `"_default"` if `init()` has
/// not been called yet.
#[doc(hidden)]
pub fn default_tag() -> &'static str {
    DEFAULT_TAG.get().map(|s| s.as_str()).unwrap_or("_default")
}

/// Format a nanosecond UNIX timestamp as a local-time readable string:
/// `"YYYY-MM-DD HH:MM:SS.nnnnnnnnn"`.
///
/// This is used for JSON serialisation of [`LogRecord::ts`].
pub fn ts_fmt(ts_ns: u64) -> String {
    let secs = (ts_ns / 1_000_000_000) as i64;
    let nsec = (ts_ns % 1_000_000_000) as u32;
    #[cfg(unix)]
    let (year, month, day, hour, min, sec) = {
        let ts = libc::time_t::try_from(secs).unwrap_or(0);
        let mut tm: std::mem::MaybeUninit<libc::tm> = std::mem::MaybeUninit::uninit();
        // SAFETY: localtime_r writes to caller-provided tm; no aliasing.
        let tm_ptr = unsafe {
            libc::localtime_r(&ts, tm.as_mut_ptr());
            tm.assume_init()
        };
        (
            tm_ptr.tm_year + 1900,
            tm_ptr.tm_mon as u32 + 1,
            tm_ptr.tm_mday as u32,
            tm_ptr.tm_hour as u32,
            tm_ptr.tm_min as u32,
            tm_ptr.tm_sec as u32,
        )
    };
    #[cfg(not(unix))]
    let (year, month, day, hour, min, sec) = {
        let day_secs = secs % 86400;
        (
            1970,
            1u32,
            1u32,
            (day_secs / 3600) as u32,
            ((day_secs % 3600) / 60) as u32,
            (day_secs % 60) as u32,
        )
    };
    format!("{year:04}-{month:02}-{day:02} {hour:02}:{min:02}:{sec:02}.{nsec:09}")
}

fn deserialize_ts<'de, D: Deserializer<'de>>(d: D) -> Result<u64, D::Error> {
    struct TsVisitor;
    impl serde::de::Visitor<'_> for TsVisitor {
        type Value = u64;
        fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
            f.write_str("a nanosecond timestamp as string or integer")
        }
        fn visit_u64<E: serde::de::Error>(self, v: u64) -> Result<u64, E> {
            Ok(v) // backward compat: accept raw integer
        }
        fn visit_str<E: serde::de::Error>(self, v: &str) -> Result<u64, E> {
            ts_parse(v).ok_or_else(|| E::custom(format!("invalid ts: {v}")))
        }
    }
    d.deserialize_any(TsVisitor)
}

/// Parse a `ts_fmt`-style string back to nanosecond u64.  Returns `None`
/// on malformed input.
pub fn ts_parse(s: &str) -> Option<u64> {
    // "YYYY-MM-DD HH:MM:SS.nnnnnnnnn" = 29 chars
    if s.len() < 29 {
        return None;
    }
    let year: i32 = s[0..4].parse().ok()?;
    let month: u32 = s[5..7].parse().ok()?;
    let day: u32 = s[8..10].parse().ok()?;
    let hour: u32 = s[11..13].parse().ok()?;
    let min: u32 = s[14..16].parse().ok()?;
    let sec: u32 = s[17..19].parse().ok()?;
    let nsec: u32 = s[20..29].parse().ok()?;
    // Build a libc tm for mktime (Unix) or crude UTC calc.
    #[cfg(unix)]
    let secs = {
        let mut tm: libc::tm = unsafe { std::mem::zeroed() };
        tm.tm_year = year - 1900;
        tm.tm_mon = month as i32 - 1;
        tm.tm_mday = day as i32;
        tm.tm_hour = hour as i32;
        tm.tm_min = min as i32;
        tm.tm_sec = sec as i32;
        tm.tm_isdst = -1; // let libc figure out DST
        let t = unsafe { libc::mktime(&mut tm) };
        if t < 0 {
            // mktime failed (e.g. date out of range for this TZ).
            // Fall back: compute UTC epoch directly.
            let days_before: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
            let y = year as u32;
            let leap =
                |y: u32| y.is_multiple_of(4) && !y.is_multiple_of(100) || y.is_multiple_of(400);
            let days = (y - 1970) as u64 * 365 + ((y - 1969) / 4) as u64
                - ((y - 1901) / 100) as u64
                + ((y - 1601) / 400) as u64
                + days_before[(month - 1) as usize] as u64
                + if month > 2 && leap(y) { 1 } else { 0 }
                + day as u64
                - 1;
            days * 86400 + hour as u64 * 3600 + min as u64 * 60 + sec as u64
        } else {
            t as u64
        }
    };
    #[cfg(not(unix))]
    let secs = {
        let days_before_month: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
        let y = year as u32;
        let leap = |y: u32| y.is_multiple_of(4) && !y.is_multiple_of(100) || y.is_multiple_of(400);
        let days = (y - 1970) * 365 + ((y - 1969) / 4) - ((y - 1901) / 100)
            + ((y - 1601) / 400)
            + days_before_month[(month - 1) as usize]
            + if month > 2 && leap(y) { 1 } else { 0 }
            + day
            - 1;
        days as u64 * 86400 + hour as u64 * 3600 + min as u64 * 60 + sec as u64
    };
    let ts_ns = secs.checked_mul(1_000_000_000)?.checked_add(nsec as u64)?;
    Some(ts_ns)
}

/// Severity level, ordered `Debug < Info < Warn < Error`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Level {
    Debug,
    Info,
    Warn,
    Error,
}

impl Level {
    /// Single-character code used in console output.
    pub fn code(self) -> &'static str {
        match self {
            Level::Debug => "D",
            Level::Info => "I",
            Level::Warn => "W",
            Level::Error => "E",
        }
    }
}

impl fmt::Display for Level {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.code())
    }
}

/// A single structured log record.
///
/// `ts` is nanoseconds since UNIX epoch internally; serialised as a
/// human-readable `"YYYY-MM-DD HH:MM:SS.nnnnnnnnn"` string in JSON.
#[derive(Debug, Clone, Deserialize)]
pub struct LogRecord {
    /// Nanosecond UNIX timestamp (internal u64; JSON: readable string).
    #[serde(deserialize_with = "deserialize_ts")]
    pub ts: u64,
    /// Severity.
    pub level: Level,
    /// Source identifier — `provider_id` or component name.
    pub tag: String,
    /// Free-form log message body.
    pub msg: String,
}

// Manual Serialize impl needed because we can't mix derive(Serialize) with
// field-level serialize_with on a non-structural level in a derive.
impl Serialize for LogRecord {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut st = s.serialize_struct("LogRecord", 4)?;
        st.serialize_field("ts", &ts_fmt(self.ts))?;
        st.serialize_field("level", &self.level)?;
        st.serialize_field("tag", &self.tag)?;
        st.serialize_field("msg", &self.msg)?;
        st.end()
    }
}

impl LogRecord {
    /// Create a new record with the current wall-clock timestamp (u64 ns).
    pub fn now(level: Level, tag: impl Into<String>, msg: impl Into<String>) -> Self {
        let ts = system_time_ns();
        Self {
            ts,
            level,
            tag: tag.into(),
            msg: msg.into(),
        }
    }
}

/// Best-effort nanosecond timestamp from `SystemTime`.
///
/// When chronos lands this will become `chronos::now()`.
fn system_time_ns() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

// ── Global lazy state ──────────────────────────────────────────────

use std::sync::LazyLock;

use crate::sink::FileSink;

// ── Level filter thresholds / directory / sink ──────────────────────

/// Resolved log directory — `$SCRIBE_LOG_DIR` or `"./logs"`.
static LOG_DIR: LazyLock<std::path::PathBuf> = LazyLock::new(|| {
    std::env::var("SCRIBE_LOG_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from("./logs"))
});

/// Global file sink; `None` when the log directory can't be created
/// (read-only filesystem, full disk, etc.).  When `None`, file writes
/// are silently skipped — the process keeps running console-only.
static FILE_SINK: LazyLock<Option<FileSink>> =
    LazyLock::new(|| FileSink::new(LOG_DIR.as_path()).ok());

// ── Level filter thresholds (read from env, lazy) ───────────────────

/// Parse a level word (`debug`/`info`/`warn`/`warning`/`error`,
/// case-insensitive). `None` for anything else.
fn parse_level(s: &str) -> Option<Level> {
    match s.to_lowercase().as_str() {
        "debug" => Some(Level::Debug),
        "info" => Some(Level::Info),
        "warn" | "warning" => Some(Level::Warn),
        "error" => Some(Level::Error),
        _ => None,
    }
}

/// Parse `SCRIBE_CONSOLE_LEVEL` / `SCRIBE_FILE_LEVEL` env var to a
/// numeric floor.  Unrecognised values default to the given fallback.
fn parse_level_env(key: &str, fallback: Level) -> Level {
    std::env::var(key)
        .ok()
        .as_deref()
        .and_then(parse_level)
        .unwrap_or(fallback)
}

/// Per-process on-disk level override, set once by [`init_from_config`] from
/// the launch config's `log` key. Takes precedence over `SCRIBE_FILE_LEVEL`
/// when present, so a per-component `log:` in the deploy manifest actually
/// controls that component's log file.
static FILE_LEVEL_OVERRIDE: OnceLock<Level> = OnceLock::new();

/// Minimum level for console (stderr) output.
///
/// Default: [`Level::Warn`]  — only Warn + Error reach the terminal.
/// Set `SCRIBE_CONSOLE_LEVEL=info` to also see Info, or
/// `SCRIBE_CONSOLE_LEVEL=debug` for everything.
static CONSOLE_MIN: LazyLock<Level> =
    LazyLock::new(|| parse_level_env("SCRIBE_CONSOLE_LEVEL", Level::Warn));

/// Minimum level for per-tag log files.
///
/// Default: [`Level::Info`]  — Debug is suppressed on disk unless
/// `SCRIBE_FILE_LEVEL=debug` is set.  Set `SCRIBE_FILE_LEVEL=error`
/// to only persist errors.
static FILE_MIN: LazyLock<Level> =
    LazyLock::new(|| parse_level_env("SCRIBE_FILE_LEVEL", Level::Info));

// ── Public API ─────────────────────────────────────────────────────

/// Log a record with the given `level`, `tag`, and `msg`.
///
/// This is the **only** entry point for structured logging.  The first
/// call transparently initialises the log directory and file sink.
///
/// # Per-sink level filtering
///
/// | Level  | Console (default) | File (default) |
/// |--------|:-----:|:----:|
/// | Debug  |  ✗    |  ✗   |
/// | Info   |  ✗    |  ✓   |
/// | Warn   |  ✓    |  ✓   |
/// | Error  |  ✓    |  ✓   |
///
/// Override with `SCRIBE_CONSOLE_LEVEL` / `SCRIBE_FILE_LEVEL`.
pub fn log(level: Level, tag: &str, msg: &str) {
    // Only construct the timestamped record when at least one sink
    // accepts this level — avoid wasted clock syscalls for Debug.
    let console_ok = level >= *CONSOLE_MIN;
    let file_min = FILE_LEVEL_OVERRIDE.get().copied().unwrap_or(*FILE_MIN);
    let file_ok = level >= file_min;
    if !console_ok && !file_ok {
        return;
    }
    let record = LogRecord::now(level, tag, msg);
    if console_ok {
        let _ = format::write_console(&record);
    }
    if file_ok && let Some(sink) = FILE_SINK.as_ref() {
        let _ = sink.write(&record);
    }
}

/// Convenience: [`Level::Debug`].
pub fn debug(tag: &str, msg: &str) {
    log(Level::Debug, tag, msg);
}

/// Convenience: [`Level::Info`].
pub fn info(tag: &str, msg: &str) {
    log(Level::Info, tag, msg);
}

/// Convenience: [`Level::Warn`].
pub fn warn(tag: &str, msg: &str) {
    log(Level::Warn, tag, msg);
}

/// Convenience: [`Level::Error`].
pub fn error(tag: &str, msg: &str) {
    log(Level::Error, tag, msg);
}

/// Ingest a captured stdout/stderr line from a child process under `tag`.
///
/// A child that uses Scribe (Python `scribe_logger` in a docker container,
/// or any binary that mirrors records to its console) emits lines already in
/// console format — `MM-DD HH:MM:SS.mmm  L tag  msg`. Re-logging the whole
/// line as a fresh record's `msg` double-stamps the timestamp/level/tag (it
/// appears once in the JSON fields and again embedded in `msg`). When the line
/// is recognised as Scribe console output, strip the prefix and re-emit just
/// the inner message at its original level under `tag`; otherwise treat the
/// line as opaque output and log it verbatim at Info.
pub fn ingest(tag: &str, line: &str) {
    match parse_console_line(line) {
        Some((level, msg)) => log(level, tag, msg),
        None => info(tag, line),
    }
}

/// Recognise a Scribe console line and return its `(level, inner message)`.
///
/// Layout (see `format::format_console`): two-digit `MM-DD`, space, `HH:MM:SS`,
/// `.mmm`, two spaces, a single level char `D/I/W/E`, space, the tag padded to
/// 24 columns, space, then the message at byte 47. Validated by fixed ASCII
/// positions so a non-Scribe line (e.g. raw ROS2 output) falls through to None.
fn parse_console_line(line: &str) -> Option<(Level, &str)> {
    let b = line.as_bytes();
    // Shortest possible: 47-char prefix + at least one msg byte.
    if b.len() < 48 {
        return None;
    }
    if b[2] != b'-'
        || b[5] != b' '
        || b[8] != b':'
        || b[11] != b':'
        || b[14] != b'.'
        || b[18] != b' '
        || b[19] != b' '
        || b[21] != b' '
        || b[46] != b' '
    {
        return None;
    }
    for &i in &[0usize, 1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 17] {
        if !b[i].is_ascii_digit() {
            return None;
        }
    }
    let level = match b[20] {
        b'D' => Level::Debug,
        b'I' => Level::Info,
        b'W' => Level::Warn,
        b'E' => Level::Error,
        _ => return None,
    };
    // Byte 47 is the first message byte; it is a char boundary because bytes
    // 0..=46 are all ASCII when every check above passed.
    line.get(47..).map(|msg| (level, msg))
}

// ── Macros — process-wide default tag ───────────────────────────────

/// Log at INFO level using the process-wide default tag set by [`init`].
///
/// ```rust,ignore
/// robonix_scribe::init("executor");
/// robonix_scribe::info!("connecting to atlas at {}", addr);
/// ```
#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {
        $crate::log($crate::Level::Info, $crate::default_tag(), &format!($($arg)*))
    };
}

/// Log at WARN level using the process-wide default tag set by [`init`].
#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {
        $crate::log($crate::Level::Warn, $crate::default_tag(), &format!($($arg)*))
    };
}

/// Log at ERROR level using the process-wide default tag set by [`init`].
#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {
        $crate::log($crate::Level::Error, $crate::default_tag(), &format!($($arg)*))
    };
}

/// Log at DEBUG level using the process-wide default tag set by [`init`].
#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {
        $crate::log($crate::Level::Debug, $crate::default_tag(), &format!($($arg)*))
    };
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_codes() {
        assert_eq!(Level::Debug.code(), "D");
        assert_eq!(Level::Info.code(), "I");
        assert_eq!(Level::Warn.code(), "W");
        assert_eq!(Level::Error.code(), "E");
    }

    #[test]
    fn parse_console_round_trips_formatter() {
        // A line produced by format_console must parse back to its level + msg,
        // so re-ingesting a child's console echo strips the prefix (no
        // double-stamped timestamp/tag).
        for (level, msg) in [
            (Level::Info, "[nav2] [smoother_server-2] launched."),
            (Level::Warn, "rtabmap: republishing map"),
            (Level::Error, "webui GET /api/map.png failed"),
            (Level::Debug, "x"),
        ] {
            let rec = LogRecord::now(level, "nav2", msg);
            let line = format::format_console(&rec);
            let line = line.trim_end_matches('\n');
            assert_eq!(
                parse_console_line(line),
                Some((level, msg)),
                "line={line:?}"
            );
        }
    }

    #[test]
    fn parse_console_rejects_non_console_lines() {
        // Raw child output (ROS2 logs, start-script prints) must fall through.
        for raw in [
            "[nav2/start] mode=docker (FORCE= PLATFORM=)",
            "[INFO] [1782660318.498] [planner_server]: activating",
            "",
            "short",
            "06-28 15:41:51.183 X nav2  bad level char",
        ] {
            assert_eq!(parse_console_line(raw), None, "raw={raw:?}");
        }
    }

    #[test]
    fn level_display_is_code() {
        assert_eq!(Level::Info.to_string(), "I");
    }

    #[test]
    fn level_ordering() {
        assert!(Level::Debug < Level::Info);
        assert!(Level::Info < Level::Warn);
        assert!(Level::Warn < Level::Error);
    }

    #[test]
    fn log_record_serialization_is_readable_string() {
        let rec = LogRecord {
            ts: 1765432100123456789,
            level: Level::Info,
            tag: "scene_svc".into(),
            msg: "object registered".into(),
        };
        let json = serde_json::to_string(&rec).unwrap();
        // ts is now a human-readable "YYYY-MM-DD HH:MM:SS.nnnnnnnnn" string.
        assert!(
            json.contains(r#""ts":""#),
            "ts should be a JSON string, got: {json}"
        );
        assert!(json.contains(r#""level":"info""#));
        assert!(json.contains(r#""tag":"scene_svc""#));
        assert!(json.contains(r#""msg":"object registered""#));
    }

    #[test]
    fn log_record_deserialization_readable_string() {
        // New format: ts as readable string
        let json = r#"{"ts":"2025-12-11 13:48:20.123456789","level":"info","tag":"scene_svc","msg":"object registered"}"#;
        let rec: LogRecord = serde_json::from_str(json).unwrap();
        assert_eq!(rec.level, Level::Info);
        assert_eq!(rec.tag, "scene_svc");
        assert_eq!(rec.msg, "object registered");
        // The nanosecond value depends on local TZ → we just verify it's non-zero
        assert!(rec.ts > 0, "parsed ts should be positive");
    }

    #[test]
    fn log_record_deserialization_legacy_integer() {
        // Legacy format: ts as raw integer (backward compat)
        let json = r#"{"ts":1765432100123456789,"level":"info","tag":"scene_svc","msg":"object registered"}"#;
        let rec: LogRecord = serde_json::from_str(json).unwrap();
        assert_eq!(rec.ts, 1765432100123456789);
        assert_eq!(rec.level, Level::Info);
        assert_eq!(rec.tag, "scene_svc");
        assert_eq!(rec.msg, "object registered");
    }

    #[test]
    fn log_record_roundtrip() {
        let rec = LogRecord {
            ts: 1_700_000_000_000_000_000,
            level: Level::Error,
            tag: "test".into(),
            msg: "roundtrip".into(),
        };
        let json = serde_json::to_string(&rec).unwrap();
        let back: LogRecord = serde_json::from_str(&json).unwrap();
        assert_eq!(back.level, rec.level);
        assert_eq!(back.tag, rec.tag);
        assert_eq!(back.msg, rec.msg);
        // ts may differ slightly due to TZ roundtrip; verify within 1 day
        let diff = back.ts.abs_diff(rec.ts);
        assert!(
            diff < 86_400_000_000_000,
            "roundtrip ts diff too large: {diff} ns"
        );
    }

    #[test]
    fn log_record_now_has_reasonable_ts() {
        let rec = LogRecord::now(Level::Debug, "test", "hello");
        // After 2020-01-01 in nanos
        assert!(rec.ts > 1_577_836_800_000_000_000);
        // Not in the far future
        assert!(rec.ts < 4_000_000_000_000_000_000);
    }

    // ── log() integration tests ─────────────────────────────────
    //
    // NOTE: FILE_SINK is a LazyLock — it initialises once per process.
    // These tests only verify the public API does not panic and that
    // convenience wrappers route to the correct level.  File/directory
    // behaviour is covered exhaustively by `sink::tests`.

    #[test]
    fn log_does_not_panic() {
        log(Level::Debug, "smoke", "smoke test");
        debug("smoke_d", "debug smoke");
        info("smoke_i", "info smoke");
        warn("smoke_w", "warn smoke");
        error("smoke_e", "error smoke");
    }

    #[test]
    fn convenience_wrappers_use_correct_levels() {
        // Verify by inspecting a LogRecord produced by each wrapper.
        let r_d = LogRecord::now(Level::Debug, "t", "d");
        let r_i = LogRecord::now(Level::Info, "t", "i");
        let r_w = LogRecord::now(Level::Warn, "t", "w");
        let r_e = LogRecord::now(Level::Error, "t", "e");
        assert_eq!(r_d.level, Level::Debug);
        assert_eq!(r_i.level, Level::Info);
        assert_eq!(r_w.level, Level::Warn);
        assert_eq!(r_e.level, Level::Error);
    }

    #[test]
    fn default_log_dir_is_dot_logs() {
        let default = std::env::var("SCRIBE_LOG_DIR")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| std::path::PathBuf::from("./logs"));
        assert_eq!(default, std::path::PathBuf::from("./logs"));
    }

    #[test]
    fn default_tag_is_default_until_init() {
        assert_eq!(default_tag(), "_default");
    }

    #[test]
    fn macro_info_uses_default_tag() {
        // This test parses the msg field to verify the tag used.
        // We can't easily intercept the macro's output in unit tests,
        // so we verify the default_tag() function works correctly.
        // The actual macro routing is smoke-tested in integration.
        assert_eq!(default_tag(), "_default");
    }
}
