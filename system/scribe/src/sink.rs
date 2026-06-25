//! Per-tag file sink — JSON-lines output, one file per tag.
//!
//! A [`FileSink`] owns a directory (`$SCRIBE_LOG_DIR` or `./logs`) and a
//! `Mutex<HashMap<tag, File>>`.  The first write for a given `tag` opens
//! (or creates) `{dir}/{tag}.log`; subsequent writes reuse the handle.
//!
//! Every line is a JSON-serialised [`LogRecord`] followed by `\n`.  The
//! sink flushes after each write so a crash does not lose the last
//! record.

use crate::LogRecord;
use std::collections::HashMap;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

/// Per-tag file sink.
///
/// # Thread safety
///
/// All writes go through a single `Mutex`.  Contention is low because
/// writes are small and every record is flushed immediately.
pub struct FileSink {
    dir: PathBuf,
    writers: Mutex<HashMap<String, File>>,
}

impl FileSink {
    /// Create a new file sink rooted at `log_dir`.
    ///
    /// Creates the directory (and any missing parents) if it does not
    /// already exist.
    pub fn new(log_dir: impl Into<PathBuf>) -> io::Result<Self> {
        let dir = log_dir.into();
        fs::create_dir_all(&dir)?;
        Ok(Self {
            dir,
            writers: Mutex::new(HashMap::new()),
        })
    }

    /// Write one [`LogRecord`] to `{dir}/{record.tag}.log`.
    ///
    /// Opens the file in append mode on first use for a given tag; caches
    /// the handle thereafter.  Serialises the record as a single JSON
    /// line, writes it, and flushes.
    ///
    /// # Errors
    ///
    /// Returns `io::Error` if the file can't be opened or written.
    /// The caller (`log()` in lib.rs) treats this as best-effort and
    /// silently drops the file write on failure.
    pub fn write(&self, record: &LogRecord) -> io::Result<()> {
        let json_line = serde_json::to_string(record)?;
        let mut guard = self.writers.lock().unwrap_or_else(|e| e.into_inner());
        // or_insert_with can't use `?`; use entry + insert to propagate errors.
        let writer: &mut File = if let Some(w) = guard.get_mut(&record.tag) {
            w
        } else {
            let path = self.tag_path(&record.tag);
            let f = OpenOptions::new().create(true).append(true).open(&path)?;
            guard.entry(record.tag.clone()).or_insert(f)
        };
        writeln!(writer, "{json_line}")?;
        writer.flush()?;
        Ok(())
    }

    /// Absolute path for a given tag's log file.
    ///
    /// Non-safe characters (anything outside `[A-Za-z0-9._-]`) are
    /// replaced with `_` to prevent path traversal / invalid filenames.
    fn tag_path(&self, tag: &str) -> PathBuf {
        let safe: String = tag
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-' {
                    c
                } else {
                    '_'
                }
            })
            .collect();
        self.dir.join(format!("{safe}.log"))
    }

    /// Return the log directory path (for diagnostics).
    #[allow(dead_code)]
    pub fn dir(&self) -> &Path {
        &self.dir
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Level;
    use std::io::BufRead;

    fn test_sink() -> (tempfile::TempDir, FileSink) {
        let tmp = tempfile::tempdir().unwrap();
        let sink = FileSink::new(tmp.path()).unwrap();
        (tmp, sink)
    }

    fn read_lines(path: &Path) -> Vec<String> {
        let f = File::open(path).unwrap();
        io::BufReader::new(f).lines().map(|l| l.unwrap()).collect()
    }

    #[test]
    fn first_write_creates_file() {
        let (tmp, sink) = test_sink();
        let rec = LogRecord {
            ts: 1234567890,
            level: Level::Info,
            tag: "atlas".into(),
            msg: "hello".into(),
        };
        sink.write(&rec).unwrap();

        let log_path = tmp.path().join("atlas.log");
        assert!(log_path.exists(), "expected {log_path:?} to exist");

        let lines = read_lines(&log_path);
        assert_eq!(lines.len(), 1);
        let parsed: LogRecord = serde_json::from_str(&lines[0]).unwrap();
        assert_eq!(parsed.tag, "atlas");
        assert_eq!(parsed.msg, "hello");
    }

    #[test]
    fn same_tag_appends() {
        let (tmp, sink) = test_sink();
        let rec1 = LogRecord {
            ts: 1,
            level: Level::Debug,
            tag: "a".into(),
            msg: "m1".into(),
        };
        let rec2 = LogRecord {
            ts: 2,
            level: Level::Info,
            tag: "a".into(),
            msg: "m2".into(),
        };
        sink.write(&rec1).unwrap();
        sink.write(&rec2).unwrap();

        let lines = read_lines(&tmp.path().join("a.log"));
        assert_eq!(lines.len(), 2);
        let r1: LogRecord = serde_json::from_str(&lines[0]).unwrap();
        let r2: LogRecord = serde_json::from_str(&lines[1]).unwrap();
        assert_eq!(r1.msg, "m1");
        assert_eq!(r2.msg, "m2");
    }

    #[test]
    fn different_tags_different_files() {
        let (tmp, sink) = test_sink();
        sink.write(&LogRecord {
            ts: 0,
            level: Level::Warn,
            tag: "x".into(),
            msg: "mx".into(),
        })
        .unwrap();
        sink.write(&LogRecord {
            ts: 0,
            level: Level::Error,
            tag: "y".into(),
            msg: "my".into(),
        })
        .unwrap();

        let x_path = tmp.path().join("x.log");
        let y_path = tmp.path().join("y.log");
        assert!(x_path.exists());
        assert!(y_path.exists());

        let x_lines = read_lines(&x_path);
        let y_lines = read_lines(&y_path);
        assert_eq!(x_lines.len(), 1);
        assert_eq!(y_lines.len(), 1);

        let rx: LogRecord = serde_json::from_str(&x_lines[0]).unwrap();
        let ry: LogRecord = serde_json::from_str(&y_lines[0]).unwrap();
        assert_eq!(rx.tag, "x");
        assert_eq!(ry.tag, "y");
    }

    #[test]
    fn concurrent_writes_no_data_loss() {
        use std::sync::Arc;
        use std::thread;

        let tmp = tempfile::tempdir().unwrap();
        let sink = Arc::new(FileSink::new(tmp.path()).unwrap());
        const THREADS: usize = 8;
        const MSGS_PER_THREAD: usize = 100;

        let mut handles = vec![];
        for t in 0..THREADS {
            let sink = Arc::clone(&sink);
            handles.push(thread::spawn(move || {
                for i in 0..MSGS_PER_THREAD {
                    sink.write(&LogRecord {
                        ts: (t * MSGS_PER_THREAD + i) as u64,
                        level: Level::Info,
                        tag: "shared".into(),
                        msg: format!("t{t}-m{i}"),
                    })
                    .unwrap();
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        let lines = read_lines(&tmp.path().join("shared.log"));
        assert_eq!(lines.len(), THREADS * MSGS_PER_THREAD);
        // Every line should be parseable
        for line in &lines {
            let _rec: LogRecord = serde_json::from_str(line).unwrap();
        }
    }

    #[test]
    fn sanitize_tag_replaces_dangerous_chars() {
        let (tmp, sink) = test_sink();
        // Tag with `/`, `..`, and spaces — should be sanitized into a safe filename.
        let rec = LogRecord {
            ts: 0,
            level: Level::Info,
            tag: "a/../b c".into(),
            msg: "m".into(),
        };
        sink.write(&rec).unwrap();
        // `/` becomes `_`, `.` is safe, ` ` becomes `_` → "a_.._b_c"
        let safe_path = tmp.path().join("a_.._b_c.log");
        assert!(
            safe_path.exists(),
            "expected {safe_path:?} to exist after sanitization"
        );
    }

    #[test]
    fn write_does_not_panic_on_bad_path() {
        // Create a sink pointed at a path that can't be a directory.
        let sink = FileSink::new("/proc/sysrq-trigger"); // existing file, not a dir
        assert!(
            sink.is_err(),
            "should fail to create sink under a non-directory"
        );
    }

    #[test]
    fn sink_creation_fails_gracefully() {
        // /dev/null is a file, not a directory — create_dir_all should fail.
        let result = FileSink::new("/dev/null/logs");
        assert!(result.is_err());
    }
}
