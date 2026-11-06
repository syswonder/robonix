// SPDX-License-Identifier: MulanPSL-2.0

#![cfg(target_os = "linux")]

use robonix_scribe::{Level, LogRecord};
use std::io::{BufRead, Write};
use std::process::Stdio;
include!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../chronos/clock/tests/support/mod.rs"
));
use self::mode as clock_mode;

#[test]
fn clock_failures_preserve_log_evidence() {
    if std::env::var("ROBONIX_CLOCK_TEST_CHILD").as_deref() == Ok("scribe-forwarder") {
        for line in std::io::stdin().lock().lines() {
            robonix_scribe::ingest("forwarded", &line.unwrap());
        }
        return;
    }
    let directory = tempfile::tempdir().unwrap();
    let logs = directory.path().join("logs");
    let Some(output) = run_case_with_env(
        "clock_failures_preserve_log_evidence",
        &[
            ("SCRIBE_LOG_DIR", logs.as_os_str()),
            ("SCRIBE_CONSOLE_LEVEL", std::ffi::OsStr::new("warn")),
            ("SCRIBE_FILE_LEVEL", std::ffi::OsStr::new("info")),
        ],
        || {
            robonix_scribe::log(Level::Info, "clock-test", "clock normal message");
            for (mode, message) in [(1, "clock failure message"), (2, "clock negative message")] {
                clock_mode(mode);
                assert!(LogRecord::now(Level::Info, "clock-test", message).is_err());
                robonix_scribe::log(Level::Info, "clock-test", message);
                robonix_scribe::log(
                    Level::Error,
                    "clock-test\x1b[31m",
                    "error message\nforged line",
                );
            }
            clock_mode(4);
            robonix_scribe::log(Level::Info, "clock-test", "clock quality message");
            clock_mode(0);
            robonix_scribe::log(Level::Info, "clock-test", "clock recovered message");
        },
    ) else {
        return;
    };
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("clock failure message [Chronos time read failed:"));
    assert!(stderr.contains("clock negative message [Chronos time read failed:"));
    assert!(stderr.contains("does not fit u64"));
    assert!(!output.stderr.contains(&0x1b));
    assert_eq!(stderr.lines().count(), 4);
    assert!(stderr.contains("error message\\nforged line"));
    let records: Vec<LogRecord> = std::fs::read_to_string(logs.join("clock-test.log"))
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    assert_eq!(records.len(), 3);
    assert_eq!(records[0].msg, "clock normal message");
    assert!(
        records[1]
            .msg
            .starts_with("clock quality message [Chronos wall clock unavailable:")
    );
    assert_eq!(records[2].msg, "clock recovered message");
    assert!(records.iter().all(|record| record.ts > 0));

    let forwarded_logs = directory.path().join("forwarded");
    let mut receiver = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "clock_failures_preserve_log_evidence",
            "--nocapture",
        ])
        .env("ROBONIX_CLOCK_TEST_CHILD", "scribe-forwarder")
        .env("SCRIBE_LOG_DIR", &forwarded_logs)
        .env("SCRIBE_FILE_LEVEL", "error")
        .env("SCRIBE_CONSOLE_LEVEL", "error")
        .env_remove("LD_PRELOAD")
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    receiver
        .stdin
        .take()
        .unwrap()
        .write_all(&output.stderr)
        .unwrap();
    let forwarded = receiver.wait_with_output().unwrap();
    assert!(
        forwarded.status.success(),
        "{}",
        String::from_utf8_lossy(&forwarded.stderr)
    );
    let records: Vec<LogRecord> = std::fs::read_to_string(forwarded_logs.join("forwarded.log"))
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    assert_eq!(records.len(), 2);
    assert!(records.iter().all(|record| record.level == Level::Error
        && record.msg.contains("error message\\nforged line")
        && record.msg.contains("Chronos time read failed")));
}
