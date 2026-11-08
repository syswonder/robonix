// SPDX-License-Identifier: MulanPSL-2.0
//! `rbnx llm-usage` — sum the per-request model usage records that
//! `robonix_api.llm.metrics` and `robonix-llm-metrics` append.

use anyhow::{Context, Result};
use serde::Serialize;
use serde_json::Value;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Requests and reported tokens of one consumer/operation/host/model group.
#[derive(Debug, Default, PartialEq, Eq, Serialize)]
struct Total {
    /// Component that made the requests.
    consumer: String,
    /// What the component used the model for.
    operation: String,
    /// Endpoint host and port; joined with `model` against a price list.
    host: String,
    /// Model named in the requests.
    model: String,
    /// Records in the group, one per call.
    calls: u64,
    /// Requests sent, retries included.
    attempts: u64,
    /// Calls per outcome (`ok`, `http_error`, ...).
    status: BTreeMap<String, u64>,
    /// Sum of reported prompt tokens.
    input_tokens: u64,
    /// Sum of reported generated tokens.
    output_tokens: u64,
    /// Sum of reported cached prompt tokens.
    cached_input_tokens: u64,
    /// Sum of reported reasoning tokens.
    reasoning_tokens: u64,
    /// Calls whose input or output count was not reported; their tokens are
    /// missing from the sums.
    unreported: u64,
}

pub fn execute(dir: Option<PathBuf>, boot: Option<&str>, json: bool) -> Result<()> {
    let dir = dir.unwrap_or_else(|| {
        std::env::var_os("ROBONIX_LLM_METRICS_DIR")
            .filter(|d| !d.is_empty())
            .map_or_else(|| PathBuf::from("./rbnx-boot/llm-metrics"), PathBuf::from)
    });
    let (totals, skipped) = sum(&dir, boot)?;
    if json {
        for total in &totals {
            println!("{}", serde_json::to_string(total)?);
        }
    } else {
        print_table(&totals);
    }
    if skipped > 0 {
        eprintln!("rbnx llm-usage: skipped {skipped} unreadable line(s) or file(s)");
    }
    Ok(())
}

/// Totals of every `*.jsonl` record under `dir`, and the count of lines
/// that were not a record: a line still being written, or an unreadable file
/// (counted as one).
fn sum(dir: &Path, boot: Option<&str>) -> Result<(Vec<Total>, u64)> {
    let entries = std::fs::read_dir(dir).with_context(|| {
        format!(
            "{} is not readable; pass -d or set ROBONIX_LLM_METRICS_DIR",
            dir.display()
        )
    })?;
    let mut groups: BTreeMap<[String; 4], Total> = BTreeMap::new();
    let mut skipped = 0;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("jsonl") {
            continue;
        }
        let Ok(text) = std::fs::read_to_string(&path) else {
            skipped += 1;
            continue;
        };
        for line in text.lines().filter(|line| !line.trim().is_empty()) {
            let Ok(record) = serde_json::from_str::<Value>(line) else {
                skipped += 1;
                continue;
            };
            if boot.is_some_and(|boot| record["boot_id"].as_str() != Some(boot)) {
                continue;
            }
            let field = |name: &str| record[name].as_str().unwrap_or_default().to_owned();
            let key = [
                field("consumer"),
                field("operation"),
                field("host"),
                field("model"),
            ];
            let total = groups.entry(key.clone()).or_insert_with(|| {
                let [consumer, operation, host, model] = key;
                Total {
                    consumer,
                    operation,
                    host,
                    model,
                    ..Total::default()
                }
            });
            let count = |name: &str| record[name].as_u64();
            total.calls += 1;
            total.attempts += count("attempts").unwrap_or(1);
            *total.status.entry(field("status")).or_default() += 1;
            total.input_tokens += count("input_tokens").unwrap_or(0);
            total.output_tokens += count("output_tokens").unwrap_or(0);
            total.cached_input_tokens += count("cached_input_tokens").unwrap_or(0);
            total.reasoning_tokens += count("reasoning_tokens").unwrap_or(0);
            if count("input_tokens").is_none() || count("output_tokens").is_none() {
                total.unreported += 1;
            }
        }
    }
    Ok((groups.into_values().collect(), skipped))
}

fn print_table(totals: &[Total]) {
    println!(
        "{:<14} {:<16} {:<24} {:<24} {:>6} {:>8} {:>10} {:>10} {:>10} {:>10} {:>10}  status",
        "consumer",
        "operation",
        "host",
        "model",
        "calls",
        "attempts",
        "input",
        "output",
        "cached",
        "reasoning",
        "unreported"
    );
    for t in totals {
        let status = t
            .status
            .iter()
            .map(|(name, count)| format!("{name}={count}"))
            .collect::<Vec<_>>()
            .join(" ");
        println!(
            "{:<14} {:<16} {:<24} {:<24} {:>6} {:>8} {:>10} {:>10} {:>10} {:>10} {:>10}  {status}",
            t.consumer,
            t.operation,
            t.host,
            t.model,
            t.calls,
            t.attempts,
            t.input_tokens,
            t.output_tokens,
            t.cached_input_tokens,
            t.reasoning_tokens,
            t.unreported
        );
    }
}

#[cfg(test)]
mod tests {
    use super::sum;

    #[test]
    fn sums_by_group_and_boot_and_skips_partial_lines() {
        let directory = tempfile::tempdir().unwrap();
        let records = [
            r#"{"consumer":"pilot","operation":"plan","host":"h","model":"m","status":"ok","attempts":2,"input_tokens":10,"output_tokens":3,"cached_input_tokens":4,"boot_id":"a"}"#,
            r#"{"consumer":"pilot","operation":"plan","host":"h","model":"m","status":"timeout","attempts":1,"input_tokens":null,"output_tokens":null,"boot_id":"a"}"#,
            r#"{"consumer":"scene","operation":"caption","host":"h","model":"m","status":"ok","input_tokens":5,"output_tokens":1,"boot_id":"b"}"#,
            r#"{"consumer":"pilot","operation":"pl"#,
        ];
        std::fs::write(directory.path().join("pilot-1.jsonl"), records.join("\n")).unwrap();
        std::fs::write(directory.path().join("notes.txt"), "not a record").unwrap();
        std::fs::write(directory.path().join("scene-2.jsonl"), [0xff, 0xfe]).unwrap();

        let (totals, skipped) = sum(directory.path(), None).unwrap();
        assert_eq!(skipped, 2);
        assert_eq!(totals.len(), 2);
        let plan = &totals[0];
        assert_eq!((plan.calls, plan.attempts), (2, 3));
        assert_eq!(
            (
                plan.input_tokens,
                plan.output_tokens,
                plan.cached_input_tokens
            ),
            (10, 3, 4)
        );
        assert_eq!(plan.unreported, 1);
        assert_eq!(plan.status.get("timeout"), Some(&1));
        // Python writers before `attempts` count as one request.
        assert_eq!(totals[1].attempts, 1);

        let (totals, _) = sum(directory.path(), Some("b")).unwrap();
        assert_eq!(totals.len(), 1);
        assert_eq!(totals[0].consumer, "scene");
    }
}
