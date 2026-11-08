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
    /// Reported latencies, kept only to compute the summary below.
    #[serde(skip)]
    latencies: Vec<u64>,
    /// Calls that reported a latency.
    latency_samples: u64,
    /// Median reported latency in milliseconds.
    latency_p50_ms: Option<u64>,
    /// 95th percentile reported latency in milliseconds.
    latency_p95_ms: Option<u64>,
    /// Maximum reported latency in milliseconds.
    latency_max_ms: Option<u64>,
}

impl Total {
    /// Sort once, then fill the latency summary fields.
    fn summarize_latency(&mut self) {
        self.latency_samples = self.latencies.len() as u64;
        if self.latencies.is_empty() {
            return;
        }
        self.latencies.sort_unstable();
        self.latency_p50_ms = Some(percentile(&self.latencies, 0.50));
        self.latency_p95_ms = Some(percentile(&self.latencies, 0.95));
        self.latency_max_ms = self.latencies.last().copied();
    }
}

/// Nearest-rank percentile of a sorted slice; `fraction` is in `0..=1`.
fn percentile(sorted: &[u64], fraction: f64) -> u64 {
    let rank = (fraction * sorted.len() as f64).ceil() as usize;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
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
            if let Some(latency) = count("latency_ms") {
                total.latencies.push(latency);
            }
            if count("input_tokens").is_none() || count("output_tokens").is_none() {
                total.unreported += 1;
            }
        }
    }
    let mut totals: Vec<Total> = groups.into_values().collect();
    for total in &mut totals {
        total.summarize_latency();
    }
    Ok((totals, skipped))
}

fn print_table(totals: &[Total]) {
    println!(
        "{:<14} {:<16} {:<24} {:<24} {:>6} {:>8} {:>10} {:>10} {:>10} {:>10} {:>10} {:>7} {:>8} {:>8} {:>8}  status",
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
        "unreported",
        "lat_n",
        "p50_ms",
        "p95_ms",
        "max_ms"
    );
    for t in totals {
        let status = t
            .status
            .iter()
            .map(|(name, count)| format!("{name}={count}"))
            .collect::<Vec<_>>()
            .join(" ");
        println!(
            "{:<14} {:<16} {:<24} {:<24} {:>6} {:>8} {:>10} {:>10} {:>10} {:>10} {:>10} {:>7} {:>8} {:>8} {:>8}  {status}",
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
            t.unreported,
            t.latency_samples,
            fmt_ms(t.latency_p50_ms),
            fmt_ms(t.latency_p95_ms),
            fmt_ms(t.latency_max_ms)
        );
    }
}

/// A latency cell, or `-` when no record reported one.
fn fmt_ms(value: Option<u64>) -> String {
    value.map_or_else(|| "-".to_owned(), |value| value.to_string())
}

#[cfg(test)]
mod tests {
    use super::sum;

    #[test]
    fn sums_by_group_and_boot_and_skips_partial_lines() {
        let directory = tempfile::tempdir().unwrap();
        let records = [
            r#"{"consumer":"pilot","operation":"plan","host":"h","model":"m","status":"ok","attempts":2,"input_tokens":10,"output_tokens":3,"cached_input_tokens":4,"latency_ms":100,"boot_id":"a"}"#,
            r#"{"consumer":"pilot","operation":"plan","host":"h","model":"m","status":"timeout","attempts":1,"input_tokens":null,"output_tokens":null,"latency_ms":300,"boot_id":"a"}"#,
            r#"{"consumer":"scene","operation":"caption","host":"h","model":"m","status":"ok","input_tokens":5,"output_tokens":1,"latency_ms":200,"boot_id":"b"}"#,
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
        assert_eq!(
            (
                plan.latency_samples,
                plan.latency_p50_ms,
                plan.latency_p95_ms,
                plan.latency_max_ms
            ),
            (2, Some(100), Some(300), Some(300))
        );
        // Python writers before `attempts` count as one request.
        assert_eq!(totals[1].attempts, 1);
        assert_eq!(totals[1].latency_p50_ms, Some(200));

        let (totals, _) = sum(directory.path(), Some("b")).unwrap();
        assert_eq!(totals.len(), 1);
        assert_eq!(totals[0].consumer, "scene");
    }
}
