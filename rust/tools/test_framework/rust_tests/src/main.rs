// SPDX-License-Identifier: MulanPSL-2.0
// Rust CLI Stress Test - Single Client Process
use anyhow::Result;
use clap::Parser;
use robonix_core::ros_idl::task::{SubmitTaskRequest, SubmitTaskResponse};
use ros2_client::{
    service::AService, Context, Name, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
};
use rustdds::{policy, Duration, QosPolicyBuilder};
use std::time::{Duration as StdDuration, Instant};

#[derive(Parser, Debug)]
struct Args {
    #[arg(long, default_value_t = 0)]
    client_id: usize,
    #[arg(short, long, default_value_t = 1000)]
    requests: usize,
    #[arg(short, long, default_value_t = 100)]
    rate: u64,
    #[arg(short, long, default_value_t = 0)]
    duration: u64,
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    let args = Args::parse();
    let context = Context::new().map_err(|e| anyhow::anyhow!("{:?}", e))?;
    let mut node = context
        .new_node(
            NodeName::new("/rbnx", &format!("stress_test_{}", args.client_id)).unwrap(),
            NodeOptions::new().enable_rosout(false),
        )
        .map_err(|e| anyhow::anyhow!("{:?}", e))?;

    let qos = QosPolicyBuilder::new()
        .history(policy::History::KeepLast { depth: 10 })
        .reliability(policy::Reliability::Reliable {
            max_blocking_time: Duration::from_millis(100),
        })
        .durability(policy::Durability::Volatile)
        .build();

    let client = node
        .create_client::<AService<SubmitTaskRequest, SubmitTaskResponse>>(
            ServiceMapping::Enhanced,
            &Name::new("/rbnx/task", "submit").unwrap(),
            &ServiceTypeName::new("robonix_sdk", "SubmitTask"),
            qos.clone(),
            qos.clone(),
        )
        .map_err(|e| anyhow::anyhow!("{:?}", e))?;

    tokio::time::sleep(StdDuration::from_secs(5)).await;

    let mut latencies = Vec::new();
    let (mut success, mut failed) = (0, 0);
    let start_time = Instant::now();
    let interval = if args.rate > 0 {
        StdDuration::from_secs(1) / args.rate as u32
    } else {
        StdDuration::from_millis(0)
    };
    let mut last_req = Instant::now();
    let end_time = if args.duration > 0 {
        Some(start_time + StdDuration::from_secs(args.duration))
    } else {
        None
    };

    for i in 0..args.requests {
        if let Some(end) = end_time {
            if Instant::now() >= end {
                break;
            }
        }
        let elapsed = last_req.elapsed();
        if elapsed < interval {
            tokio::time::sleep(interval - elapsed).await;
        }
        last_req = Instant::now();

        let req = SubmitTaskRequest {
            description: format!("task {}", i),
            params: "{}".to_string(),
        };
        let call_start = Instant::now();

        match tokio::time::timeout(StdDuration::from_secs(5), client.async_call_service(req)).await
        {
            Ok(Ok(_)) => {
                success += 1;
                latencies.push(call_start.elapsed().as_secs_f64() * 1000.0);
            }
            _ => failed += 1,
        }
    }

    latencies.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let total = success + failed;
    let avg = if latencies.is_empty() {
        0.0
    } else {
        latencies.iter().sum::<f64>() / latencies.len() as f64
    };
    let p = |pct: f64| {
        if latencies.is_empty() {
            0.0
        } else {
            latencies[((latencies.len() - 1) as f64 * pct / 100.0) as usize]
        }
    };

    println!("\nOverall Statistics:");
    println!(
        "Total: {}, Success: {}, Failed: {}, Rate: {:.2}%",
        total,
        success,
        failed,
        (success as f64 / total as f64) * 100.0
    );
    println!(
        "Latency (ms): Avg: {:.2}, Min: {:.2}, Max: {:.2}, P50: {:.2}, P95: {:.2}, P99: {:.2}, P999: {:.2}",
        avg,
        p(0.0),
        p(100.0),
        p(50.0),
        p(95.0),
        p(99.0),
        p(99.9)
    );

    Ok(())
}
