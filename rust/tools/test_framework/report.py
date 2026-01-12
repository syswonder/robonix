#!/usr/bin/env python3
import json, os, sys, re, argparse, statistics
from pathlib import Path
from datetime import datetime

class BenchmarkReport:
    def __init__(self, log_dir, output_file=None, concurrency=None, requests=None, rate=None, duration=None):
        self.log_dir = Path(log_dir)
        self.output_file = Path(output_file) if output_file else (self.log_dir / "benchmark_report.json")
        self.results = {
            "stats": {}, 
            "metadata": {
                "timestamp": datetime.now().isoformat(), 
                "log_dir": str(self.log_dir), 
                "concurrency": concurrency,
                "requests": requests,
                "rate": rate,
                "duration": duration
            }
        }

    def parse_log(self, content):
        stats = {"total": 0, "success": 0, "failed": 0, "avg": 0, "min": 0, "max": 0, "p50": 0, "p95": 0, "p99": 0, "p999": 0}
        m = re.search(r'Overall Statistics:.*?Total: (\d+), Success: (\d+), Failed: (\d+).*?Latency \(us\): Avg: ([\d.]+), Min: ([\d.]+), Max: ([\d.]+), P50: ([\d.]+), P95: ([\d.]+), P99: ([\d.]+), P999: ([\d.]+)', content, re.DOTALL)
        if m:
            stats.update({"total": int(m.group(1)), "success": int(m.group(2)), "failed": int(m.group(3)),
                          "avg": float(m.group(4)), "min": float(m.group(5)), "max": float(m.group(6)),
                          "p50": float(m.group(7)), "p95": float(m.group(8)), "p99": float(m.group(9)), "p999": float(m.group(10))})
        return stats

    def aggregate_logs(self, pattern):
        logs = list(self.log_dir.glob(pattern))
        if not logs: return None
        all_stats = [self.parse_log(l.read_text()) for l in logs]
        # Filter out failed parses
        all_stats = [s for s in all_stats if s["total"] > 0]
        if not all_stats: return None
        
        agg = {"total": sum(s["total"] for s in all_stats), "success": sum(s["success"] for s in all_stats), "failed": sum(s["failed"] for s in all_stats)}
        agg.update({"avg": statistics.mean(s["avg"] for s in all_stats), 
                    "min": min(s["min"] for s in all_stats), 
                    "max": max(s["max"] for s in all_stats),
                    "p50": statistics.mean(s["p50"] for s in all_stats), 
                    "p95": max(s["p95"] for s in all_stats), 
                    "p99": max(s["p99"] for s in all_stats), 
                    "p999": max(s["p999"] for s in all_stats)})
        return agg

    def collect_results(self):
        rust_stats = self.aggregate_logs("rust_test_*.log")
        if rust_stats: self.results["stats"]["rust"] = rust_stats
        
        python_stats = self.aggregate_logs("python_test_*.log")
        if python_stats: self.results["stats"]["python"] = python_stats

        cpp_stats = self.aggregate_logs("cpp_test_*.log")
        if cpp_stats: self.results["stats"]["cpp"] = cpp_stats
        
        return self.results

    def save(self):
        self.output_file.write_text(json.dumps(self.results, indent=2))
        print(f"Report saved to: {self.output_file}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-dir', type=str, default='logs')
    parser.add_argument('--output', type=str)
    parser.add_argument('--requests', type=int)
    parser.add_argument('--rate', type=int)
    parser.add_argument('--duration', type=int)
    parser.add_argument('--concurrency', type=int)
    args = parser.parse_args()
    report = BenchmarkReport(args.log_dir, args.output, 
                             concurrency=args.concurrency,
                             requests=args.requests,
                             rate=args.rate,
                             duration=args.duration)
    report.collect_results()
    report.save()
