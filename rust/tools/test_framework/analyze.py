#!/usr/bin/env python3
import json, sys, re, argparse, subprocess
from pathlib import Path
from report import BenchmarkReport
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Update font cache and try to use Latin Modern Sans
try:
    subprocess.run(['fc-cache', '-f'], check=False)
except:
    pass

available_fonts = [f.name for f in matplotlib.font_manager.fontManager.ttflist]
if "Latin Modern Sans" in available_fonts:
    plt.rcParams['font.family'] = 'Latin Modern Sans'
else:
    plt.rcParams['font.family'] = 'sans-serif'

def main():
    if len(sys.argv) > 1:
        log_base_dir = Path(sys.argv[1])
    else:
        # If no dir provided, find the latest benchmark_* folder in logs
        logs_root = Path("logs")
        if not logs_root.exists(): return
        bench_dirs = sorted([d for d in logs_root.iterdir() if d.is_dir() and d.name.startswith("benchmark_")], reverse=True)
        if not bench_dirs: return
        log_base_dir = bench_dirs[0]
    
    print(f"Processing benchmark directory: {log_base_dir}")
    test_dirs = sorted([d for d in log_base_dir.iterdir() if d.is_dir() and "_c" in d.name])
    if not test_dirs: 
        print("No test directories found.")
        return

    all_results = []
    for d in test_dirs:
        c_match = re.search(r'_c(\d+)(?:_|$)', d.name)
        concurrency = int(c_match.group(1)) if c_match else 0
        report = BenchmarkReport(d, concurrency=concurrency)
        res = report.collect_results()
        for t_type, stats in res["stats"].items():
            all_results.append({"type": t_type, "concurrency": concurrency, "stats": stats, "name": d.name})

    if not all_results:
        print("No results to report.")
        return

    # Generate Text Report
    generate_text_report(all_results, log_base_dir)

    # Prepare data for plotting
    concurrencies = sorted(list(set(r["concurrency"] for r in all_results)))
    
    plot_data = {
        'rust': {'p50': [], 'p99': [], 'p999': [], 'fail_rate': []},
        'python': {'p50': [], 'p99': [], 'p999': [], 'fail_rate': []}
    }

    for c in concurrencies:
        for t in ['rust', 'python']:
            match = [r for r in all_results if r['type'] == t and r['concurrency'] == c]
            if match:
                match.sort(key=lambda x: x["name"], reverse=True)
                s = match[0]['stats']
                plot_data[t]['p50'].append(s.get('p50', 0))
                plot_data[t]['p99'].append(s.get('p99', 0))
                plot_data[t]['p999'].append(s.get('p999', 0))
                fail_rate = (s['failed'] / s['total'] * 100) if s['total'] > 0 else 0
                plot_data[t]['fail_rate'].append(fail_rate)
            else:
                for k in plot_data[t]: plot_data[t][k].append(None)

    # Plot Latency Curves - Separate for Rust and Python
    for t in ['rust', 'python']:
        plt.figure(figsize=(12, 8))
        color = 'b' if t == 'rust' else 'r'
        plt.plot(concurrencies, plot_data[t]['p50'], f'{color}o-', label='P50')
        plt.plot(concurrencies, plot_data[t]['p99'], f'{color}s--', label='P99')
        plt.plot(concurrencies, plot_data[t]['p999'], f'{color}^:', label='P999')
        
        plt.xlabel('Concurrency')
        plt.ylabel('Latency (ms)')
        plt.title(f'{t.capitalize()} Latency Percentiles by Concurrency')
        plt.grid(True, which='both', linestyle='--', alpha=0.5)
        plt.legend()
        plt.savefig(log_base_dir / f"latency_curves_{t}.png")
        plt.close()
        print(f"{t.capitalize()} latency curves saved to {log_base_dir / f'latency_curves_{t}.png'}")

    # Plot Failure Rate Curves - Keep together for comparison
    plt.figure(figsize=(12, 8))
    plt.plot(concurrencies, plot_data['rust']['fail_rate'], 'bo-', label='Rust Failure Rate')
    plt.plot(concurrencies, plot_data['python']['fail_rate'], 'ro-', label='Python Failure Rate')
    plt.xlabel('Concurrency')
    plt.ylabel('Failure Rate (%)')
    plt.title('Failure Rate Comparison by Concurrency')
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.legend()
    plt.savefig(log_base_dir / "failure_rate_comparison.png")
    plt.close()
    print(f"Failure rate comparison saved to {log_base_dir / 'failure_rate_comparison.png'}")

def generate_text_report(all_results, output_dir):
    summary = {} # concurrency -> {type -> stats}
    for r in all_results:
        c = r["concurrency"]
        if c not in summary: summary[c] = {}
        if r["type"] not in summary[c] or r["name"] > summary[c][r["type"]]["name"]:
            summary[c][r["type"]] = r

    report_lines = []
    report_lines.append("=" * 100)
    report_lines.append(f"{'Robonix Benchmark Summary Report':^100}")
    report_lines.append("=" * 100)
    report_lines.append(f"{'Conc':<6} {'Type':<10} {'Total':>8} {'Succ':>8} {'Fail':>8} {'Rate':>8} {'Avg':>8} {'P50':>8} {'P99':>8} {'P999':>8}")
    report_lines.append("-" * 100)

    for c in sorted(summary.keys()):
        for t in sorted(summary[c].keys()):
            r = summary[c][t]
            s = r["stats"]
            rate = (s['success'] / s['total'] * 100) if s['total'] > 0 else 0
            report_lines.append(f"{c:<6} {t:<10} {s['total']:>8} {s['success']:>8} {s['failed']:>8} {rate:>7.1f}% {s['avg']:>8.2f} {s['p50']:>8.2f} {s['p99']:>8.2f} {s['p999']:>8.2f}")
        report_lines.append("-" * 100)

    report_text = "\n".join(report_lines)
    print(report_text)
    (output_dir / "benchmark_summary.txt").write_text(report_text)
    print(f"Summary report saved to {output_dir / 'benchmark_summary.txt'}")

if __name__ == '__main__': main()
