#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Summarize latency benchmark results from JSON files into a comparison table and plot."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_results(results_dir: Path) -> list[dict]:
    files = sorted(results_dir.glob("*.json"), key=lambda p: p.stat().st_mtime)
    seen: dict[tuple[str, str], dict] = {}
    for f in files:
        with open(f) as fp:
            data = json.load(fp)
        transport = data.get("transport", "?")
        rmw = data.get("rmw") or ""
        key = (transport, rmw)
        stats = data.get("stats", {})
        mean = stats.get("mean_us", 0)
        std = stats.get("std_us", 0)
        cv = (std / mean * 100) if mean > 0 else 0
        row = {
            "name": f"{transport} ({rmw})" if rmw else transport,
            "startup_us": stats.get("startup_us", stats.get("first_request_us", 0)),
            "mean_us": mean,
            "median_us": stats.get("median_us", 0),
            "p99_us": stats.get("p99_us", 0),
            "min_us": stats.get("min_us", 0),
            "max_us": stats.get("max_us", 0),
            "std_us": std,
            "cv_pct": cv,
        }
        seen[key] = row
    order = ["grpc", "zmq", "http", "ros2"]
    return sorted(seen.values(), key=lambda r: (
        order.index(r["name"].split(" ")[0]) if r["name"].split(" ")[0] in order else 99,
        r["name"],
    ))


def plot_results(rows: list[dict], output_path: Path) -> None:
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = ["Latin Modern Sans", "LM Sans 10", "DejaVu Sans"] + plt.rcParams["font.sans-serif"]

    names = [r["name"] for r in rows]
    x = np.arange(len(names))
    width = 0.2

    startup_vals = [r.get("startup_us", 0) for r in rows]
    mean_vals = np.array([r["mean_us"] for r in rows])
    median_vals = [r["median_us"] for r in rows]
    p99_vals = [r["p99_us"] for r in rows]
    std_vals = np.array([r["std_us"] for r in rows])

    # Asymmetric error bars: lower = min(mean, std) so bar never goes below 0
    err_lower = np.minimum(mean_vals, std_vals)
    err_upper = std_vals
    yerr = np.array([err_lower, err_upper])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    def _fmt_us(val):
        if val >= 1e6:
            return f"{val/1e6:.2f} s"
        elif val >= 1e3:
            return f"{val/1e3:.1f} ms"
        else:
            return f"{val:.1f} us"

    # Left: startup only (log scale: ROS2 can be 100x+ slower than gRPC/ZMQ)
    startup_plot = np.maximum(startup_vals, 1)  # avoid log(0)
    bars1 = ax1.bar(x, startup_plot, width=0.5, color="#c73e1d")
    ax1.bar_label(bars1, labels=[_fmt_us(v) for v in startup_vals], fontsize=8)
    ax1.set_yscale("log")
    ax1.set_ylabel(r"Latency ($\mu$s, log scale)", fontsize=11)
    ax1.set_title("Startup (init to first response)", fontsize=12)
    ax1.set_xticks(x)
    ax1.set_xticklabels(names, rotation=15, ha="right")
    ax1.grid(axis="y", alpha=0.3)

    # Right: steady-state (mean, median, p99)
    w = 0.25
    bars_mean = ax2.bar(x - w, mean_vals, w, yerr=yerr, capsize=2, label="mean ± std", color="#2e86ab")
    bars_med = ax2.bar(x, median_vals, w, label="median", color="#a23b72")
    bars_p99 = ax2.bar(x + w, p99_vals, w, label="p99", color="#f18f01")
    ax2.bar_label(bars_mean, labels=[_fmt_us(v) for v in mean_vals], fontsize=7)
    ax2.bar_label(bars_med, labels=[_fmt_us(v) for v in median_vals], fontsize=7)
    ax2.bar_label(bars_p99, labels=[_fmt_us(v) for v in p99_vals], fontsize=7)
    ax2.set_ylabel(r"Latency ($\mu$s)", fontsize=11)
    ax2.set_title("Steady-state RTT", fontsize=12)
    ax2.set_ylim(bottom=0)
    ax2.set_xticks(x)
    ax2.set_xticklabels(names, rotation=15, ha="right")
    ax2.legend()
    ax2.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Plot saved to {output_path}")


def format_summary(rows: list[dict]) -> str:
    lines = [
        "",
        "=== Latency Benchmark Summary (μs) ===",
        "",
        f"{'Transport':<30} {'startup':>10} {'mean':>10} {'std':>10} {'cv%':>8} {'median':>10} {'p99':>10} {'min':>10} {'max':>10}",
        "-" * 118,
    ]
    for r in rows:
        startup = r.get("startup_us", 0)
        lines.append(
            f"{r['name']:<30} {startup:>10.2f} {r['mean_us']:>10.2f} {r['std_us']:>10.2f} {r['cv_pct']:>7.2f}% "
            f"{r['median_us']:>10.2f} {r['p99_us']:>10.2f} {r['min_us']:>10.2f} {r['max_us']:>10.2f}"
        )
    lines.append("")
    lines.append("startup = init to first response (rclpy.init, discovery, connect, etc.); std = std dev; cv% = coefficient of variation")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dir", nargs="?", default="results", help="Results directory")
    parser.add_argument("-o", "--output", metavar="FILE", help="Override summary txt path (default: <dir>/summary.txt)")
    parser.add_argument("-p", "--plot", metavar="FILE", help="Override plot path (default: <dir>/latency_plot.png)")
    parser.add_argument("--no-txt", action="store_true", help="Do not save summary.txt")
    parser.add_argument("--no-plot", action="store_true", help="Do not save latency_plot.png")
    args = parser.parse_args()

    results_dir = Path(args.dir)
    if not results_dir.exists():
        print(f"Directory not found: {results_dir}")
        return

    rows = load_results(results_dir)
    if not rows:
        print(f"No JSON files in {results_dir}")
        return

    summary = format_summary(rows)
    print(summary)

    if not args.no_txt:
        txt_path = Path(args.output) if args.output else results_dir / "summary.txt"
        txt_path.parent.mkdir(parents=True, exist_ok=True)
        txt_path.write_text(summary, encoding="utf-8")
        print(f"Summary saved to {txt_path}")

    if not args.no_plot:
        plot_path = Path(args.plot) if args.plot else results_dir / "latency_plot.png"
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        plot_results(rows, plot_path)


if __name__ == "__main__":
    main()
