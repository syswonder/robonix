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
    files = sorted(results_dir.glob("*.json"))
    rows = []
    for f in files:
        with open(f) as fp:
            data = json.load(fp)
        transport = data.get("transport", "?")
        rmw = data.get("rmw") or ""
        name = f"{transport} ({rmw})" if rmw else transport
        stats = data.get("stats", {})
        mean = stats.get("mean_us", 0)
        std = stats.get("std_us", 0)
        cv = (std / mean * 100) if mean > 0 else 0
        rows.append({
            "name": name,
            "mean_us": mean,
            "median_us": stats.get("median_us", 0),
            "p99_us": stats.get("p99_us", 0),
            "min_us": stats.get("min_us", 0),
            "max_us": stats.get("max_us", 0),
            "std_us": std,
            "cv_pct": cv,
        })
    return rows


def plot_results(rows: list[dict], output_path: Path) -> None:
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = ["Latin Modern Sans", "LM Sans 10", "DejaVu Sans"] + plt.rcParams["font.sans-serif"]

    names = [r["name"] for r in rows]
    x = np.arange(len(names))
    width = 0.25

    mean_vals = np.array([r["mean_us"] for r in rows])
    median_vals = [r["median_us"] for r in rows]
    p99_vals = [r["p99_us"] for r in rows]
    std_vals = np.array([r["std_us"] for r in rows])

    # Asymmetric error bars: lower = min(mean, std) so bar never goes below 0
    err_lower = np.minimum(mean_vals, std_vals)
    err_upper = std_vals
    yerr = np.array([err_lower, err_upper])

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(x - width, mean_vals, width, yerr=yerr, capsize=2, label="mean ± std", color="#2e86ab")
    ax.bar(x, median_vals, width, label="median", color="#a23b72")
    ax.bar(x + width, p99_vals, width, label="p99", color="#f18f01")

    ax.set_ylabel(r"Latency ($\mu$s)", fontsize=11)
    ax.set_title("Latency Benchmark: Transport Comparison (error bars = std)", fontsize=12)
    ax.set_ylim(bottom=0)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, ha="right")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Plot saved to {output_path}")


def format_summary(rows: list[dict]) -> str:
    lines = [
        "",
        "=== Latency Benchmark Summary (μs) ===",
        "",
        f"{'Transport':<30} {'mean':>10} {'std':>10} {'cv%':>8} {'median':>10} {'p99':>10} {'min':>10} {'max':>10}",
        "-" * 108,
    ]
    for r in rows:
        lines.append(
            f"{r['name']:<30} {r['mean_us']:>10.2f} {r['std_us']:>10.2f} {r['cv_pct']:>7.2f}% "
            f"{r['median_us']:>10.2f} {r['p99_us']:>10.2f} {r['min_us']:>10.2f} {r['max_us']:>10.2f}"
        )
    lines.append("")
    lines.append("Stability: std = standard deviation, cv% = coefficient of variation (std/mean*100, lower = more stable)")
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
