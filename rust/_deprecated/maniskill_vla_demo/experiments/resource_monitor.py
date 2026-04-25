#!/usr/bin/env python3
"""Light-weight resource sampler. Polls nvidia-smi + /proc for one or more PIDs
at a fixed rate and writes a CSV. Absolute unix timestamps are written so the
eval driver can splice time-aligned segment annotations.

Usage:
    python3 resource_monitor.py --pids 12345,67890 --gpus 2,3 \
            --out /tmp/resource.csv --hz 2
"""
import argparse
import csv
import os
import signal
import subprocess
import sys
import time


RUN = True
def _stop(*_):
    global RUN
    RUN = False
signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)


def sample_nvidia_smi(gpu_idx_csv: str) -> list[dict]:
    out = subprocess.run([
        "nvidia-smi",
        f"--query-gpu=index,utilization.gpu,memory.used,memory.total",
        "--format=csv,noheader,nounits",
        "-i", gpu_idx_csv,
    ], capture_output=True, text=True)
    rows = []
    for line in out.stdout.strip().split("\n"):
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 4:
            rows.append({
                "gpu_idx": int(parts[0]),
                "gpu_util": float(parts[1]),
                "gpu_mem_used_mb": float(parts[2]),
                "gpu_mem_total_mb": float(parts[3]),
            })
    return rows


def sample_pid(pid: int) -> dict | None:
    try:
        with open(f"/proc/{pid}/status") as f:
            rss_kb = 0
            for line in f:
                if line.startswith("VmRSS:"):
                    rss_kb = int(line.split()[1])
                    break
        # CPU % via /proc/<pid>/stat — simplify: report accumulated user+sys jiffies
        with open(f"/proc/{pid}/stat") as f:
            s = f.read().split()
        # fields 14,15 are utime,stime (0-indexed: idx 13,14 but pid name has spaces; use rsplit)
        utime = int(s[13])
        stime = int(s[14])
        return {"pid": pid, "rss_mb": rss_kb / 1024.0,
                "cpu_jiffies_utime_stime": utime + stime}
    except FileNotFoundError:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pids", default="", help="comma-separated PIDs to sample (RSS, CPU)")
    ap.add_argument("--gpus", default="", help="comma-separated GPU indices")
    ap.add_argument("--out", required=True)
    ap.add_argument("--hz", type=float, default=2.0)
    args = ap.parse_args()

    period = 1.0 / args.hz
    gpu_csv = args.gpus
    pids = [int(p) for p in args.pids.split(",") if p.strip()]

    with open(args.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([
            "ts",
            "gpu_idx", "gpu_util_pct", "gpu_mem_used_mb", "gpu_mem_total_mb",
            "pid", "rss_mb", "cpu_jiffies",
        ])
        fh.flush()

        while RUN:
            t = time.time()
            # GPU rows
            if gpu_csv:
                try:
                    rows = sample_nvidia_smi(gpu_csv)
                except Exception as e:
                    rows = []
                for r in rows:
                    w.writerow([
                        f"{t:.3f}",
                        r["gpu_idx"], r["gpu_util"],
                        r["gpu_mem_used_mb"], r["gpu_mem_total_mb"],
                        "", "", "",
                    ])
            # Per-PID rows
            for pid in pids:
                rec = sample_pid(pid)
                if rec is not None:
                    w.writerow([
                        f"{t:.3f}",
                        "", "", "", "",
                        rec["pid"], f"{rec['rss_mb']:.1f}",
                        rec["cpu_jiffies_utime_stime"],
                    ])
            fh.flush()
            # Sleep
            elapsed = time.time() - t
            wait = period - elapsed
            if wait > 0:
                time.sleep(wait)


if __name__ == "__main__":
    main()
