#!/usr/bin/env python3
"""
Robonix Multi-Process Zero-Copy Benchmark
==========================================

Launches 3 independent processes (camera, yolo, edge) that communicate
via POSIX SHM + CUDA IPC, collects per-stage timing, and reports results.

Run:  python -m zero_copy_demo.benchmark [--frames 200]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


def _wait_all(procs: dict[str, subprocess.Popen], timeout: float = 300):
    """Wait for all subprocesses, kill stragglers on timeout."""
    deadline = time.monotonic() + timeout
    for name, p in procs.items():
        remaining = max(0, deadline - time.monotonic())
        try:
            p.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            print(f"[!] {name} timed out, killing", file=sys.stderr)
            p.kill()
            p.wait()
        if p.returncode != 0:
            print(f"[!] {name} exited with code {p.returncode}", file=sys.stderr)


def run(args) -> dict:
    W, H = args.width, args.height
    total_frames = args.warmup + args.frames
    frame_mb = W * H * 3 / 1e6
    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    gpu_name = "unknown"
    try:
        import torch
        if torch.cuda.is_available():
            gpu_name = f"{torch.cuda.get_device_name(0)} (CUDA {torch.version.cuda})"
    except ImportError:
        pass

    print(f"[+] GPU: {gpu_name}", file=sys.stderr)
    print(f"[+] Frame: {W}x{H} RGB8 ({frame_mb:.2f} MB)", file=sys.stderr)
    print(f"[+] Multi-process benchmark: {total_frames} frames "
          f"({args.warmup} warmup + {args.frames} timed)", file=sys.stderr)
    print(f"[+] Server: {args.server}", file=sys.stderr)

    base_args = [sys.executable, "-m"]
    common = ["--server", args.server, "--frames", str(total_frames),
              "--results-dir", str(results_dir)]

    # Camera registers SHM first, then waits --start-delay seconds for
    # YOLO to negotiate, load the model, and be ready to consume frames.
    # fps=30 keeps camera from outpacing YOLO (~25-30 ms/frame for inference).
    cam_delay = 10  # seconds: covers YOLO model load + Edge IPC setup

    print(f"\n[1/3] Starting camera_node (start-delay={cam_delay}s)...", file=sys.stderr)
    cam = subprocess.Popen(
        base_args + ["zero_copy_demo.nodes.camera_node",
                      "--width", str(W), "--height", str(H),
                      "--fps", "30",
                      "--start-delay", str(cam_delay)] + common,
        stderr=sys.stderr)
    time.sleep(1)

    print(f"[2/3] Starting yolo_node...", file=sys.stderr)
    yolo = subprocess.Popen(
        base_args + ["zero_copy_demo.nodes.yolo_node"] + common,
        stderr=sys.stderr)
    time.sleep(5)

    print(f"[3/3] Starting edge_node...", file=sys.stderr)
    edge = subprocess.Popen(
        base_args + ["zero_copy_demo.nodes.edge_node"] + common,
        stderr=sys.stderr)

    procs = {"camera": cam, "yolo": yolo, "edge": edge}
    _wait_all(procs)

    # ── Collect results ──
    cam_file = results_dir / "rbnx_camera.json"
    yolo_file = results_dir / "rbnx_yolo.json"
    edge_file = results_dir / "rbnx_edge.json"

    for f in (cam_file, yolo_file, edge_file):
        if not f.exists():
            print(f"[!] Missing results: {f}", file=sys.stderr)
            sys.exit(1)

    with open(cam_file) as f:
        cam_data = json.load(f)
    with open(yolo_file) as f:
        yolo_data = json.load(f)
    with open(edge_file) as f:
        edge_data = json.load(f)

    # Skip warmup frames from each node's timing
    w = args.warmup
    dma = np.array(cam_data["dma_ns"][w:], dtype=np.float64)
    y_read = np.array(yolo_data["read_ns"][w:], dtype=np.float64)
    y_h2d = np.array(yolo_data["h2d_ns"][w:], dtype=np.float64)
    y_pre = np.array(yolo_data["preprocess_ns"][w:], dtype=np.float64)
    y_inf = np.array(yolo_data["infer_ns"][w:], dtype=np.float64)
    y_d2d = np.array(yolo_data["d2d_ns"][w:], dtype=np.float64)
    e_ipc = np.array(edge_data["ipc_read_ns"][w:], dtype=np.float64)
    e_cmp = np.array(edge_data["compute_ns"][w:], dtype=np.float64)

    n = min(len(dma), len(y_read), len(e_ipc))
    dma, y_read, y_h2d, y_pre, y_inf, y_d2d = (
        dma[:n], y_read[:n], y_h2d[:n], y_pre[:n], y_inf[:n], y_d2d[:n])
    e_ipc, e_cmp = e_ipc[:n], e_cmp[:n]

    total_ns = dma + y_read + y_h2d + y_pre + y_inf + y_d2d + e_ipc + e_cmp

    def us(a):
        return float(np.mean(a) / 1e3)

    report = {
        "path": "robonix",
        "config": {"width": W, "height": H, "frames": int(n),
                   "frame_mb": frame_mb, "gpu": gpu_name},
        "driver_dma_us": us(dma),
        "node_a": {
            "read_us": us(y_read),
            "h2d_us": us(y_h2d),
            "preprocess_gpu_us": us(y_pre),
            "infer_us": us(y_inf),
            "d2d_us": us(y_d2d),
        },
        "node_b": {
            "gpu_read_us": us(e_ipc),
            "compute_us": us(e_cmp),
        },
        "total_us": us(total_ns),
        "total_std_us": float(np.std(total_ns / 1e3)),
    }

    sep = "=" * 72
    print(f"\n{sep}", file=sys.stderr)
    print(f"  ROBONIX Multi-Process Pipeline -- {W}x{H} RGB8 ({frame_mb:.2f} MB)", file=sys.stderr)
    print(f"  {n} frames  |  GPU: {gpu_name}  |  server: {args.server}", file=sys.stderr)
    print(f"  3 independent OS processes (camera / yolo / edge)", file=sys.stderr)
    print(sep, file=sys.stderr)
    for label, v, tag in [
        ("1. Camera: DMA -> SHM",              report["driver_dma_us"],               "WRITE"),
        ("2. YOLO: zero-copy read (mmap)",      report["node_a"]["read_us"],           "FREE"),
        ("3. YOLO: pinned H2D (async DMA)",     report["node_a"]["h2d_us"],            "DMA"),
        ("4. YOLO: preprocess (ON GPU)",        report["node_a"]["preprocess_gpu_us"], "FREE"),
        ("5. YOLO: inference",                  report["node_a"]["infer_us"],          ""),
        ("6. YOLO: D2D copy to IPC buffer",     report["node_a"]["d2d_us"],            "D2D"),
        ("7. Sobel: CUDA IPC D2D read",         report["node_b"]["gpu_read_us"],       "D2D"),
        ("8. Sobel: compute (GPU)",             report["node_b"]["compute_us"],        ""),
    ]:
        mark = f"  <- {tag}" if tag else ""
        print(f"  {label:<42s} {v:>8.0f} us{mark}", file=sys.stderr)
    print(f"  {'-' * 68}", file=sys.stderr)
    print(f"  TOTAL  {report['total_us']:>8.0f} us  (sigma = {report['total_std_us']:.1f} us)",
          file=sys.stderr)
    print(f"  Data copies: 1 H2D DMA + 2 D2D (all GPU-side)  |  CPU copies: 0",
          file=sys.stderr)
    print(f"{sep}\n", file=sys.stderr)

    return report


def main():
    ap = argparse.ArgumentParser(description="Robonix multi-process pipeline benchmark")
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--save-dir", "--results-dir", default="results", dest="results_dir")
    args = ap.parse_args()
    report = run(args)
    if args.json:
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
