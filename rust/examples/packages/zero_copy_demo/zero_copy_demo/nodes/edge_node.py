#!/usr/bin/env python3
"""Sobel edge-detection node — CUDA IPC consumer.

Receives GPU tensor from YOLO node via CUDA IPC (cross-process GPU
memory sharing). Performs Sobel edge detection entirely on the GPU.

Launch:  rbnx start -p zero_copy_demo -n edge
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import signal
import sys
import time

import torch

from ..common import connect_server, sobel_gpu
from ..rbnx_buffer import (
    cuda_ipc_open_handle, cuda_ipc_close_handle,
    cuda_memcpy_d2d, cuda_device_sync,
)
from ..common import pb, pb_grpc  # noqa: F401


def _ns():
    return time.perf_counter_ns()


class IpcGpuView:
    """Wraps a CUDA IPC device pointer for repeated D2D reads."""

    def __init__(self, dev_ptr: int, h: int, w: int, c: int):
        self.dev_ptr = dev_ptr
        self.size = h * w * c
        self.tensor = torch.empty(h, w, c, dtype=torch.uint8, device="cuda")

    def read(self) -> torch.Tensor:
        cuda_memcpy_d2d(self.tensor.data_ptr(), self.dev_ptr, self.size)
        cuda_device_sync()
        return self.tensor


def main():
    ap = argparse.ArgumentParser(description="Robonix Sobel edge node (CUDA IPC consumer)")
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    ap.add_argument("--frames", type=int, default=int(os.environ.get("RBNX_FRAMES", "0")))
    ap.add_argument("--results-dir", default=None,
                    help="Write per-frame timing JSON (benchmark mode)")
    args = ap.parse_args()

    if not torch.cuda.is_available():
        print("[edge] FATAL: CUDA not available", file=sys.stderr)
        sys.exit(1)

    device = torch.device("cuda")
    node_id = "com.robonix.demo.edge_detector"

    stub = connect_server(args.server)
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=node_id, namespace="robonix/sys/perception", kind="service",
    ))

    neg_resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
        consumer_id=node_id, provider_node_id="com.robonix.demo.yolo_detector",
        interface_name="gpu_tensor", transport="shared_memory",
    ))
    yolo_meta = json.loads(neg_resp.metadata_json) if neg_resp.metadata_json else {}
    W = yolo_meta.get("width", 1920)
    H = yolo_meta.get("height", 1080)
    C = yolo_meta.get("channels", 3)
    ipc_handle_b64 = yolo_meta.get("cuda_ipc_handle_b64")
    gpu_buf_size = yolo_meta.get("gpu_buf_size", H * W * C)

    if not ipc_handle_b64:
        print("[edge] FATAL: No cuda_ipc_handle_b64 in channel metadata", file=sys.stderr)
        sys.exit(1)

    ipc_handle = base64.b64decode(ipc_handle_b64)
    dev_ptr = cuda_ipc_open_handle(ipc_handle)
    print(f"[edge] CUDA IPC opened: {W}x{H}x{C} ({gpu_buf_size} bytes), handle: {ipc_handle_b64}", file=sys.stderr)

    ipc_view = IpcGpuView(dev_ptr, H, W, C)

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    sobel_gpu(torch.randn(H, W, C, device=device).byte())
    torch.cuda.synchronize()

    processed = 0
    T = {"ipc_read_ns": [], "compute_ns": [], "total_ns": []}
    print("[edge] Processing...", file=sys.stderr)

    try:
        while running and (args.frames == 0 or processed < args.frames):
            t_total = _ns()

            t = _ns()
            gpu_frame = ipc_view.read()
            T["ipc_read_ns"].append(_ns() - t)

            t = _ns()
            with torch.no_grad():
                sobel_gpu(gpu_frame)
            torch.cuda.synchronize()
            T["compute_ns"].append(_ns() - t)

            T["total_ns"].append(_ns() - t_total)
            processed += 1
            if processed % 100 == 0:
                print(f"[edge] Processed {processed} frames (Sobel)", file=sys.stderr)
            if not args.results_dir:
                time.sleep(0.001)
    finally:
        print(f"[edge] Stopping after {processed} frames", file=sys.stderr)
        if args.results_dir:
            import pathlib
            p = pathlib.Path(args.results_dir)
            p.mkdir(parents=True, exist_ok=True)
            with open(p / "rbnx_edge.json", "w") as f:
                json.dump(T, f)
        cuda_ipc_close_handle(dev_ptr)
        stub.UnregisterNode(pb.UnregisterNodeRequest(node_id=node_id))


if __name__ == "__main__":
    main()
