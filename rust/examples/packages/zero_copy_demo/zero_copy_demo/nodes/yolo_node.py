#!/usr/bin/env python3
"""YOLO node — SHM consumer, CUDA IPC producer.

Reads frames from camera's SHM buffer (zero-copy mmap), performs
pinned async H2D + GPU preprocessing + YOLOv8 inference, then exports
the GPU tensor via CUDA IPC so downstream nodes (e.g. edge_node)
can access GPU VRAM directly without any CPU copy.

Launch:  rbnx start -p zero_copy_demo -n yolo
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

from ..common import connect_server, pad32, warmup_models
from ..rbnx_buffer import (
    RobonixBufferManager, FORMAT_RGB8,
    cuda_ipc_get_handle, cuda_device_malloc, cuda_device_free,
    cuda_memcpy_d2d, cuda_device_sync,
)
from ..common import pb, pb_grpc  # noqa: F401


def _ns():
    return time.perf_counter_ns()


def main():
    ap = argparse.ArgumentParser(description="Robonix YOLO node (SHM consumer, CUDA IPC producer)")
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    ap.add_argument("--frames", type=int, default=int(os.environ.get("RBNX_FRAMES", "0")))
    ap.add_argument("--results-dir", default=None,
                    help="Write per-frame timing JSON (benchmark mode)")
    args = ap.parse_args()

    if not torch.cuda.is_available():
        print("[yolo] FATAL: CUDA not available", file=sys.stderr)
        sys.exit(1)

    device = torch.device("cuda")
    node_id = "com.robonix.demo.yolo_detector"

    stub = connect_server(args.server)
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=node_id, namespace="robonix/sys/perception", kind="service",
    ))

    neg_resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
        consumer_id=node_id, provider_node_id="com.robonix.demo.camera_sim",
        interface_name="rgb", transport="shared_memory",
    ))
    cam_shm = neg_resp.endpoint
    cam_meta = json.loads(neg_resp.metadata_json) if neg_resp.metadata_json else {}
    W = cam_meta.get("width", 1920)
    H = cam_meta.get("height", 1080)
    C = cam_meta.get("channels", 3)
    print(f"[yolo] Camera SHM: {cam_shm}  ({W}x{H}x{C})", file=sys.stderr)

    gpu_buf_size = H * W * C
    gpu_out_ptr = cuda_device_malloc(gpu_buf_size)
    ipc_handle = cuda_ipc_get_handle(gpu_out_ptr)
    ipc_handle_b64 = base64.b64encode(ipc_handle).decode("ascii")

    print(f"[yolo] buffer size: {gpu_buf_size}", file=sys.stderr)
    print(f"[yolo] cuda ipc handle: {ipc_handle_b64}", file=sys.stderr)

    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        node_id=node_id, name="gpu_tensor",
        supported_transports=["shared_memory"],
        metadata_json=json.dumps({
            "width": W, "height": H, "channels": C,
            "format": FORMAT_RGB8, "memory_domain": "gpu",
            "cuda_ipc_handle_b64": ipc_handle_b64,
            "gpu_buf_size": gpu_buf_size,
            "msg_type": "robonix_msg/ZeroCopyFrame",
        }),
        contract_id="robonix/sys/perception/yolo/gpu_tensor",
    ))

    mgr = RobonixBufferManager()
    cons = mgr.open(cam_shm, pin_for_gpu=True)
    cons_view = cons.as_numpy()
    print(f"[yolo] Opened camera SHM, pinned={cons.is_pinned}", file=sys.stderr)

    print("[yolo] Loading YOLOv8n...", file=sys.stderr)
    from ultralytics import YOLO
    yolo_model = YOLO("yolov8n.pt").model.to(device).eval()
    warmup_models(device, yolo_model, H, W)
    print("[yolo] Model ready", file=sys.stderr)

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    last_seq = 0
    processed = 0
    T = {"read_ns": [], "h2d_ns": [], "preprocess_ns": [],
         "infer_ns": [], "d2d_ns": [], "total_ns": []}

    print("[yolo] Waiting for frames...", file=sys.stderr)
    try:
        while running and (args.frames == 0 or processed < args.frames):
            cur_seq = cons.seq
            if cur_seq <= last_seq:
                time.sleep(0.0005)
                continue
            last_seq = cur_seq
            t_total = _ns()

            t = _ns()
            tensor_u8 = torch.as_tensor(cons_view)
            T["read_ns"].append(_ns() - t)

            t = _ns()
            gpu_u8 = tensor_u8.to(device, non_blocking=True)
            torch.cuda.synchronize()
            T["h2d_ns"].append(_ns() - t)

            t = _ns()
            gpu_a = gpu_u8.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
            T["preprocess_ns"].append(_ns() - t)

            t = _ns()
            with torch.no_grad():
                yolo_model(pad32(gpu_a))
            torch.cuda.synchronize()
            T["infer_ns"].append(_ns() - t)

            t = _ns()
            gpu_u8_contig = gpu_u8.contiguous()
            cuda_memcpy_d2d(gpu_out_ptr, gpu_u8_contig.data_ptr(), gpu_buf_size)
            cuda_device_sync()
            T["d2d_ns"].append(_ns() - t)

            T["total_ns"].append(_ns() - t_total)
            processed += 1
            if processed % 100 == 0:
                print(f"[yolo] Processed {processed} frames", file=sys.stderr)
    finally:
        print(f"[yolo] Stopping after {processed} frames", file=sys.stderr)
        if args.results_dir:
            import pathlib
            p = pathlib.Path(args.results_dir)
            p.mkdir(parents=True, exist_ok=True)
            with open(p / "rbnx_yolo.json", "w") as f:
                json.dump(T, f)
        mgr.detach(cons.handle_id)
        mgr.destroy()
        cuda_device_free(gpu_out_ptr)
        stub.UnregisterNode(pb.UnregisterNodeRequest(node_id=node_id))


if __name__ == "__main__":
    main()
