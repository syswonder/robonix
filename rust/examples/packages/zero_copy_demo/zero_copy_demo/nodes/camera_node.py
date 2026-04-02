#!/usr/bin/env python3
"""Camera node — standalone process, SHM producer.

Simulates a V4L2/CSI camera driver. DMA writes directly into a
Robonix SHM buffer allocated through the control plane. Downstream
consumers in other processes get zero-copy access via mmap.

Launch:  rbnx start -p zero_copy_demo -n camera
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time

from ..common import FakeCameraDriver, connect_server
from ..rbnx_buffer import RobonixBufferManager, FORMAT_RGB8, _get_lib

from ..common import pb, pb_grpc  # noqa: F401


def main():
    ap = argparse.ArgumentParser(description="Robonix camera node (SHM producer)")
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    ap.add_argument("--width", type=int, default=int(os.environ.get("RBNX_WIDTH", "1920")))
    ap.add_argument("--height", type=int, default=int(os.environ.get("RBNX_HEIGHT", "1080")))
    ap.add_argument("--fps", type=float, default=float(os.environ.get("RBNX_FPS", "30")))
    ap.add_argument("--frames", type=int, default=int(os.environ.get("RBNX_FRAMES", "0")),
                    help="0 = infinite loop")
    ap.add_argument("--results-dir", default=None,
                    help="Write per-frame timing JSON (benchmark mode)")
    ap.add_argument("--start-delay", type=float, default=0,
                    help="Seconds to wait after SHM creation before producing (for consumer startup)")
    args = ap.parse_args()

    W, H, C = args.width, args.height, 3
    node_id = "com.robonix.demo.camera_sim"

    stub = connect_server(args.server)
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=node_id, namespace="robonix/prm/camera", kind="primitive",
    ))
    resp = stub.DeclareInterface(pb.DeclareInterfaceRequest(
        node_id=node_id, name="rgb",
        supported_transports=["shared_memory"],
        metadata_json=json.dumps({
            "width": W, "height": H, "channels": C,
            "format": FORMAT_RGB8, "memory_domain": "cpu",
            "msg_type": "robonix_msg/ZeroCopyFrame",
        }),
        contract_id="robonix/prm/camera/rgb",
    ))
    shm_name = resp.allocated_endpoint
    print(f"[camera] SHM endpoint: {shm_name}", file=sys.stderr)

    mgr = RobonixBufferManager()
    prod = mgr.allocate(shm_name, W, H, C, fmt=FORMAT_RGB8)
    prod_view = prod.as_numpy()
    driver = FakeCameraDriver(H, W, args.frames or 10000, target_view=prod_view)

    bench_mode = args.results_dir is not None
    frame_interval = 1.0 / args.fps if args.fps > 0 else 0.0
    running = True
    dma_ns: list[int] = []

    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    if args.start_delay > 0:
        print(f"[camera] Waiting {args.start_delay:.0f}s for consumers to start...",
              file=sys.stderr)
        time.sleep(args.start_delay)

    rate_str = f" @ {args.fps} fps" if args.fps > 0 else " (no rate limit)"
    print(f"[camera] Streaming {W}x{H}{rate_str}  (SHM={shm_name})", file=sys.stderr)
    seq = 0
    try:
        while running and (args.frames == 0 or seq < args.frames):
            t0 = time.monotonic()
            _, t_dma = driver.capture(seq)
            _get_lib().rbnx_buf_signal_write(mgr._ptr, prod.handle_id)
            dma_ns.append(t_dma)
            seq += 1
            elapsed = time.monotonic() - t0
            if frame_interval > elapsed:
                time.sleep(frame_interval - elapsed)
    finally:
        print(f"[camera] Stopping after {seq} frames", file=sys.stderr)
        if args.results_dir:
            import pathlib
            p = pathlib.Path(args.results_dir)
            p.mkdir(parents=True, exist_ok=True)
            with open(p / "rbnx_camera.json", "w") as f:
                json.dump({"dma_ns": dma_ns}, f)
        mgr.release(prod.handle_id)
        mgr.destroy()
        stub.UnregisterNode(pb.UnregisterNodeRequest(node_id=node_id))


if __name__ == "__main__":
    main()
