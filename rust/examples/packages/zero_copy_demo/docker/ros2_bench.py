#!/usr/bin/env python3
"""
ROS 2 / FastDDS Multi-Process Pipeline Benchmark  (runs inside Docker)
======================================================================

Three independent processes communicate via ROS 2 DDS topics:
  Process 1 (camera):  publishes sensor_msgs/Image
  Process 2 (yolo):    subscribes, deserializes, YOLO inference
  Process 3 (edge):    subscribes, deserializes, Sobel compute

Each process measures per-frame latency. The orchestrator (--role omitted)
launches all three and produces a combined report.

Run inside container:
    python3 /bench/ros2_bench.py --server 127.0.0.1:50051 --frames 200
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

# ── Per-role implementations (each runs in its own OS process) ────────

def _run_camera(args):
    """Camera publisher — separate process."""
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image

    sys.path.insert(0, "/bench")
    from common import FakeCameraDriver

    QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)

    W, H, C = args.width, args.height, 3
    total = args.warmup + args.frames

    driver = FakeCameraDriver(H, W, total)

    rclpy.init()
    node = rclpy.create_node("bench_camera_pub")
    pub = node.create_publisher(Image, "/camera/image_raw", QOS)

    print(f"[ros2-camera] Waiting for 2 subscribers...", file=sys.stderr)
    while pub.get_subscription_count() < 2:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"[ros2-camera] Publishing {total} frames ({W}x{H})", file=sys.stderr)
    dma_ns, serialize_ns = [], []

    for seq in range(total):
        buf, t_dma = driver.capture(seq)
        dma_ns.append(t_dma)

        msg = Image()
        msg.header.frame_id = str(seq)
        msg.height, msg.width = H, W
        msg.encoding = "rgb8"
        msg.step = W * 3
        msg.is_bigendian = False

        t = time.perf_counter_ns()
        msg.data = buf.tobytes()
        serialize_ns.append(time.perf_counter_ns() - t)

        pub.publish(msg)
        time.sleep(0.001)

    time.sleep(1)
    rclpy.shutdown()
    print(f"[ros2-camera] Done, {total} frames published", file=sys.stderr)

    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    with open(results_dir / "ros2_camera.json", "w") as f:
        json.dump({"dma_ns": dma_ns, "serialize_ns": serialize_ns}, f)


def _run_subscriber(args, role: str):
    """YOLO or Edge subscriber — separate process."""
    import rclpy
    import torch
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image

    sys.path.insert(0, "/bench")
    from common import pad32, sobel_gpu, warmup_models

    QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)

    total = args.warmup + args.frames
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu
                          else "cpu")

    yolo_model = None
    work_fn = None
    if role == "yolo":
        from ultralytics import YOLO
        print(f"[ros2-yolo] Loading YOLOv8n...", file=sys.stderr)
        yolo_model = YOLO("yolov8n.pt").model.to(device).eval()
        warmup_models(device, yolo_model, args.height, args.width)
        print(f"[ros2-yolo] Model ready, waiting for frames", file=sys.stderr)
    else:
        work_fn = sobel_gpu
        sobel_gpu(torch.randn(args.height, args.width, 3, device=device).byte())
        if device.type == "cuda":
            torch.cuda.synchronize()
        print(f"[ros2-edge] Ready, waiting for frames", file=sys.stderr)

    results: list[dict] = []
    done = False

    def callback(msg: Image):
        nonlocal done
        if done:
            return
        t_start = time.perf_counter_ns()

        t = time.perf_counter_ns()
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            msg.height, msg.width, 3).copy()
        t_deser = time.perf_counter_ns() - t

        t = time.perf_counter_ns()
        tensor_cpu = torch.from_numpy(arr).permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        t_preprocess = time.perf_counter_ns() - t

        t = time.perf_counter_ns()
        tensor_gpu = tensor_cpu.to(device)
        if device.type == "cuda":
            torch.cuda.synchronize()
        t_h2d = time.perf_counter_ns() - t

        t = time.perf_counter_ns()
        if yolo_model is not None:
            with torch.no_grad():
                yolo_model(pad32(tensor_gpu))
        elif work_fn is not None:
            with torch.no_grad():
                work_fn(tensor_gpu.squeeze(0).permute(1, 2, 0).mul_(255.0).byte())
        if device.type == "cuda":
            torch.cuda.synchronize()
        t_compute = time.perf_counter_ns() - t

        results.append({
            "deser_ns": t_deser,
            "preprocess_ns": t_preprocess,
            "h2d_ns": t_h2d,
            "compute_ns": t_compute,
            "total_ns": time.perf_counter_ns() - t_start,
        })

        if len(results) >= total:
            done = True

    rclpy.init()
    node = rclpy.create_node(f"bench_{role}_sub")
    node.create_subscription(Image, "/camera/image_raw", callback, QOS)

    while not done:
        rclpy.spin_once(node, timeout_sec=0.5)

    rclpy.shutdown()
    print(f"[ros2-{role}] Done, {len(results)} frames processed", file=sys.stderr)

    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    with open(results_dir / f"ros2_{role}.json", "w") as f:
        json.dump(results, f)


# ── Orchestrator ──────────────────────────────────────────────────────

def _run_orchestrate(args):
    """Launch 3 ROS 2 processes, collect timing, produce report."""
    import torch

    W, H = args.width, args.height
    total = args.warmup + args.frames
    frame_mb = W * H * 3 / 1e6
    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    gpu_name = "CPU"
    if not args.cpu and torch.cuda.is_available():
        gpu_name = f"{torch.cuda.get_device_name(0)} (CUDA {torch.version.cuda})"
    print(f"[+] GPU: {gpu_name}", file=sys.stderr)
    print(f"[+] Frame: {W}x{H} RGB8 ({frame_mb:.2f} MB)", file=sys.stderr)

    # Register nodes with robonix-server
    sys.path.insert(0, str(Path("/bench/proto_gen")))
    sys.path.insert(0, "/bench")
    from common import connect_server
    import robonix_runtime_pb2 as pb

    server_addr = args.server.removeprefix("http://").removeprefix("https://")
    print(f"[+] Connecting to robonix-server @ {server_addr}...", file=sys.stderr)
    stub = connect_server(server_addr)

    cam_id = "com.robonix.bench.ros2.camera"
    yolo_id = "com.robonix.bench.ros2.yolo"
    edge_id = "com.robonix.bench.ros2.edge"

    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=cam_id, namespace="robonix/prm/camera", kind="primitive"))
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        node_id=cam_id, name="rgb", supported_transports=["ros2"],
        metadata_json=f'{{"width":{W},"height":{H},"encoding":"rgb8","msg_type":"sensor_msgs/Image"}}',
        abstract_interface_id="robonix/prm/camera/rgb"))
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=yolo_id, namespace="robonix/sys/perception", kind="service"))
    stub.NegotiateChannel(pb.NegotiateChannelRequest(
        consumer_id=yolo_id, provider_node_id=cam_id,
        interface_name="rgb", transport="ros2"))
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=edge_id, namespace="robonix/sys/perception", kind="service"))
    stub.NegotiateChannel(pb.NegotiateChannelRequest(
        consumer_id=edge_id, provider_node_id=cam_id,
        interface_name="rgb", transport="ros2"))
    print(f"[+] Nodes registered (transport=ros2, 2 channels: cam→yolo, cam→edge)",
          file=sys.stderr)

    common_args = ["--width", str(W), "--height", str(H),
                    "--frames", str(args.frames), "--warmup", str(args.warmup),
                    "--results-dir", str(results_dir), "--server", args.server]
    if args.cpu:
        common_args.append("--cpu")

    # Start subscribers first, then publisher
    print(f"\n[1/3] Starting ros2-yolo process...", file=sys.stderr)
    yolo_proc = subprocess.Popen(
        [sys.executable, __file__, "--role", "yolo"] + common_args,
        stderr=sys.stderr)

    print(f"[2/3] Starting ros2-edge process...", file=sys.stderr)
    edge_proc = subprocess.Popen(
        [sys.executable, __file__, "--role", "edge"] + common_args,
        stderr=sys.stderr)

    time.sleep(3)

    print(f"[3/3] Starting ros2-camera process...", file=sys.stderr)
    cam_proc = subprocess.Popen(
        [sys.executable, __file__, "--role", "camera"] + common_args,
        stderr=sys.stderr)

    for name, p in [("camera", cam_proc), ("yolo", yolo_proc), ("edge", edge_proc)]:
        try:
            p.wait(timeout=300)
        except subprocess.TimeoutExpired:
            print(f"[!] {name} timed out, killing", file=sys.stderr)
            p.kill(); p.wait()
        if p.returncode != 0:
            print(f"[!] {name} exited with code {p.returncode}", file=sys.stderr)

    # Unregister
    for nid in (cam_id, yolo_id, edge_id):
        stub.UnregisterNode(pb.UnregisterNodeRequest(node_id=nid))

    # ── Collect results ──
    cam_data = json.load(open(results_dir / "ros2_camera.json"))
    yolo_data = json.load(open(results_dir / "ros2_yolo.json"))
    edge_data = json.load(open(results_dir / "ros2_edge.json"))

    w = args.warmup
    dma_us = [v / 1e3 for v in cam_data["dma_ns"][w:]]
    ser_us = [v / 1e3 for v in cam_data["serialize_ns"][w:]]
    ra = yolo_data[w:]
    rb = edge_data[w:]
    n = min(len(dma_us), len(ser_us), len(ra), len(rb))
    dma_us, ser_us, ra, rb = dma_us[:n], ser_us[:n], ra[:n], rb[:n]

    def avg(lst):
        return sum(lst) / len(lst) if lst else 0.0

    report = {
        "path": "ros2_fastdds",
        "config": {"width": W, "height": H, "frames": n,
                   "frame_mb": frame_mb, "gpu": gpu_name},
        "driver_dma_us": avg(dma_us),
        "serialize_us": avg(ser_us),
        "node_a": {
            "deser_us": avg([r["deser_ns"] / 1e3 for r in ra]),
            "preprocess_us": avg([r["preprocess_ns"] / 1e3 for r in ra]),
            "h2d_us": avg([r["h2d_ns"] / 1e3 for r in ra]),
            "compute_us": avg([r["compute_ns"] / 1e3 for r in ra]),
            "total_us": avg([r["total_ns"] / 1e3 for r in ra]),
        },
        "node_b": {
            "deser_us": avg([r["deser_ns"] / 1e3 for r in rb]),
            "preprocess_us": avg([r["preprocess_ns"] / 1e3 for r in rb]),
            "h2d_us": avg([r["h2d_ns"] / 1e3 for r in rb]),
            "compute_us": avg([r["compute_ns"] / 1e3 for r in rb]),
            "total_us": avg([r["total_ns"] / 1e3 for r in rb]),
        },
    }
    all_total = [dma_us[i] + ser_us[i] + ra[i]["total_ns"] / 1e3
                 + rb[i]["total_ns"] / 1e3 for i in range(n)]
    report["total_us"] = avg(all_total)
    report["total_std_us"] = float(np.std(all_total))

    sep = "=" * 72
    print(f"\n{sep}", file=sys.stderr)
    print(f"  ROS 2 / FastDDS Multi-Process -- {W}x{H} RGB8 ({frame_mb:.2f} MB)",
          file=sys.stderr)
    print(f"  {n} frames  |  GPU: {gpu_name}  |  server: {server_addr}", file=sys.stderr)
    print(f"  3 independent OS processes (camera / yolo / edge)", file=sys.stderr)
    print(sep, file=sys.stderr)
    cc = 0
    for label, v, tag in [
        ("1.  Camera: DMA -> internal buffer",       report["driver_dma_us"],           ""),
        ("2.  Camera: serialize (tobytes)",           report["serialize_us"],            "COPY"),
        ("3.  YOLO: DDS deserialize + copy",         report["node_a"]["deser_us"],      "COPY"),
        ("4.  YOLO: preprocess (CPU)",               report["node_a"]["preprocess_us"], "COPY"),
        ("5.  YOLO: CPU->GPU (pageable H2D)",        report["node_a"]["h2d_us"],        "COPY"),
        ("6.  YOLO: inference",                      report["node_a"]["compute_us"],    ""),
        ("7.  Sobel: DDS deserialize + copy",        report["node_b"]["deser_us"],      "COPY"),
        ("8.  Sobel: preprocess (CPU)",              report["node_b"]["preprocess_us"], "COPY"),
        ("9.  Sobel: CPU->GPU (pageable H2D)",       report["node_b"]["h2d_us"],        "COPY"),
        ("10. Sobel: compute",                       report["node_b"]["compute_us"],    ""),
    ]:
        mark = f"  <- {tag}" if tag else ""
        print(f"  {label:<45s} {v:>8.0f} us{mark}", file=sys.stderr)
        if tag == "COPY":
            cc += 1
    print(f"  {'-' * 68}", file=sys.stderr)
    print(f"  TOTAL  {report['total_us']:>8.0f} us  (sigma = {report['total_std_us']:.1f} us)",
          file=sys.stderr)
    print(f"  Data copies: {cc}  |  CPU BW: ~{frame_mb * cc:.1f} MB/frame", file=sys.stderr)
    print(f"{sep}\n", file=sys.stderr)

    print(json.dumps(report, indent=2))


# ── Entry point ───────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="ROS 2 multi-process benchmark")
    ap.add_argument("--role", choices=["camera", "yolo", "edge"], default=None,
                    help="Internal: run a single node process")
    ap.add_argument("--server", default=os.environ.get("ROBONIX_SERVER", "127.0.0.1:50051"))
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--save-dir", "--results-dir", default="/results", dest="results_dir")
    args = ap.parse_args()

    if args.role == "camera":
        _run_camera(args)
    elif args.role in ("yolo", "edge"):
        _run_subscriber(args, args.role)
    else:
        _run_orchestrate(args)


if __name__ == "__main__":
    main()
