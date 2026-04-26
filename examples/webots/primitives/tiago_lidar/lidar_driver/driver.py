#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago lidar driver — primitive cap.

Subscribes to the planar lidar topic (default /scanner), exposes:
  robonix/primitive/lidar/snapshot  → lidar_snapshot(Empty) → LaserScan

Env vars:
  ROBONIX_ATLAS          atlas endpoint (default 127.0.0.1:50051)
  TIAGO_LIDAR_MCP_PORT   MCP HTTP port (default 50113)
  TIAGO_SCAN_TOPIC       lidar topic (default /scanner)
"""
import json
import logging
import os
import sys
import threading
import time
from pathlib import Path


def _ensure_proto_gen() -> None:
    """Locate rbnx-build/codegen/proto_gen produced by `rbnx codegen
    --out-dir rbnx-build/codegen`. All build artefacts live under
    `<pkg>/rbnx-build/`; nothing should land at package root."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            if str(pg) not in sys.path:
                sys.path.insert(0, str(pg))
            return
        d = d.parent


def _ensure_mcp_types() -> None:
    """Locate rbnx-build/codegen/robonix_mcp_types from
    `rbnx codegen --mcp --out-dir rbnx-build/codegen`."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        mt = d / "rbnx-build" / "codegen" / "robonix_mcp_types"
        if mt.is_dir() and (mt / "__init__.py").exists():
            if str(mt) not in sys.path:
                sys.path.insert(0, str(mt))
            return
        d = d.parent


def _ensure_robonix_py() -> None:
    """Find pylib/robonix-py — host walk-up or `rbnx path` fallback."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for cand in (d / "pylib" / "robonix-py", d / "robonix-py"):
            if cand.is_dir() and (cand / "robonix_py" / "__init__.py").exists():
                if str(cand) not in sys.path:
                    sys.path.insert(0, str(cand))
                return
        d = d.parent
    import subprocess
    try:
        out = subprocess.run(
            ["rbnx", "path", "robonix-py"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        if out.returncode == 0:
            lib = Path(out.stdout.strip())
            if lib.is_dir() and str(lib) not in sys.path:
                sys.path.insert(0, str(lib))
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass


_ensure_proto_gen()
_ensure_mcp_types()
_ensure_robonix_py()

for _logger_name in (
    "mcp", "mcp.server", "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
    "httpx", "httpcore", "uvicorn", "uvicorn.access",
):
    logging.getLogger(_logger_name).setLevel(logging.WARNING)

import grpc
import atlas_pb2 as pb
import atlas_pb2_grpc as pb_grpc

import builtin_interfaces_mcp
import std_msgs_mcp
from sensor_msgs_mcp import LaserScan
from std_msgs_mcp import Empty

from mcp.server.fastmcp import FastMCP
from robonix_py import mcp_contract

# ── ROS2 lazy imports ────────────────────────────────────────────────────────

_rclpy = None
_LaserScan = None


def _import_ros2():
    global _rclpy, _LaserScan
    import rclpy  # type: ignore
    from sensor_msgs.msg import LaserScan as RosLaserScan  # type: ignore
    _rclpy = rclpy
    _LaserScan = RosLaserScan


# ── shared state ─────────────────────────────────────────────────────────────

mcp = FastMCP("tiago-lidar")

_lock = threading.Lock()
_latest_scan_msg = None
_ros_node = None


# ── conversion ───────────────────────────────────────────────────────────────

def _ros_laserscan_to_mcp(msg) -> LaserScan:
    h = msg.header
    stamp = builtin_interfaces_mcp.Time(sec=int(h.stamp.sec), nanosec=int(h.stamp.nanosec))
    header = std_msgs_mcp.Header(stamp=stamp, frame_id=str(h.frame_id))
    intensities = [float(x) for x in msg.intensities] if len(msg.intensities) else []
    return LaserScan(
        header=header,
        angle_min=float(msg.angle_min),
        angle_max=float(msg.angle_max),
        angle_increment=float(msg.angle_increment),
        time_increment=float(msg.time_increment),
        scan_time=float(msg.scan_time),
        range_min=float(msg.range_min),
        range_max=float(msg.range_max),
        ranges=[float(r) for r in msg.ranges],
        intensities=intensities,
    )


def _mcp_header_now(frame_id: str) -> std_msgs_mcp.Header:
    now = time.time()
    sec = int(now)
    ns = int((now % 1) * 1e9) % 1_000_000_000
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=ns),
        frame_id=frame_id,
    )


def _laserscan_error(message: str) -> LaserScan:
    fid = "error:" + message.replace("\n", " ")[:200]
    return LaserScan(header=_mcp_header_now(fid), ranges=[], intensities=[])


# ── ROS2 callback ────────────────────────────────────────────────────────────

def _on_scan(msg):
    global _latest_scan_msg
    with _lock:
        _latest_scan_msg = msg


# ── MCP tool ─────────────────────────────────────────────────────────────────

@mcp_contract(mcp, contract_id="robonix/primitive/lidar/snapshot")
def snapshot(msg: Empty) -> LaserScan:
    """Get the latest planar lidar scan. Returns sensor_msgs/LaserScan;
    `ranges[i]` is the distance (m) at angle `angle_min + i*angle_increment`.
    Useful for "obstacle in front?" / "where's the nearest open space?"
    Contract: robonix/primitive/lidar/snapshot."""
    _ = msg
    with _lock:
        ros_scan = _latest_scan_msg
    if ros_scan is None:
        return _laserscan_error("no lidar scan received yet")
    return _ros_laserscan_to_mcp(ros_scan)


# ── runtime wiring ───────────────────────────────────────────────────────────

def _start_ros2():
    global _ros_node
    _import_ros2()
    _rclpy.init()
    from rclpy.executors import SingleThreadedExecutor  # type: ignore

    node = _rclpy.create_node("tiago_lidar_driver")
    _ros_node = node
    scan_topic = os.environ.get("TIAGO_SCAN_TOPIC", "/scanner")
    node.create_subscription(_LaserScan, scan_topic, _on_scan, 1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print(f"[tiago_lidar] ROS2 ready: sub {scan_topic}")
    while _rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


def _heartbeat_loop(stub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.Heartbeat(pb.HeartbeatRequest(capability_id=node_id))
        except Exception as e:
            print(f"[tiago_lidar] heartbeat failed: {e}")


def _decl_mcp(stub, cap_id: str, contract_id: str, port: int, fn) -> None:
    """Atlas registration derived from the @mcp_contract handler:
    docstring → description, codegen input class → input_schema_json."""
    description = (fn.__doc__ or "").strip()
    input_cls = getattr(fn, "_robonix_input_cls", None)
    schema_json = json.dumps(
        input_cls.json_schema() if input_cls is not None
        else {"type": "object", "properties": {}, "required": []}
    )
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_MCP,
        endpoint=f"http://127.0.0.1:{port}/mcp/",
        params=pb.TransportParams(mcp=pb.McpParams(
            description=description,
            input_schema_json=schema_json,
        )),
    ))


def main() -> None:
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    port = int(os.environ.get("TIAGO_LIDAR_MCP_PORT", "50113"))
    cap_id = os.environ.get("ROBONIX_CAPABILITY_ID", "com.robonix.primitive.tiago_lidar")

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.AtlasStub(channel)

    try:
        pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
        md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=cap_id,
            namespace="robonix/primitive/lidar",
            capability_md_path=md_path,
        ))
        _decl_mcp(stub, cap_id, "robonix/primitive/lidar/snapshot", port, snapshot)
        print(f"[tiago_lidar] registered cap {cap_id} → 1 interface on port {port}")
    except Exception as e:
        print(f"[tiago_lidar] WARN: atlas registration failed: {e}")

    threading.Thread(target=_heartbeat_loop, args=(stub, cap_id), daemon=True).start()
    threading.Thread(target=_start_ros2, daemon=True).start()

    print(f"[tiago_lidar] MCP HTTP serving on 0.0.0.0:{port}")
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
