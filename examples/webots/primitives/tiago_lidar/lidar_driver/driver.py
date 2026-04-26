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
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for pg in (d / "rbnx-build" / "codegen" / "proto_gen", d / "proto_gen"):
            if pg.is_dir() and (pg / "atlas_legacy_pb2.py").exists():
                if str(pg) not in sys.path:
                    sys.path.insert(0, str(pg))
                return
        d = d.parent


def _ensure_mcp_types() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for mt in (d / "rbnx-build" / "codegen" / "robonix_mcp_types", d / "robonix_mcp_types"):
            if mt.is_dir() and (mt / "__init__.py").exists():
                if str(mt) not in sys.path:
                    sys.path.insert(0, str(mt))
                return
        d = d.parent


_ensure_proto_gen()
_ensure_mcp_types()

for _logger_name in (
    "mcp", "mcp.server", "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
    "httpx", "httpcore", "uvicorn", "uvicorn.access",
):
    logging.getLogger(_logger_name).setLevel(logging.WARNING)

import grpc
import atlas_legacy_pb2 as pb
import atlas_legacy_pb2_grpc as pb_grpc

import builtin_interfaces_mcp
import std_msgs_mcp
from sensor_msgs_mcp import LaserScan
from std_msgs_mcp import Empty

from mcp.server.fastmcp import FastMCP

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

@mcp.tool(name="snapshot")
def snapshot() -> dict:
    """Get the latest planar lidar scan as sensor_msgs/LaserScan.
    Contract: robonix/primitive/lidar/snapshot."""
    with _lock:
        ros_scan = _latest_scan_msg
    if ros_scan is None:
        return _laserscan_error("no lidar scan received yet").to_dict()
    return _ros_laserscan_to_mcp(ros_scan).to_dict()


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
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[tiago_lidar] heartbeat failed: {e}")


def _single_tool_meta(tool_name: str, description: str, input_schema: dict) -> str:
    return json.dumps({
        "tools": [{"name": tool_name, "description": description, "input_schema": input_schema}]
    })


def main() -> None:
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    port = int(os.environ.get("TIAGO_LIDAR_MCP_PORT", "50113"))
    node_id = os.environ.get("ROBONIX_NODE_ID", "com.robonix.primitive.tiago_lidar")

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    try:
        stub.RegisterNode(pb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/primitive/lidar",
            kind="primitive",
            skill_md="# tiago_lidar\nPlanar lidar snapshot.",
        ))
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="snapshot",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "snapshot",
                "Get the latest 2D lidar scan. Returns sensor_msgs/LaserScan.",
                Empty.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/primitive/lidar/snapshot",
        ))
        print(f"[tiago_lidar] registered node {node_id} → 1 cap on port {port}")
    except Exception as e:
        print(f"[tiago_lidar] WARN: atlas registration failed: {e}")

    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_ros2, daemon=True).start()

    print(f"[tiago_lidar] MCP HTTP serving on 0.0.0.0:{port}")
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
