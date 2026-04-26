#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago camera driver — primitive cap.

Subscribes to RGB + depth image topics, exposes two on-demand MCP tools:
  robonix/primitive/camera/snapshot        → camera_snapshot(Empty) → Image (JPEG)
  robonix/primitive/camera/depth_snapshot  → camera_depth_snapshot(Empty) → Image (JPEG)

Env vars:
  ROBONIX_ATLAS              atlas endpoint (default 127.0.0.1:50051)
  TIAGO_CAMERA_MCP_PORT      MCP HTTP port (default 50112)
  TIAGO_RGB_TOPIC            ROS2 topic for RGB (default /head_front_camera/rgb/image_raw)
  TIAGO_DEPTH_TOPIC          ROS2 topic for depth (default /head_front_camera/depth_registered/image_raw)
  TIAGO_RGB_FRAME_ID         frame_id stamped on returned RGB Image
  TIAGO_DEPTH_FRAME_ID       frame_id stamped on returned depth Image
"""
import json
import logging
import os
import sys
import threading
import time
from io import BytesIO
from pathlib import Path

import numpy as np


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


def _ensure_robonix_py() -> None:
    """Find the robonix_py helper lib (sibling pylib/robonix-py/ on host
    or /robonix_pkgs/pylib/robonix-py/ inside the sim container) by
    walking up from this driver. Falls back to `rbnx path` only if a
    walk-up doesn't turn it up — that fallback never fires inside the
    container because rbnx isn't installed there."""
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
import atlas_legacy_pb2 as pb
import atlas_legacy_pb2_grpc as pb_grpc

import builtin_interfaces_mcp
import std_msgs_mcp
from sensor_msgs_mcp import Image
from std_msgs_mcp import Empty

from robonix_py import mcp_contract
from mcp.server.fastmcp import FastMCP

# ── ROS2 lazy imports ────────────────────────────────────────────────────────

_rclpy = None
_Image = None


def _import_ros2():
    global _rclpy, _Image
    import rclpy  # type: ignore
    from sensor_msgs.msg import Image as RosImage  # type: ignore
    _rclpy = rclpy
    _Image = RosImage


# ── shared state ─────────────────────────────────────────────────────────────

mcp = FastMCP("tiago-camera")

_lock = threading.Lock()
_latest_rgb: bytes | None = None
_latest_depth: bytes | None = None
_ros_node = None


# ── conversion helpers ───────────────────────────────────────────────────────

def _ros2_image_to_jpeg(msg) -> bytes:
    h, w = msg.height, msg.width
    encoding = msg.encoding.lower()
    if encoding == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif encoding == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
    elif encoding == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif encoding == "16uc1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = (raw / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else np.zeros((h, w), np.uint8)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif encoding == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        valid = np.isfinite(raw)
        if valid.any():
            mn, mx = raw[valid].min(), raw[valid].max()
            norm = np.where(valid, (raw - mn) / max(mx - mn, 1e-6) * 255, 0).astype(np.uint8)
        else:
            norm = np.zeros((h, w), np.uint8)
        arr = np.stack([norm, norm, norm], axis=-1)
    else:
        raise ValueError(f"unsupported image encoding: {encoding}")

    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.fromarray(np.ascontiguousarray(arr)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _mcp_header_now(frame_id: str) -> std_msgs_mcp.Header:
    now = time.time()
    sec = int(now)
    ns = int((now % 1) * 1e9) % 1_000_000_000
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=ns),
        frame_id=frame_id,
    )


def _image_error(message: str) -> Image:
    raw = message.encode("utf-8")
    return Image(
        header=_mcp_header_now("tiago_camera_error"),
        height=0, width=0,
        encoding="error",
        is_bigendian=0,
        step=len(raw),
        data=raw,
    )


def _jpeg_to_image_mcp(jpg: bytes, frame_id: str) -> Image:
    from PIL import Image as PILImage
    im = PILImage.open(BytesIO(jpg))
    w, h = im.size
    return Image(
        header=_mcp_header_now(frame_id),
        height=h, width=w,
        encoding="jpeg",
        is_bigendian=0,
        step=len(jpg),
        data=jpg,
    )


# ── ROS2 callbacks ───────────────────────────────────────────────────────────

def _on_rgb(msg):
    global _latest_rgb
    try:
        with _lock:
            _latest_rgb = _ros2_image_to_jpeg(msg)
    except Exception as e:
        print(f"[tiago_camera] RGB conversion error: {e}")


def _on_depth(msg):
    global _latest_depth
    try:
        with _lock:
            _latest_depth = _ros2_image_to_jpeg(msg)
    except Exception as e:
        print(f"[tiago_camera] Depth conversion error: {e}")


# ── MCP tools ────────────────────────────────────────────────────────────────

@mcp_contract(mcp, contract_id="robonix/primitive/camera/snapshot")
def snapshot(msg: Empty) -> Image:
    """Get the current RGB head-camera frame as a JPEG-encoded sensor_msgs/Image.
    Contract: robonix/primitive/camera/snapshot."""
    _ = msg
    with _lock:
        data = _latest_rgb
    if data is None:
        return _image_error("no RGB image received yet")
    return _jpeg_to_image_mcp(
        data, os.environ.get("TIAGO_RGB_FRAME_ID", "head_front_camera_rgb_optical_frame")
    )


@mcp_contract(mcp, contract_id="robonix/primitive/camera/depth_snapshot")
def depth_snapshot(msg: Empty) -> Image:
    """Get the current depth head-camera frame as a JPEG-encoded sensor_msgs/Image
    (depth normalized to grayscale).
    Contract: robonix/primitive/camera/depth_snapshot."""
    _ = msg
    with _lock:
        data = _latest_depth
    if data is None:
        return _image_error("no depth image received yet")
    return _jpeg_to_image_mcp(
        data, os.environ.get("TIAGO_DEPTH_FRAME_ID", "head_front_camera_depth_optical_frame")
    )


# ── runtime wiring ───────────────────────────────────────────────────────────

def _start_ros2():
    global _ros_node
    _import_ros2()
    _rclpy.init()
    from rclpy.executors import SingleThreadedExecutor  # type: ignore

    node = _rclpy.create_node("tiago_camera_driver")
    _ros_node = node
    rgb_topic = os.environ.get("TIAGO_RGB_TOPIC", "/head_front_camera/rgb/image_raw")
    depth_topic = os.environ.get("TIAGO_DEPTH_TOPIC", "/head_front_camera/depth_registered/image_raw")
    node.create_subscription(_Image, rgb_topic, _on_rgb, 1)
    node.create_subscription(_Image, depth_topic, _on_depth, 1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print(f"[tiago_camera] ROS2 ready: sub {rgb_topic} + {depth_topic}")
    while _rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


def _heartbeat_loop(stub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[tiago_camera] heartbeat failed: {e}")


def _single_tool_meta(tool_name: str, description: str, input_schema: dict) -> str:
    return json.dumps({
        "tools": [{"name": tool_name, "description": description, "input_schema": input_schema}]
    })


def main() -> None:
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    port = int(os.environ.get("TIAGO_CAMERA_MCP_PORT", "50112"))
    node_id = os.environ.get("ROBONIX_NODE_ID", "com.robonix.primitive.tiago_camera")

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    try:
        stub.RegisterNode(pb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/primitive/camera",
            kind="primitive",
            skill_md="# tiago_camera\nHead camera RGB + depth snapshots.",
        ))
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="snapshot",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "snapshot",
                "Get current RGB head-camera frame as sensor_msgs/Image (JPEG in data).",
                Empty.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/primitive/camera/snapshot",
        ))
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="depth_snapshot",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "depth_snapshot",
                "Get current depth head-camera frame as grayscale-JPEG sensor_msgs/Image.",
                Empty.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/primitive/camera/depth_snapshot",
        ))
        print(f"[tiago_camera] registered node {node_id} → 2 caps on port {port}")
    except Exception as e:
        print(f"[tiago_camera] WARN: atlas registration failed: {e}")

    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_ros2, daemon=True).start()

    print(f"[tiago_camera] MCP HTTP serving on 0.0.0.0:{port}")
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
