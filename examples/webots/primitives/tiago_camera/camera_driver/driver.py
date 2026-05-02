#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago camera driver — primitive cap.

Subscribes to RGB + depth image topics, exposes:
  robonix/primitive/camera/snapshot        → MCP snapshot(Empty) → Image (JPEG)
  robonix/primitive/camera/depth_snapshot  → MCP depth_snapshot(Empty) → Image (JPEG)
  robonix/primitive/camera/driver          → gRPC LifecycleDriver.Driver(INIT/...)

The `/driver` interface gates `rbnx boot`: INIT here waits for the first
RGB frame to arrive, so by the time the LLM gets `camera_snapshot` in its
tool list we know the topic is actually delivering data.

Env vars:
  ROBONIX_ATLAS              atlas endpoint (default 127.0.0.1:50051)
  TIAGO_CAMERA_MCP_PORT      MCP HTTP port (default 50112)
  TIAGO_CAMERA_DRIVER_PORT   LifecycleDriver gRPC port (default 50212)
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
    """Find pylib/robonix-py — host-side via `rbnx path`, in-container via
    walk-up from this file. Provides @mcp_contract."""
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
from sensor_msgs_mcp import Image
from std_msgs_mcp import Empty

from mcp.server.fastmcp import FastMCP
from robonix_py import mcp_contract

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
    elif encoding == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif encoding == "bgra8":
        # Webots head camera publishes BGRA8 — drop alpha + swap to RGB.
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
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
    """PRIMARY perception tool. Use freely — between every chassis/cmd
    burst — to see what's in front of the robot and decide what to do
    next. Returns the current RGB head-camera frame as a JPEG-encoded
    sensor_msgs/Image (`data` is base64).
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
    """Get the current depth head-camera frame as a JPEG-encoded
    sensor_msgs/Image (depth normalized to grayscale; binary `data` is
    base64). Use to gauge stand-off distance / find open space.
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
            stub.Heartbeat(pb.HeartbeatRequest(capability_id=node_id))
        except Exception as e:
            print(f"[tiago_camera] heartbeat failed: {e}")


def _decl_mcp(stub, cap_id: str, contract_id: str, port: int, fn) -> None:
    """Atlas registration derived from the @mcp_contract-decorated handler:
    docstring → description, codegen input class → input_schema_json."""
    description = (fn.__doc__ or "").strip()
    input_cls = getattr(fn, "_robonix_input_cls", None)
    if input_cls is None:
        schema_json = json.dumps({"type": "object", "properties": {}, "required": []})
    else:
        schema_json = json.dumps(input_cls.json_schema())
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


# ── lifecycle Driver gRPC server ────────────────────────────────────────────

import lifecycle_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402

_CMD_INIT = 0
_CMD_SHUTDOWN = 1


def _wait_for_first_rgb(timeout_s: float) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with _lock:
            have = _latest_rgb is not None
        if have:
            return True, ""
        time.sleep(0.1)
    return False, "no RGB frame received within timeout"


class _CameraDriverServicer(contracts_grpc.PrimitiveCameraDriverServicer):
    def Driver(self, request, context):
        cmd = request.command
        if cmd == _CMD_INIT:
            ok, err = _wait_for_first_rgb(timeout_s=15.0)
            return lifecycle_pb2.Driver_Response(
                ok=ok, state="ready" if ok else "error", error=err
            )
        if cmd == _CMD_SHUTDOWN:
            return lifecycle_pb2.Driver_Response(ok=True, state="ready", error="")
        return lifecycle_pb2.Driver_Response(ok=False, state="error", error="invalid command")


def _start_driver_grpc(port: int) -> None:
    from concurrent import futures
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    contracts_grpc.add_PrimitiveCameraDriverServicer_to_server(
        _CameraDriverServicer(), server
    )
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    print(f"[tiago_camera] LifecycleDriver gRPC serving on 0.0.0.0:{port}")


def _decl_driver(stub, cap_id: str, port: int) -> None:
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=cap_id,
        contract_id="robonix/primitive/camera/driver",
        transport=pb.TRANSPORT_GRPC,
        endpoint=f"127.0.0.1:{port}",
        params=pb.TransportParams(grpc=pb.GrpcParams(
            proto_file="robonix_contracts.proto",
            service_name="LifecycleDriver",
            method="Driver",
        )),
    ))


def _decl_topic_out(stub, cap_id: str, contract_id: str, topic: str, qos_profile: str = "") -> None:
    """Tell atlas which ROS topic carries this contract's data plane.
    Consumers (scene, telemetry) discover the topic by querying atlas
    for the contract over TRANSPORT_ROS2."""
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_ROS2,
        endpoint=topic,
        params=pb.TransportParams(ros2=pb.Ros2Params(qos_profile=qos_profile)),
    ))


def main() -> None:
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    mcp_port = int(os.environ.get("TIAGO_CAMERA_MCP_PORT", "50112"))
    driver_port = int(os.environ.get("TIAGO_CAMERA_DRIVER_PORT", "50212"))
    cap_id = os.environ.get("ROBONIX_CAPABILITY_ID", "com.robonix.primitive.tiago_camera")

    threading.Thread(target=_start_ros2, daemon=True).start()
    _start_driver_grpc(driver_port)

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.AtlasStub(channel)

    try:
        pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
        md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=cap_id,
            namespace="robonix/primitive/camera",
            capability_md_path=md_path,
        ))
        _decl_driver(stub, cap_id, driver_port)
        _decl_mcp(stub, cap_id, "robonix/primitive/camera/snapshot",       mcp_port, snapshot)
        _decl_mcp(stub, cap_id, "robonix/primitive/camera/depth_snapshot", mcp_port, depth_snapshot)
        rgb_topic = os.environ.get("TIAGO_RGB_TOPIC", "/head_front_camera/rgb/image_raw")
        depth_topic = os.environ.get("TIAGO_DEPTH_TOPIC", "/head_front_camera/depth_registered/image_raw")
        _decl_topic_out(stub, cap_id, "robonix/primitive/camera/rgb",   rgb_topic,   "reliable")
        _decl_topic_out(stub, cap_id, "robonix/primitive/camera/depth", depth_topic, "reliable")
        print(f"[tiago_camera] registered cap {cap_id} → driver:{driver_port}, mcp:{mcp_port}, "
              f"ros2: rgb={rgb_topic} depth={depth_topic}")
    except Exception as e:
        print(f"[tiago_camera] WARN: atlas registration failed: {e}")

    threading.Thread(target=_heartbeat_loop, args=(stub, cap_id), daemon=True).start()

    print(f"[tiago_camera] MCP HTTP serving on 0.0.0.0:{mcp_port}")
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=mcp_port, log_level="warning")


if __name__ == "__main__":
    main()
