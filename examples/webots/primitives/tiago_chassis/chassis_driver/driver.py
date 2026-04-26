#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago chassis driver — primitive cap.

Subscribes to /amcl_pose, publishes to /cmd_vel, exposes two MCP tools:
  robonix/primitive/chassis/state  → state()  → RobotState (as dict)
  robonix/primitive/chassis/cmd    → cmd(...) → ack JSON string

Runs inside the sim docker container (sim/start.sh brings it up, then
the package's `start:` block does `docker exec robonix_tiago_sim ...`).

Env vars:
  ROBONIX_ATLAS                   atlas endpoint (default 127.0.0.1:50051)
  TIAGO_CHASSIS_MCP_PORT          MCP HTTP port (default 50111)
  TIAGO_CHASSIS_CMD_DURATION_SEC  how long to publish cmd_vel per call (default 1.0)
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
    """Find pylib/robonix-py — host-side via `rbnx path`, in-container via
    walk-up from this file. Provides @mcp_contract: turns the codegen IO
    classes on the function signature into a FastMCP tool."""
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

# Quiet down noisy upstreams.
for _logger_name in (
    "mcp", "mcp.server", "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
    "httpx", "httpcore", "uvicorn", "uvicorn.access",
):
    logging.getLogger(_logger_name).setLevel(logging.WARNING)

import grpc
import atlas_pb2 as pb
import atlas_pb2_grpc as pb_grpc

from base_mcp import MoveCommand, RobotState
from geometry_msgs_mcp import PoseStamped
from std_msgs_mcp import Empty, String

from mcp.server.fastmcp import FastMCP
from robonix_py import mcp_contract

# ── ROS2 lazy imports ────────────────────────────────────────────────────────

_rclpy = None
_PoseWithCovarianceStamped = None
_Twist = None


def _import_ros2():
    global _rclpy, _PoseWithCovarianceStamped, _Twist
    import rclpy  # type: ignore
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist  # type: ignore
    _rclpy = rclpy
    _PoseWithCovarianceStamped = PoseWithCovarianceStamped
    _Twist = Twist


# ── shared state ─────────────────────────────────────────────────────────────

mcp = FastMCP("tiago-chassis")

_lock = threading.Lock()
_latest_pose: dict | None = None
_ros_node = None
_cmd_vel_pub = None


# ── ROS2 subscriber callbacks ────────────────────────────────────────────────

def _on_pose(msg):
    global _latest_pose
    p = msg.pose.pose
    with _lock:
        _latest_pose = {
            "frame": msg.header.frame_id,
            "position": {"x": p.position.x, "y": p.position.y, "z": p.position.z},
            "orientation": {
                "x": p.orientation.x, "y": p.orientation.y,
                "z": p.orientation.z, "w": p.orientation.w,
            },
        }


# ── MCP tools ────────────────────────────────────────────────────────────────

@mcp_contract(mcp, contract_id="robonix/primitive/chassis/state")
def state(msg: Empty) -> RobotState:
    """Get the chassis pose. Returns base/RobotState; `base_pose` is the
    latest /amcl_pose when AMCL has localized, empty otherwise. (Other
    fields like joint_state / gripper / tcp_pose are leftover from the
    legacy whole-robot RobotState message and stay empty for this sim.)
    Contract: robonix/primitive/chassis/state."""
    _ = msg
    with _lock:
        pose = _latest_pose
    out = RobotState()
    if pose is not None:
        bp = PoseStamped()
        bp.header.frame_id = pose.get("frame", "map")
        pos = pose.get("position", {})
        bp.pose.position.x = float(pos.get("x", 0.0))
        bp.pose.position.y = float(pos.get("y", 0.0))
        bp.pose.position.z = float(pos.get("z", 0.0))
        ori = pose.get("orientation", {})
        bp.pose.orientation.x = float(ori.get("x", 0.0))
        bp.pose.orientation.y = float(ori.get("y", 0.0))
        bp.pose.orientation.z = float(ori.get("z", 0.0))
        bp.pose.orientation.w = float(ori.get("w", 1.0))
        out.base_pose = bp
    return out


@mcp_contract(mcp, contract_id="robonix/primitive/chassis/move")
def move(msg: MoveCommand) -> String:
    """PRIMARY motion controller. Drives the chassis: publishes Twist on
    /cmd_vel for TIAGO_CHASSIS_CMD_DURATION_SEC seconds (default 1.0),
    then a zero Twist to stop. Pair with camera/snapshot for visual
    exploration: snapshot → reason → short move burst → snapshot again.
    Contract: robonix/primitive/chassis/move."""
    if _ros_node is None or _cmd_vel_pub is None:
        return String(data=json.dumps({"error": "ROS2 not initialized"}))
    duration = float(os.environ.get("TIAGO_CHASSIS_CMD_DURATION_SEC", "1.0"))
    tw = _Twist()
    tw.linear.x = float(msg.linear_x)
    tw.linear.y = float(msg.linear_y)
    tw.linear.z = float(msg.linear_z)
    tw.angular.x = float(msg.angular_x)
    tw.angular.y = float(msg.angular_y)
    tw.angular.z = float(msg.angular_z)
    stop = _Twist()
    steps = max(1, int(duration / 0.1))
    for _ in range(steps):
        _cmd_vel_pub.publish(tw)
        time.sleep(0.1)
    _cmd_vel_pub.publish(stop)
    return String(data=json.dumps({
        "status": "done",
        "linear_x": msg.linear_x,
        "angular_z": msg.angular_z,
        "duration": duration,
    }))


# ── runtime wiring ───────────────────────────────────────────────────────────

def _start_ros2():
    global _ros_node, _cmd_vel_pub
    _import_ros2()
    _rclpy.init()
    from rclpy.executors import SingleThreadedExecutor  # type: ignore

    node = _rclpy.create_node("tiago_chassis_driver")
    _ros_node = node
    node.create_subscription(_PoseWithCovarianceStamped, "/amcl_pose", _on_pose, 1)
    _cmd_vel_pub = node.create_publisher(_Twist, "/cmd_vel", 1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print("[tiago_chassis] ROS2 ready: sub /amcl_pose, pub /cmd_vel")
    while _rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


def _heartbeat_loop(stub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.Heartbeat(pb.HeartbeatRequest(capability_id=node_id))
        except Exception as e:
            print(f"[tiago_chassis] heartbeat failed: {e}")


def _decl_mcp(stub, cap_id: str, contract_id: str, port: int, fn) -> None:
    """Register one MCP-transport interface with atlas. The single source
    of truth is the @mcp_contract-decorated function: its docstring is
    the description, and its single parameter's codegen class supplies
    the JSON Schema (codegen messages emit `.json_schema()`)."""
    description = (fn.__doc__ or "").strip()
    # The decorator stashes the resolved input class so we don't have
    # to re-reflect the annotation here.
    input_cls = getattr(fn, "_robonix_input_cls", None)
    if input_cls is None:
        # Bare @mcp.tool() (no contract decorator): empty schema.
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


def main() -> None:
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    port = int(os.environ.get("TIAGO_CHASSIS_MCP_PORT", "50111"))
    cap_id = os.environ.get(
        "ROBONIX_CAPABILITY_ID", "com.robonix.primitive.tiago_chassis"
    )

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.AtlasStub(channel)

    try:
        pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
        md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=cap_id,
            namespace="robonix/primitive/chassis",
            capability_md_path=md_path,
        ))
        _decl_mcp(stub, cap_id, "robonix/primitive/chassis/state", port, state)
        _decl_mcp(stub, cap_id, "robonix/primitive/chassis/move",  port, move)
        print(f"[tiago_chassis] registered cap {cap_id} → 2 interfaces on port {port}")
    except Exception as e:
        print(f"[tiago_chassis] WARN: atlas registration failed: {e}")

    threading.Thread(target=_heartbeat_loop, args=(stub, cap_id), daemon=True).start()
    threading.Thread(target=_start_ros2, daemon=True).start()

    print(f"[tiago_chassis] MCP HTTP serving on 0.0.0.0:{port}")
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
