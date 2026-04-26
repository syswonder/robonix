#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago chassis driver — primitive cap.

Subscribes to /amcl_pose, publishes to /cmd_vel, exposes two MCP tools:
  robonix/primitive/chassis/state  → chassis_state(Empty) → RobotState
  robonix/primitive/chassis/cmd    → chassis_cmd(MoveCommand) → String

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
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for pg in (d / "rbnx-build" / "codegen" / "proto_gen", d / "proto_gen"):
            if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
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
    """Pull in robonix-py (mcp_contract decorator)."""
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
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

import builtin_interfaces_mcp
import std_msgs_mcp
from prm_base_mcp import MoveCommand, RobotState
from geometry_msgs_mcp import PoseStamped
from std_msgs_mcp import Empty, String

from robonix_py import mcp_contract
from mcp.server.fastmcp import FastMCP

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
def chassis_state(msg: Empty) -> RobotState:
    """Get the chassis pose (latest /amcl_pose). Returns codegen prm_base/RobotState
    with `base_pose` populated from AMCL when available; empty otherwise.
    Contract: robonix/primitive/chassis/state."""
    _ = msg
    with _lock:
        pose = _latest_pose
    state = RobotState()
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
        state.base_pose = bp
    return state


@mcp_contract(mcp, contract_id="robonix/primitive/chassis/cmd")
def chassis_cmd(msg: MoveCommand) -> String:
    """Send a velocity command to the chassis (publishes Twist on /cmd_vel
    for `TIAGO_CHASSIS_CMD_DURATION_SEC` seconds, default 1.0).
    Contract: robonix/primitive/chassis/cmd."""
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
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[tiago_chassis] heartbeat failed: {e}")


def _single_tool_meta(tool_name: str, description: str, input_schema: dict) -> str:
    return json.dumps({
        "tools": [{
            "name": tool_name,
            "description": description,
            "input_schema": input_schema,
        }]
    })


def main() -> None:
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    port = int(os.environ.get("TIAGO_CHASSIS_MCP_PORT", "50111"))
    node_id = os.environ.get(
        "ROBONIX_NODE_ID", "com.robonix.primitive.tiago_chassis"
    )

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    try:
        stub.RegisterNode(pb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/primitive/chassis",
            kind="primitive",
            skill_md="# tiago_chassis\nPose readout (/amcl_pose) + velocity control (/cmd_vel).",
        ))

        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="chassis_state",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "chassis_state",
                "Get the chassis pose. Returns prm_base/RobotState; base_pose is the latest /amcl_pose.",
                Empty.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/primitive/chassis/state",
        ))
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="chassis_cmd",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "chassis_cmd",
                "Send a velocity command. prm_base/MoveCommand wire JSON. "
                "Publish duration: env TIAGO_CHASSIS_CMD_DURATION_SEC (default 1.0).",
                MoveCommand.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/primitive/chassis/cmd",
        ))
        print(f"[tiago_chassis] registered node {node_id} → 2 caps on port {port}")
    except Exception as e:
        print(f"[tiago_chassis] WARN: atlas registration failed: {e}")

    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_ros2, daemon=True).start()

    print(f"[tiago_chassis] MCP HTTP serving on 0.0.0.0:{port}")
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
