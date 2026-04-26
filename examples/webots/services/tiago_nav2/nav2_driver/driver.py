#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Tiago Nav2 service — wraps Nav2's navigate_to_pose action.

Capabilities:
  robonix/service/navigation/navigate  → nav_navigate(PoseStamped) → String
  robonix/service/navigation/status    → nav_status(String goal_id) → String
  robonix/service/navigation/cancel    → nav_cancel(String goal_id) → String

start.sh launches nav2_bringup before exec'ing this script, so by the
time we open the ActionClient the action server is already live (or we
fall back to publishing /goal_pose if it doesn't come up in time).
"""
import json
import logging
import math
import os
import queue
import sys
import threading
import time
import uuid
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

import std_msgs_mcp
from std_msgs_mcp import String
from geometry_msgs_mcp import PoseStamped

from robonix_py import mcp_contract
from mcp.server.fastmcp import FastMCP

# ── ROS2 lazy imports ────────────────────────────────────────────────────────

_rclpy = None
_PoseStamped = None
_NavigateToPose = None
_GoalStatus = None


def _import_ros2():
    global _rclpy, _PoseStamped, _NavigateToPose, _GoalStatus
    import rclpy  # type: ignore
    from geometry_msgs.msg import PoseStamped as RosPoseStamped  # type: ignore
    _rclpy = rclpy
    _PoseStamped = RosPoseStamped
    try:
        from nav2_msgs.action import NavigateToPose  # type: ignore
        _NavigateToPose = NavigateToPose
    except ImportError:
        print("[tiago_nav2] WARNING: nav2_msgs not available, /goal_pose fallback only")
    try:
        from action_msgs.msg import GoalStatus  # type: ignore
        _GoalStatus = GoalStatus
    except ImportError:
        _GoalStatus = None


# ── shared state ─────────────────────────────────────────────────────────────

mcp = FastMCP("tiago-nav2")

_lock = threading.Lock()
_ros_node = None
_goal_pub = None
_nav_client = None
_nav_action_ready = False
_nav_queue: "queue.Queue[tuple[str, float, float, float, str]]" = queue.Queue()
_goal_states: dict[str, dict] = {}
_goal_handles: dict[str, object] = {}


def _goal_status_name(status: int) -> str:
    if _GoalStatus is None:
        return str(int(status))
    g = _GoalStatus
    m = {
        int(g.STATUS_UNKNOWN): "UNKNOWN",
        int(g.STATUS_ACCEPTED): "ACCEPTED",
        int(g.STATUS_EXECUTING): "EXECUTING",
        int(g.STATUS_CANCELING): "CANCELING",
        int(g.STATUS_SUCCEEDED): "SUCCEEDED",
        int(g.STATUS_CANCELED): "CANCELED",
        int(g.STATUS_ABORTED): "ABORTED",
    }
    return m.get(int(status), str(int(status)))


def _make_pose_stamped(node, frame_id: str, x: float, y: float, yaw: float):
    g = _PoseStamped()
    g.header.frame_id = frame_id
    g.header.stamp = node.get_clock().now().to_msg()
    g.pose.position.x = float(x)
    g.pose.position.y = float(y)
    g.pose.position.z = 0.0
    g.pose.orientation.z = math.sin(yaw / 2.0)
    g.pose.orientation.w = math.cos(yaw / 2.0)
    return g


def _feedback_cb(gid, _fb):
    with _lock:
        if gid in _goal_states:
            _goal_states[gid]["feedback"] = "navigating"


def _goal_response_cb(fut, gid: str):
    try:
        gh = fut.result()
    except Exception as e:
        with _lock:
            _goal_states[gid] = {"status": "FAILED", "accepted": False, "error": str(e)}
        return
    if not gh.accepted:
        with _lock:
            _goal_states[gid] = {"status": "REJECTED", "accepted": False}
        return
    with _lock:
        _goal_handles[gid] = gh
        _goal_states[gid] = {"status": "ACCEPTED", "accepted": True}
    res_fut = gh.get_result_async()
    res_fut.add_done_callback(lambda f: _result_cb(f, gid))


def _result_cb(fut, gid: str):
    try:
        res = fut.result()
        status = getattr(res, "status", None)
        st_name = _goal_status_name(status) if status is not None else "UNKNOWN"
        with _lock:
            _goal_states[gid] = {"status": st_name, "accepted": True, "terminal": True}
            _goal_handles.pop(gid, None)
    except Exception as e:
        with _lock:
            _goal_states[gid] = {
                "status": "FAILED", "accepted": True, "error": str(e), "terminal": True,
            }
            _goal_handles.pop(gid, None)


def _dispatch_nav_goal(node, gid, x, y, yaw, frame_id):
    pose = _make_pose_stamped(node, frame_id, x, y, yaw)
    if _nav_client is not None and _nav_action_ready:
        goal_msg = _NavigateToPose.Goal()
        goal_msg.pose = pose
        send_future = _nav_client.send_goal_async(  # type: ignore[union-attr]
            goal_msg, feedback_callback=lambda fb: _feedback_cb(gid, fb),
        )
        send_future.add_done_callback(lambda f, g=gid: _goal_response_cb(f, g))
        with _lock:
            _goal_states[gid] = {"status": "SENT", "accepted": False}
        return
    _goal_pub.publish(pose)
    with _lock:
        _goal_states[gid] = {"status": "PUBLISHED_TOPIC", "accepted": True, "topic": "/goal_pose"}


# ── MCP tools ────────────────────────────────────────────────────────────────

@mcp_contract(mcp, contract_id="robonix/service/navigation/navigate")
def navigate(msg: PoseStamped) -> String:
    """Send the robot to a target pose via Nav2's navigate_to_pose action.
    Returns std_msgs/String whose `data` is JSON `{goal_id, status}`.
    Use the goal_id with nav_status / nav_cancel to track the goal.
    Contract: robonix/service/navigation/navigate."""
    if _ros_node is None:
        return String(data=json.dumps({"error": "ROS2 not initialized"}))
    frame_id = msg.header.frame_id or "map"
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w
    yaw = 2.0 * math.atan2(qz, qw)
    gid = str(uuid.uuid4())
    _nav_queue.put(
        (gid, float(msg.pose.position.x), float(msg.pose.position.y), float(yaw), frame_id)
    )
    with _lock:
        _goal_states[gid] = {"status": "QUEUED", "accepted": False}
    return String(data=json.dumps({
        "goal_id": gid, "status": "queued", "nav_action": _nav_action_ready,
    }))


@mcp_contract(mcp, contract_id="robonix/service/navigation/status")
def status(msg: String) -> String:
    """Return navigation status for a goal_id obtained from nav_navigate.
    msg.data is the goal_id. Returns std_msgs/String JSON.
    Contract: robonix/service/navigation/status."""
    gid = msg.data
    with _lock:
        st = _goal_states.get(gid)
    if st is None:
        return String(data=json.dumps({"error": "unknown goal_id", "goal_id": gid}))
    return String(data=json.dumps({"goal_id": gid, **st}))


@mcp_contract(mcp, contract_id="robonix/service/navigation/cancel")
def cancel(msg: String) -> String:
    """Cancel an in-flight navigation goal (Nav2 action only — /goal_pose
    fallback goals can't be cancelled).
    msg.data is the goal_id. Returns std_msgs/String JSON.
    Contract: robonix/service/navigation/cancel."""
    gid = msg.data
    with _lock:
        gh = _goal_handles.get(gid)
    if gh is None:
        return String(data=json.dumps({"error": "no active goal handle", "goal_id": gid}))
    gh.cancel_goal_async()  # type: ignore[union-attr]
    return String(data=json.dumps({"goal_id": gid, "status": "cancel_requested"}))


# ── runtime wiring ───────────────────────────────────────────────────────────

def _start_ros2():
    global _ros_node, _goal_pub, _nav_client, _nav_action_ready
    _import_ros2()
    _rclpy.init()
    from rclpy.executors import MultiThreadedExecutor  # type: ignore
    from rclpy.action import ActionClient  # type: ignore

    node = _rclpy.create_node("tiago_nav2_driver")
    _ros_node = node
    _goal_pub = node.create_publisher(_PoseStamped, "/goal_pose", 1)

    if _NavigateToPose is not None:
        _nav_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        wait_sec = float(os.environ.get("TIAGO_NAV2_WAIT_SEC", "30"))
        _nav_action_ready = _nav_client.wait_for_server(timeout_sec=wait_sec)
        if not _nav_action_ready:
            print("[tiago_nav2] WARNING: navigate_to_pose action not ready; using /goal_pose fallback")
        else:
            print("[tiago_nav2] navigate_to_pose ActionClient ready")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    while _rclpy.ok():
        executor.spin_once(timeout_sec=0.05)
        while True:
            try:
                gid, x, y, yaw, frame_id = _nav_queue.get_nowait()
            except queue.Empty:
                break
            _dispatch_nav_goal(node, gid, x, y, yaw, frame_id)


def _heartbeat_loop(stub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[tiago_nav2] heartbeat failed: {e}")


def _single_tool_meta(tool_name: str, description: str, input_schema: dict) -> str:
    return json.dumps({
        "tools": [{"name": tool_name, "description": description, "input_schema": input_schema}]
    })


def main() -> None:
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    port = int(os.environ.get("TIAGO_NAV2_MCP_PORT", "50121"))
    node_id = os.environ.get("ROBONIX_NODE_ID", "com.robonix.service.tiago_nav2")

    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    try:
        stub.RegisterNode(pb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/service/navigation",
            kind="service",
            skill_md="# tiago_nav2\nNav2 navigate_to_pose wrapper.",
        ))
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="navigate",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "navigate",
                "Send a navigation goal. Arguments: geometry_msgs/PoseStamped JSON. "
                "Returns std_msgs/String; data is JSON {goal_id, status}.",
                PoseStamped.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/service/navigation/navigate",
        ))
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="status",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "status",
                "Get navigation status. std_msgs/String; data is goal_id from nav_navigate.",
                std_msgs_mcp.String.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/service/navigation/status",
        ))
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=node_id, name="cancel",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "cancel",
                "Cancel a navigation goal. std_msgs/String; data is goal_id from nav_navigate.",
                std_msgs_mcp.String.json_schema(),
            ),
            listen_port=port,
            contract_id="robonix/service/navigation/cancel",
        ))
        print(f"[tiago_nav2] registered node {node_id} → 3 caps on port {port}")
    except Exception as e:
        print(f"[tiago_nav2] WARN: atlas registration failed: {e}")

    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_ros2, daemon=True).start()

    print(f"[tiago_nav2] MCP HTTP serving on 0.0.0.0:{port}")
    import uvicorn
    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
