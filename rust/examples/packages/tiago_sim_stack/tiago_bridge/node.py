#!/usr/bin/env python3
# pyright: reportArgumentType=false
"""Tiago robot bridge node: ROS2 ↔ MCP for robonix agent.

Subscribes to ROS2 topics (camera, pose, scan) and exposes them as MCP tools
that the robonix agent can call.  Data-plane port is allocated by robonix-atlas.

When `nav2_msgs` is available, navigation uses the Nav2 **navigate_to_pose** action;
otherwise goals are published on `/goal_pose` (PoseStamped).

Optional env vars:
  ROBONIX_ATLAS       Control-plane address (default: localhost:50051)
  ROBONIX_NODE_ID      Registry node id, reverse-DNS (default: com.robonix.prm.tiago)
  ROBONIX_NAMESPACE    Registry namespace (default: robonix/prm/camera)
  ROBONIX_DISTRO       Distro label for QueryNodes (default: humble)
  ROBONIX_CONTAINER_ID Container label for QueryNodes (default: empty)
  ROBONIX_MCP_LISTEN_PORT  If unset or 0, pick a free TCP port for MCP HTTP (recommended).
  ROBONIX_PRM_GRPC_LISTEN_PORT  Fixed port for PRM camera gRPC (optional; default: ephemeral)
  TIAGO_RGB_STREAM_HZ  gRPC PrmCameraRgb.Stream max rate (default: 2)
  TIAGO_RGB_FRAME_ID   frame_id in streamed sensor_msgs/Image (default: head_front_camera_rgb_optical_frame)
  TIAGO_ROS_DOMAIN     ROS_DOMAIN_ID (default: unset)

PRM POC: registers standard port `rgb` for both transports:
- `grpc`: server-stream of sensor_msgs/Image (`robonix.contracts.PrmCameraRgb` / `Stream`)
- `ros2`: republish to the robonix-allocated ROS2 endpoint
See `robonix/prm/camera/rgb` in `rust/robonix-interfaces/README.md`.
"""
import asyncio
import base64
import json
import logging
import os
import socket
import queue
import sys
import threading
import time
import uuid
from io import BytesIO
from pathlib import Path

import numpy as np

for _logger_name in (
    "mcp", "mcp.server", "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
    "httpx", "httpcore", "uvicorn", "uvicorn.access",
):
    logging.getLogger(_logger_name).setLevel(logging.WARNING)

from mcp.server.fastmcp import FastMCP


def _ensure_proto_gen() -> None:
    """Resolve rust/examples/proto_gen for repo layout or /app/proto_gen in Docker."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()
import grpc
from concurrent import futures as _grpc_futures
from google.protobuf import empty_pb2

import robonix_contracts_pb2_grpc
import sensor_msgs_pb2
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

# ── ROS2 imports (deferred so the file can be syntax-checked without ROS) ────

_rclpy = None
_Image = None
_CompressedImage = None
_PoseWithCovarianceStamped = None
_PoseStamped = None
_Twist = None
_LaserScan = None
_NavigateToPose = None
_GoalStatus = None


def _import_ros2():
    """Lazy-import ROS2 dependencies."""
    global _rclpy, _Image, _CompressedImage, _PoseWithCovarianceStamped
    global _PoseStamped, _Twist, _LaserScan, _NavigateToPose, _GoalStatus

    import rclpy  # type: ignore
    from sensor_msgs.msg import Image, CompressedImage, LaserScan  # type: ignore
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist  # type: ignore

    _rclpy = rclpy
    _Image = Image
    _CompressedImage = CompressedImage
    _PoseWithCovarianceStamped = PoseWithCovarianceStamped
    _PoseStamped = PoseStamped
    _Twist = Twist
    _LaserScan = LaserScan

    try:
        from nav2_msgs.action import NavigateToPose  # type: ignore
        _NavigateToPose = NavigateToPose
    except ImportError:
        print("[tiago-node] WARNING: nav2_msgs not available, navigate action disabled")

    try:
        from action_msgs.msg import GoalStatus  # type: ignore
        _GoalStatus = GoalStatus
    except ImportError:
        _GoalStatus = None


# ── MCP tool definitions ─────────────────────────────────────────────────────

mcp = FastMCP("tiago-node")

# Shared state updated by ROS2 subscriber callbacks
_latest_rgb: bytes | None = None
_latest_depth: bytes | None = None
_latest_pose: dict | None = None
_latest_scan: dict | None = None
_ros_node = None
_lock = threading.Lock()

# PRM camera_rgb POC: ROS2 republish topic (allocated by robonix-atlas) + gRPC stream
_RGB_ROS_TOPIC: str | None = None
_rgb_remap_pub = None

_nav_queue: "queue.Queue[tuple[str, float, float, float, str]]" = queue.Queue()
_nav_client = None
_nav_action_ready = False
_goal_states: dict[str, dict] = {}
_goal_handles: dict[str, object] = {}

_goal_pub = None
_cmd_vel_pub = None


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


def _ros2_image_to_jpeg(msg) -> bytes:
    """Convert a ROS2 sensor_msgs/Image to JPEG bytes."""
    h, w = msg.height, msg.width
    encoding = msg.encoding.lower()

    if encoding in ("rgb8",):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif encoding in ("bgr8",):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        arr = arr[:, :, ::-1]  # BGR→RGB
    elif encoding in ("rgba8",):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif encoding in ("bgra8",):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
        arr = arr[:, :, 2::-1]  # BGRA→RGB
    elif encoding in ("mono8", "8uc1"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif encoding in ("16uc1", "mono16"):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = (raw / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else np.zeros((h, w), np.uint8)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif encoding in ("32fc1",):
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


@mcp.tool()
def get_camera_image() -> str:
    """Get the current RGB camera image from the robot's head camera.
    Returns a base64-encoded JPEG string."""
    with _lock:
        data = _latest_rgb
    if data is None:
        return json.dumps({"error": "no RGB image received yet"})
    return json.dumps({
        "image_base64": base64.b64encode(data).decode("ascii"),
        "format": "jpeg",
    })


@mcp.tool()
def get_depth_image() -> str:
    """Get the current depth image from the robot's head camera.
    Returns a base64-encoded JPEG (depth normalized to grayscale)."""
    with _lock:
        data = _latest_depth
    if data is None:
        return json.dumps({"error": "no depth image received yet"})
    return json.dumps({
        "image_base64": base64.b64encode(data).decode("ascii"),
        "format": "jpeg",
    })


@mcp.tool()
def get_robot_pose() -> str:
    """Get the robot's current pose (from AMCL localization).
    Returns position {x, y, z} and orientation {x, y, z, w} in the map frame."""
    with _lock:
        pose = _latest_pose
    if pose is None:
        return json.dumps({"error": "no pose received yet (is AMCL running?)"})
    return json.dumps(pose)


@mcp.tool()
def get_lidar_scan() -> str:
    """Get a summary of the latest lidar scan (min/max range, number of points).
    Full point cloud data is too large; this returns statistics."""
    with _lock:
        scan = _latest_scan
    if scan is None:
        return json.dumps({"error": "no lidar scan received yet"})
    return json.dumps(scan)


def _enqueue_nav_goal(x: float, y: float, yaw: float, frame_id: str) -> str:
    gid = str(uuid.uuid4())
    _nav_queue.put((gid, float(x), float(y), float(yaw), frame_id))
    with _lock:
        _goal_states[gid] = {"status": "QUEUED", "accepted": False}
    return json.dumps({
        "goal_id": gid,
        "status": "queued",
        "nav_action": _nav_action_ready,
    })


@mcp.tool()
def navigate_to(x: float, y: float, yaw: float = 0.0, frame_id: str = "map") -> str:
    """Send the robot to (x, y) in `frame_id` with heading yaw (rad).
    Uses Nav2 navigate_to_pose action when available; else publishes PoseStamped on /goal_pose."""
    if _ros_node is None:
        return json.dumps({"error": "ROS2 node not initialized"})
    return _enqueue_nav_goal(x, y, yaw, frame_id)


@mcp.tool()
def send_nav_goal(x: float, y: float, yaw: float = 0.0, frame_id: str = "map") -> str:
    """Design-contract alias: queue a navigation goal (same as navigate_to)."""
    return navigate_to(x, y, yaw, frame_id)


@mcp.tool()
def get_nav_status(goal_id: str) -> str:
    """Return stored status for a goal_id from navigate_to / send_nav_goal."""
    with _lock:
        st = _goal_states.get(goal_id)
    if st is None:
        return json.dumps({"error": "unknown goal_id", "goal_id": goal_id})
    return json.dumps({"goal_id": goal_id, **st})


@mcp.tool()
def cancel_nav_goal(goal_id: str) -> str:
    """Request cancellation of a navigation goal (Nav2 action only)."""
    with _lock:
        gh = _goal_handles.get(goal_id)
    if gh is None:
        return json.dumps({"error": "no active goal handle for goal_id", "goal_id": goal_id})
    gh.cancel_goal_async()  # type: ignore[union-attr]
    return json.dumps({"goal_id": goal_id, "status": "cancel_requested"})


@mcp.tool()
def move_base(linear_x: float = 0.0, angular_z: float = 0.0, duration: float = 1.0) -> str:
    """Send a velocity command to the robot base.
    linear_x: forward speed (m/s), angular_z: rotation speed (rad/s).
    Publishes for `duration` seconds then stops."""
    if _ros_node is None:
        return json.dumps({"error": "ROS2 node not initialized"})

    from geometry_msgs.msg import Twist  # type: ignore

    cmd = Twist()
    cmd.linear.x = float(linear_x)
    cmd.angular.z = float(angular_z)

    stop = Twist()
    steps = max(1, int(duration / 0.1))
    for _ in range(steps):
        _cmd_vel_pub.publish(cmd)
        time.sleep(0.1)
    _cmd_vel_pub.publish(stop)

    return json.dumps({
        "status": "done",
        "linear_x": linear_x,
        "angular_z": angular_z,
        "duration": duration,
    })


# ── ROS2 subscriber callbacks ────────────────────────────────────────────────

def _on_rgb(msg):
    global _latest_rgb
    try:
        data = _ros2_image_to_jpeg(msg)
        with _lock:
            _latest_rgb = data
        pub = _rgb_remap_pub
        if pub is not None:
            pub.publish(msg)
    except Exception as e:
        print(f"[tiago-node] RGB conversion error: {e}")


def _on_depth(msg):
    global _latest_depth
    try:
        data = _ros2_image_to_jpeg(msg)
        with _lock:
            _latest_depth = data
    except Exception as e:
        print(f"[tiago-node] Depth conversion error: {e}")


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


def _on_scan(msg):
    global _latest_scan
    ranges = np.array(msg.ranges, dtype=np.float32)
    valid = ranges[np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)]
    with _lock:
        _latest_scan = {
            "num_points": len(ranges),
            "valid_points": len(valid),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "closest": float(valid.min()) if len(valid) else None,
            "farthest": float(valid.max()) if len(valid) else None,
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
        }


def _make_pose_stamped(node, frame_id: str, x: float, y: float, yaw: float):
    import math
    from geometry_msgs.msg import PoseStamped  # type: ignore

    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.position.z = 0.0
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def _feedback_cb(gid: str, _feedback):
    with _lock:
        if gid in _goal_states:
            _goal_states[gid]["feedback"] = "navigating"


def _goal_response_cb(fut, gid: str):
    global _goal_handles
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
            _goal_states[gid] = {
                "status": st_name,
                "accepted": True,
                "terminal": True,
            }
            _goal_handles.pop(gid, None)
    except Exception as e:
        with _lock:
            _goal_states[gid] = {"status": "FAILED", "accepted": True, "error": str(e), "terminal": True}
            _goal_handles.pop(gid, None)


def _dispatch_nav_goal(node, gid: str, x: float, y: float, yaw: float, frame_id: str):
    global _nav_client

    pose = _make_pose_stamped(node, frame_id, x, y, yaw)

    if _nav_client is not None and _nav_action_ready:
        goal_msg = _NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = _nav_client.send_goal_async(  # type: ignore[union-attr]
            goal_msg,
            feedback_callback=lambda fb: _feedback_cb(gid, fb),
        )
        send_future.add_done_callback(lambda f, g=gid: _goal_response_cb(f, g))
        with _lock:
            _goal_states[gid] = {"status": "SENT", "accepted": False}
        return

    _goal_pub.publish(pose)
    with _lock:
        _goal_states[gid] = {"status": "PUBLISHED_TOPIC", "accepted": True, "topic": "/goal_pose"}


def _start_ros2():
    """Initialize ROS2 node, subscriptions, optional Nav2 action client, spin."""
    global _ros_node, _goal_pub, _cmd_vel_pub, _nav_client, _nav_action_ready, _rgb_remap_pub
    _import_ros2()
    _rclpy.init()

    from rclpy.executors import MultiThreadedExecutor  # type: ignore
    from rclpy.action import ActionClient  # type: ignore

    node = _rclpy.create_node("tiago_robonix_bridge")
    _ros_node = node

    rgb_topic = os.environ.get("TIAGO_RGB_TOPIC", "/head_front_camera/rgb/image_raw")
    depth_topic = os.environ.get("TIAGO_DEPTH_TOPIC", "/head_front_camera/depth_registered/image_raw")
    scan_topic = os.environ.get("TIAGO_SCAN_TOPIC", "/scanner/scan")

    node.create_subscription(_Image, rgb_topic, _on_rgb, 1)
    node.create_subscription(_Image, depth_topic, _on_depth, 1)

    if _RGB_ROS_TOPIC:
        _rgb_remap_pub = node.create_publisher(_Image, _RGB_ROS_TOPIC, 1)
        print(f"[tiago-node] PRM camera_rgb ROS2 republish → {_RGB_ROS_TOPIC!r}")
    node.create_subscription(_PoseWithCovarianceStamped, "/amcl_pose", _on_pose, 1)
    node.create_subscription(_LaserScan, scan_topic, _on_scan, 1)

    _goal_pub = node.create_publisher(_PoseStamped, "/goal_pose", 1)
    _cmd_vel_pub = node.create_publisher(_Twist, "/cmd_vel", 1)

    if _NavigateToPose is not None:
        _nav_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        _nav_action_ready = _nav_client.wait_for_server(timeout_sec=float(os.environ.get("TIAGO_NAV2_WAIT_SEC", "30")))
        if not _nav_action_ready:
            print("[tiago-node] WARNING: navigate_to_pose action server not ready; using /goal_pose fallback")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    print("[tiago-node] ROS2 subscriptions active")
    while _rclpy.ok():
        executor.spin_once(timeout_sec=0.05)
        while True:
            try:
                gid, x, y, yaw, frame_id = _nav_queue.get_nowait()
            except queue.Empty:
                break
            _dispatch_nav_goal(node, gid, x, y, yaw, frame_id)


# ── MCP tool schema extraction (for DeclareInterface metadata) ───────────────

def _mcp_tools_list() -> list[dict]:
    async def _list():
        return await mcp.list_tools()

    tools = asyncio.run(_list())
    out = []
    for t in tools:
        schema = t.inputSchema
        if not isinstance(schema, dict):
            schema = dict(schema)
        out.append({
            "name": t.name,
            "description": t.description or "",
            "input_schema": schema,
        })
    return out


def _iface_meta_mcp() -> str:
    return json.dumps({"tools": _mcp_tools_list()})


def _start_mcp_http(port: int) -> None:
    import uvicorn
    app = mcp.streamable_http_app()
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def _pick_grpc_listen_port() -> int:
    raw = os.environ.get("ROBONIX_PRM_GRPC_LISTEN_PORT", "").strip()
    if raw.isdigit():
        p = int(raw)
        if 1 <= p <= 65535:
            return p
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 0))
    p = int(s.getsockname()[1])
    s.close()
    return p


def _jpeg_to_sensor_image_proto(jpg: bytes):
    from PIL import Image as PILImage

    im = PILImage.open(BytesIO(jpg))
    w, h = im.size
    img = sensor_msgs_pb2.Image()
    img.header.frame_id = os.environ.get("TIAGO_RGB_FRAME_ID", "head_front_camera_rgb_optical_frame")
    now = time.time()
    img.header.stamp.sec = int(now)
    img.header.stamp.nanosec = int((now % 1) * 1e9) % 1_000_000_000
    img.width = w
    img.height = h
    img.encoding = "jpeg"
    img.is_bigendian = 0
    img.step = len(jpg)
    img.data = jpg
    return img


class _PrmCameraRgbServicer(robonix_contracts_pb2_grpc.PrmCameraRgbServicer):
    """Implements contract `robonix/prm/camera/rgb` (`PrmCameraRgb.Stream` in robonix_contracts.proto)."""

    def Stream(self, request: empty_pb2.Empty, context):
        _ = request  # google.protobuf.Empty
        hz = float(os.environ.get("TIAGO_RGB_STREAM_HZ", "2"))
        period = 1.0 / max(hz, 0.25)
        while context.is_active():
            with _lock:
                jpg = _latest_rgb
            if jpg:
                try:
                    yield _jpeg_to_sensor_image_proto(jpg)
                except Exception as e:
                    print(f"[tiago-node] gRPC PrmCameraRgb.Stream encode error: {e}")
            time.sleep(period)


def _run_grpc_prm_server(port: int) -> None:
    server = grpc.server(_grpc_futures.ThreadPoolExecutor(max_workers=4))
    robonix_contracts_pb2_grpc.add_PrmCameraRgbServicer_to_server(_PrmCameraRgbServicer(), server)
    server.add_insecure_port(f"0.0.0.0:{port}")
    server.start()
    print(f"[tiago-node] robonix.contracts.PrmCameraRgb (Stream) on 0.0.0.0:{port}")
    server.wait_for_termination()


def _pick_mcp_listen_port() -> int:
    raw = os.environ.get("ROBONIX_MCP_LISTEN_PORT", "").strip()
    if raw.isdigit():
        p = int(raw)
        if 1 <= p <= 65535:
            return p
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", 0))
    p = int(s.getsockname()[1])
    s.close()
    return p


def _heartbeat_loop(stub: pb_grpc.RobonixRuntimeStub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[tiago-node] heartbeat failed: {e}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    global _RGB_ROS_TOPIC

    channel = grpc.insecure_channel(os.environ.get("ROBONIX_ATLAS", "localhost:50051"))
    stub = pb_grpc.RobonixRuntimeStub(channel)

    node_id = os.environ.get("ROBONIX_NODE_ID", "com.robonix.prm.tiago")

    # Capability-first namespace: `robonix/prm/camera` + standard port leaf `rgb`.
    # Multiple implementations (grpc stream + ros2 topic) are disambiguated by transport,
    # not by inventing new leaf names.
    stub.RegisterNode(
        pb.RegisterNodeRequest(
            node_id=node_id,
            namespace=os.environ.get("ROBONIX_NAMESPACE", "robonix/prm/camera"),
            kind="primitive",
            skill_md="",
            distro=os.environ.get("ROBONIX_DISTRO", "humble"),
            container_id=os.environ.get("ROBONIX_CONTAINER_ID", ""),
        )
    )

    mcp_port = _pick_mcp_listen_port()
    resp = stub.DeclareInterface(
        pb.DeclareInterfaceRequest(
            node_id=node_id,
            name="mcp_tools",
            supported_transports=["mcp"],
            metadata_json=_iface_meta_mcp(),
            listen_port=mcp_port,
        )
    )
    mcp_endpoint = resp.allocated_endpoint
    _, mcp_port_str = mcp_endpoint.rsplit(":", 1)
    assert int(mcp_port_str) == mcp_port, (mcp_endpoint, mcp_port)

    prm_grpc_port = _pick_grpc_listen_port()
    # Stable contract id matches rust/contracts and system interface catalog.
    resp_cam_g = stub.DeclareInterface(
        pb.DeclareInterfaceRequest(
            node_id=node_id,
            name="rgb",
            supported_transports=["grpc"],
            metadata_json="{}",
            listen_port=prm_grpc_port,
            contract_id="robonix/prm/camera/rgb",
        )
    )
    resp_cam_r = stub.DeclareInterface(
        pb.DeclareInterfaceRequest(
            node_id=node_id,
            name="rgb",
            supported_transports=["ros2"],
            metadata_json="{}",
            listen_port=0,
            contract_id="robonix/prm/camera/rgb",
        )
    )
    _RGB_ROS_TOPIC = resp_cam_r.allocated_endpoint

    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()

    threading.Thread(target=_run_grpc_prm_server, args=(prm_grpc_port,), daemon=True).start()
    print(
        f"[tiago-node] PRM camera rgb gRPC stream endpoint {resp_cam_g.allocated_endpoint} "
        f"(same port reused if multiple grpc on node)"
    )

    ros_thread = threading.Thread(target=_start_ros2, daemon=True)
    ros_thread.start()

    threading.Thread(target=_start_mcp_http, args=(mcp_port,), daemon=True).start()
    print(f"[tiago-node] MCP HTTP on port {mcp_port} (server-allocated)")
    print("[tiago-node] ready — ROS2 + MCP + PRM camera rgb (gRPC stream + ROS2 topic) POC active")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
