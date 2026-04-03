#!/usr/bin/env python3
"""RTAB-Map bridge: env_node gRPC → ROS2 topics → RTAB-Map → SlamDataService gRPC.

Runs inside a ROS2 Humble Docker container.  Polls env_node for RGB-D
observations via gRPC, publishes them as standard ROS2 camera topics and TF
transforms so RTAB-Map can consume them, then aggregates RTAB-Map output
(PointCloud2 + OccupancyGrid) and exposes it via a lightweight SlamDataService
gRPC endpoint.

The host-side viz_node discovers and polls this endpoint via robonix-atlas
NegotiateChannel and logs the data to Rerun.  No rerun dependency is needed
inside the container.

Data flow:
  env_node (host gRPC)
    → /camera/color/image_raw       (sensor_msgs/Image, rgb8)
    → /camera/depth/image_rect_raw  (sensor_msgs/Image, 32FC1, metres)
    → /camera/color/camera_info     (sensor_msgs/CameraInfo)
    → odom → camera_link TF         (ground-truth camera pose)
  RTAB-Map (ROS2, same container)
    → /rtabmap/cloud_map            (PointCloud2)
    → /rtabmap/grid_map             (OccupancyGrid)
  SlamDataService (gRPC, this process)
    → GetSlamData() → SlamData{cloud_map, occupancy_map}
  viz_node (host, polls SlamDataService)
    → Rerun

Env vars:
  ROBONIX_ATLAS       robonix-atlas gRPC address  (default: localhost:50051)
  ENV_GRPC_ENDPOINT    skip NegotiateChannel, use this endpoint directly
  BRIDGE_FPS           env_node poll rate in Hz     (default: 10)
"""
from __future__ import annotations

import math
import os
import socket
import struct
import sys
import threading
import time
from concurrent import futures as _futures

import numpy as np

# ---------------------------------------------------------------------------
# Proto path setup (stubs copied into /app/proto_gen by Dockerfile)
# ---------------------------------------------------------------------------
sys.path.insert(0, "/app/proto_gen")
sys.path.insert(0, "/app")

import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc
import maniskill_env_pb2 as env_pb
import maniskill_env_pb2_grpc as env_pb_grpc

# ---------------------------------------------------------------------------
# ROS2 imports (available inside the Docker container)
# ---------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
BRIDGE_FPS = float(os.environ.get("BRIDGE_FPS", "10"))
NODE_ID = "com.robonix.demo.mapping"

# ---------------------------------------------------------------------------
# Shared SLAM state (updated by ROS2 callbacks, read by gRPC servicer)
# ---------------------------------------------------------------------------
_slam_lock = threading.Lock()
_latest_cloud: env_pb.SlamCloudMap | None = None
_latest_occ: env_pb.SlamOccupancyMap | None = None

# ---------------------------------------------------------------------------
# gRPC SlamDataService implementation
# ---------------------------------------------------------------------------


class _SlamDataServicer(env_pb_grpc.SlamDataServiceServicer):
    def GetSlamData(self, request, context):
        with _slam_lock:
            cloud = _latest_cloud
            occ = _latest_occ
        data = env_pb.SlamData()
        if cloud is not None:
            data.cloud_map.CopyFrom(cloud)
        if occ is not None:
            data.occupancy_map.CopyFrom(occ)
        return data


def _pick_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _start_slam_grpc(port: int) -> None:
    server = grpc.server(_futures.ThreadPoolExecutor(max_workers=4))
    env_pb_grpc.add_SlamDataServiceServicer_to_server(_SlamDataServicer(), server)
    server.add_insecure_port(f"0.0.0.0:{port}")
    server.start()
    print(f"[rtabmap-bridge] SlamDataService gRPC on :{port}", file=sys.stderr)
    server.wait_for_termination()


# ---------------------------------------------------------------------------
# gRPC env_node connection
# ---------------------------------------------------------------------------
_env_stub: env_pb_grpc.EnvDataServiceStub | None = None


def _discover_env_grpc() -> str:
    override = os.environ.get("ENV_GRPC_ENDPOINT", "").strip()
    if override:
        return override

    server_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    channel = grpc.insecure_channel(server_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    try:
        stub.RegisterNode(pb.RegisterNodeRequest(
            node_id=NODE_ID,
            namespace="robonix/viz",
            kind="primitive",
        ))
    except grpc.RpcError as e:
        print(f"[rtabmap-bridge] register failed (non-fatal): {e}", file=sys.stderr)

    for attempt in range(60):
        try:
            resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
                consumer_id=NODE_ID,
                provider_node_id="com.robonix.demo.maniskill",
                interface_name="env_data",
                transport="grpc",
            ))
            return resp.endpoint
        except grpc.RpcError:
            if attempt < 59:
                print(f"[rtabmap-bridge] waiting for env_node… ({attempt+1}/60)",
                      file=sys.stderr)
                time.sleep(2)
            else:
                raise
    raise RuntimeError("could not discover env gRPC endpoint")


def _register_slam_interface(server_addr: str, slam_grpc_port: int) -> None:
    """Register this node's SlamDataService interface so viz_node can discover it."""
    channel = grpc.insecure_channel(server_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)
    try:
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=NODE_ID,
            name="slam_data",
            supported_transports=["grpc"],
            metadata_json="{}",
            listen_port=slam_grpc_port,
        ))
        print(f"[rtabmap-bridge] declared slam_data interface on port {slam_grpc_port}",
              file=sys.stderr)
    except grpc.RpcError as e:
        print(f"[rtabmap-bridge] DeclareInterface failed (non-fatal): {e}", file=sys.stderr)


def _start_heartbeat(server_addr: str) -> None:
    channel = grpc.insecure_channel(server_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    def _loop():
        while True:
            time.sleep(15.0)
            try:
                stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=NODE_ID))
            except Exception:
                pass

    threading.Thread(target=_loop, daemon=True).start()


# ---------------------------------------------------------------------------
# PointCloud2 parsing
# ---------------------------------------------------------------------------

def _float_rgb_to_u8(rgb: float) -> tuple[int, int, int]:
    """PCL packed RGB stored as a float32 (common in PointCloud2)."""
    packed = struct.unpack("I", struct.pack("f", float(rgb)))[0]
    return (
        (packed >> 16) & 0xFF,
        (packed >> 8) & 0xFF,
        packed & 0xFF,
    )


def _pointcloud2_to_numpy(msg: PointCloud2):
    """Extract (N,3) float32 positions and optional (N,3) uint8 colors.

    RTAB-Map often publishes ``x/y/z`` as FLOAT64; reading them as float32
    yields garbage → NaNs → 0 points after filtering.  Prefer
    ``sensor_msgs_py.point_cloud2.read_points`` (handles dtypes + endianness).
    """
    try:
        from sensor_msgs_py import point_cloud2 as pc2
    except ImportError:
        pc2 = None

    names = {f.name for f in msg.fields}

    if pc2 is not None:
        fields = ("x", "y", "z", "rgb") if "rgb" in names else ("x", "y", "z")
        try:
            rows = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
        except ValueError:
            fields = ("x", "y", "z")
            rows = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
        if not rows:
            return np.zeros((0, 3), dtype=np.float32), None
        pts = np.asarray([r[:3] for r in rows], dtype=np.float32)
        colors = None
        if len(fields) == 4:
            cols = []
            for r in rows:
                v = r[3]
                if isinstance(v, (float, np.floating)):
                    cols.append(_float_rgb_to_u8(v))
                else:
                    iv = int(v) & 0xFFFFFFFF
                    cols.append(
                        ((iv >> 16) & 0xFF, (iv >> 8) & 0xFF, iv & 0xFF)
                    )
            colors = np.asarray(cols, dtype=np.uint8)
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if colors is not None:
            colors = colors[valid]
        return pts, colors

    # Fallback: respect PointField datatype for x/y/z (float32 vs float64).
    from sensor_msgs.msg import PointField

    field_map = {f.name: f for f in msg.fields}
    point_step = msg.point_step
    n_pts = msg.width * msg.height
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_pts, point_step)

    def _xyz_dtype(f):
        if f.datatype == PointField.FLOAT64:
            return np.float64
        if f.datatype == PointField.FLOAT32:
            return np.float32
        return np.float32

    def _xyz_width(f):
        return 8 if f.datatype == PointField.FLOAT64 else 4

    xo, yo, zo = field_map["x"].offset, field_map["y"].offset, field_map["z"].offset
    wx, wy, wz = _xyz_width(field_map["x"]), _xyz_width(field_map["y"]), _xyz_width(field_map["z"])
    dtx, dty, dtz = _xyz_dtype(field_map["x"]), _xyz_dtype(field_map["y"]), _xyz_dtype(field_map["z"])

    xs = np.frombuffer(data[:, xo:xo + wx].tobytes(), dtype=dtx)
    ys = np.frombuffer(data[:, yo:yo + wy].tobytes(), dtype=dty)
    zs = np.frombuffer(data[:, zo:zo + wz].tobytes(), dtype=dtz)
    points = np.column_stack([xs, ys, zs]).astype(np.float32)

    valid = np.isfinite(points).all(axis=1)
    points = points[valid]

    colors = None
    if "rgb" in field_map:
        rf = field_map["rgb"]
        rgb_off = rf.offset
        if rf.datatype == PointField.FLOAT32:
            rgbf = np.frombuffer(data[valid, rgb_off:rgb_off + 4].tobytes(), dtype=np.float32)
            colors = np.array([_float_rgb_to_u8(t) for t in rgbf], dtype=np.uint8)
        else:
            rgb_bytes = data[valid, rgb_off:rgb_off + 4]
            r = rgb_bytes[:, 2].astype(np.uint8)
            g = rgb_bytes[:, 1].astype(np.uint8)
            b = rgb_bytes[:, 0].astype(np.uint8)
            colors = np.column_stack([r, g, b])

    return points, colors


# ---------------------------------------------------------------------------
# ROS2 Bridge Node
# ---------------------------------------------------------------------------

class RtabmapBridge(Node):
    def __init__(self):
        super().__init__("rtabmap_bridge")

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._rgb_pub   = self.create_publisher(Image,      "/camera/color/image_raw",        qos)
        self._depth_pub = self.create_publisher(Image,      "/camera/depth/image_rect_raw",   qos)
        self._info_pub  = self.create_publisher(CameraInfo, "/camera/color/camera_info",       qos)
        self._tf_broadcaster = TransformBroadcaster(self)

        # RTAB-Map now publishes with latch=False (VOLATILE + RELIABLE).
        # Use a plain VOLATILE subscription with generous depth.
        # map_always_update=True ensures RTAB-Map re-publishes every ~0.5 s
        # so missing the first message is not a problem.
        map_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._map_sub = self.create_subscription(
            PointCloud2,   "/rtabmap/cloud_map", self._on_cloud_map, map_qos)
        self._occ_sub = self.create_subscription(
            OccupancyGrid, "/rtabmap/grid_map",  self._on_grid_map,  map_qos)
        print("[rtabmap-bridge] subscribed to cloud_map + grid_map "
              "(RELIABLE + VOLATILE, depth=10)", file=sys.stderr)

        # Diagnostic: after 5 s print RTAB-Map publisher QoS so we know which
        # durability it actually uses (to keep only the matching subscription).
        self._diag_timer = self.create_timer(5.0, self._print_topic_qos)

        interval = 1.0 / max(1.0, BRIDGE_FPS)
        self._timer = self.create_timer(interval, self._poll_env)
        self._frame = 0

    def _print_topic_qos(self):
        """One-shot: log publisher/subscriber QoS for the map topics."""
        self.destroy_timer(self._diag_timer)
        for topic in ("/rtabmap/cloud_map", "/rtabmap/grid_map"):
            pubs = self.get_publishers_info_by_topic(topic)
            subs = self.get_subscriptions_info_by_topic(topic)
            lines = [f"[rtabmap-bridge] QoS audit: {topic}"]
            for p in pubs:
                q = p.qos_profile
                lines.append(
                    f"  PUB  reliability={q.reliability.name}"
                    f"  durability={q.durability.name}"
                    f"  depth={q.depth}"
                )
            for s in subs:
                q = s.qos_profile
                lines.append(
                    f"  SUB  reliability={q.reliability.name}"
                    f"  durability={q.durability.name}"
                    f"  depth={q.depth}"
                )
            print("\n".join(lines), file=sys.stderr)

    # -- Publish env_node obs as ROS2 topics --------------------------------

    def _poll_env(self):
        if _env_stub is None:
            return
        try:
            obs = _env_stub.GetObs(env_pb.Empty())
        except grpc.RpcError as e:
            self.get_logger().error(f"GetObs failed: {e}", throttle_duration_sec=5.0)
            return

        stamp    = self.get_clock().now().to_msg()
        frame_id = "camera_link"

        self._publish_rgb(obs, stamp, frame_id)
        self._publish_depth(obs, stamp, frame_id)
        self._publish_camera_info(obs, stamp, frame_id)
        self._publish_tf(obs, stamp)
        self._frame += 1

    def _publish_rgb(self, obs, stamp, frame_id: str):
        msg = Image()
        msg.header       = Header(stamp=stamp, frame_id=frame_id)
        msg.height       = obs.height
        msg.width        = obs.width
        msg.encoding     = "rgb8"
        msg.is_bigendian = False
        msg.step         = obs.width * 3
        msg.data         = obs.rgb
        self._rgb_pub.publish(msg)

    def _publish_depth(self, obs, stamp, frame_id: str):
        if not obs.depth:
            return
        msg = Image()
        msg.header       = Header(stamp=stamp, frame_id=frame_id)
        msg.height       = obs.height
        msg.width        = obs.width
        msg.encoding     = "32FC1"   # env_node already converts mm → metres
        msg.is_bigendian = False
        msg.step         = obs.width * 4
        msg.data         = obs.depth
        self._depth_pub.publish(msg)

    def _publish_camera_info(self, obs, stamp, frame_id: str):
        if obs.fx <= 0 or obs.fy <= 0:
            return
        msg = CameraInfo()
        msg.header            = Header(stamp=stamp, frame_id=frame_id)
        msg.height            = obs.height
        msg.width             = obs.width
        msg.distortion_model  = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [obs.fx, 0.0, obs.cx,
                 0.0, obs.fy, obs.cy,
                 0.0, 0.0,   1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [obs.fx, 0.0, obs.cx, 0.0,
                 0.0, obs.fy, obs.cy, 0.0,
                 0.0, 0.0,   1.0,    0.0]
        self._info_pub.publish(msg)

    def _publish_tf(self, obs, stamp):
        """Publish odom → camera_link from the ground-truth camera pose."""
        if len(obs.camera_pose) < 16:
            return
        T = np.array(obs.camera_pose, dtype=np.float64).reshape(4, 4)
        tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
        qw, qx, qy, qz = _rotation_matrix_to_quaternion(T[:3, :3])

        t = TransformStamped()
        t.header.stamp        = stamp
        t.header.frame_id     = "odom"
        t.child_frame_id      = "camera_link"
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = float(tz)
        t.transform.rotation.x    = float(qx)
        t.transform.rotation.y    = float(qy)
        t.transform.rotation.z    = float(qz)
        t.transform.rotation.w    = float(qw)
        self._tf_broadcaster.sendTransform(t)

    # -- RTAB-Map output subscribers → shared state -------------------------

    def _on_cloud_map(self, msg: PointCloud2):
        global _latest_cloud
        print(f"[rtabmap-bridge] cloud_map CB: width={msg.width} height={msg.height} "
              f"point_step={msg.point_step}", file=sys.stderr, flush=True)
        points, colors = _pointcloud2_to_numpy(msg)
        n = len(points)
        if n == 0:
            print("[rtabmap-bridge] cloud_map: 0 valid pts (filtered)", file=sys.stderr)
            self.get_logger().warn(
                "cloud_map received but 0 valid points after NaN filter",
                throttle_duration_sec=10.0)
            return
        cloud = env_pb.SlamCloudMap(
            xyz=points.astype(np.float32).tobytes(),
            n_points=n,
        )
        if colors is not None:
            cloud.rgb = colors.tobytes()
        with _slam_lock:
            _latest_cloud = cloud
        print(f"[rtabmap-bridge] cloud_map: {n} pts", file=sys.stderr)
        self.get_logger().info(
            f"cloud_map: {n} pts", throttle_duration_sec=10.0)

    def _on_grid_map(self, msg: OccupancyGrid):
        """Convert OccupancyGrid to grayscale uint8 image and store as SlamOccupancyMap.

        ROS values (int8): 0=free→255, 100=occupied→0, -1=unknown→128.
        Vertically flipped so top-left = map north.
        """
        global _latest_occ
        w = msg.info.width
        h = msg.info.height
        print(f"[rtabmap-bridge] grid_map CB: {w}×{h}", file=sys.stderr, flush=True)
        if w == 0 or h == 0:
            return

        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        img  = np.full((h, w), 128, dtype=np.uint8)
        img[data == 0]   = 255
        img[data == 100] = 0
        img = np.flipud(img)   # ROS origin is bottom-left → flip to top-left

        occ = env_pb.SlamOccupancyMap(
            image      = img.tobytes(),
            width      = w,
            height     = h,
            resolution = msg.info.resolution,
            origin_x   = msg.info.origin.position.x,
            origin_y   = msg.info.origin.position.y,
        )
        with _slam_lock:
            _latest_occ = occ
        print(f"[rtabmap-bridge] grid_map: {w}×{h} @ {msg.info.resolution:.3f} m/cell",
              file=sys.stderr)
        self.get_logger().info(
            f"grid_map: {w}×{h} @ {msg.info.resolution:.3f} m/cell",
            throttle_duration_sec=10.0,
        )


# ---------------------------------------------------------------------------
# Quaternion from rotation matrix
# ---------------------------------------------------------------------------

def _rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert 3×3 rotation matrix to (qw, qx, qy, qz)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s  = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s  = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s  = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s  = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qw, qx, qy, qz


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global _env_stub

    print("[rtabmap-bridge] starting…", file=sys.stderr)

    endpoint = _discover_env_grpc()
    env_channel = grpc.insecure_channel(endpoint)
    _env_stub = env_pb_grpc.EnvDataServiceStub(env_channel)
    print(f"[rtabmap-bridge] connected to env gRPC at {endpoint}", file=sys.stderr)

    server_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")

    # Start SlamDataService gRPC server and register it with robonix-atlas
    slam_port = _pick_port()
    threading.Thread(target=_start_slam_grpc, args=(slam_port,), daemon=True).start()
    _register_slam_interface(server_addr, slam_port)
    _start_heartbeat(server_addr)

    rclpy.init()
    node = RtabmapBridge()
    print(f"[rtabmap-bridge] ROS2 publishing at {BRIDGE_FPS} Hz", file=sys.stderr)
    print("[rtabmap-bridge] ready", file=sys.stderr)

    # _poll_env makes a blocking gRPC call (GetObs) that can take 100-500 ms when
    # rt-fast ray-tracing is enabled.  rclpy.spin() runs a single-threaded
    # executor, so any blocking timer callback starves _on_cloud_map and
    # _on_grid_map subscriptions — they simply never fire.
    # MultiThreadedExecutor gives subscription callbacks their own threads so
    # they can run in parallel with the blocking timer.
    # _poll_env blocks for ~200-500 ms per call (rt-fast rendering).
    # At 10 Hz that means up to 5 concurrent blocking calls.
    # Use 16 threads so subscription callbacks always have a free thread.
    executor = MultiThreadedExecutor(num_threads=16)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
