# SPDX-License-Identifier: MulanPSL-2.0
"""ROS2 subscribers — scene's primary ingest path. Runs an rclpy node
in a dedicated thread, subscribes to robot-self-pose / 2D laser /
RGB / depth topics, and stashes the latest sample of each behind an
asyncio-friendly accessor.
The asyncio side never touches rclpy callbacks directly — they post
into thread-safe slots; the periodic ticks that consume the data
(VLM perception, self-tracker, plane extraction) read those slots
on their own schedule. Keeps GIL contention predictable and means
the relation engine / MCP server don't stall behind a fat pointcloud
callback.

Topic names come from atlas channel declarations or
`RBNX_CONFIG_FILE.observations[]` so a new robot does not need a code edit —
just declare which topic publishes which observation kind.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

log = logging.getLogger(__name__)


# These imports are deferred until start() is called so that scene's
# import-time code path doesn't blow up on hosts that don't have
# rclpy (e.g. running the unit tests). The container build always
# has rclpy.
def _import_ros():
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy  # type: ignore
    from rclpy.duration import Duration  # type: ignore
    from rclpy.time import Time  # type: ignore
    from sensor_msgs.msg import Image, LaserScan, PointCloud2, CameraInfo  # type: ignore
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped  # type: ignore
    from nav_msgs.msg import Odometry, OccupancyGrid  # type: ignore
    # tf2 is the authoritative source for `map → base_link` and
    # `map → camera_optical`. Reading /odom directly puts the robot in
    # *odom-frame* — once SLAM accumulates drift, the map → odom
    # transform is non-identity, and the web UI shows the robot offset
    # from where rviz (which goes through tf) shows it.
    from tf2_ros import Buffer, TransformListener  # type: ignore
    # map/msg/MapLifecycle comes from the generated ros2_idl overlay
    # (rbnx codegen --ros2 + colcon build), NOT the ROS distro. Import it
    # defensively: a deploy without the overlay must lose only the
    # lifecycle subscription, not every scene subscription.
    try:
        from map.msg import MapLifecycle  # type: ignore
    except ImportError:
        MapLifecycle = None
    return {
        "rclpy": rclpy,
        "Node": Node,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
        "Duration": Duration,
        "Time": Time,
        "Image": Image,
        "LaserScan": LaserScan,
        "CameraInfo": CameraInfo,
        # mapping declares /rtabmap/cloud_map under
        # robonix/service/map/pointcloud — scene auto-classifies it
        # as kind=lidar3d. Without this import scene crashes the
        # moment mapping appears.
        "PointCloud2": PointCloud2,
        "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
        "TransformStamped": TransformStamped,
        "Odometry": Odometry,
        "OccupancyGrid": OccupancyGrid,
        # None when the ros2_idl overlay is missing — _subscribe skips the
        # spec with a warning instead of crashing the hub.
        "MapLifecycle": MapLifecycle,
        "Buffer": Buffer,
        "TransformListener": TransformListener,
    }


@dataclass
class TopicSpec:
    """One observation kind's wiring. `kind` is the abstract name
    (rgb / depth / lidar2d / pose / odom); `topic` is the concrete
    ROS topic; `msg_type` selects which sensor_msgs / nav_msgs class
    to import. Optional `qos_profile` lets a config override the
    default reliable-latched profile when needed (e.g. /amcl_pose
    needs RELIABLE + KEEP_LAST(1))."""
    kind: str
    topic: str
    msg_type: str            # "Image" | "LaserScan" | "PoseWithCovarianceStamped" | "Odometry"
    qos_profile: str = "default"


class _LatestSlot:
    """Thread-safe cache of the most recent message on one topic.
    Writers are rclpy callbacks (background thread); readers are
    asyncio ticks. We don't queue — VLM / relation engine only ever
    care about the latest frame."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._msg: Any = None
        self._stamp_unix: float = 0.0
        self._count: int = 0

    def write(self, msg: Any) -> None:
        with self._lock:
            self._msg = msg
            self._stamp_unix = time.time()
            self._count += 1

    def read(self) -> tuple[Any, float, int]:
        with self._lock:
            return self._msg, self._stamp_unix, self._count


class SubscribersHub:
    """Owns the rclpy.Node + spin thread + per-kind latest slots.
    Lifecycle:

      hub = SubscribersHub(specs=[...])
      await hub.start()        # spawns spin thread, creates subs
      ...
      await hub.stop()         # destroys node, joins thread
    """

    def __init__(self, specs: list[TopicSpec], *, node_name: str = "scene_subscribers") -> None:
        self.specs = specs
        self.node_name = node_name
        self._slots: dict[str, _LatestSlot] = {s.kind: _LatestSlot() for s in specs}
        self._ros: dict[str, Any] | None = None
        self._node: Any = None
        self._spin_thread: threading.Thread | None = None
        self._stop_evt = threading.Event()
        # tf2 buffer + listener — populated in start() once rclpy is up.
        self._tf_buffer: Any = None
        self._tf_listener: Any = None

    # ── lifecycle ──────────────────────────────────────────────────────────
    async def start(self) -> None:
        if self._ros is not None:
            return
        try:
            self._ros = _import_ros()
        except ImportError as e:
            log.error("[scene-ros] rclpy not available: %s", e)
            self._ros = None
            return
        rclpy = self._ros["rclpy"]
        rclpy.init(args=None)
        Node = self._ros["Node"]
        node = Node(self.node_name)
        self._node = node
        # tf2 buffer + listener spin alongside the topic subs. The
        # buffer accumulates /tf + /tf_static; consumers call
        # `lookup_transform(map, base_link, ...)` to get the SLAM-
        # corrected pose without ever touching /odom or /amcl_pose.
        Buffer = self._ros["Buffer"]
        TransformListener = self._ros["TransformListener"]
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        for spec in self.specs:
            self._subscribe(spec)
        self._spin_thread = threading.Thread(
            target=self._spin_loop,
            name="scene-rclpy-spin",
            daemon=True,
        )
        self._spin_thread.start()
        log.info("[scene-ros] hub up: %d topic(s) + tf2", len(self.specs))

    def add_spec(self, spec: TopicSpec) -> bool:
        """Add a new (kind, topic) subscription to a hub already up.
        Used by the background reconciler to absorb topics that come
        online after start(). Returns True if added, False if the
        kind was already known — or if the subscription could not be
        created (missing msg class): committing the slot anyway would
        make has_kinds() claim a subscription that doesn't exist and
        pair a success log with the skip warning."""
        if self._ros is None:
            return False
        if spec.kind in self._slots:
            return False
        self._slots[spec.kind] = _LatestSlot()
        if not self._subscribe(spec):
            del self._slots[spec.kind]
            return False
        self.specs.append(spec)
        log.info("[scene-ros] dynamic add: %s on %s", spec.kind, spec.topic)
        return True

    def has_kinds(self) -> set[str]:
        return set(self._slots.keys())

    async def stop(self) -> None:
        if self._ros is None:
            return
        self._stop_evt.set()
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._ros["rclpy"].shutdown()
        except Exception:  # noqa: BLE001
            pass
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        self._ros = None
        self._node = None

    # ── spin loop ──────────────────────────────────────────────────────────
    def _spin_loop(self) -> None:
        rclpy = self._ros["rclpy"]  # type: ignore[index]
        while not self._stop_evt.is_set():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:  # noqa: BLE001
                log.debug("[scene-ros] spin tick: %s", e)

    # ── subscription wiring ────────────────────────────────────────────────
    def _subscribe(self, spec: TopicSpec) -> bool:
        """Create the rclpy subscription for one spec. Returns False (with
        a warning) when the msg class is unavailable — the hub and every
        other subscription keep running."""
        assert self._ros is not None
        msg_cls = self._ros.get(spec.msg_type)
        if msg_cls is None:
            # Unknown or unavailable msg class (e.g. MapLifecycle without
            # the ros2_idl overlay). One subscription degrades, the hub —
            # and every other subscription — keeps running.
            log.warning(
                "[scene-ros] msg type %r unavailable — skipping %s on %s "
                "(generated interface overlay missing?)",
                spec.msg_type, spec.kind, spec.topic,
            )
            return False
        QoSProfile = self._ros["QoSProfile"]
        ReliabilityPolicy = self._ros["ReliabilityPolicy"]
        DurabilityPolicy = self._ros["DurabilityPolicy"]
        HistoryPolicy = self._ros["HistoryPolicy"]

        # Per-topic QoS. ROS2's compatibility rule: a subscriber's
        # reliability must be ≤ the publisher's, so RELIABLE-pub +
        # BEST_EFFORT-sub silently drops every message — caught by
        # `ros2 topic info -v` showing zero data on a `topic echo`.
        # Webots's camera plugin publishes RELIABLE, Nav2 publishes
        # /amcl_pose RELIABLE, etc. So we default to RELIABLE here
        # and trust ROS to coerce when a publisher is BEST_EFFORT.
        # If a deployment needs to opt back into BEST_EFFORT for a
        # specific topic (e.g. a real lidar at 100 Hz), the spec
        # could grow a `qos_profile` field; not needed for v1.
        if spec.kind in ("rgb", "depth"):
            # KEEP_LAST(1): cameras spam at >10 Hz; only the freshest
            # frame is useful for the periodic VLM tick.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        elif spec.kind == "lidar2d":
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=2,
            )
        elif spec.kind == "occupancy_grid":
            # slam_toolbox publishes /map RELIABLE + TRANSIENT_LOCAL
            # (the standard "latched" map). A late subscriber needs
            # TRANSIENT_LOCAL to receive the cached snapshot.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        elif spec.kind == "intrinsics":
            # Pinhole K (primitive/camera/intrinsics). Real cameras
            # (e.g. realsense2_camera) publish camera_info CONTINUOUSLY
            # alongside every frame with DURABILITY=VOLATILE — a
            # TRANSIENT_LOCAL subscriber is then QoS-incompatible and
            # receives NOTHING ("waiting for camera intrinsics" forever).
            # VOLATILE is compatible with both a volatile continuous
            # publisher and a transient-local latched one, so use it.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
        elif spec.kind == "map_lifecycle":
            # mapping's identity broadcast is latched (published once per
            # lifecycle transition); TRANSIENT_LOCAL picks up the cached
            # sample on a late start.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        elif spec.kind == "camera_extrinsics":
            # Static camera mount transform (primitive/camera/extrinsics):
            # genuinely latched (published once), so TRANSIENT_LOCAL is
            # needed to pick up the cached value on a late start.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        else:
            # Pose / odom are typically reliable + small.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

        slot = self._slots[spec.kind]
        _kind = spec.kind  # closure capture

        def _cb(msg: Any, _slot: _LatestSlot = slot, _k: str = _kind) -> None:
            _slot.write(msg)
            # First arrival debug — proves the subscription wiring is alive
            # end-to-end. Skipped after count > 1 so we don't spam. Critical
            # for diagnosing the "subscribed but no msg arrives" class of bug
            # (e.g. DDS GUID mismatch after a republisher restart).
            if _slot._count == 1:
                log.info("[scene-ros] FIRST msg on %s (kind=%s)", spec.topic, _k)

        self._node.create_subscription(msg_cls, spec.topic, _cb, qos)
        log.info("[scene-ros] subscribed: %s on %s (%s, qos=%s)",
                 spec.kind, spec.topic, spec.msg_type, spec.kind)
        return True

    # ── consumer-side accessors ────────────────────────────────────────────
    def latest(self, kind: str) -> tuple[Any, float, int]:
        """Returns (msg, stamp_unix, count). msg is None until the
        first callback arrives; readers should branch on stamp_unix > 0."""
        slot = self._slots.get(kind)
        if slot is None:
            return None, 0.0, 0
        return slot.read()

    def has(self, kind: str) -> bool:
        return kind in self._slots

    # ── tf2 accessors ────────────────────────────────────────────────────
    def lookup_xy_yaw(
        self,
        target_frame: str = "base_link",
        source_frame: str = "map",
    ) -> Optional[tuple[float, float, float, float]]:
        """Return (x, y, z, yaw) of `target_frame` expressed in
        `source_frame` via tf2, or None when the transform isn't
        available yet. This is the SLAM-corrected pose: it goes
        through /tf (rtabmap publishes map→odom; chassis publishes
        odom→base_link), so it tracks rviz exactly. Reading /odom
        instead leaves the consumer in odom-frame and drifts away
        from the map once SLAM corrects.
        """
        if self._ros is None or self._tf_buffer is None:
            return None
        try:
            Time = self._ros["Time"]
            Duration = self._ros["Duration"]
            tf = self._tf_buffer.lookup_transform(
                source_frame, target_frame,
                Time(),                        # latest available
                Duration(seconds=0.1),
            )
        except Exception as e:  # noqa: BLE001
            log.debug("[scene-ros] tf2 lookup %s→%s failed: %s",
                      source_frame, target_frame, e)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # Quaternion → yaw (around Z).
        import math
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return float(t.x), float(t.y), float(t.z), float(yaw)

    def lookup_transform_4x4(
        self,
        target_frame: str,
        source_frame: str = "map",
    ):
        """Return a 4x4 numpy homogeneous transform that maps points
        from `target_frame` into `source_frame`. None when tf isn't
        available. Used by perception to project camera-optical
        depth points directly into map frame without composing odom-
        frame chassis pose by hand."""
        if self._ros is None or self._tf_buffer is None:
            return None
        try:
            import numpy as np
            Time = self._ros["Time"]
            Duration = self._ros["Duration"]
            tf = self._tf_buffer.lookup_transform(
                source_frame, target_frame,
                Time(),
                Duration(seconds=0.1),
            )
        except Exception as e:  # noqa: BLE001
            log.debug("[scene-ros] tf2 4x4 %s→%s failed: %s",
                      source_frame, target_frame, e)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # Quaternion → 3x3 rotation.
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        n = x*x + y*y + z*z + w*w
        if n < 1e-12:
            return None
        s = 2.0 / n
        wx, wy, wz = s*w*x, s*w*y, s*w*z
        xx, xy, xz = s*x*x, s*x*y, s*x*z
        yy, yz, zz = s*y*y, s*y*z, s*z*z
        R = np.array([
            [1.0 - (yy + zz), xy - wz,         xz + wy],
            [xy + wz,         1.0 - (xx + zz), yz - wx],
            [xz - wy,         yz + wx,         1.0 - (xx + yy)],
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [float(t.x), float(t.y), float(t.z)]
        return T
