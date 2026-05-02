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

Topic names come from `RBNX_CONFIG_FILE.observations[]` so a new
robot doesn't need a code edit — just declare which topic publishes
which observation kind. Default mapping matches Webots Tiago.
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
    from sensor_msgs.msg import Image, LaserScan  # type: ignore
    from geometry_msgs.msg import PoseWithCovarianceStamped  # type: ignore
    from nav_msgs.msg import Odometry  # type: ignore
    return {
        "rclpy": rclpy,
        "Node": Node,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
        "Image": Image,
        "LaserScan": LaserScan,
        "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
        "Odometry": Odometry,
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


# Default topic mapping per robot platform. The Soma adapter in
# service.py reads RBNX_CONFIG_FILE.observations[] and overrides
# this; this dict is only the fallback for an empty config.
DEFAULT_WEBOTS_TIAGO_TOPICS = [
    TopicSpec(kind="rgb",     topic="/head_front_camera/rgb/image_raw",     msg_type="Image"),
    TopicSpec(kind="depth",   topic="/head_front_camera/depth_registered/image_raw", msg_type="Image"),
    TopicSpec(kind="lidar2d", topic="/scan",                                msg_type="LaserScan"),
    TopicSpec(kind="pose",    topic="/amcl_pose",                           msg_type="PoseWithCovarianceStamped"),
    TopicSpec(kind="odom",    topic="/odom",                                msg_type="Odometry"),
]


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
        for spec in self.specs:
            self._subscribe(spec)
        self._spin_thread = threading.Thread(
            target=self._spin_loop,
            name="scene-rclpy-spin",
            daemon=True,
        )
        self._spin_thread.start()
        log.info("[scene-ros] hub up: %d topic(s)", len(self.specs))

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
    def _subscribe(self, spec: TopicSpec) -> None:
        assert self._ros is not None
        msg_cls = self._ros[spec.msg_type]
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
        else:
            # Pose / odom are typically reliable + small.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

        slot = self._slots[spec.kind]

        def _cb(msg: Any, _slot: _LatestSlot = slot) -> None:
            _slot.write(msg)

        self._node.create_subscription(msg_cls, spec.topic, _cb, qos)
        log.info("[scene-ros] subscribed: %s on %s (%s, qos=%s)",
                 spec.kind, spec.topic, spec.msg_type, spec.kind)

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
