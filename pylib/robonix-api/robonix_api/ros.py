# SPDX-License-Identifier: MulanPSL-2.0
"""Lazy rclpy wrapper. We do NOT import rclpy at module load — it's heavy and
not present on every robonix host. The singleton is initialised on first use.

The node owns one MultiThreadedExecutor + thread; every Capability publisher /
subscription / sentinel runs against this single node. Caller code never sees
rclpy directly through Capability methods — but if you want raw rclpy you can
get the underlying node via `RosBackend.get().node`.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

log = logging.getLogger("robonix_api.ros")


class RosBackend:
    """Process-wide singleton. Capability.create_publisher / create_subscription /
    wait_for_topic all funnel through this so we don't fork multiple rclpy
    contexts."""

    _instance: "RosBackend | None" = None
    _instance_lock = threading.Lock()

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node = None
        self._executor = None
        self._spin_thread: threading.Thread | None = None
        self._started = False

    @classmethod
    def get(cls) -> "RosBackend":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = RosBackend()
            return cls._instance

    def _ensure_started(self, node_name: str = "robonix_capability") -> None:
        with self._lock:
            if self._started:
                return
            import rclpy  # type: ignore
            from rclpy.executors import MultiThreadedExecutor  # type: ignore
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node(node_name)
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(
                target=self._spin, name="robonix-ros-spin", daemon=True,
            )
            self._spin_thread.start()
            self._started = True
            log.info("rclpy backend started (node=%s)", node_name)

    def _spin(self) -> None:
        import rclpy  # type: ignore
        try:
            from rclpy.handle import InvalidHandle  # type: ignore
        except ImportError:
            # Humble exposes InvalidHandle only from the pybind module;
            # newer rclpy releases provide the public rclpy.handle path.
            from rclpy._rclpy_pybind11 import InvalidHandle  # type: ignore

        while rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except InvalidHandle as e:
                # A short-lived sentinel subscription may be destroyed by the
                # lifecycle thread while the executor is rebuilding its wait
                # set.  rclpy reports that normal entity hand-off as a
                # transient InvalidHandle; the next spin rebuilds the set.
                # Exiting here would permanently stop every ROS callback in
                # the provider (including latched camera extrinsics).
                log.debug("rclpy entity changed during spin; retrying: %s", e)
                time.sleep(0.01)
            except Exception as e:  # noqa: BLE001
                log.warning("rclpy spin loop exited: %s", e)
                return

    @property
    def node(self):
        self._ensure_started()
        return self._node

    def create_publisher(self, msg_type: type, topic: str, qos=10):
        """qos accepts a string ('best_effort' / 'reliable' / 'latched'), an int
        (queue depth, history KEEP_LAST), or an rclpy QoSProfile passed through."""
        self._ensure_started()
        return self._node.create_publisher(msg_type, topic, self.qos(qos))

    def create_subscription(self, msg_type: type, topic: str,
                            callback: Callable[[Any], None], qos=10):
        self._ensure_started()
        return self._node.create_subscription(msg_type, topic, callback, self.qos(qos))

    def wait_for_topic(
        self, topic: str, msg_type: type, timeout_s: float,
    ) -> bool:
        """Subscribe transiently and block until first message or timeout."""
        self._ensure_started()
        seen = threading.Event()
        sub = self.create_subscription(
            msg_type, topic, lambda _m: seen.set(), qos="best_effort",
        )
        try:
            return seen.wait(timeout=timeout_s)
        finally:
            try:
                self._node.destroy_subscription(sub)
            except Exception:  # noqa: BLE001
                pass

    @staticmethod
    def qos(spec):
        """Coerce a friendly QoS spec into an rclpy QoSProfile.
            str:  "best_effort" | "reliable" | "latched" (transient_local + reliable)
            int:  queue depth, history=KEEP_LAST, durability=VOLATILE, reliability=RELIABLE
            QoSProfile: passed through unchanged."""
        from rclpy.qos import (  # type: ignore
            QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
        )
        if isinstance(spec, str):
            name = spec.lower()
            if name == "latched":
                return QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )
            rel = {
                "best_effort": ReliabilityPolicy.BEST_EFFORT,
                "reliable":    ReliabilityPolicy.RELIABLE,
            }.get(name, ReliabilityPolicy.RELIABLE)
            return QoSProfile(
                reliability=rel,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        if isinstance(spec, int):
            return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=spec)
        return spec  # already a QoSProfile

    def shutdown(self) -> None:
        with self._lock:
            if not self._started:
                return
            try:
                import rclpy  # type: ignore
                if self._executor is not None:
                    self._executor.shutdown()
                if self._node is not None:
                    self._node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:  # noqa: BLE001
                log.debug("rclpy shutdown raised: %s", e)
            self._started = False


def resolve_msg_type(name: str) -> type:
    """`'sensor_msgs/Image'` / `'sensor_msgs/msg/Image'` / `'PointCloud2'` →
    actual rclpy message class. Raises ImportError on miss."""
    n = name.strip()
    if "/" in n:
        parts = [p for p in n.split("/") if p and p != "msg"]
        if len(parts) != 2:
            raise ValueError(f"unrecognized msg type {name!r}")
        pkg, cls = parts
    else:
        # Bare class name; try the common packages in order.
        candidates = ("sensor_msgs", "std_msgs", "geometry_msgs", "nav_msgs")
        for pkg in candidates:
            try:
                mod = __import__(f"{pkg}.msg", fromlist=[n])
                return getattr(mod, n)
            except (ImportError, AttributeError):
                continue
        raise ImportError(f"could not locate msg type {name!r} in {candidates}")
    mod = __import__(f"{pkg}.msg", fromlist=[cls])
    return getattr(mod, cls)
