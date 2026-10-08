# SPDX-License-Identifier: MulanPSL-2.0
"""Which map scene is bound to.

Mapping's latched lifecycle broadcast wins; the manifest's map_id, then
SCENE_MAP_ID, then "default" cover a mapping that is not up yet or does not
broadcast. `generation` is mapping's map-frame epoch; the watcher in
service.py flushes derived objects when it changes.
"""
from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger("scene.map_binding")

# Every map_id-partitioned store keys on this one rule. Unsafe in a directory
# name or URL segment: control characters, path separators and the set
# Windows reserves. Non-ASCII names are legal.
_MAP_ID_UNSAFE = re.compile(r'[\x00-\x1f\x7f/\\:*?"<>|]')


def sanitize_map_id(raw: Optional[str]) -> str:
    """A map id safe as a directory name and URL segment; all-dot names
    become "default" because "." and ".." name existing directories."""
    cleaned = _MAP_ID_UNSAFE.sub("_", (raw or "").strip())
    # Some filesystems drop trailing dots/spaces. A leading dot stays: it is
    # how the reserved `.live*` partitions are recognised.
    cleaned = cleaned.rstrip(". ")
    if not cleaned or set(cleaned) <= {"."}:
        return "default"
    return cleaned[:120]


@dataclass(frozen=True)
class MapBinding:
    """The resolved binding and where it came from."""
    map_id: str
    source: str                      # "lifecycle" | "config" | "env" | "default"
    mode: str = ""                   # only set when source == "lifecycle"
    generation: Optional[int] = None  # only set when source == "lifecycle"


def choose_map_binding(
    broadcast: Optional[dict],
    config_map_id: object,
    env_map_id: Optional[str],
) -> MapBinding:
    """broadcast > config.map_id > SCENE_MAP_ID > "default". A broadcast with
    an empty map_id (mapping runs without a named map) counts as none."""
    if broadcast and str(broadcast.get("map_id") or ""):
        return MapBinding(
            map_id=str(broadcast["map_id"]),
            source="lifecycle",
            mode=str(broadcast.get("mode") or ""),
            generation=int(broadcast.get("generation") or 0),
        )
    if config_map_id:
        return MapBinding(map_id=str(config_map_id), source="config")
    if env_map_id:
        return MapBinding(map_id=str(env_map_id), source="env")
    return MapBinding(map_id="default", source="default")


def read_latched_lifecycle(topic: str, timeout_s: float) -> Optional[dict]:
    """One read of the latched MapLifecycle sample, on a private rclpy
    context so the hub's later rclpy.init() does not clash. None on timeout
    or when rclpy / the ros2_idl overlay is missing."""
    try:
        import rclpy  # type: ignore
        from rclpy.executors import SingleThreadedExecutor  # type: ignore
        from rclpy.qos import (  # type: ignore
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        from map.msg import MapLifecycle  # type: ignore  # ros2_idl overlay
    except ImportError as e:
        log.warning(
            "[scene] lifecycle probe unavailable (%s) — falling back to "
            "static map binding (is the ros2_idl overlay built + sourced?)", e,
        )
        return None

    ctx = None
    try:
        # Inside the guard: a broken RMW env must degrade, not stop startup.
        ctx = rclpy.Context()
        rclpy.init(context=ctx, args=None)
        node = rclpy.create_node("scene_map_binding_probe", context=ctx)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        got: list = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            # Latched: a late probe still gets the sample.
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(MapLifecycle, topic, got.append, qos)
        deadline = time.monotonic() + timeout_s
        while not got and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
        if not got:
            return None
        msg = got[0]
        return {
            "map_id": str(msg.map_id),
            "mode": str(msg.mode),
            "generation": int(msg.generation),
        }
    except Exception as e:  # noqa: BLE001
        log.warning("[scene] lifecycle probe failed: %s", e)
        return None
    finally:
        if ctx is not None:
            try:
                rclpy.shutdown(context=ctx)
            except Exception:  # noqa: BLE001
                pass
