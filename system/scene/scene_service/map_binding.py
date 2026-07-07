# SPDX-License-Identifier: MulanPSL-2.0
"""Map identity binding — how scene learns WHICH map it is scoped to.

The mapping service broadcasts its current identity {map_id, mode,
generation} latched on the `robonix/service/map/lifecycle` topic (see
capabilities/service/map/lifecycle.v1.toml). At startup scene probes that
broadcast once and, when present, binds to it — the broadcast is
authoritative, so the deploy no longer needs to hand-copy the same map_id
into two config blocks. Static fallbacks (manifest config / SCENE_MAP_ID
env / "default") remain for deployments where mapping is not up yet when
scene starts (the normal full-boot order) or does not broadcast at all.

`generation` is mapping's map-frame epoch: it bumps whenever the map
origin may have changed (mapping-mode session start, reset_map) and stays
put across localization round-trips. P2 records it and warns on runtime
change; acting on it (flush + re-anchor) is the P3 lifecycle linkage.

Split from service.py so the pure precedence rule and the one-shot ROS
read are unit-testable without atlas.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger("scene.map_binding")


@dataclass(frozen=True)
class MapBinding:
    """The resolved binding plus where it came from (for the startup log
    and for the runtime mismatch watcher)."""
    map_id: str
    source: str                      # "lifecycle" | "config" | "env" | "default"
    mode: str = ""                   # only set when source == "lifecycle"
    generation: Optional[int] = None  # only set when source == "lifecycle"


def choose_map_binding(
    broadcast: Optional[dict],
    config_map_id: object,
    env_map_id: Optional[str],
) -> MapBinding:
    """Precedence: mapping's lifecycle broadcast (authoritative source of
    map identity) > manifest config.map_id > SCENE_MAP_ID env > "default".
    A broadcast with an empty map_id means mapping runs ephemeral (no named
    map) — treated as "no broadcast", scene falls back to static binding."""
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
    """One-shot read of the latched MapLifecycle sample on `topic`.

    Runs on a PRIVATE rclpy context + executor so it neither initialises
    nor disturbs the default context the ingest hub owns (the hub calls
    rclpy.init() later and would raise on a double init). Returns
    {map_id, mode, generation} or None on timeout. Degrades to None with
    one warning when rclpy or the generated `map` interface package is
    missing (ros2_idl overlay not built) — binding then falls back to
    config/env, scene itself keeps working.
    """
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
        # Context/init INSIDE the guard: a broken RMW env must degrade to
        # None per the docstring promise, not escape and kill scene startup.
        ctx = rclpy.Context()
        rclpy.init(context=ctx, args=None)
        node = rclpy.create_node("scene_map_binding_probe", context=ctx)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        got: list = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            # TRANSIENT_LOCAL: the broadcast is latched; a late-joining
            # probe must receive the cached sample.
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
