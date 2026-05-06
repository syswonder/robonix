# SPDX-License-Identifier: MulanPSL-2.0
"""Relation engine — pure-geometric predicates over the object registry.
Runs at 1 Hz as its own asyncio task. Output is a list of (subject,
predicate, target) triples cached on the engine and exposed via
`current()` / served by the snapshot tools.
"""
from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from typing import Iterable, Optional

from .object_registry import ObjectRegistry, Pose3D, SceneObject


# Public predicate names — kept as a tuple so the MCP-tool input
# validator can sanity-check `RelationConstraint.predicate`.
RELATION_PREDICATES = ("on", "inside", "near", "reachable_by")

# Per-(subject_cls, target_cls) override for `near`. Falls back to a
# default radius when no entry exists.
_NEAR_THRESHOLDS_M: dict[tuple[str, str], float] = {
    ("person", "robot"): 1.5,
    ("robot", "person"): 1.5,
    ("cup", "tray"): 0.4,
    ("cup", "table"): 0.6,
}
_DEFAULT_NEAR_THRESHOLD_M = 1.0

# Reachability v1: pure distance from gripper to object. Real
# kinematics-based reachability waits for Soma.
# TODO(soma-kinematics)
_REACHABLE_RADIUS_M = 1.0
_GRIPPER_OBJECT_PREFIX = "robot.right_gripper"  # canonical name used by self-tracker


@dataclass(frozen=True)
class RelationTriple:
    subject_object_id: str
    predicate: str             # "on" | "inside" | "near" | "reachable_by"
    target_object_id: str


def _dist3(a: Pose3D, b: Pose3D) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _xy_overlap(a: SceneObject, b: SceneObject) -> bool:
    """Axis-aligned XY overlap between two bboxes centered on their poses."""
    ax_lo, ax_hi = a.pose.x - a.bbox.half_x, a.pose.x + a.bbox.half_x
    ay_lo, ay_hi = a.pose.y - a.bbox.half_y, a.pose.y + a.bbox.half_y
    bx_lo, bx_hi = b.pose.x - b.bbox.half_x, b.pose.x + b.bbox.half_x
    by_lo, by_hi = b.pose.y - b.bbox.half_y, b.pose.y + b.bbox.half_y
    return ax_hi >= bx_lo and bx_hi >= ax_lo and ay_hi >= by_lo and by_hi >= ay_lo


def _on(a: SceneObject, b: SceneObject) -> bool:
    """`a` rests on top of `b`: a's bottom is within 5cm of b's top, and
    their xy projections overlap. Order matters."""
    a_bottom_z = a.pose.z - a.bbox.half_z
    b_top_z = b.pose.z + b.bbox.half_z
    if abs(a_bottom_z - b_top_z) > 0.05:
        return False
    return _xy_overlap(a, b)


def _inside(a: SceneObject, b: SceneObject) -> bool:
    """Center of `a` lies inside `b`'s axis-aligned bbox."""
    return (
        b.pose.x - b.bbox.half_x <= a.pose.x <= b.pose.x + b.bbox.half_x
        and b.pose.y - b.bbox.half_y <= a.pose.y <= b.pose.y + b.bbox.half_y
        and b.pose.z - b.bbox.half_z <= a.pose.z <= b.pose.z + b.bbox.half_z
    )


def _near(a: SceneObject, b: SceneObject) -> bool:
    threshold = _NEAR_THRESHOLDS_M.get((a.cls, b.cls), _DEFAULT_NEAR_THRESHOLD_M)
    return _dist3(a.pose, b.pose) <= threshold


def _reachable_by(a: SceneObject, gripper: Optional[SceneObject]) -> bool:
    """Distance-only reachability stub. Returns False if the gripper
    object isn't in the registry yet (no self-tracker, or robot not
    yet observed); never raises so the relation engine keeps running.
    TODO(soma-kinematics): replace with proper IK reach check."""
    if gripper is None:
        return False
    return _dist3(a.pose, gripper.pose) <= _REACHABLE_RADIUS_M


def _find_gripper(objects: Iterable[SceneObject]) -> Optional[SceneObject]:
    for o in objects:
        if o.object_id.startswith(_GRIPPER_OBJECT_PREFIX):
            return o
    # Fallback: any robot self-object will do (single-arm v1).
    for o in objects:
        if o.attributes.get("is_robot"):
            return o
    return None


def _compute_relations(objects: list[SceneObject]) -> list[RelationTriple]:
    """O(N²) over objects. N is small (<50 in practice); profile if it
    grows. Skip self-pairs and skip pairs where either side is missing
    (Pilot can still ask about missing objects but their relations
    aren't refreshed)."""
    out: list[RelationTriple] = []
    fresh = [o for o in objects if not o.missing]
    gripper = _find_gripper(fresh)
    for a in fresh:
        for b in fresh:
            if a is b:
                continue
            if _on(a, b):
                out.append(RelationTriple(a.object_id, "on", b.object_id))
            if _inside(a, b):
                out.append(RelationTriple(a.object_id, "inside", b.object_id))
            if a.object_id < b.object_id and _near(a, b):
                # `near` is symmetric; emit once per pair.
                out.append(RelationTriple(a.object_id, "near", b.object_id))
        if gripper is not None and a is not gripper and _reachable_by(a, gripper):
            out.append(RelationTriple(a.object_id, "reachable_by", gripper.object_id))
    return out


class RelationEngine:
    """Periodic recomputation of pairwise relations + cached snapshot.

    Read path: `engine.current()` returns the latest cached list. Reads
    don't touch the registry lock — they grab the engine's local cache
    that the periodic tick refreshed atomically. So MCP `query` calls
    don't have to wait for ingest.
    """

    def __init__(self, registry: ObjectRegistry, *, period_s: float = 1.0) -> None:
        self.registry = registry
        self.period_s = period_s
        self._relations: list[RelationTriple] = []
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()

    def current(self) -> list[RelationTriple]:
        # Tuples are immutable, list is replaced atomically in tick(),
        # so callers see a consistent snapshot.
        return list(self._relations)

    async def start(self) -> None:
        if self._task is not None:
            return
        self._stop.clear()
        self._task = asyncio.create_task(self._run(), name="scene-relation-engine")

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._task = None

    async def _run(self) -> None:
        while not self._stop.is_set():
            try:
                await self._tick()
            except Exception as e:  # noqa: BLE001 — never let the engine die
                # Logged at WARN; we'd rather show stale relations than crash.
                import logging
                logging.warning("[scene-relations] tick failed: %s", e)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=self.period_s)
            except asyncio.TimeoutError:
                pass

    async def _tick(self) -> None:
        objects, _ = await self.registry.snapshot()
        rels = _compute_relations(list(objects.values()))
        self._relations = rels
