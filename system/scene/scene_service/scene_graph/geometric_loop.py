# SPDX-License-Identifier: MulanPSL-2.0
"""Fast geometric relation loop (a few Hz).

Publishes the node set and the edges geometry decides on its own:
`reachable_by` (gripper distance) and, with SCENE_RELATIONS=geometric (the
default), strict contact/containment plus capped `near`. The builder adds
the semantic edges on its slower cadence.
"""
from __future__ import annotations

import asyncio
import logging
import math
import os
from typing import Iterable, Optional

from ..state.object_registry import Pose3D, SceneObject
from .builder import _is_stable, _object_to_node
from .geometry import CONF_MED, strict_geometric_relations
from .store import SceneGraphStore
from .types import INVERSE_RELATIONS, SceneGraphEdge

log = logging.getLogger(__name__)


def _env_float(key: str, default: float) -> float:
    try:
        return float(os.environ.get(key, str(default)))
    except ValueError:
        return default


def _env_int(key: str, default: int) -> int:
    try:
        return int(os.environ.get(key, str(default)))
    except ValueError:
        return default


# Distance-only reachability until Soma provides kinematics.
_REACHABLE_RADIUS_M = 1.0
_RELATIONS_MODE = os.environ.get("SCENE_RELATIONS", "geometric").strip().lower()  # geometric | reachable_only
# `near` is capped per object so a crowded room does not become a complete graph.
_NEAR_MAX_PER_OBJECT = _env_int("SCENE_NEAR_MAX_PER_OBJECT", 3)
_NEAR_RADIUS_M = _env_float("SCENE_NEAR_RADIUS_M", 1.5)
_GRIPPER_OBJECT_PREFIX = "robot.right_gripper"


def _dist3(a: Pose3D, b: Pose3D) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _find_gripper(objects: Iterable[SceneObject]) -> Optional[SceneObject]:
    """The gripper self-object, else any robot self-object."""
    for o in objects:
        if o.object_id.startswith(_GRIPPER_OBJECT_PREFIX):
            return o
    for o in objects:
        if o.attributes.get("is_robot"):
            return o
    return None


class GeometricRelationLoop:
    """Owns the store's geometric slice; debounces edges against pose jitter."""

    def __init__(
        self,
        registry,
        store: SceneGraphStore,
        *,
        hz: Optional[float] = None,
        min_observations: Optional[int] = None,
    ) -> None:
        self.registry = registry
        self.store = store
        self.period_s = 1.0 / (hz or _env_float("SCENE_RELATION_HZ", 3.0))
        self.min_observations = (
            min_observations
            if min_observations is not None
            else _env_int("SCENE_GRAPH_MIN_OBSERVATIONS", 2)
        )
        # An edge is emitted at score >= _enter and dropped at 0; _max caps
        # how long it coasts through dropouts.
        self._enter = 2
        self._max = 3
        self._scores: dict[tuple[str, str, str], tuple[int, SceneGraphEdge]] = {}
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()

    async def start(self) -> None:
        if self._task is not None:
            return
        self._stop.clear()
        self._task = asyncio.create_task(self._run(), name="scene-geometric-loop")
        log.info(
            "[scene-geo] geometric relation loop started (%.1f Hz, min_obs=%d)",
            1.0 / self.period_s,
            self.min_observations,
        )

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
            except Exception as e:  # noqa: BLE001 — never let the loop die
                log.warning("[scene-geo] tick failed: %s", e)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=self.period_s)
            except asyncio.TimeoutError:
                pass

    async def _tick(self) -> None:
        objs_dict = await self.registry.snapshot()
        objects = list(objs_dict.values())
        stable = [o for o in objects if _is_stable(o, self.min_observations)]
        nodes = [_object_to_node(o) for o in stable]
        raw = self._reachable_edges(objects, stable)
        if _RELATIONS_MODE == "geometric":
            raw.extend(self._object_relations(nodes))
        self.store.set_geometric(nodes, self._debounce(raw))

    def _reachable_edges(
        self, objects: list[SceneObject], stable: list[SceneObject]
    ) -> list[SceneGraphEdge]:
        """object→gripper edges within reach. The gripper is not a node, so
        these edges point outside the node set."""
        gripper = _find_gripper(objects)
        if gripper is None:
            return []
        return [
            SceneGraphEdge(
                source_id=o.object_id,
                target_id=gripper.object_id,
                relation="reachable_by",
                confidence=CONF_MED,
                method="geometric",
                reason="geometry: within gripper reach radius (distance-only)",
            )
            for o in stable
            if o.object_id != gripper.object_id
            and _dist3(o.pose, gripper.pose) <= _REACHABLE_RADIUS_M
        ]

    def _object_relations(self, nodes) -> list[SceneGraphEdge]:
        """Contact/containment both ways; `near` only for each object's closest
        `_NEAR_MAX_PER_OBJECT` neighbours and only when nothing stronger holds."""
        out: list[SceneGraphEdge] = []
        near_candidates: dict[str, list[tuple[float, str]]] = {}
        for i, a in enumerate(nodes):
            for b in nodes[i + 1:]:
                for rel, conf, dist in strict_geometric_relations(a, b, _NEAR_RADIUS_M):
                    if rel == "near":
                        near_candidates.setdefault(a.object_id, []).append((dist, b.object_id))
                        near_candidates.setdefault(b.object_id, []).append((dist, a.object_id))
                        continue
                    reason = f"geometry: {rel} (centre distance {dist:.2f} m)"
                    out.append(SceneGraphEdge(a.object_id, b.object_id, rel, conf, "geometric", reason))
                    out.append(SceneGraphEdge(
                        b.object_id, a.object_id, INVERSE_RELATIONS[rel], conf, "geometric", reason))
        seen: set[tuple[str, str]] = set()
        for src, cands in near_candidates.items():
            for dist, dst in sorted(cands)[:_NEAR_MAX_PER_OBJECT]:
                if (src, dst) in seen:
                    continue
                seen.add((src, dst))
                out.append(SceneGraphEdge(
                    src, dst, "near", CONF_MED, "geometric",
                    f"geometry: within {_NEAR_RADIUS_M:.1f} m (centre distance {dist:.2f} m)",
                ))
        return out

    def _debounce(self, edges: list[SceneGraphEdge]) -> list[SceneGraphEdge]:
        """Present keys gain a point (capped at _max), absent keys lose one."""
        present = {(e.source_id, e.target_id, e.relation): e for e in edges}
        new_state: dict[tuple[str, str, str], tuple[int, SceneGraphEdge]] = {}
        emitted: list[SceneGraphEdge] = []
        for key in set(self._scores) | set(present):
            prev_score, prev_edge = self._scores.get(key, (0, None))
            if key in present:
                score, edge = min(self._max, prev_score + 1), present[key]
            else:
                score, edge = prev_score - 1, prev_edge
            if score <= 0 or edge is None:
                continue
            new_state[key] = (score, edge)
            if score >= self._enter:
                emitted.append(edge)
        self._scores = new_state
        return emitted
