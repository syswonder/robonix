# SPDX-License-Identifier: MulanPSL-2.0
"""Fast geometric relation loop.

Produces the one metric, robot-actionable relation geometry still owns under
the VLM-primary scene graph — ``reachable_by`` (a real 3D gripper→object
distance) — at a high rate so it reaches Pilot within ~1-2 s of an object
stabilizing, independent of (and far faster than) the LLM scene-graph builder.
Writes the *geometric slice* (nodes + edges) into ``SceneGraphStore``; the
image-grounded builder fills the relational + semantic edges asynchronously.

Contact/containment (``on_top_of``/``under``/``inside``/``contains``) is **no
longer** computed here: the AABB tests misfired on full-volume boxes and
same-surface objects, so the image-grounded VLM owns those now. ``near`` is not
emitted as an edge either — proximity is served as a query
(``get_object_context.nearby_objects``).
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
from .types import SceneGraphEdge

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


# Reachability v1: pure gripper→object distance (real IK waits for Soma).
# Ported from the retired RelationEngine. TODO(soma-kinematics).
_REACHABLE_RADIUS_M = 1.0
# Deterministic object↔object relations (ThinkGraphs/BBQ style: geometry
# decides, no LLM in the loop). `near` is capped per object so a crowded room
# does not become a complete graph.
_RELATIONS_MODE = os.environ.get("SCENE_RELATIONS", "geometric").strip().lower()  # geometric | reachable_only
_NEAR_MAX_PER_OBJECT = _env_int("SCENE_NEAR_MAX_PER_OBJECT", 3)
_NEAR_RADIUS_M = _env_float("SCENE_NEAR_RADIUS_M", 1.5)
_GRIPPER_OBJECT_PREFIX = "robot.right_gripper"  # canonical self-tracker name


def _dist3(a: Pose3D, b: Pose3D) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _find_gripper(objects: Iterable[SceneObject]) -> Optional[SceneObject]:
    """Locate the robot's gripper self-object (for reachability). Prefers
    the canonical gripper id, else any robot self-object (single-arm v1)."""
    for o in objects:
        if o.object_id.startswith(_GRIPPER_OBJECT_PREFIX):
            return o
    for o in objects:
        if o.attributes.get("is_robot"):
            return o
    return None


class GeometricRelationLoop:
    """High-rate geometric relation producer; owns the store's geometric
    slice.

    Each tick recomputes contact/containment + reachable_by from the
    registry and runs the result through a debounce hysteresis band so EMA
    pose jitter (object_registry alpha=0.3) does not flicker the graph,
    while keeping emission latency under ~1 s. Read path: callers use
    ``SceneGraphStore.get_snapshot()``; this loop only writes.
    """

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
        # Debounce band: emit at score >= _ENTER (≈0.7 s at 3 Hz), drop at
        # score <= 0 (≈0.7 s grace after disappearance). _MAX caps how long
        # a long-lived edge can coast through dropouts.
        self._enter = 2
        self._max = 3
        # key -> (score, last edge seen for that key)
        self._scores: dict[tuple[str, str, str], tuple[int, SceneGraphEdge]] = {}
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()

    # ── lifecycle ────────────────────────────────────────────────────────

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

    # ── per-tick ─────────────────────────────────────────────────────────

    async def _tick(self) -> None:
        """Recompute geometric edges, debounce, and publish the slice.

        Stable non-robot objects become nodes (captions read from the
        builder's cache, falling back to the label); reachable_by uses the
        gripper self-object, which is excluded from the node set on
        purpose — its edge target is therefore an id outside ``nodes``."""
        objs_dict, _ = await self.registry.snapshot()
        objects = list(objs_dict.values())
        stable = [o for o in objects if _is_stable(o, self.min_observations)]

        nodes = [_object_to_node(o) for o in stable]
        for node in nodes:
            node.caption = self.store.get_cached_caption(node) or node.label

        # VLM-primary: geometry owns only `reachable_by` (a real 3D gripper
        # distance the robot acts on). Contact/containment (on_top_of/under/
        # inside/contains) are no longer emitted here — they came from
        # volumetric-AABB tests that misfire on full-volume boxes and
        # same-surface objects; the image-grounded VLM pass (scene-graph
        # builder) owns them now. `near` was already a proximity query, not an
        # edge. This loop still publishes the node set.
        raw = self._reachable_edges(objects, stable)
        if _RELATIONS_MODE == "geometric":
            raw.extend(self._object_relations(nodes))
        emitted = self._debounce(raw)
        self.store.set_geometric(nodes, emitted)

    def _reachable_edges(
        self, objects: list[SceneObject], stable: list[SceneObject]
    ) -> list[SceneGraphEdge]:
        """`reachable_by` edges object→gripper for stable objects within the
        reach radius. Empty when no gripper/robot self-object is tracked."""
        gripper = _find_gripper(objects)
        if gripper is None:
            return []
        out: list[SceneGraphEdge] = []
        for o in stable:
            if o.object_id == gripper.object_id:
                continue
            if _dist3(o.pose, gripper.pose) <= _REACHABLE_RADIUS_M:
                out.append(SceneGraphEdge(
                    source_id=o.object_id,
                    target_id=gripper.object_id,
                    relation="reachable_by",
                    confidence=CONF_MED,
                    method="geometric",
                    reason="geometry: within gripper reach radius (distance-only)",
                ))
        return out

    def _object_relations(self, nodes) -> list[SceneGraphEdge]:
        """Pairwise deterministic relations between stable objects.

        Support/containment edges are emitted in both directions (the
        vocabulary has explicit inverses). `near` edges are kept only for
        each object's closest `_NEAR_MAX_PER_OBJECT` neighbours within
        `_NEAR_RADIUS_M`, and only when no stronger relation links the pair.
        """
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
                    inv = {"on_top_of": "under", "under": "on_top_of", "inside": "contains", "contains": "inside"}[rel]
                    out.append(SceneGraphEdge(b.object_id, a.object_id, inv, conf, "geometric", reason))
        seen: set[tuple[str, str]] = set()
        for src, cands in near_candidates.items():
            for dist, dst in sorted(cands)[:_NEAR_MAX_PER_OBJECT]:
                key = (src, dst)
                if key in seen:
                    continue
                seen.add(key)
                out.append(SceneGraphEdge(
                    src, dst, "near", CONF_MED, "geometric",
                    f"geometry: within {_NEAR_RADIUS_M:.1f} m (centre distance {dist:.2f} m)",
                ))
        return out

    def _debounce(self, edges: list[SceneGraphEdge]) -> list[SceneGraphEdge]:
        """Hysteresis band over (source, target, relation) keys. Present
        keys gain a point (capped at _max), absent keys lose one; a key is
        emitted while its score >= _enter and forgotten once it hits 0.
        This both delays emission of a flickering new edge and grants a
        brief grace before removing one that momentarily drops out."""
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
