# SPDX-License-Identifier: MulanPSL-2.0
"""SceneObject registry — the canonical store for everything `system/scene`
tracks about the world. Pure-Python in-memory at runtime. A boot starts
fresh; stable objects come back via `restore_object()` when the operator
Loads a saved map in the map UI (or, in the legacy `SCENE_RESTORE_ON_START`
mode, at boot) — see `scene_service/persistence.py` — instead of being
re-accumulated through the `min_observations` filter from scratch.
"""
from __future__ import annotations

import asyncio
import math
import re
import time
from dataclasses import dataclass, field
from typing import Iterable, Optional


# ── Attribute schema ────────────────────────────────────────────────────────
# Free-form `attributes: dict[str, Any]` is convenient but easy to drift;
# document the keys we actually emit here so adding a new one is a
# code-change with a comment, not a typo. Anyone adding a new key should
# extend this list and the per-class defaults below.
OBJECT_ATTRIBUTE_KEYS = (
    "graspable",     # bool — small enough to grasp; from class default
    "movable",       # bool — not bolted to the floor
    "fragile",       # bool — drop carefully (cup, glass, …)
    "is_robot",      # bool — the tracked self-object
    "source",        # str  — "perception" | "planar_extraction" | "self"
    # Internal tracking keys (not semantic object properties):
    "cg_uuid",       # str  — the concept-graphs MapObjectList uuid currently
                     #        bound to this record (per-process, ephemeral)
    "restored",      # bool — restored from persistence (map Load / legacy
                     #        boot restore) and not yet re-observed;
                     #        perception re-binds it by class+pose, then clears
)

# Per-class defaults. Hardcoded for v1; intent is to drive these from a
# config file eventually so the same scene service can run on different
# robots without code edits.
_CLASS_ATTRIBUTE_DEFAULTS: dict[str, dict[str, object]] = {
    "cup":      {"graspable": True,  "movable": True,  "fragile": True,  "is_robot": False},
    "bottle":   {"graspable": True,  "movable": True,  "fragile": True,  "is_robot": False},
    "tool":     {"graspable": True,  "movable": True,  "fragile": False, "is_robot": False},
    "tray":     {"graspable": True,  "movable": True,  "fragile": False, "is_robot": False},
    "table":    {"graspable": False, "movable": True,  "fragile": False, "is_robot": False},
    "chair":    {"graspable": False, "movable": True,  "fragile": False, "is_robot": False},
    "door":     {"graspable": False, "movable": True,  "fragile": False, "is_robot": False},
    "person":   {"graspable": False, "movable": True,  "fragile": False, "is_robot": False},
    "robot":    {"graspable": False, "movable": True,  "fragile": False, "is_robot": True},
    "surface":  {"graspable": False, "movable": False, "fragile": False, "is_robot": False},
    "wall":     {"graspable": False, "movable": False, "fragile": False, "is_robot": False},
    "floor":    {"graspable": False, "movable": False, "fragile": False, "is_robot": False},
}

DEFAULT_ATTRIBUTES: dict[str, object] = {
    "graspable": False, "movable": True, "fragile": False, "is_robot": False, "source": "perception",
}


# ── Lightweight pose / bbox structs ────────────────────────────────────────
# Python-native (no ROS msg dependency at this layer) so the registry
# stays cleanly testable without rclpy installed. Conversion to/from the
# IDL `Object`/`FrameMapping` happens at the MCP boundary only.
@dataclass
class Pose3D:
    x: float
    y: float
    z: float
    yaw: float = 0.0  # radians; yaw-only orientation suffices for v1
    frame_id: str = ""


@dataclass
class BBox3D:
    """Axis-aligned bounding box in `frame_id`, centered on the object pose."""
    size_x: float = 0.1
    size_y: float = 0.1
    size_z: float = 0.1
    yaw: float = 0.0
    frame_id: str = ""

    @property
    def half_x(self) -> float:
        return self.size_x * 0.5

    @property
    def half_y(self) -> float:
        return self.size_y * 0.5

    @property
    def half_z(self) -> float:
        return self.size_z * 0.5


@dataclass
class SceneObject:
    """Stable object record. id format: `scene.object.<cls>_<NNN>`.

    Pose is in the explicit world frame supplied by ingest after its
    coordinate transform; never a guessed or raw sensor frame. Confidence is an EMA over per-observation
    confidences; pose is updated by the data_assoc layer with EMA
    `alpha=0.3` toward each new pose. `last_seen` is wall-clock unix
    seconds (Chronos TODO)."""
    object_id: str
    cls: str
    pose: Pose3D
    bbox: BBox3D
    confidence: float
    first_seen: float
    last_seen: float
    observation_count: int = 1
    missing: bool = False
    attributes: dict[str, object] = field(default_factory=dict)


@dataclass
class SceneSurface:
    """Planar surface registered by geom/plane_extract. Same id-namespace
    rules as SceneObject (`scene.surface.<NNN>`); we expose surfaces in
    snapshots so `on(cup, table)` can resolve via plane lookups when no
    bounding-box "table" object is present.
    """
    surface_id: str
    pose: Pose3D                 # centroid
    normal: tuple[float, float, float]
    extent_x: float              # along plane local x
    extent_y: float              # along plane local y
    last_seen: float


# ── Registry ────────────────────────────────────────────────────────────────
class ObjectRegistry:
    """Async-safe object + surface store. All read/write paths go through
    `with reg.lock(): ...`; readers take an atomic snapshot via
    `await reg.snapshot()` if they want to release the lock fast.

    Stable id allocation is a per-class monotonic counter."""

    def __init__(self, *, grace_period_s: float = 5.0) -> None:
        self._lock = asyncio.Lock()
        self._objects: dict[str, SceneObject] = {}
        self._surfaces: dict[str, SceneSurface] = {}
        self._counters: dict[str, int] = {}
        self._surface_counter: int = 0
        self.grace_period_s = grace_period_s

    # ── locking ────────────────────────────────────────────────────────────
    def lock(self) -> "asyncio.Lock":
        return self._lock

    async def snapshot(self) -> tuple[dict[str, SceneObject], dict[str, SceneSurface]]:
        """Atomic shallow copy. Cheap because dataclasses are referenced,
        not copied — callers must NOT mutate returned values."""
        async with self._lock:
            return dict(self._objects), dict(self._surfaces)

    # ── id allocation ──────────────────────────────────────────────────────
    def _alloc_id(self, cls: str) -> str:
        n = self._counters.get(cls, 0) + 1
        self._counters[cls] = n
        return f"scene.object.{cls}_{n:03d}"

    def _alloc_surface_id(self) -> str:
        self._surface_counter += 1
        return f"scene.surface.{self._surface_counter:03d}"

    def clear_objects(self) -> int:
        """Drop every object and surface. Caller MUST hold `lock()`.

        Used by the lifecycle linkage when the map frame epoch changes
        (mapping reset / re-init): every stored map-frame coordinate is no
        longer anchored, and this state is derived — re-observation rebuilds
        it in the new frame. Id counters are NOT reset, so ids stay unique
        across the flush (restored/old ids never collide with new ones).
        Returns the number of objects dropped."""
        n = len(self._objects)
        self._objects.clear()
        self._surfaces.clear()
        return n

    # ── object CRUD (caller MUST hold the lock) ────────────────────────────
    def insert_object(
        self,
        cls: str,
        pose: Pose3D,
        bbox: BBox3D,
        confidence: float,
        now: float,
        *,
        is_robot: bool = False,
        source: str = "perception",
    ) -> SceneObject:
        """Allocate a new SceneObject. Caller must hold `self._lock`."""
        oid = self._alloc_id(cls)
        attrs = dict(DEFAULT_ATTRIBUTES)
        attrs.update(_CLASS_ATTRIBUTE_DEFAULTS.get(cls, {}))
        attrs["source"] = source
        if is_robot:
            attrs["is_robot"] = True
        obj = SceneObject(
            object_id=oid,
            cls=cls,
            pose=pose,
            bbox=bbox,
            confidence=confidence,
            first_seen=now,
            last_seen=now,
            observation_count=1,
            missing=False,
            attributes=attrs,
        )
        self._objects[oid] = obj
        return obj

    def restore_object(self, obj: SceneObject) -> None:
        """Re-insert a persisted object verbatim under its existing
        `object_id` (warm restore at boot). Caller must hold `self._lock`.

        Advances the per-class id counter past the restored numeric suffix so
        a later `_alloc_id(cls)` can never collide with — or reuse — a restored
        id. A new object of the same class therefore continues numbering after
        the highest restored one. Objects with an unparseable id (not the
        `scene.object.<cls>_<NNN>` shape) are still stored; only the counter
        bump is skipped for them.

        The restored object is a *remembered-but-unseen* record: its persisted
        `cg_uuid` belonged to a now-dead perception process and is meaningless
        to the new MapObjectList, so it is dropped and the object is flagged
        `restored`. That flag tells the perception reconcile not to evict it on
        the uuid-membership rule until a live detection re-binds it by
        class+pose (see `ConceptGraphsDetector._apply_snapshot`)."""
        obj.attributes.pop("cg_uuid", None)
        obj.attributes["restored"] = True
        self._objects[obj.object_id] = obj
        m = re.search(r"_(\d+)$", obj.object_id)
        if m:
            n = int(m.group(1))
            if n > self._counters.get(obj.cls, 0):
                self._counters[obj.cls] = n

    def get_object(self, oid: str) -> Optional[SceneObject]:
        return self._objects.get(oid)

    def all_objects(self) -> Iterable[SceneObject]:
        return self._objects.values()

    def all_surfaces(self) -> Iterable[SceneSurface]:
        return self._surfaces.values()

    def update_object_pose(
        self,
        obj: SceneObject,
        new_pose: Pose3D,
        new_confidence: float,
        now: float,
        *,
        ema_pose: float = 0.3,
        ema_conf: float = 0.3,
    ) -> None:
        """EMA-blend new observation into the existing record. Caller
        must hold the lock. Yaw is averaged on the unit circle to
        avoid wrap-around; confidence is bounded to [0, 1]."""
        a = ema_pose
        obj.pose = Pose3D(
            x=(1 - a) * obj.pose.x + a * new_pose.x,
            y=(1 - a) * obj.pose.y + a * new_pose.y,
            z=(1 - a) * obj.pose.z + a * new_pose.z,
            yaw=_circular_ema(obj.pose.yaw, new_pose.yaw, a),
            frame_id=new_pose.frame_id,
        )
        obj.confidence = max(0.0, min(1.0, (1 - ema_conf) * obj.confidence + ema_conf * new_confidence))
        obj.observation_count += 1
        obj.last_seen = now
        if obj.missing:
            obj.missing = False  # re-acquired

    def mark_stale(self, now: float) -> int:
        """Set `missing=True` on objects past the grace period. Returns
        how many transitioned this tick (0 most of the time). Never
        deletes — Pilot may still ask about missing objects.

        Caller must hold the lock."""
        flipped = 0
        for obj in self._objects.values():
            if obj.missing or obj.attributes.get("is_robot"):
                continue
            if (now - obj.last_seen) > self.grace_period_s:
                obj.missing = True
                flipped += 1
        return flipped

    # ── cross-tick re-bind / soft eviction ─────────────────────────────────
    # The perception layer (concept-graphs) releases an object's tracking
    # uuid whenever a cleanup tick culls or re-keys it; without these, the
    # record is hard-deleted and a re-detection a few ticks later mints a
    # fresh id at observation_count=1 (the "9 objects → 1" collapse). Keeping
    # the record `missing` and re-binding it by class+pose preserves the id
    # and accumulated count across the uuid gap. This generalizes the same
    # class+distance gate the perception layer already uses for warm-restore
    # re-binding (`_rebind_restored`).

    def find_rebindable(
        self,
        cls: str,
        pose: Pose3D,
        max_d: float,
        *,
        only_missing: bool = False,
        exclude_oids: Optional[Iterable[str]] = None,
    ) -> Optional[SceneObject]:
        """Nearest non-robot, non-restored record of class `cls` whose centroid
        is within `max_d` of `pose`, else None.

        With `only_missing=True` only soft-evicted (`missing`) records are
        considered — the cross-tick re-bind case, where a re-detected object
        reclaims the id it held before a cull. `exclude_oids` skips records
        already claimed this tick. Restored records are excluded (the
        perception layer re-binds those via its own warm-restore path). Caller
        must hold the lock."""
        excl = set(exclude_oids or ())
        best: Optional[SceneObject] = None
        best_d = max_d
        for obj in self._objects.values():
            if obj.object_id in excl or obj.cls != cls:
                continue
            if obj.attributes.get("is_robot") or obj.attributes.get("restored"):
                continue
            if only_missing and not obj.missing:
                continue
            d = _dist3(obj.pose, pose)
            if d <= best_d:
                best_d = d
                best = obj
        return best

    def soft_evict(self, obj: SceneObject) -> None:
        """Mark a perception record `missing` and release its concept-graphs
        uuid binding instead of deleting it, so a later re-detection can
        re-bind the same id (and accumulated observation_count) via
        `find_rebindable`. Bounded by `prune_expired`. Caller must hold the
        lock."""
        obj.missing = True
        obj.attributes.pop("cg_uuid", None)

    def prune_expired(self, now: float, ttl_s: float) -> list[str]:
        """Hard-delete `missing` perception records whose `last_seen` is older
        than `ttl_s`; returns the deleted object_ids. Bounds growth of
        soft-evicted records — dedup survivors' stale twins and objects that
        truly left the scene. The robot self-record and warm-restored records
        (kept indefinitely until re-seen) are never pruned here. Caller must
        hold the lock."""
        doomed = [
            oid for oid, o in self._objects.items()
            if o.missing
            and not o.attributes.get("is_robot")
            and not o.attributes.get("restored")
            and (now - o.last_seen) > ttl_s
        ]
        for oid in doomed:
            del self._objects[oid]
        return doomed

    # ── surface insert ─────────────────────────────────────────────────────
    def insert_or_update_surface(
        self,
        pose: Pose3D,
        normal: tuple[float, float, float],
        extent_x: float,
        extent_y: float,
        now: float,
        *,
        merge_dist_m: float = 0.30,
    ) -> SceneSurface:
        """If an existing surface lives within `merge_dist_m` of pose AND
        has a near-parallel normal, update it; otherwise allocate a new
        one. Avoids spamming hundreds of nearly-identical planes when
        plane_extract sees the same table every frame.

        Caller must hold the lock."""
        for s in self._surfaces.values():
            if _dist3(s.pose, pose) < merge_dist_m and _normal_parallel(s.normal, normal, tol_rad=0.20):
                s.pose = pose
                s.normal = normal
                s.extent_x = max(s.extent_x, extent_x)
                s.extent_y = max(s.extent_y, extent_y)
                s.last_seen = now
                return s
        sid = self._alloc_surface_id()
        s = SceneSurface(
            surface_id=sid,
            pose=pose,
            normal=normal,
            extent_x=extent_x,
            extent_y=extent_y,
            last_seen=now,
        )
        self._surfaces[sid] = s
        return s

    def stats(self) -> dict[str, int]:
        return {
            "objects": len(self._objects),
            "missing": sum(1 for o in self._objects.values() if o.missing),
            "surfaces": len(self._surfaces),
        }


# ── helpers ─────────────────────────────────────────────────────────────────
def _circular_ema(current: float, new: float, alpha: float) -> float:
    """EMA on the unit circle: average sin/cos, take atan2. Avoids the
    "yaw flips by 2π and we lerp through zero" bug."""
    cur_s, cur_c = math.sin(current), math.cos(current)
    new_s, new_c = math.sin(new), math.cos(new)
    s = (1 - alpha) * cur_s + alpha * new_s
    c = (1 - alpha) * cur_c + alpha * new_c
    return math.atan2(s, c)


def _dist3(a: Pose3D, b: Pose3D) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _normal_parallel(n1: tuple[float, float, float], n2: tuple[float, float, float], *, tol_rad: float) -> bool:
    """True if the angle between n1 and n2 is within tol_rad (or its
    supplement — opposite-pointing normals are still 'parallel surfaces').
    """
    nn1 = _norm(n1); nn2 = _norm(n2)
    if nn1 == 0.0 or nn2 == 0.0:
        return False
    dot = (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]) / (nn1 * nn2)
    dot = max(-1.0, min(1.0, dot))
    angle = math.acos(abs(dot))
    return angle <= tol_rad


def _norm(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def now_unix() -> float:
    """Wall-clock unix seconds. TODO(chronos) replace with the unified
    time service once it lands."""
    return time.time()
