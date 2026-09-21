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
    "label_confidence",       # float — confidence-weighted winning label share
    "label_provisional",      # bool — label has not met the stability gate
    "label_evidence_count",   # int — recent observations used for the label
    "label_candidates",       # list — ranked label evidence for diagnostics
    "navigation_grade",       # bool — geometry and label passed nav admission
    "geometry_navigation_grade",  # bool — geometry alone passed nav admission
    "geometry_source",        # str — measured or explicit operator bbox
    "bbox_method",            # str — method/provenance for the current bbox
    "geometry_point_count",   # int — measured points supporting geometry
    "geometry_view_count",    # int — fused views supporting geometry
    "operator_geometry",      # bool — pose/bbox was explicitly corrected
    "label_source",           # str — "model", "model_clip", or "operator"
    "operator_label",         # str — sticky human/supervisor label override
    "operator_label_previous",  # dict — model label state for explicit undo
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


# A sentence or two. The persistence column is 4096 and the embedding is
# computed over this text, so the cap is about what stays readable in a
# panel rather than about storage.
_MAX_CAPTION_LEN = 512


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
    # One sentence about this object, in a person's words. Written by the
    # captioner and editable by whoever is looking at the map, which is why
    # it lives here rather than on the scene-graph node it used to: a graph
    # node is rebuilt from the registry every pass, so an edit made there
    # survived until the next one.
    #
    # Distinct from `cls` on purpose. "my favourite desk" is not a category,
    # and writing it into `cls` -- which is what the label correction does --
    # stops the thing being a desk to `find`, to the relation loop and to
    # goal_near.
    caption: str = ""
    caption_updated_at: float = 0.0
    caption_source: str = ""      # "model" | "operator" | "" (none yet)
    attributes: dict[str, object] = field(default_factory=dict)

    @property
    def display_name(self) -> str:
        """The short heading for one row: class plus this object's number.

        Built rather than stored, so it follows a class correction. An object
        minted while the detector said `sink` and corrected to `cabinet`
        reads `cabinet_0003`, not `sink_0003` -- the number is this object's
        identity, not a count of cabinets.

        Deliberately not the caption. A caption is a sentence and a row needs
        a heading; the sentence belongs where there is room to read it.
        """
        import re as _re

        m = _re.search(r"(\d+)$", self.object_id)
        return f"{self.cls}_{m.group(1)}" if m else (self.cls or self.object_id)


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
        # One counter, not one per class: an object id must not encode
        # anything that can change. See `_alloc_id`.
        self._object_counter: int = 0
        self._surface_counter: int = 0
        # Ids that have left, and where they went. Insertion-ordered so the
        # bound evicts the oldest forwarding address first.
        self._departed: dict[str, dict] = {}
        self.grace_period_s = grace_period_s

    # ── locking ────────────────────────────────────────────────────────────
    def lock(self) -> "asyncio.Lock":
        return self._lock

    async def snapshot(self) -> tuple[dict[str, SceneObject], dict[str, SceneSurface]]:
        """Atomic shallow copy. Cheap because dataclasses are referenced,
        not copied — callers must NOT mutate returned values."""
        async with self._lock:
            return dict(self._objects), dict(self._surfaces)

    # ── departures ─────────────────────────────────────────────────────────
    # Bounded so a long session cannot grow this without limit; oldest first,
    # because a forwarding address matters most while something still holds
    # the old id.
    _MAX_DEPARTURES = 5000

    def _record_departure(
        self,
        obj: "SceneObject",
        reason: str,
        *,
        superseded_by: Optional[str] = None,
        inferred: bool = False,
    ) -> None:
        """Note that `obj`'s id has left, and where it went if anywhere.

        `inferred` marks a successor we decided by proximity rather than were
        told. Callers that must not be wrong -- a user's confirmed choice --
        can look at it and decline to follow."""
        if len(self._departed) >= self._MAX_DEPARTURES:
            self._departed.pop(next(iter(self._departed)))
        self._departed[obj.object_id] = {
            "object_id": obj.object_id,
            "cls": obj.cls,
            "reason": reason,
            "at": obj.last_seen,
            "superseded_by": superseded_by,
            "inferred": bool(inferred and superseded_by),
        }

    def resolve_id(
        self, object_id: str, *, follow_inferred: bool = True,
    ) -> tuple[Optional[str], Optional[dict]]:
        """Map a possibly-stale id to the live id that now stands for it.

        Returns `(live_id, departure)`. A live id resolves to itself with no
        departure. A superseded one follows the chain; a tombstone resolves to
        None and hands back the record saying what happened, which is the
        answer a caller needs to tell "never existed" from "existed, gone".

        `follow_inferred=False` stops at a successor that was guessed rather
        than known -- the right setting when acting on a human's confirmation,
        where going to the wrong object is worse than admitting the id is
        stale. Caller must hold the lock."""
        if object_id in self._objects:
            return object_id, None
        seen: set[str] = set()
        current = object_id
        record: Optional[dict] = None
        while current in self._departed and current not in seen:
            seen.add(current)
            record = self._departed[current]
            nxt = record.get("superseded_by")
            if not nxt:
                return None, record
            if record.get("inferred") and not follow_inferred:
                return None, record
            if nxt in self._objects:
                return nxt, record
            current = nxt
        return None, record

    def departures(self) -> dict[str, dict]:
        """Every recorded departure, newest last. Caller must hold the lock."""
        return dict(self._departed)

    def _successor_for(
        self, obj: "SceneObject", max_d: float,
    ) -> Optional["SceneObject"]:
        """The live record most likely to be what `obj` was folded into.

        Same class, still observed, nearest within `max_d` -- the same gate
        re-adoption uses, pointed forwards instead of backwards. It is a
        guess, and is recorded as one."""
        if max_d <= 0.0:
            return None
        best: Optional[SceneObject] = None
        best_d = max_d
        for other in self._objects.values():
            if other.object_id == obj.object_id or other.cls != obj.cls:
                continue
            if other.missing or other.attributes.get("is_robot"):
                continue
            d = _dist3(other.pose, obj.pose)
            if d <= best_d:
                best_d = d
                best = other
        return best

    # ── id allocation ──────────────────────────────────────────────────────
    def _alloc_id(self) -> str:
        """An opaque, stable id. It encodes nothing about the object.

        Ids used to read `scene.object.<cls>_<NNN>`, which put the detector's
        first guess into a permanent name. Correct the class and the id went
        on saying the old one for ever -- `sink_003` after it turned out to be
        a cabinet. It also broke the collision guard in `restore_object`,
        which bumped the counter for the object's *current* class while the
        suffix had come from the class it was minted under, leaving the
        original class free to mint that number a second time and overwrite
        the restored object.

        What a person reads is the display name, which is built from the
        current class (or the name they gave it) and this number. Readability
        belongs there, where it can follow a correction; an identifier's job
        is to stay the same.
        """
        self._object_counter += 1
        return f"scene.object.{self._object_counter:04d}"

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
        for obj in self._objects.values():
            # No successor by construction: the frame these were anchored in
            # is gone, so nothing in the new one stands for them.
            self._record_departure(obj, "epoch_flush")
        self._objects.clear()
        self._surfaces.clear()
        return n

    # ── object CRUD (caller MUST hold the lock) ────────────────────────────
    def clear_derived_objects(self) -> int:
        """Delete non-robot objects and all derived surfaces."""
        doomed = [
            oid
            for oid, obj in self._objects.items()
            if not obj.attributes.get("is_robot")
        ]
        for oid in doomed:
            self._record_departure(self._objects[oid], "derived_cleared")
            del self._objects[oid]
        self._surfaces.clear()
        return len(doomed)

    def delete_derived_object(self, object_id: str) -> SceneObject:
        """Delete one non-robot object or raise a precise lookup error."""
        obj = self._objects.get(object_id)
        if obj is None:
            raise KeyError(f"unknown Scene object {object_id!r}")
        if obj.attributes.get("is_robot"):
            raise ValueError("the robot self-object cannot be deleted")
        # A person said this is not a thing. It has no successor, and saying
        # so is the point: a later reference to it should read as deleted,
        # not as unknown.
        self._record_departure(obj, "operator_deleted")
        del self._objects[object_id]
        return obj

    def set_object_caption(self, object_id: str, caption: str, *,
                           source: str, now: float) -> SceneObject:
        """Describe one object, or clear the description with an empty string.

        A caption is not a class correction and does not touch `cls`: the
        thing stays a table to `find`, to the relation loop and to goal_near
        while being "my favourite desk" to whoever wrote that. Correcting a
        misdetection is `update_object_label`, a different operation with
        different consequences.

        `source` records who wrote it. An operator's sentence is not
        overwritten by the captioner on its next pass -- a description a
        person took the trouble to write is not a cache entry.
        """
        obj = self._objects.get(object_id)
        if obj is None:
            raise KeyError(f"unknown Scene object {object_id!r}")
        if source not in ("model", "operator"):
            raise ValueError(f"caption source must be model or operator, got {source!r}")
        if (source == "model" and obj.caption_source == "operator"
                and obj.caption):
            return obj
        cleaned = " ".join(str(caption or "").split())
        if len(cleaned) > _MAX_CAPTION_LEN:
            raise ValueError(
                f"a caption is at most {_MAX_CAPTION_LEN} characters, "
                f"got {len(cleaned)}")
        obj.caption = cleaned
        obj.caption_updated_at = float(now)
        obj.caption_source = source if cleaned else ""
        return obj

    def update_object_label(self, object_id: str, label: str) -> SceneObject:
        """Apply a sticky operator-owned semantic label correction."""
        obj = self._objects.get(object_id)
        if obj is None:
            raise KeyError(f"unknown Scene object {object_id!r}")
        if obj.attributes.get("is_robot"):
            raise ValueError("the robot self-object label cannot be edited")
        if not obj.attributes.get("operator_label"):
            obj.attributes["operator_label_previous"] = {
                "label": obj.cls,
                "label_source": obj.attributes.get("label_source", "model"),
                "label_confidence": obj.attributes.get("label_confidence", 0.0),
                "label_provisional": obj.attributes.get(
                    "label_provisional",
                    True,
                ),
                "label_evidence_count": obj.attributes.get(
                    "label_evidence_count",
                    0,
                ),
                "label_candidates": obj.attributes.get("label_candidates", []),
                "graspable": obj.attributes.get("graspable", False),
                "movable": obj.attributes.get("movable", True),
                "fragile": obj.attributes.get("fragile", False),
            }
        obj.cls = label
        for key in ("graspable", "movable", "fragile"):
            obj.attributes[key] = DEFAULT_ATTRIBUTES[key]
        obj.attributes.update(_CLASS_ATTRIBUTE_DEFAULTS.get(label, {}))
        obj.attributes["operator_label"] = label
        obj.attributes["label_source"] = "operator"
        obj.attributes["label_confidence"] = 1.0
        obj.attributes["label_provisional"] = False
        obj.attributes["navigation_grade"] = bool(
            obj.attributes.get("geometry_navigation_grade")
        )
        return obj

    def update_model_label(
        self,
        object_id: str,
        label: str,
        *,
        confidence: float,
        source: str = "model_vlm",
    ) -> bool:
        """Publish accepted model naming without creating an operator override.

        Physical identity and object id are unchanged. A human/admin label is
        sticky and always wins over later model evidence. Caller must hold the
        registry lock.
        """

        obj = self._objects.get(object_id)
        normalized = " ".join(str(label or "").strip().lower().split())
        if (
            obj is None
            or not normalized
            or obj.attributes.get("is_robot")
            or obj.attributes.get("operator_label")
        ):
            return False
        obj.cls = normalized
        for key in ("graspable", "movable", "fragile"):
            obj.attributes[key] = DEFAULT_ATTRIBUTES[key]
        obj.attributes.update(_CLASS_ATTRIBUTE_DEFAULTS.get(normalized, {}))
        obj.attributes["label_source"] = str(source or "model_vlm")
        obj.attributes["label_confidence"] = max(
            0.0,
            min(1.0, float(confidence)),
        )
        obj.attributes["label_provisional"] = False
        obj.attributes["navigation_grade"] = bool(
            obj.attributes.get("geometry_navigation_grade")
        )
        return True

    def clear_object_label_override(self, object_id: str) -> SceneObject:
        """Restore the model-owned label state captured before operator edit."""
        obj = self._objects.get(object_id)
        if obj is None:
            raise KeyError(f"unknown Scene object {object_id!r}")
        if obj.attributes.get("is_robot"):
            raise ValueError("the robot self-object label cannot be edited")
        previous = obj.attributes.get("operator_label_previous")
        if not isinstance(previous, dict) or not obj.attributes.get(
            "operator_label"
        ):
            raise ValueError(
                f"Scene object {object_id!r} has no operator label override"
            )
        obj.cls = str(previous.get("label") or obj.cls)
        for key in (
            "label_source",
            "label_confidence",
            "label_provisional",
            "label_evidence_count",
            "label_candidates",
            "graspable",
            "movable",
            "fragile",
        ):
            if key in previous:
                value = previous[key]
                obj.attributes[key] = value
                # label_candidates is mutable; do not retain the undo payload's
                # list object after the payload is removed below.
                if key == "label_candidates":
                    obj.attributes[key] = list(value or ())
        obj.attributes.pop("operator_label", None)
        obj.attributes.pop("operator_label_previous", None)
        obj.attributes["navigation_grade"] = bool(
            obj.attributes.get("geometry_navigation_grade")
            and not obj.attributes.get("label_provisional", True)
        )
        return obj

    def update_object_geometry(
        self,
        object_id: str,
        pose: Pose3D,
        bbox: BBox3D,
    ) -> SceneObject:
        """Install a provenance-marked, deliberately non-nav operator bbox."""
        obj = self._objects.get(object_id)
        if obj is None:
            raise KeyError(f"unknown Scene object {object_id!r}")
        if obj.attributes.get("is_robot"):
            raise ValueError("the robot self-object geometry cannot be edited")
        obj.pose = pose
        obj.bbox = bbox
        obj.attributes["operator_geometry"] = True
        obj.attributes["geometry_source"] = "operator_bbox"
        obj.attributes["bbox_method"] = "operator_yaw_bbox"
        obj.attributes["geometry_point_count"] = 0
        obj.attributes["geometry_view_count"] = 0
        obj.attributes["geometry_navigation_grade"] = False
        obj.attributes["navigation_grade"] = False
        return obj

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
        oid = self._alloc_id()
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

        Advances the id counter past the restored numeric suffix so a later
        `_alloc_id` can neither collide with nor reuse a restored id. There is
        one counter and the id encodes no class, so the bump cannot land on
        the wrong one -- which is what happened while ids carried a class: the
        suffix was minted under the class the object had then, the bump was
        keyed on the class it has now, and a correction between the two left
        the original class free to mint that number again.

        Ids from before that change (`scene.object.<cls>_<NNN>`) restore
        verbatim and still bump the counter: the trailing number is read
        whatever precedes it. They keep their old spelling for ever, which is
        the point of an opaque id -- nothing reads it but the keys.

        The restored object is a *remembered-but-unseen* record: its persisted
        `cg_uuid` belonged to a now-dead perception process and is meaningless
        to the new MapObjectList, so it is dropped and the object is flagged
        `restored`. That flag tells the perception reconcile not to evict it on
        the uuid-membership rule until a live detection re-binds it by
        class+pose (see `ConceptGraphsDetector._apply_snapshot`)."""
        obj.attributes.pop("cg_uuid", None)
        obj.attributes["restored"] = True
        self._objects[obj.object_id] = obj
        m = re.search(r"(\d+)$", obj.object_id)
        if m:
            n = int(m.group(1))
            if n > self._object_counter:
                self._object_counter = n

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
            # A human-corrected pose outranks perception's absence evidence:
            # the stale CG cloud at the old footprint would otherwise flip a
            # freshly corrected object to missing.
            if obj.attributes.get("operator_geometry"):
                continue
            # RGB-D tracks own staleness through healthy, visibility-checked
            # negative observations. Wall-clock silence can mean an occlusion,
            # out-of-FOV object, disconnected sensor, or failed model and must
            # not be converted into evidence that the object disappeared.
            if obj.attributes.get("observation_lifecycle") == "visibility":
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
        obj.attributes["missing_reason"] = "cg_orphan"
        obj.attributes.pop("cg_uuid", None)

    def prune_expired(
        self, now: float, ttl_s: float, *, merge_dist_m: float = 0.0,
    ) -> list[str]:
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
            # Operator corrections are never TTL-deleted: a relocated
            # object's stale CG cloud reads as clear absence at its old
            # footprint, which must not erase the human's edit.
            and not o.attributes.get("operator_label")
            and not o.attributes.get("operator_geometry")
            and (now - o.last_seen) > ttl_s
        ]
        for oid in doomed:
            obj = self._objects[oid]
            # This is the moment a merge's loser finally goes, and the
            # survivor has had a full TTL to settle, so it is the best point
            # to guess where the id went. Recorded as a guess.
            successor = self._successor_for(obj, merge_dist_m)
            self._record_departure(
                obj, "ttl_pruned",
                superseded_by=successor.object_id if successor else None,
                inferred=successor is not None,
            )
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

    # ── duplicate collapse ─────────────────────────────────────────────────
    # Association answers "which object is this detection?" and never "are
    # these two objects the same?", so a detection that lands outside the
    # gate mints a second record for one thing and nothing ever puts them
    # back together. Both the operator's list and `find`'s candidate set
    # then carry copies that neither a person nor a ranker can tell apart.

    def merge_duplicates(
        self,
        now: float,
        *,
        xy_m: float = 0.35,
        z_m: float = 1.20,
        max_merges: int = 64,
    ) -> list[tuple[str, str]]:
        """Fold same-class records that describe one object into one record.

        Returns the `(absorbed_id, survivor_id)` pairs applied, so a caller
        can log or publish them. Each absorbed id is recorded as departed and
        superseded, which is what keeps a stale reference working: a request
        naming the absorbed id resolves to the survivor.

        Gating is deliberately tighter across the floor than association's --
        a wrong merge erases a real distinction, and that is worse than one
        object showing twice. Height is gated loosely for the same reason
        association's is: a single camera's depth estimate is the least
        reliable number in the record, and letting it decide is what produced
        the duplicates in the first place.

        `max_merges` bounds one pass. A registry that has drifted badly is
        repaired over several ticks rather than in one long hold of the lock.

        Caller must hold the lock.
        """
        if xy_m <= 0.0:
            return []
        merged: list[tuple[str, str]] = []
        # Most evidence first, so the survivor of each pair is settled before
        # anything is folded into it and a chain cannot form mid-pass.
        candidates = [
            obj for obj in self._objects.values()
            if not obj.attributes.get("is_robot")
        ]
        candidates.sort(key=lambda o: (-self._merge_rank(o), o.object_id))

        absorbed: set[str] = set()
        for survivor in candidates:
            if survivor.object_id in absorbed:
                continue
            for other in candidates:
                if len(merged) >= max_merges:
                    return merged
                if other.object_id in absorbed or other is survivor:
                    continue
                if not self._same_thing(survivor, other, xy_m, z_m):
                    continue
                # An operator-touched record is never absorbed: a human
                # looked at this object and said something about it, and
                # perception's opinion does not outrank that.
                if self._operator_touched(other) and not self._operator_touched(
                        survivor):
                    continue
                self._absorb(survivor, other, now)
                absorbed.add(other.object_id)
                merged.append((other.object_id, survivor.object_id))

        for object_id in absorbed:
            self._objects.pop(object_id, None)
        return merged

    @staticmethod
    def _merge_rank(obj: "SceneObject") -> float:
        """How much this record deserves to be the survivor.

        Observations first, because that is accumulated evidence. An
        operator-touched record outranks any amount of it -- a human's
        statement about an object is not something a detector count
        overrides -- and a live record outranks a missing one, since the
        survivor should be the one perception can still confirm.
        """
        rank = float(obj.observation_count)
        if obj.missing:
            rank -= 1e6
        if obj.attributes.get("operator_geometry") or obj.attributes.get(
                "operator_label"):
            rank += 1e9
        return rank

    @staticmethod
    def _operator_touched(obj: "SceneObject") -> bool:
        return bool(obj.attributes.get("operator_geometry")
                    or obj.attributes.get("operator_label"))

    @staticmethod
    def _same_thing(
        a: "SceneObject", b: "SceneObject", xy_m: float, z_m: float,
    ) -> bool:
        """Whether two records are close enough to be one object.

        Same class and same frame, then separately gated across the floor
        and in height. Coordinates from different frames are not comparable
        however near their numbers look.
        """
        if a.cls != b.cls:
            return False
        frame_a = str(a.pose.frame_id or "").strip()
        frame_b = str(b.pose.frame_id or "").strip()
        if not frame_a or frame_a != frame_b:
            return False
        if math.hypot(a.pose.x - b.pose.x, a.pose.y - b.pose.y) > xy_m:
            return False
        return abs(a.pose.z - b.pose.z) <= z_m

    def _absorb(
        self, survivor: "SceneObject", other: "SceneObject", now: float,
    ) -> None:
        """Move what `other` knew onto `survivor` and retire its id.

        The survivor's pose stands: it was chosen for having more evidence
        behind it, and averaging in a pose that was wrong enough to create a
        duplicate would drag it off the object. What does transfer is the
        observation count -- those sightings were of this object, and a
        survivor that under-reports them looks less established than it is --
        and first_seen, which is when the object was actually first seen
        whichever record happened to hold it.
        """
        survivor.observation_count += max(0, other.observation_count)
        survivor.first_seen = min(survivor.first_seen, other.first_seen)
        survivor.last_seen = max(survivor.last_seen, other.last_seen)
        if not survivor.missing:
            pass
        elif not other.missing:
            # The survivor was only missing because this record held the
            # recent sightings.
            survivor.missing = False
        survivor.confidence = max(survivor.confidence, other.confidence)
        self._record_departure(
            other, "merged_duplicate",
            superseded_by=survivor.object_id,
            # Decided by proximity, like every other succession this table
            # records. A caller acting on a human's confirmed choice can
            # still decline to follow it.
            inferred=True,
        )

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
