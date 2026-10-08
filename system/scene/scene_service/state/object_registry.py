# SPDX-License-Identifier: MulanPSL-2.0
"""SceneObject registry — the canonical store for everything `system/scene` tracks
about the world.
"""
from __future__ import annotations

import asyncio
import math
import warnings
import re
import time
from dataclasses import dataclass, field
from typing import Iterable, Optional


# Free-form `attributes: dict[str, Any]` is convenient but easy to drift;
# document the keys we actually emit here so adding a new one is a code-change
# with a comment, not a typo. Anyone adding a new key should extend this list
# and the per-class defaults below.
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

# Per-class defaults.
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


# Python-native (no ROS msg dependency at this layer) so the registry stays
# cleanly testable without rclpy installed.
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

# A sentence or two.
_MAX_CAPTION_LEN = 512


@dataclass
class SceneObject:
    """Stable object record."""
    object_id: str
    label: str
    pose: Pose3D
    bbox: BBox3D
    confidence: float
    first_seen: float
    last_seen: float
    observation_count: int = 1
    missing: bool = False
    # One sentence about this object, in a person's words.
    caption: str = ""
    caption_updated_at: float = 0.0
    caption_source: str = ""      # "model" | "operator" | "" (none yet)
    attributes: dict[str, object] = field(default_factory=dict)

    @property
    def settled(self) -> bool:
        """Part of the semantic map: not the robot and not a provisional label.
        A remembered object counts though it is out of view (a loaded map's
        objects all are). What the map pages and the captioner show; the rest
        is the registry's working state, for the debug view.
        """
        return (not self.attributes.get("is_robot")
                and not self.attributes.get("label_provisional", False))

    @property
    def cls(self) -> str:
        """Deprecated: the class is `label`."""
        warnings.warn("SceneObject.cls is deprecated; use label",
                      DeprecationWarning, stacklevel=2)
        return self.label

    @cls.setter
    def cls(self, value: str) -> None:
        warnings.warn("SceneObject.cls is deprecated; use label",
                      DeprecationWarning, stacklevel=2)
        self.label = value

    @property
    def display_name(self) -> str:
        """The short heading for one row: class plus this object's number."""

        m = re.search(r"(\d+)$", self.object_id)
        return f"{self.label}_{m.group(1)}" if m else (self.label or self.object_id)


class ObjectRegistry:
    """Async-safe object store."""

    def __init__(self, *, grace_period_s: float = 5.0) -> None:
        self._lock = asyncio.Lock()
        self._objects: dict[str, SceneObject] = {}
        # One counter, not one per class: an object id must not encode
        # anything that can change. See `_alloc_id`.
        self._object_counter: int = 0
        # Ids that have left, and where they went. Insertion-ordered so the
        # bound evicts the oldest forwarding address first.
        self._departed: dict[str, dict] = {}
        self.grace_period_s = grace_period_s

    def lock(self) -> "asyncio.Lock":
        return self._lock

    async def snapshot(self) -> dict[str, SceneObject]:
        """Atomic shallow copy."""
        async with self._lock:
            return dict(self._objects)

    # Bounded so a long session cannot grow this without limit; oldest first,
    # because a forwarding address matters most while something still holds the
    # old id.
    _MAX_DEPARTURES = 5000

    def _record_departure(
        self,
        obj: "SceneObject",
        reason: str,
        *,
        superseded_by: Optional[str] = None,
        inferred: bool = False,
    ) -> None:
        """Note that `obj`'s id has left, and where it went if anywhere."""
        if len(self._departed) >= self._MAX_DEPARTURES:
            self._departed.pop(next(iter(self._departed)))
        self._departed[obj.object_id] = {
            "object_id": obj.object_id,
            "label": obj.label,
            "reason": reason,
            "at": obj.last_seen,
            "superseded_by": superseded_by,
            "inferred": bool(inferred and superseded_by),
        }

    def resolve_id(
        self, object_id: str, *, follow_inferred: bool = True,
    ) -> tuple[Optional[str], Optional[dict]]:
        """Map a possibly-stale id to the live id that now stands for it."""
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

    def _successor_for(
        self, obj: "SceneObject", max_d: float,
    ) -> Optional["SceneObject"]:
        """The live record most likely to be what `obj` was folded into."""
        if max_d <= 0.0:
            return None
        best: Optional[SceneObject] = None
        best_d = max_d
        for other in self._objects.values():
            if other.object_id == obj.object_id or other.label != obj.label:
                continue
            if other.missing or other.attributes.get("is_robot"):
                continue
            d = _dist3(other.pose, obj.pose)
            if d <= best_d:
                best_d = d
                best = other
        return best

    def _alloc_id(self) -> str:
        """An opaque, stable id."""
        self._object_counter += 1
        return f"scene.object.{self._object_counter:04d}"

    def clear_objects(self) -> int:
        """Drop every object."""
        n = len(self._objects)
        for obj in self._objects.values():
            # No successor by construction: the frame these were anchored in
            # is gone, so nothing in the new one stands for them.
            self._record_departure(obj, "epoch_flush")
        self._objects.clear()
        return n

    def clear_derived_objects(self) -> int:
        """Delete non-robot objects."""
        doomed = [
            oid
            for oid, obj in self._objects.items()
            if not obj.attributes.get("is_robot")
        ]
        for oid in doomed:
            self._record_departure(self._objects[oid], "derived_cleared")
            del self._objects[oid]
        return len(doomed)

    def delete_derived_object(self, object_id: str) -> SceneObject:
        """Delete one non-robot object or raise a precise lookup error."""
        obj = self._objects.get(object_id)
        if obj is None:
            raise KeyError(f"unknown Scene object {object_id!r}")
        if obj.attributes.get("is_robot"):
            raise ValueError("the robot self-object cannot be deleted")
        # A person said this is not a thing.
        self._record_departure(obj, "operator_deleted")
        del self._objects[object_id]
        return obj

    def set_object_caption(self, object_id: str, caption: str, *,
                           source: str, now: float) -> SceneObject:
        """Describe one object, or clear the description with an empty string.
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
                "label": obj.label,
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
        obj.label = label
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
        obj.label = str(previous.get("label") or obj.label)
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
        label: str,
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
        attrs.update(_CLASS_ATTRIBUTE_DEFAULTS.get(label, {}))
        attrs["source"] = source
        if is_robot:
            attrs["is_robot"] = True
        obj = SceneObject(
            object_id=oid,
            label=label,
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
        """Re-insert a persisted object verbatim under its existing `object_id`
        (warm restore at boot).
        """
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
        """EMA-blend new observation into the existing record."""
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
        """Set `missing=True` on objects past the grace period."""
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
            # negative observations.
            if obj.attributes.get("observation_lifecycle") == "visibility":
                continue
            if (now - obj.last_seen) > self.grace_period_s:
                obj.missing = True
                flipped += 1
        return flipped

    # The perception layer (concept-graphs) releases an object's tracking uuid
    # whenever a cleanup tick culls or re-keys it; without these, the record is
    # hard-deleted and a re-detection a few ticks later mints a fresh id at
    # observation_count=1 (the "9 objects → 1" collapse).

    def find_rebindable(
        self,
        label: str,
        pose: Pose3D,
        max_d: float,
        *,
        only_missing: bool = False,
        exclude_oids: Optional[Iterable[str]] = None,
    ) -> Optional[SceneObject]:
        """Nearest non-robot, non-restored record of class `label` whose centroid
        is within `max_d` of `pose`, else None.
        """
        excl = set(exclude_oids or ())
        best: Optional[SceneObject] = None
        best_d = max_d
        for obj in self._objects.values():
            if obj.object_id in excl or obj.label != label:
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
        """Mark a perception record `missing` and release its concept-graphs uuid
        binding instead of deleting it, so a later re-detection can re-bind the
        same id (and accumulated observation_count) via `find_rebindable`.
        """
        obj.missing = True
        obj.attributes["missing_reason"] = "cg_orphan"
        obj.attributes.pop("cg_uuid", None)

    def prune_expired(
        self, now: float, ttl_s: float, *, merge_dist_m: float = 0.0,
    ) -> list[str]:
        """Hard-delete `missing` perception records whose `last_seen` is older
        than `ttl_s`; returns the deleted object_ids.
        """
        doomed = [
            oid for oid, o in self._objects.items()
            if o.missing
            and not o.attributes.get("is_robot")
            and not o.attributes.get("restored")
            # Operator corrections are never TTL-deleted: a relocated object's
            # stale CG cloud reads as clear absence at its old footprint, which
            # must not erase the human's edit.
            and not self._operator_touched(o)
            and (now - o.last_seen) > ttl_s
        ]
        for oid in doomed:
            obj = self._objects[oid]
            # This is the moment a merge's loser finally goes, and the survivor
            # has had a full TTL to settle, so it is the best point to guess
            # where the id went.
            successor = self._successor_for(obj, merge_dist_m)
            self._record_departure(
                obj, "ttl_pruned",
                superseded_by=successor.object_id if successor else None,
                inferred=successor is not None,
            )
            del self._objects[oid]
        return doomed

    # Association answers "which object is this detection?" and never "are
    # these two objects the same?", so a detection that lands outside the gate
    # mints a second record for one thing and nothing ever puts them back
    # together.

    def merge_duplicates(
        self,
        now: float,
        *,
        xy_m: float = 0.35,
        z_m: float = 1.20,
        max_merges: int = 64,
    ) -> list[tuple[str, str]]:
        """Fold same-class records that describe one object into one record."""
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
                # An operator-touched record is never absorbed: a human looked
                # at this object and said something about it, and perception's
                # opinion does not outrank that.
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
        """How much this record deserves to be the survivor."""
        rank = float(obj.observation_count)
        if obj.missing:
            rank -= 1e6
        if ObjectRegistry._operator_touched(obj):
            rank += 1e9
        return rank

    @staticmethod
    def _operator_touched(obj: "SceneObject") -> bool:
        return bool(obj.attributes.get("operator_geometry")
                    or obj.attributes.get("operator_label")
                    or obj.caption_source == "operator")

    @staticmethod
    def _same_thing(
        a: "SceneObject", b: "SceneObject", xy_m: float, z_m: float,
    ) -> bool:
        """Whether two records are close enough to be one object."""
        if a.label != b.label:
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
        """Move what `other` knew onto `survivor` and retire its id."""
        survivor.observation_count += max(0, other.observation_count)
        survivor.first_seen = min(survivor.first_seen, other.first_seen)
        survivor.last_seen = max(survivor.last_seen, other.last_seen)
        # The survivor may only be missing because this record held the
        # recent sightings.
        survivor.missing = survivor.missing and other.missing
        survivor.confidence = max(survivor.confidence, other.confidence)
        # A person's caption outlives the record it was written on; otherwise
        # keep whichever caption exists.
        if other.caption and (other.caption_source == "operator"
                              and survivor.caption_source != "operator"
                              or not survivor.caption):
            survivor.caption = other.caption
            survivor.caption_source = other.caption_source
            survivor.caption_updated_at = other.caption_updated_at
        self._record_departure(
            other, "merged_duplicate",
            superseded_by=survivor.object_id,
            # Decided by proximity, like every other succession this table
            # records.
            inferred=True,
        )

    def stats(self) -> dict[str, int]:
        return {
            "objects": len(self._objects),
            "missing": sum(1 for o in self._objects.values() if o.missing),
        }


def _circular_ema(current: float, new: float, alpha: float) -> float:
    """EMA on the unit circle: average sin/cos, take atan2. Avoids the "yaw flips
    by 2π and we lerp through zero" bug.
    """
    cur_s, cur_c = math.sin(current), math.cos(current)
    new_s, new_c = math.sin(new), math.cos(new)
    s = (1 - alpha) * cur_s + alpha * new_s
    c = (1 - alpha) * cur_c + alpha * new_c
    return math.atan2(s, c)


def _dist3(a: Pose3D, b: Pose3D) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def now_unix() -> float:
    """Wall-clock unix seconds."""
    return time.time()
