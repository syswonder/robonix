# SPDX-License-Identifier: MulanPSL-2.0
"""Snapshot scoping — turn a SnapshotSpec into the (objects, relations,
surfaces) subset the MCP tool returns. Centralises:

  * radius rejection (>50 m → error)
  * freshness filter (drop objects last_seen > freshness_s ago)
  * include_stale toggle (missing flag)
  * min_confidence floor
  * max_objects cap (after spatial sort by distance from region center)

The TF transform from `region_frame` to `map` is the caller's job —
we operate purely on map-frame poses here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .object_registry import Pose3D, SceneObject, SceneSurface
from .relations import RelationTriple


@dataclass(frozen=True)
class SnapshotScope:
    """Resolved scoping inputs (in map frame)."""
    layers: tuple[str, ...]
    center: Pose3D
    radius_m: float
    freshness_s: float
    include_stale: bool
    min_confidence: float
    max_objects: int


_DEFAULT_LAYERS = ("object", "relation")
_VALID_LAYERS = ("object", "relation", "surface")
_RADIUS_CAP_M = 50.0


def validate_scope(
    *,
    layers: list[str] | None,
    region_frame: str,
    region_center_xyz: tuple[float, float, float],
    region_radius_m: float,
    freshness_s: float,
    include_stale: bool,
    min_confidence: float,
    max_objects: int,
    transform_to_map: "callable[[Pose3D], Pose3D] | None" = None,
) -> SnapshotScope:
    """Resolve sentinels, validate, transform region to map frame.

    Raises ValueError on bad inputs (radius > cap, unknown layer name,
    negative cap, …) so the MCP wrapper turns them into a clean tool
    error rather than a server-side traceback.

    `transform_to_map` is supplied by the service entrypoint and wraps
    tf2 lookups; passing it through here keeps state/ free of ROS
    deps. Pass None if frame is already 'map' or unknown."""
    if region_radius_m <= 0.0:
        raise ValueError("region_radius_m must be > 0")
    if region_radius_m > _RADIUS_CAP_M:
        raise ValueError(
            f"region_radius_m={region_radius_m:.1f} exceeds cap {_RADIUS_CAP_M:.0f} "
            "(scene snapshots must be region-scoped to keep payloads small)"
        )
    if max_objects < 0:
        raise ValueError("max_objects must be >= 0")

    eff_layers = tuple(layers) if layers else _DEFAULT_LAYERS
    bad = [l for l in eff_layers if l not in _VALID_LAYERS]
    if bad:
        raise ValueError(f"unknown snapshot layer(s) {bad!r}; valid: {_VALID_LAYERS}")

    eff_frame = region_frame or "base_link"
    center = Pose3D(*region_center_xyz, frame_id=eff_frame)
    if eff_frame != "map" and transform_to_map is not None:
        center = transform_to_map(center)
    elif eff_frame != "map":
        # No tf available; treat the request as already-in-map-coords.
        # Better than failing — Pilot can override frame to `map`
        # explicitly if it cares.
        center = Pose3D(center.x, center.y, center.z, center.yaw, frame_id="map")

    return SnapshotScope(
        layers=eff_layers,
        center=center,
        radius_m=region_radius_m,
        freshness_s=freshness_s,
        include_stale=include_stale,
        min_confidence=min_confidence,
        max_objects=max_objects,
    )


def _dist_xy(a: Pose3D, b: Pose3D) -> float:
    """Horizontal distance — region radius is interpreted as a
    cylinder around the vertical axis, not a sphere. Matters when the
    robot is on the floor and the scene includes objects at different
    heights."""
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def scope_snapshot(
    scope: SnapshotScope,
    *,
    objects: dict[str, SceneObject],
    surfaces: dict[str, SceneSurface],
    relations: list[RelationTriple],
    now: float,
) -> tuple[list[SceneObject], list[RelationTriple], list[SceneSurface]]:
    """Apply scope to the in-memory snapshot. Returns scoped (objects,
    relations, surfaces). Sorted by distance from region center so the
    `max_objects` cap is meaningful (closest first)."""

    # ── object filter
    if "object" in scope.layers:
        candidates: list[tuple[float, SceneObject]] = []
        for obj in objects.values():
            if not scope.include_stale and obj.missing:
                continue
            if obj.confidence < scope.min_confidence:
                continue
            age = now - obj.last_seen
            if age > scope.freshness_s and not scope.include_stale:
                continue
            d = _dist_xy(obj.pose, scope.center)
            if d > scope.radius_m:
                continue
            candidates.append((d, obj))
        candidates.sort(key=lambda t: t[0])
        scoped_objects = [o for _, o in candidates[: scope.max_objects]]
    else:
        scoped_objects = []

    # ── relation filter — keep only relations between scoped objects
    if "relation" in scope.layers and scoped_objects:
        scoped_ids = {o.object_id for o in scoped_objects}
        scoped_relations = [
            r for r in relations
            if r.subject_object_id in scoped_ids and r.target_object_id in scoped_ids
        ]
    else:
        scoped_relations = []

    # ── surface filter — separate radius pass; surfaces don't have
    # confidence so the freshness filter is the only fade-out.
    if "surface" in scope.layers:
        scoped_surfaces: list[SceneSurface] = []
        for s in surfaces.values():
            age = now - s.last_seen
            if age > scope.freshness_s and not scope.include_stale:
                continue
            if _dist_xy(s.pose, scope.center) > scope.radius_m:
                continue
            scoped_surfaces.append(s)
    else:
        scoped_surfaces = []

    return scoped_objects, scoped_relations, scoped_surfaces
