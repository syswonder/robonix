# SPDX-License-Identifier: MulanPSL-2.0
"""FastMCP tool definitions — thin wrappers over `state/`. All five
tools are read-only; writes happen on the ingest path. Inputs are
codegen-derived ROS dataclasses (`semantic_map_mcp.*` / `std_msgs_mcp.*`),
matching the pattern from system/memory: handler annotates with the type
of the srv's first field (no Request/Response wrapper exists in MCP
codegen output)."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from .state import (
    BBox3D,
    ObjectRegistry,
    Pose3D,
    SceneObject,
    SceneSurface,
    scope_snapshot,
)
from .state.relations import RelationEngine, RelationTriple, RELATION_PREDICATES
from .state.snapshot import validate_scope

# Resolved at import time. PYTHONPATH is set by package_manifest.yaml's
# `start:` block to include rbnx-build/codegen/{proto_gen,robonix_mcp_types}.
import semantic_map_mcp  # type: ignore
import std_msgs_mcp  # type: ignore
import builtin_interfaces_mcp  # type: ignore
from semantic_map_mcp import (  # type: ignore
    BoundingBox,
    FrameMapping,
    GetObjectResult,
    Object,
    Point3D,
    QueryConstraints,
    QueryResults,
    Region,
    Relation,
    RelationConstraint as RelationConstraintIDL,
    RelationType,
    SafetyContext,
    SceneSnapshot,
    SemanticMapSlice,
    SnapshotSpec,
)
from std_msgs_mcp import String

from mcp.server.fastmcp import FastMCP
from robonix_py import mcp_contract

log = logging.getLogger(__name__)


# ── Module-level state pointers, set by service.py at startup ──────────────
_REGISTRY: ObjectRegistry | None = None
_RELATIONS: RelationEngine | None = None
_TRANSFORM_TO_MAP = None  # callable(Pose3D)->Pose3D, or None


def attach_state(
    *,
    registry: ObjectRegistry,
    relations: RelationEngine,
    transform_to_map=None,
) -> None:
    global _REGISTRY, _RELATIONS, _TRANSFORM_TO_MAP
    _REGISTRY = registry
    _RELATIONS = relations
    _TRANSFORM_TO_MAP = transform_to_map


# ── conversions: Python state → IDL ────────────────────────────────────────

_REL_NAME_TO_ENUM: dict[str, int] = {
    "child_of": 0,
    "on_top": 1,
    "on": 1,            # alias used by relations.py
    "inside": 2,
    "near": 3,
    "custom": 4,
    "reachable_by": 5,
}


def _enum_for(predicate: str) -> RelationType:
    val = _REL_NAME_TO_ENUM.get(predicate.lower(), 4)
    return RelationType(type=val, custom_type="" if val != 4 else predicate)


def _unix_to_time(t: float):
    """Wall-clock unix → builtin_interfaces/Time."""
    sec = int(t)
    nsec = int((t - sec) * 1_000_000_000)
    return builtin_interfaces_mcp.Time(sec=sec, nanosec=nsec)


def _scene_object_to_idl(o: SceneObject) -> Object:
    bbox = BoundingBox(
        scale_x=o.bbox.size_x,
        scale_y=o.bbox.size_y,
        scale_z=o.bbox.size_z,
        yaw=o.bbox.yaw,
    )
    fm = FrameMapping(
        center=Point3D(x=o.pose.x, y=o.pose.y, z=o.pose.z),
        bbox=[bbox],
        texture=[],
        frame_id=o.pose.frame_id,
    )
    rels = [
        Relation(relation_type=_enum_for(rel_pred), target_entity_id=tgt)
        for rel_pred, tgt in _flatten_relations_for(o.object_id)
    ]
    return Object(
        id=o.object_id,
        label=o.cls,
        relations=rels,
        registered_skills=[],
        registered_primitives=[],
        frame_mapping=[fm],
        confidence=o.confidence,
        first_seen=_unix_to_time(o.first_seen),
        last_seen=_unix_to_time(o.last_seen),
        observation_count=o.observation_count,
        missing=o.missing,
    )


def _scene_surface_to_idl(s: SceneSurface) -> Object:
    """Surfaces are exposed as Object records with cls='surface'. Cleaner
    than a parallel IDL message; Pilot consumes them through the same
    snapshot pipeline."""
    bbox = BoundingBox(
        scale_x=max(s.extent_x, 0.05),
        scale_y=max(s.extent_y, 0.05),
        scale_z=0.02,
        yaw=0.0,
    )
    fm = FrameMapping(
        center=Point3D(x=s.pose.x, y=s.pose.y, z=s.pose.z),
        bbox=[bbox],
        texture=[],
        frame_id=s.pose.frame_id,
    )
    return Object(
        id=s.surface_id,
        label="surface",
        relations=[],
        registered_skills=[],
        registered_primitives=[],
        frame_mapping=[fm],
        confidence=1.0,
        first_seen=_unix_to_time(s.last_seen),
        last_seen=_unix_to_time(s.last_seen),
        observation_count=1,
        missing=False,
    )


def _flatten_relations_for(object_id: str) -> list[tuple[str, str]]:
    if _RELATIONS is None:
        return []
    return [
        (r.predicate, r.target_object_id)
        for r in _RELATIONS.current()
        if r.subject_object_id == object_id
    ]


def _make_relation_idl(r: RelationTriple) -> Relation:
    return Relation(relation_type=_enum_for(r.predicate), target_entity_id=r.target_object_id)


def _empty_object(id_echo: str = "") -> Object:
    """Sentinel for `get_object` not-found case. Pilot keys on the bool
    `found`, but the embedded Object can't be None — codegen wants a
    structured value either way."""
    return Object(
        id=id_echo,
        label="",
        relations=[],
        registered_skills=[],
        registered_primitives=[],
        frame_mapping=[],
        confidence=0.0,
        first_seen=_unix_to_time(0.0),
        last_seen=_unix_to_time(0.0),
        observation_count=0,
        missing=True,
    )


# ── @mcp_contract handlers ─────────────────────────────────────────────────

mcp = FastMCP("scene_provider")


@mcp_contract(mcp, contract_id="robonix/system/scene/get_snapshot")
async def get_snapshot(spec: SnapshotSpec) -> SceneSnapshot:
    """Return a region-scoped snapshot of objects, relations, and
    surfaces. Pilot calls this every LLM round; payload size is
    bounded by `region_radius_m` (must be 0 < r <= 50) and `max_objects`.
    Contract: robonix/system/scene/get_snapshot."""
    if _REGISTRY is None or _RELATIONS is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")

    scope = validate_scope(
        layers=list(spec.layers) if spec.layers else None,
        region_frame=spec.region_frame,
        region_center_xyz=(spec.region_center_x, spec.region_center_y, spec.region_center_z),
        region_radius_m=spec.region_radius_m,
        freshness_s=spec.freshness_s if spec.freshness_s > 0 else 30.0,
        include_stale=spec.include_stale,
        min_confidence=spec.min_confidence,
        max_objects=int(spec.max_objects) if spec.max_objects > 0 else 50,
        transform_to_map=_TRANSFORM_TO_MAP,
    )

    objs, surfs = await _REGISTRY.snapshot()
    rels = _RELATIONS.current()
    now = time.time()
    sc_objs, sc_rels, sc_surfs = scope_snapshot(
        scope, objects=objs, surfaces=surfs, relations=rels, now=now,
    )
    return SceneSnapshot(
        objects=[_scene_object_to_idl(o) for o in sc_objs],
        relations=[_make_relation_idl(r) for r in sc_rels],
        surfaces=[_scene_surface_to_idl(s) for s in sc_surfs],
        stamp_unix=now,
        frame_id="map",
    )


@mcp_contract(mcp, contract_id="robonix/system/scene/query")
async def query(constraints: QueryConstraints) -> QueryResults:
    """Composable query over the object registry. Filters by class,
    relation constraints, spatial radius, and confidence. Returns
    matching objects (current state — episodic memory belongs to
    spatial_memory_service).
    Contract: robonix/system/scene/query."""
    if _REGISTRY is None or _RELATIONS is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")

    objs, _ = await _REGISTRY.snapshot()
    candidates: list[SceneObject] = list(objs.values())

    if constraints.cls:
        wanted = {c.lower() for c in constraints.cls}
        candidates = [o for o in candidates if o.cls.lower() in wanted]

    if constraints.min_confidence > 0:
        candidates = [o for o in candidates if o.confidence >= constraints.min_confidence]

    if constraints.pose_radius_m and constraints.pose_radius_m > 0:
        cx, cy = constraints.pose_center_x, constraints.pose_center_y
        r = constraints.pose_radius_m
        candidates = [
            o for o in candidates
            if (o.pose.x - cx) ** 2 + (o.pose.y - cy) ** 2 <= r * r
        ]

    if constraints.relations:
        relations = _RELATIONS.current()
        matched: list[SceneObject] = []
        for o in candidates:
            ok = True
            for rc in constraints.relations:
                pred = rc.predicate.lower()
                if pred not in RELATION_PREDICATES:
                    ok = False
                    break
                hits = [
                    r for r in relations
                    if r.subject_object_id == o.object_id and r.predicate == pred
                ]
                if not hits:
                    ok = False
                    break
                if rc.target_object_id:
                    hits = [r for r in hits if r.target_object_id == rc.target_object_id]
                if rc.target_cls:
                    target_cls = rc.target_cls.lower()
                    hits = [
                        r for r in hits
                        if (r.target_object_id in objs and objs[r.target_object_id].cls.lower() == target_cls)
                    ]
                if not hits:
                    ok = False
                    break
            if ok:
                matched.append(o)
        candidates = matched

    return QueryResults(results=[_scene_object_to_idl(o) for o in candidates])


@mcp_contract(mcp, contract_id="robonix/system/scene/get_object")
async def get_object(object_id: String) -> GetObjectResult:
    """Direct lookup by stable object_id. Returns full record with
    first_seen / last_seen / observation_count / missing. `found`
    is False for unknown ids (and `obj` is then a placeholder).
    Contract: robonix/system/scene/get_object."""
    if _REGISTRY is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")
    objs, _ = await _REGISTRY.snapshot()
    o = objs.get(object_id.data)
    if o is None:
        return GetObjectResult(obj=_empty_object(id_echo=object_id.data), found=False)
    return GetObjectResult(obj=_scene_object_to_idl(o), found=True)


@mcp_contract(mcp, contract_id="robonix/system/scene/get_semantic_map")
async def get_semantic_map(region: Region) -> SemanticMapSlice:
    """Region-scoped surfaces + a reference to Nav2's 2D occupancy
    grid (we don't ship voxel data through MCP). When mapping isn't
    running, occupancy_grid_topic is empty; consumers should still
    handle the surfaces list.
    Contract: robonix/system/scene/get_semantic_map."""
    if _REGISTRY is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")
    objs, surfs = await _REGISTRY.snapshot()
    cx, cy = region.center_x, region.center_y
    r = region.radius_m if region.radius_m > 0 else 5.0
    in_region = [
        s for s in surfs.values()
        if (s.pose.x - cx) ** 2 + (s.pose.y - cy) ** 2 <= r * r
    ]
    import os
    return SemanticMapSlice(
        surfaces=[_scene_surface_to_idl(s) for s in in_region],
        occupancy_grid_topic=os.environ.get("ROBONIX_SCENE_OCCUPANCY_TOPIC", ""),
        occupancy_grid_frame=os.environ.get("ROBONIX_SCENE_OCCUPANCY_FRAME", "map"),
        stamp_unix=time.time(),
    )


@mcp_contract(mcp, contract_id="robonix/system/scene/get_safety_context")
async def get_safety_context(scope: String) -> SafetyContext:
    """Stub. v1 always returns status="not_implemented" with empty
    arrays so Pilot can call without exception. Sentinel will fill it.
    Contract: robonix/system/scene/get_safety_context."""
    _ = scope
    return SafetyContext(
        status="not_implemented",
        forbidden_zones=[],
        human_distances_json="{}",
    )


__all__ = [
    "mcp",
    "attach_state",
    "get_snapshot",
    "query",
    "get_object",
    "get_semantic_map",
    "get_safety_context",
]


if TYPE_CHECKING:  # pragma: no cover
    _ = (RelationConstraintIDL,)
