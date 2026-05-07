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
    GetSafeGoalNearObject_Request,
    GetSafeGoalNearObject_Response,
    Object,
    Point3D,
    QueryConstraints,
    QueryResults,
    Relation,
    RelationConstraint as RelationConstraintIDL,
    RelationType,
    SafetyContext,
    SceneSnapshot,
    SnapshotSpec,
)
from geometry_msgs_mcp import PoseStamped, Point, Quaternion  # type: ignore
from std_msgs_mcp import String

from mcp.server.fastmcp import FastMCP
from robonix_py import mcp_contract

log = logging.getLogger(__name__)


# ── Module-level state pointers, set by service.py at startup ──────────────
_REGISTRY: ObjectRegistry | None = None
_RELATIONS: RelationEngine | None = None
_TRANSFORM_TO_MAP = None  # callable(Pose3D)->Pose3D, or None
_HUB = None  # SubscribersHub, exposes .latest("occupancy_grid") for safe-goal BFS


def attach_state(
    *,
    registry: ObjectRegistry,
    relations: RelationEngine,
    transform_to_map=None,
    hub=None,
) -> None:
    global _REGISTRY, _RELATIONS, _TRANSFORM_TO_MAP, _HUB
    _REGISTRY = registry
    _RELATIONS = relations
    _TRANSFORM_TO_MAP = transform_to_map
    _HUB = hub


# ── conversions: Python state → IDL ────────────────────────────────────────

_REL_NAME_TO_ENUM: dict[str, int] = {
    "child_of": 0,
    "on_top": 1,
    "on": 1,  # alias used by relations.py
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
        # registered_skills=[],
        # registered_primitives=[],
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
        # registered_skills=[],
        # registered_primitives=[],
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
    return Relation(
        relation_type=_enum_for(r.predicate), target_entity_id=r.target_object_id
    )


def _empty_object(id_echo: str = "") -> Object:
    """Sentinel for `get_object` not-found case. Pilot keys on the bool
    `found`, but the embedded Object can't be None — codegen wants a
    structured value either way."""
    return Object(
        id=id_echo,
        label="",
        relations=[],
        # registered_skills=[],
        # registered_primitives=[],
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
        region_center_xyz=(
            spec.region_center_x,
            spec.region_center_y,
            spec.region_center_z,
        ),
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
        scope,
        objects=objs,
        surfaces=surfs,
        relations=rels,
        now=now,
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
        candidates = [
            o for o in candidates if o.confidence >= constraints.min_confidence
        ]

    if constraints.pose_radius_m and constraints.pose_radius_m > 0:
        cx, cy = constraints.pose_center_x, constraints.pose_center_y
        r = constraints.pose_radius_m
        candidates = [
            o
            for o in candidates
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
                    r
                    for r in relations
                    if r.subject_object_id == o.object_id and r.predicate == pred
                ]
                if not hits:
                    ok = False
                    break
                if rc.target_object_id:
                    hits = [
                        r for r in hits if r.target_object_id == rc.target_object_id
                    ]
                if rc.target_cls:
                    target_cls = rc.target_cls.lower()
                    hits = [
                        r
                        for r in hits
                        if (
                            r.target_object_id in objs
                            and objs[r.target_object_id].cls.lower() == target_cls
                        )
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


@mcp_contract(mcp, contract_id="robonix/system/scene/get_safe_goal_near_object")
async def get_safe_goal_near_object(
    req: GetSafeGoalNearObject_Request,
) -> GetSafeGoalNearObject_Response:
    """Find a navigation-safe approach pose near a registered scene object.
    Returns a PoseStamped in the map frame, facing back toward the object
    centre. Use this instead of guessing map coordinates and feed the result
    straight into `service/navigation/navigate`.

    Algorithm (when an OccupancyGrid is available via the ingest hub):
      1. Compute the bbox-derived standoff distance.
      2. Build an inflated obstacle mask: occupied (>50) and unknown (<0)
         cells; inflate by `(robot_radius + clearance) / resolution` cells.
      3. Sweep candidate goals on rings around the object centre at
         standoff..standoff+search_radius_m, sweeping ±90° about the robot's
         approach direction. Pick the first inflation-free cell.
      4. Set yaw to face the object centre.

    Without an occupancy grid (mapping not yet up) we fall back to the
    bbox-derived approach point and report the limitation in `reason`.

    Contract: robonix/system/scene/get_safe_goal_near_object."""
    import math

    if _REGISTRY is None:
        return GetSafeGoalNearObject_Response(
            reachable=False,
            pose=PoseStamped(),
            reason="scene mcp_tools.attach_state was never called",
        )
    objs, _ = await _REGISTRY.snapshot()
    o = objs.get(req.object_id.data)
    if o is None:
        return GetSafeGoalNearObject_Response(
            reachable=False,
            pose=PoseStamped(),
            reason=f"unknown object_id {req.object_id.data!r}",
        )

    clearance = float(req.clearance_m) if req.clearance_m > 0 else 0.4
    search_r = float(req.search_radius_m) if req.search_radius_m > 0 else 3.0
    cx, cy = float(o.pose.x), float(o.pose.y)
    # BBox3D is axis-aligned (modulo yaw) and centered on the object pose;
    # half_x / half_y are the bbox extents we need for the standoff radius.
    ox = oy = 0.0
    if o.bbox is not None:
        ox = float(o.bbox.half_x)
        oy = float(o.bbox.half_y)
    standoff = max(0.5, math.hypot(ox, oy)) + clearance

    # Approach direction = from robot toward the object. Without a self
    # pose any direction is as valid as another; -x is fine for nav.
    self_xy = None
    if _RELATIONS is not None and getattr(_RELATIONS, "_self_tracker", None):
        latest = _RELATIONS._self_tracker.latest_xy_yaw()  # type: ignore[attr-defined]
        if latest:
            self_xy = (latest[0], latest[1])
    if self_xy is None:
        appr_dx, appr_dy = -1.0, 0.0
    else:
        dx, dy = cx - self_xy[0], cy - self_xy[1]
        n = math.hypot(dx, dy)
        appr_dx, appr_dy = (dx / n, dy / n) if n > 1e-3 else (-1.0, 0.0)
    appr_ang = math.atan2(appr_dy, appr_dx)

    # ── 1. real BFS on the occupancy grid (preferred) ──────────────────
    if _HUB is not None and _HUB.has("occupancy_grid"):
        try:
            import numpy as np

            msg, _stamp, count = _HUB.latest("occupancy_grid")
            if msg is not None and count > 0 and msg.info.width and msg.info.height:
                info = msg.info
                w, h = int(info.width), int(info.height)
                res = float(info.resolution)
                ogx = float(info.origin.position.x)
                ogy = float(info.origin.position.y)
                grid = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(h, w)
                # Treat unknown as obstacle: a navigation goal in unmapped
                # space is the same kind of risk as one in a wall.
                blocked = (grid > 50) | (grid < 0)

                # Robot footprint inflation. 0.3 m is a Tiago-sized default;
                # `clearance_m` from the caller bumps it further.
                robot_radius = 0.3
                infl = max(1, int(math.ceil((robot_radius + clearance) / res)))

                def is_safe(gx: int, gy: int) -> bool:
                    if (
                        gx - infl < 0
                        or gy - infl < 0
                        or gx + infl >= w
                        or gy + infl >= h
                    ):
                        return False
                    return not bool(
                        blocked[
                            gy - infl : gy + infl + 1, gx - infl : gx + infl + 1
                        ].any()
                    )

                # Sweep: rings of radius standoff..standoff+search_r at
                # 0.1 m steps; angles ±π/2 about approach in 0.2 rad steps.
                # First hit wins (closest to caller's preferred geometry).
                step_r = max(res, 0.1)
                n_rings = int(search_r / step_r) + 1
                d_angles = [0.0]
                for k in range(1, 9):  # ±0.2..±1.6 rad ≈ ±91°
                    d_angles.extend((k * 0.2, -k * 0.2))

                best = None
                for i in range(n_rings):
                    r = standoff + i * step_r
                    for dth in d_angles:
                        ang = appr_ang + dth
                        wx = cx - math.cos(ang) * r
                        wy = cy - math.sin(ang) * r
                        gx = int((wx - ogx) / res)
                        gy = int((wy - ogy) / res)
                        if 0 <= gx < w and 0 <= gy < h and is_safe(gx, gy):
                            best = (wx, wy, ang, r, dth)
                            break
                    if best is not None:
                        break

                if best is not None:
                    bx, by, _bang, br, bdth = best
                    yaw = math.atan2(cy - by, cx - bx)
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position = Point(x=float(bx), y=float(by), z=0.0)
                    pose.pose.orientation = Quaternion(
                        x=0.0,
                        y=0.0,
                        z=float(math.sin(yaw / 2.0)),
                        w=float(math.cos(yaw / 2.0)),
                    )
                    return GetSafeGoalNearObject_Response(
                        reachable=True,
                        pose=pose,
                        reason=(
                            f"occupancy grid: r={br:.2f}m "
                            f"Δθ={math.degrees(bdth):+.0f}° "
                            f"inflate={infl}cells({(infl*res):.2f}m)"
                        ),
                    )
                # Map exists but found no free goal — explicit failure rather
                # than returning a bbox guess that lands in a wall.
                return GetSafeGoalNearObject_Response(
                    reachable=False,
                    pose=PoseStamped(),
                    reason=(
                        f"occupancy grid had no free cell within "
                        f"{search_r:.1f}m of {req.object_id.data} "
                        f"at standoff={standoff:.2f}m, inflate={infl}cells"
                    ),
                )
        except Exception as e:  # noqa: BLE001
            log.warning(
                "[safe_goal] occupancy BFS errored, falling back to bbox: %s", e
            )

    # ── 2. bbox fallback (no map yet) ──────────────────────────────────
    ax = cx - appr_dx * standoff
    ay = cy - appr_dy * standoff
    yaw = math.atan2(cy - ay, cx - ax)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position = Point(x=float(ax), y=float(ay), z=0.0)
    pose.pose.orientation = Quaternion(
        x=0.0,
        y=0.0,
        z=float(math.sin(yaw / 2.0)),
        w=float(math.cos(yaw / 2.0)),
    )
    return GetSafeGoalNearObject_Response(
        reachable=True,
        pose=pose,
        reason=(
            f"no occupancy_grid yet; bbox-derived "
            f"standoff={standoff:.2f}m from {req.object_id.data}"
        ),
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
    "get_safe_goal_near_object",
    "get_safety_context",
]


if TYPE_CHECKING:  # pragma: no cover
    _ = (RelationConstraintIDL,)
