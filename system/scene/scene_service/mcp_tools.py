# SPDX-License-Identifier: MulanPSL-2.0
"""FastMCP tool definitions — two read-only handlers, that's it.

  list_objects()           → flat list of every object in the registry
  goal_near(object_id)     → reachable approach pose for a physical object
  goal_room(room_id)       → reachable pose inside a room polygon

Writes happen on the ingest path (perception → registry); these
handlers only read. Inputs are codegen-derived ROS dataclasses
(`semantic_map_mcp.*`); the @mcp_contract decorator turns each one
into a JSON-schema-typed MCP tool that Pilot discovers via atlas.
"""
from __future__ import annotations

import logging
import math
import time
from difflib import SequenceMatcher
from typing import TYPE_CHECKING

from .state import ObjectRegistry, SceneObject
from .scene_graph.store import SceneGraphStore
from .scene_graph.types import SceneGraphSnapshot
from .geometry import disc_inside_polygon, point_in_polygon, polygon_centroid

if TYPE_CHECKING:
    from .annotations import Annotation, AnnotationStore

# Resolved at import time. PYTHONPATH is set by package_manifest.yaml's
# `start:` block to include rbnx-build/codegen/{proto_gen,robonix_mcp_types}.
import semantic_map_mcp  # type: ignore
from semantic_map_mcp import (  # type: ignore
    GoalNear_Request,
    GoalNear_Response,
    GoalRoom_Request,
    GoalRoom_Response,
    GetObjectContext_Request,
    GetObjectContext_Response,
    GetSceneGraph_Request,
    GetSceneGraph_Response,
    ListObjects_Request,
    ListObjects_Response,
    ListRelations_Request,
    ListRelations_Response,
    Object,
    SceneAnnotation as SceneAnnotationIDL,
    SceneGraphEdge as SceneGraphEdgeIDL,
    SceneGraphNode as SceneGraphNodeIDL,
)

from mcp.server.fastmcp import FastMCP
from robonix_api import mcp_contract

log = logging.getLogger(__name__)


# ── Module-level state pointers, set by service.py at startup ──────────────
_REGISTRY: ObjectRegistry | None = None
_HUB = None  # SubscribersHub, exposes .latest("occupancy_grid") for goal_near BFS
_SG_STORE: SceneGraphStore | None = None
_ANNO_STORE: "AnnotationStore | None" = None


def attach_state(*, registry: ObjectRegistry, hub=None) -> None:
    global _REGISTRY, _HUB
    _REGISTRY = registry
    _HUB = hub


def attach_scene_graph_store(store: SceneGraphStore) -> None:
    global _SG_STORE
    _SG_STORE = store


def attach_annotation_store(store: "AnnotationStore | None") -> None:
    global _ANNO_STORE
    _ANNO_STORE = store


# ── conversions: SceneObject → IDL Object ──────────────────────────────────

def _to_idl(o: SceneObject) -> Object:
    return Object(
        id=o.object_id,
        label=o.cls,
        x=float(o.pose.x),
        y=float(o.pose.y),
        z=float(o.pose.z),
        yaw=float(o.pose.yaw),
        last_seen_unix=float(o.last_seen),
    )


def _annotation_centroid(a: "Annotation") -> tuple[float, float]:
    return polygon_centroid(getattr(a, "points", []) or [])


def _annotation_object_id(a: "Annotation") -> str:
    return f"scene.{a.kind}.{a.annotation_id}"


def _annotation_to_object(a: "Annotation") -> Object:
    x, y = _annotation_centroid(a)
    return Object(
        id=_annotation_object_id(a),
        label=str(a.name or a.kind),
        x=float(x),
        y=float(y),
        z=0.0,
        yaw=float(a.theta or 0.0),
        last_seen_unix=float(a.updated_at or 0.0),
    )


def _find_annotation_target(object_id: str) -> "Annotation | None":
    if _ANNO_STORE is None:
        return None
    for annotation in _ANNO_STORE.list():
        if object_id == _annotation_object_id(annotation):
            return annotation
    return None


def _normalize_room_reference(value: str) -> str:
    return " ".join(str(value or "").strip().casefold().split())


def _room_aliases(room: "Annotation") -> set[str]:
    name = _normalize_room_reference(room.name)
    aliases = {name}
    for prefix in ("room ", "room-", "房间 ", "房间"):  # i18n-ok: user room aliases
        if name.startswith(prefix) and name[len(prefix):].strip():
            aliases.add(name[len(prefix):].strip())
    return aliases


def _resolve_room_target(reference: str) -> tuple["Annotation | None", list["Annotation"]]:
    """Resolve stable ID first, then an exact unique room name/short alias.

    The second return value contains ambiguous candidates. Fuzzy matching is
    deliberately excluded: navigation must not guess between similar rooms.
    """
    exact = _find_annotation_target(reference)
    if exact is not None and exact.kind == "room":
        return exact, []
    if _ANNO_STORE is None:
        return None, []
    needle = _normalize_room_reference(reference)
    matches = [
        room for room in _ANNO_STORE.list()
        if room.kind == "room" and needle in _room_aliases(room)
    ]
    if len(matches) == 1:
        return matches[0], []
    return None, matches


def _room_id_hint() -> str:
    if _ANNO_STORE is None:
        return "no rooms are currently registered"
    rooms = [a for a in _ANNO_STORE.list() if a.kind == "room"]
    if not rooms:
        return "no rooms are currently registered"
    candidates = ", ".join(
        f"{room.name!r} (id={_annotation_object_id(room)})"
        for room in rooms[:20]
    )
    suffix = "" if len(rooms) <= 20 else f", ... {len(rooms) - 20} more"
    return f"available rooms: {candidates}{suffix}"


def _object_id_hint(reference: str, objects: list[SceneObject]) -> str:
    if not objects:
        return "no physical objects are currently registered"
    wanted = str(reference).strip().casefold()

    def score(obj: SceneObject) -> float:
        object_id = str(obj.object_id).casefold()
        label = str(obj.cls).casefold()
        return max(
            SequenceMatcher(None, wanted, object_id).ratio(),
            SequenceMatcher(None, wanted, label).ratio(),
        )

    nearest = sorted(objects, key=score, reverse=True)[:3]
    candidates = ", ".join(
        f"{obj.cls!r} (id={obj.object_id})" for obj in nearest
    )
    return f"did you mean one of: {candidates}"


# ── @mcp_contract handlers ─────────────────────────────────────────────────

mcp = FastMCP("scene_provider")


@mcp_contract(mcp, contract_id="robonix/system/scene/list_objects")
async def list_objects(_req: ListObjects_Request) -> ListObjects_Response:
    """Return every object the scene registry currently believes
    exists, plus room annotations, as a flat list. Use this tool to discover
    stable object/room IDs before goal_near or goal_room. Use get_scene_graph
    only when object relationships are needed.
    Contract: robonix/system/scene/list_objects."""
    if _REGISTRY is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")
    objs, _surfs = await _REGISTRY.snapshot()
    visible = [o for o in objs.values() if not o.missing]
    objects = [_to_idl(o) for o in visible]
    if _ANNO_STORE is not None:
        objects.extend(_annotation_to_object(a) for a in _ANNO_STORE.list() if a.kind == "room")
    return ListObjects_Response(
        objects=objects,
        stamp_unix=time.time(),
    )


# Constants for goal_near — service-side defaults, no longer schema knobs.
_GOAL_NEAR_CLEARANCE_M = 0.4   # robot inscribed_radius + safety margin
_GOAL_NEAR_SEARCH_M = 6.0      # max distance to look for a free cell
_GOAL_NEAR_ROBOT_RADIUS_M = 0.3  # Tiago-sized default for inflation
_GOAL_NEAR_RING_STEP_M = 0.1
_GOAL_NEAR_ANGLE_STEPS = [0.0, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6,
                          0.8, -0.8, 1.0, -1.0, 1.2, -1.2,
                          1.4, -1.4, 1.6, -1.6]


def _occupancy_bfs(
    grid_msg, target_x: float, target_y: float, approach_ang: float
) -> tuple[float, float, float] | None:
    """Sweep rings of free cells around (target_x, target_y) on the
    occupancy grid; pick the first one with enough clearance and face
    back at the target. Returns (x, y, yaw) or None if nothing free.
    """
    import numpy as np

    info = grid_msg.info
    w, h = int(info.width), int(info.height)
    res = float(info.resolution)
    ogx = float(info.origin.position.x)
    ogy = float(info.origin.position.y)
    grid = np.frombuffer(bytes(grid_msg.data), dtype=np.int8).reshape(h, w)
    # Occupied cells are hard blockers. Unknown cells are allowed here because
    # scene objects often sit at the edge of the explored map; rejecting all
    # unknown cells makes object-relative navigation unusable before SLAM has
    # fully painted the area.
    blocked = grid > 50
    infl = max(1, int(math.ceil(
        (_GOAL_NEAR_ROBOT_RADIUS_M + _GOAL_NEAR_CLEARANCE_M) / res)))

    def is_safe(gx: int, gy: int) -> bool:
        if gx - infl < 0 or gy - infl < 0 or gx + infl >= w or gy + infl >= h:
            return False
        return not bool(
            blocked[gy - infl: gy + infl + 1, gx - infl: gx + infl + 1].any()
        )

    standoff = 0.5  # min approach distance from object centre
    n_rings = int(_GOAL_NEAR_SEARCH_M / _GOAL_NEAR_RING_STEP_M) + 1
    for i in range(n_rings):
        r = standoff + i * _GOAL_NEAR_RING_STEP_M
        for dth in _GOAL_NEAR_ANGLE_STEPS:
            ang = approach_ang + dth
            wx = target_x - math.cos(ang) * r
            wy = target_y - math.sin(ang) * r
            gx = int((wx - ogx) / res)
            gy = int((wy - ogy) / res)
            if 0 <= gx < w and 0 <= gy < h and is_safe(gx, gy):
                yaw = math.atan2(target_y - wy, target_x - wx)
                return wx, wy, yaw
    return None


@mcp_contract(mcp, contract_id="robonix/system/scene/goal_near")
async def goal_near(req: GoalNear_Request) -> GoalNear_Response:
    """Find a navigation-safe approach pose near a physical scene object.
    Returns map-frame (x, y, yaw); pass to navigation/navigate.

    Room annotations are deliberately not accepted. Resolve those through
    goal_room so the returned pose is constrained to the room polygon.

    `reachable=false` when:
      - the object_id isn't in the registry, or
      - mapping isn't running (no occupancy_grid yet), or
      - no free cell exists within the search radius of the target on the grid.
    Contract: robonix/system/scene/goal_near."""
    if _REGISTRY is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")
    objs, _surfs = await _REGISTRY.snapshot()
    target = objs.get(req.object_id)
    if target is None:
        visible = [obj for obj in objs.values() if not obj.missing]
        return GoalNear_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=(
                f"unknown physical object_id '{req.object_id}'; "
                f"{_object_id_hint(req.object_id, visible)}; "
                "use goal_room for room annotations"
            ),
        )

    robot = next(
        (
            o for o in objs.values()
            if getattr(o, "is_robot", False) or str(o.cls).lower() == "robot"
        ),
        None,
    )
    if robot is not None:
        approach_ang = math.atan2(
            float(target.pose.y) - float(robot.pose.y),
            float(target.pose.x) - float(robot.pose.x),
        )
    else:
        # Fallback if self-tracking has not populated yet.
        approach_ang = math.pi

    if _HUB is None or not _HUB.has("occupancy_grid"):
        return GoalNear_Response(
            reachable=False,
            x=float(target.pose.x), y=float(target.pose.y), yaw=0.0,
            reason="no occupancy_grid available — mapping not running",
        )

    try:
        msg, _stamp, count = _HUB.latest("occupancy_grid")
    except Exception as e:  # noqa: BLE001
        return GoalNear_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=f"occupancy_grid hub error: {e}",
        )
    if msg is None or count == 0 or not msg.info.width or not msg.info.height:
        return GoalNear_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason="occupancy_grid empty — wait for mapping to publish",
        )

    found = _occupancy_bfs(
        msg,
        float(target.pose.x),
        float(target.pose.y),
        approach_ang,
    )
    if found is None:
        return GoalNear_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=f"no free cell within {_GOAL_NEAR_SEARCH_M:.1f}m of '{req.object_id}'",
        )
    gx, gy, yaw = found
    return GoalNear_Response(
        reachable=True, x=float(gx), y=float(gy), yaw=float(yaw),
        reason=f"approach pose for '{target.cls}' ({req.object_id})",
    )


def _occupancy_room_goal(grid_msg, points) -> tuple[float, float] | None:
    """Choose the safest free grid cell nearest a room's polygon centroid."""
    import numpy as np

    polygon = [(float(x), float(y)) for x, y in points]
    if len(polygon) < 3:
        return None
    info = grid_msg.info
    width, height = int(info.width), int(info.height)
    resolution = float(info.resolution)
    origin_x = float(info.origin.position.x)
    origin_y = float(info.origin.position.y)
    grid = np.frombuffer(bytes(grid_msg.data), dtype=np.int8).reshape(height, width)
    # A room destination must be known free space. Unknown cells are not goals.
    blocked = (grid < 0) | (grid > 50)
    footprint_radius = _GOAL_NEAR_ROBOT_RADIUS_M + _GOAL_NEAR_CLEARANCE_M
    inflation_cells = max(1, int(math.ceil(footprint_radius / resolution)))
    centroid_x, centroid_y = polygon_centroid(polygon)

    min_x = max(0, int((min(x for x, _ in polygon) - origin_x) / resolution))
    max_x = min(width - 1, int((max(x for x, _ in polygon) - origin_x) / resolution))
    min_y = max(0, int((min(y for _, y in polygon) - origin_y) / resolution))
    max_y = min(height - 1, int((max(y for _, y in polygon) - origin_y) / resolution))
    candidates = []
    for gy in range(min_y, max_y + 1):
        wy = origin_y + (gy + 0.5) * resolution
        for gx in range(min_x, max_x + 1):
            wx = origin_x + (gx + 0.5) * resolution
            if not disc_inside_polygon(wx, wy, footprint_radius, polygon):
                continue
            if (
                gx - inflation_cells < 0
                or gy - inflation_cells < 0
                or gx + inflation_cells >= width
                or gy + inflation_cells >= height
            ):
                continue
            local = blocked[
                gy - inflation_cells: gy + inflation_cells + 1,
                gx - inflation_cells: gx + inflation_cells + 1,
            ]
            if not bool(local.any()):
                candidates.append(((wx - centroid_x) ** 2 + (wy - centroid_y) ** 2, wx, wy))
    if not candidates:
        return None
    _, x, y = min(candidates)
    return x, y


@mcp_contract(mcp, contract_id="robonix/system/scene/goal_room")
async def goal_room(req: GoalRoom_Request) -> GoalRoom_Response:
    """Resolve a room annotation to a safe map-frame pose inside its polygon.

    Use this for named rooms and user-defined regions before navigation/navigate.
    The result never falls outside the room polygon.
    Contract: robonix/system/scene/goal_room.
    """
    room, ambiguous = _resolve_room_target(req.room_id)
    if ambiguous:
        candidates = ", ".join(
            f"{item.name!r} (id={_annotation_object_id(item)})"
            for item in ambiguous
        )
        return GoalRoom_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=(
                f"ambiguous room reference '{req.room_id}'; candidates: "
                f"{candidates}; pass one exact stable ID"
            ),
        )
    if room is None:
        return GoalRoom_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=(
                f"unknown room reference '{req.room_id}'; pass a stable ID or "
                f"unique room name returned by list_objects; {_room_id_hint()}"
            ),
        )
    stable_room_id = _annotation_object_id(room)
    if _HUB is None or not _HUB.has("occupancy_grid"):
        return GoalRoom_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason="no occupancy_grid available - mapping not running",
        )
    try:
        msg, _stamp, count = _HUB.latest("occupancy_grid")
    except Exception as exc:  # noqa: BLE001
        return GoalRoom_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=f"occupancy_grid hub error: {exc}",
        )
    if msg is None or count == 0 or not msg.info.width or not msg.info.height:
        return GoalRoom_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason="occupancy_grid empty - wait for mapping to publish",
        )
    found = _occupancy_room_goal(msg, room.points)
    if found is None:
        return GoalRoom_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=f"no known free pose inside room '{room.name}' ({stable_room_id})",
        )
    x, y = found
    if not point_in_polygon(x, y, room.points):
        return GoalRoom_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason="internal validation rejected a pose outside the room polygon",
        )
    return GoalRoom_Response(
        reachable=True,
        x=float(x),
        y=float(y),
        yaw=float(room.theta or 0.0),
        reason=f"safe pose inside room '{room.name}' ({stable_room_id})",
    )


# ── scene graph MCP tools ────────────────────────────────────────────────────

def _sg_snapshot() -> SceneGraphSnapshot | None:
    if _SG_STORE is None:
        return None
    return _SG_STORE.get_snapshot()


def _node_to_idl(n) -> SceneGraphNodeIDL:
    return SceneGraphNodeIDL(
        object_id=n.object_id,
        label=n.label,
        caption=n.caption or n.label,
        x=float(n.bbox_center[0]),
        y=float(n.bbox_center[1]),
        z=float(n.bbox_center[2]),
        confidence=float(n.confidence),
        observation_count=int(n.observation_count),
        last_seen_unix=float(n.last_seen or 0.0),
    )


def _edge_to_idl(e) -> SceneGraphEdgeIDL:
    return SceneGraphEdgeIDL(
        source_id=e.source_id,
        target_id=e.target_id,
        relation=e.relation,
        confidence=float(e.confidence),
        reason=e.reason,
    )


def _annotation_to_idl(a: "Annotation") -> SceneAnnotationIDL:
    points_xy: list[float] = []
    for point in a.points or []:
        if len(point) >= 2:
            points_xy.extend([float(point[0]), float(point[1])])
    return SceneAnnotationIDL(
        annotation_id=a.annotation_id,
        kind=a.kind,
        name=a.name,
        points_xy=points_xy,
        theta=float(a.theta) if a.theta is not None else 0.0,
        stale=bool(a.stale),
        stale_reason=a.stale_reason or "",
        updated_at_unix=float(a.updated_at or 0.0),
    )


def _annotation_list() -> list[SceneAnnotationIDL]:
    if _ANNO_STORE is None:
        return []
    return [_annotation_to_idl(a) for a in _ANNO_STORE.list()]


@mcp_contract(mcp, contract_id="robonix/system/scene/get_scene_graph")
async def get_scene_graph(_req: GetSceneGraph_Request) -> GetSceneGraph_Response:
    """Return the current scene graph snapshot — all stable nodes
    with captions and their LLM-inferred spatial relations.
    Contract: robonix/system/scene/get_scene_graph."""
    snap = _sg_snapshot()
    if snap is None:
        return GetSceneGraph_Response(
            nodes=[], edges=[], annotations=_annotation_list(), updated_at=0.0,
        )
    return GetSceneGraph_Response(
        nodes=[_node_to_idl(n) for n in snap.nodes.values()],
        edges=[_edge_to_idl(e) for e in snap.edges],
        annotations=_annotation_list(),
        updated_at=snap.updated_at,
    )


@mcp_contract(mcp, contract_id="robonix/system/scene/get_object_context")
async def get_object_context(req: GetObjectContext_Request) -> GetObjectContext_Response:
    """Return a single object's scene graph node, its direct relations,
    and nearby objects sorted by distance. Useful for Pilot to reason
    about one object's context without fetching the full graph.
    Contract: robonix/system/scene/get_object_context."""
    snap = _sg_snapshot()
    if snap is None or req.object_id not in snap.nodes:
        # Fall back to registry if scene graph has no data yet.
        empty_node = SceneGraphNodeIDL(
            object_id=req.object_id, label="", caption="",
            x=0.0, y=0.0, z=0.0, confidence=0.0,
            observation_count=0, last_seen_unix=0.0,
        )
        return GetObjectContext_Response(
            object=empty_node, relations=[], nearby_objects=[],
        )

    node = snap.nodes[req.object_id]

    # Direct relations involving this object.
    related_edges = [
        _edge_to_idl(e) for e in snap.edges
        if e.source_id == req.object_id or e.target_id == req.object_id
    ]

    # Nearby objects by distance (exclude self).
    cx, cy, cz = node.bbox_center
    nearby: list[tuple[float, object]] = []
    for other in snap.nodes.values():
        if other.object_id == req.object_id:
            continue
        ox, oy, oz = other.bbox_center
        d = math.sqrt((cx - ox) ** 2 + (cy - oy) ** 2 + (cz - oz) ** 2)
        nearby.append((d, other))
    nearby.sort(key=lambda t: t[0])
    nearby_objs = [
        Object(
            id=n.object_id, label=n.label,
            x=float(n.bbox_center[0]), y=float(n.bbox_center[1]),
            z=float(n.bbox_center[2]),
            last_seen_unix=float(n.last_seen or 0.0),
        )
        for _, n in nearby[:5]
    ]

    return GetObjectContext_Response(
        object=_node_to_idl(node),
        relations=related_edges,
        nearby_objects=nearby_objs,
    )


@mcp_contract(mcp, contract_id="robonix/system/scene/list_relations")
async def list_relations(req: ListRelations_Request) -> ListRelations_Response:
    """List scene graph edges, optionally filtered by relation type.
    Pass empty relation string to get all edges.
    Contract: robonix/system/scene/list_relations."""
    snap = _sg_snapshot()
    if snap is None:
        return ListRelations_Response(edges=[])

    edges = snap.edges
    if req.relation:
        edges = [e for e in edges if e.relation == req.relation]

    return ListRelations_Response(
        edges=[_edge_to_idl(e) for e in edges],
    )


__all__ = [
    "mcp",
    "attach_state",
    "attach_scene_graph_store",
    "attach_annotation_store",
    "list_objects",
    "goal_near",
    "goal_room",
    "get_scene_graph",
    "get_object_context",
    "list_relations",
]
