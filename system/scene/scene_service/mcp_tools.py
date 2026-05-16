# SPDX-License-Identifier: MulanPSL-2.0
"""FastMCP tool definitions — two read-only handlers, that's it.

  list_objects()           → flat list of every object in the registry
  goal_near(object_id)     → reachable approach pose for that object

Writes happen on the ingest path (perception → registry); these
handlers only read. Inputs are codegen-derived ROS dataclasses
(`semantic_map_mcp.*`); the @mcp_contract decorator turns each one
into a JSON-schema-typed MCP tool that Pilot discovers via atlas.
"""
from __future__ import annotations

import logging
import math
import time

from .state import ObjectRegistry, SceneObject
from .scene_graph.store import SceneGraphStore
from .scene_graph.types import SceneGraphSnapshot

# Resolved at import time. PYTHONPATH is set by package_manifest.yaml's
# `start:` block to include rbnx-build/codegen/{proto_gen,robonix_mcp_types}.
import semantic_map_mcp  # type: ignore
from semantic_map_mcp import (  # type: ignore
    GoalNear_Request,
    GoalNear_Response,
    GetObjectContext_Request,
    GetObjectContext_Response,
    GetSceneGraph_Request,
    GetSceneGraph_Response,
    ListObjects_Request,
    ListObjects_Response,
    ListRelations_Request,
    ListRelations_Response,
    Object,
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


def attach_state(*, registry: ObjectRegistry, hub=None) -> None:
    global _REGISTRY, _HUB
    _REGISTRY = registry
    _HUB = hub


def attach_scene_graph_store(store: SceneGraphStore) -> None:
    global _SG_STORE
    _SG_STORE = store


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


# ── @mcp_contract handlers ─────────────────────────────────────────────────

mcp = FastMCP("scene_provider")


@mcp_contract(mcp, contract_id="robonix/system/scene/list_objects")
async def list_objects(_req: ListObjects_Request) -> ListObjects_Response:
    """Return every object the scene registry currently believes
    exists, as a flat list. The LLM filters client-side by label /
    distance / etc. — no scoping or filter knobs in the schema.
    Contract: robonix/system/scene/list_objects."""
    if _REGISTRY is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")
    objs, _surfs = await _REGISTRY.snapshot()
    visible = [o for o in objs.values() if not o.missing]
    return ListObjects_Response(
        objects=[_to_idl(o) for o in visible],
        stamp_unix=time.time(),
    )


# Constants for goal_near — service-side defaults, no longer schema knobs.
_GOAL_NEAR_CLEARANCE_M = 0.4   # robot inscribed_radius + safety margin
_GOAL_NEAR_SEARCH_M = 3.0      # max distance to look for a free cell
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
    # Treat unknown (-1) as obstacle: a goal in unmapped space is the
    # same kind of risk as one in a wall.
    blocked = (grid > 50) | (grid < 0)
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
    """Find a navigation-safe approach pose near the named object.
    Returns map-frame (x, y, yaw); pass to navigation/navigate.

    `reachable=false` when:
      - the object_id isn't in the registry, or
      - mapping isn't running (no occupancy_grid yet), or
      - no free cell exists within 3 m of the target on the grid.
    Contract: robonix/system/scene/goal_near."""
    if _REGISTRY is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")
    objs, _surfs = await _REGISTRY.snapshot()
    target = objs.get(req.object_id)
    if target is None:
        return GoalNear_Response(
            reachable=False, x=0.0, y=0.0, yaw=0.0,
            reason=f"unknown object_id '{req.object_id}'",
        )

    # Default approach direction: -x in map frame. Cheap and good
    # enough; the BFS sweeps ±90° so picking a slightly off direction
    # just shifts which ring sample wins.
    approach_ang = math.pi  # i.e. atan2(0, -1)

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


@mcp_contract(mcp, contract_id="robonix/system/scene/get_scene_graph")
async def get_scene_graph(_req: GetSceneGraph_Request) -> GetSceneGraph_Response:
    """Return the current scene graph snapshot — all stable nodes
    with captions and their LLM-inferred spatial relations.
    Contract: robonix/system/scene/get_scene_graph."""
    snap = _sg_snapshot()
    if snap is None:
        return GetSceneGraph_Response(
            nodes=[], edges=[], updated_at=0.0,
        )
    return GetSceneGraph_Response(
        nodes=[_node_to_idl(n) for n in snap.nodes.values()],
        edges=[_edge_to_idl(e) for e in snap.edges],
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
    "list_objects",
    "goal_near",
    "get_scene_graph",
    "get_object_context",
    "list_relations",
]
