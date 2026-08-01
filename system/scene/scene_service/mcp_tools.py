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

import asyncio
import heapq
import logging
import math
import os
import time
from collections.abc import Iterable
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
    GetRobotContext_Request,
    GetRobotContext_Response,
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

# Scene Hook: when list_objects detects visible objects, automatically
# capture the latest RGB frame and POST to memgraph's Scene Hook HTTP
# endpoint so the robot's memory is updated without Pilot involvement.
# Set MEMGRAPH_HOOK_URL to override the memgraph Scene Hook address.
# Default 127.0.0.1 works when Scene runs with --network host or on
# the same machine as memgraph.  For Docker without host networking
# use "http://172.17.0.1:37798" (default bridge gateway).
_MEMGRAPH_HOOK_URL = os.environ.get(
    "MEMGRAPH_HOOK_URL",
    "http://127.0.0.1:37798",
)
_SAVE_COOLDOWN_S = 2.0
_last_save_ts: float = 0.0
_last_save_ids: frozenset = frozenset()


async def _try_save_observation(visible_objects: list) -> None:
    """Fire-and-forget: capture RGB frame + save observation to memgraph.

    Never raises — all failures are logged at debug/warning level so
    the caller (list_objects) is never affected.

    Log trace (every stage emits a structured log line so the full
    lifecycle is grep-able):
      scene_hook: triggered — N objects: [...]           (info, entry)
      scene_hook: rgb WxH encoding (seq=N, age=Ts)       (info)
      scene_hook: encoded WxH → JPEG N bytes (ratio=X%)  (info)
      scene_hook: POST N bytes → memgraph ...            (info)
      scene_hook: ← memgraph 200 node=N (Tms)            (info, success)
      scene_hook: ← memgraph NNN <reason>                (warning, failure)
      scene_hook: ! <exception>                           (warning, crash)
    """
    global _last_save_ts, _last_save_ids
    t0 = time.time()

    # ── throttle: skip when the same objects were just saved ──────
    now = t0
    obj_ids = frozenset(o.object_id for o in visible_objects)
    if obj_ids == _last_save_ids and (now - _last_save_ts) < _SAVE_COOLDOWN_S:
        log.debug("scene_hook: throttled (same %d objects, %.1fs ago)",
                  len(obj_ids), now - _last_save_ts)
        return

    labels = [o.cls for o in visible_objects if o.cls]
    n_objs = len(visible_objects)
    log.info("scene_hook: triggered — %d objects: %s",
             n_objs, ", ".join(labels) if labels else "<none>")

    try:
        # ── grab latest RGB frame from ROS hub ────────────────────
        if _HUB is None or not _HUB.has("rgb"):
            log.info("scene_hook: skip — no rgb subscriber on hub")
            return
        rgb_msg, stamp_unix, seq = _HUB.latest("rgb")
        if rgb_msg is None:
            log.info("scene_hook: skip — no rgb frame yet (hub has slot, seq=0)")
            return

        h, w, enc = rgb_msg.height, rgb_msg.width, rgb_msg.encoding
        frame_age_s = (time.time() - stamp_unix) if stamp_unix > 0 else -1
        log.info("scene_hook: rgb %dx%d %s (seq=%d, age=%.1fs)",
                 w, h, enc, seq, frame_age_s)

        # ── raw RGB8 → JPEG ───────────────────────────────────────
        t_encode = time.time()
        import numpy as np

        try:
            import cv2
        except ImportError:
            log.info("scene_hook: skip — cv2 unavailable")
            return

        raw = bytes(rgb_msg.data)
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, -1)
        if enc == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            log.warning("scene_hook: cv2.imencode returned False")
            return
        import base64
        jpg_bytes = jpg.tobytes()
        img_b64 = base64.b64encode(jpg_bytes).decode("ascii")
        raw_kb, jpg_kb = len(raw) / 1024, len(jpg_bytes) / 1024
        encode_ms = (time.time() - t_encode) * 1000
        log.info("scene_hook: encoded %dx%d → JPEG %.1f KB (raw %.1f KB, ratio %.0f%%, %dms)",
                 w, h, jpg_kb, raw_kb, 100 * jpg_kb / max(raw_kb, 1), round(encode_ms))

        # ── build remember request ────────────────────────────────
        msg = (
            f"observed {', '.join(labels)} in the scene"
            if labels
            else "observed scene"
        )
        spatial_objects = [
            {
                "obj_id": o.object_id,
                "label": o.cls,
                "x": float(o.pose.x),
                "y": float(o.pose.y),
                "z": float(o.pose.z),
            }
            for o in visible_objects
        ]
        payload = {
            "session_id": "scene-auto",
            "plan_id": "scene-auto",
            "log_record": {
                "ts": time.time_ns(),
                "level": "Info",
                "tag": "scene",
                "msg": msg,
            },
            "spatial": {"origin": "world", "objects": spatial_objects},
            "image_base64": img_b64,
        }
        body_bytes = len(img_b64)  # approximate — base64 dominates

        # ── POST to memgraph Scene Hook ───────────────────────────
        t_post = time.time()
        import httpx

        log.info("scene_hook: POST → memgraph (%d objects, b64len=%d, body≈%.1f KB)",
                 n_objs, len(img_b64), body_bytes / 1024)
        async with httpx.AsyncClient(timeout=5.0) as client:
            r = await client.post(_MEMGRAPH_HOOK_URL, json=payload)
        post_ms = (time.time() - t_post) * 1000

        if r.status_code >= 400:
            log.warning("scene_hook: ← memgraph %d (%dms): %s",
                        r.status_code, round(post_ms), r.text[:200])
            return

        result = r.json()
        node_id = result.get("node_id", "?")
        _last_save_ts = now
        _last_save_ids = obj_ids
        total_ms = (time.time() - t0) * 1000
        log.info("scene_hook: ← memgraph 200 node=%s (%dms post, %dms total)",
                 node_id, round(post_ms), round(total_ms))
    except Exception:
        total_ms = (time.time() - t0) * 1000
        log.warning("scene_hook: ! exception after %dms", round(total_ms), exc_info=True)


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

    # Scene Hook: auto-save observation when objects are visible.
    # This is fire-and-forget — list_objects returns immediately
    # regardless of whether the save succeeds.
    if visible:
        asyncio.create_task(_try_save_observation(visible))

    return ListObjects_Response(
        objects=objects,
        stamp_unix=time.time(),
    )


def _polygon_area(points) -> float:
    polygon = [(float(x), float(y)) for x, y in (points or [])]
    if len(polygon) < 3:
        return 0.0
    return abs(sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1])
    )) * 0.5


@mcp_contract(mcp, contract_id="robonix/system/scene/get_robot_context")
async def get_robot_context(_req: GetRobotContext_Request) -> GetRobotContext_Response:
    """Return one coherent map-frame spatial snapshot for Pilot."""
    snapshot_at = time.time()
    if _REGISTRY is None:
        raise RuntimeError("scene mcp_tools.attach_state was never called")
    objects, _surfaces = await _REGISTRY.snapshot()
    robot = next((
        item for item in objects.values()
        if not item.missing
        and (getattr(item, "is_robot", False) or str(item.cls).lower() == "robot")
    ), None)
    map_id = _ANNO_STORE.map_id if _ANNO_STORE is not None else ""
    if robot is None:
        return GetRobotContext_Response(
            pose_known=False, map_id=map_id, x=0.0, y=0.0, z=0.0, yaw=0.0,
            room_id="", room_name="", containing_area_ids=[],
            containing_area_names=[], nearby_objects=[], observed_at_unix=0.0,
            snapshot_at_unix=snapshot_at, stale=True,
            reason="robot pose is not available from Scene",
        )

    x, y = float(robot.pose.x), float(robot.pose.y)
    containing = []
    if _ANNO_STORE is not None:
        containing = [
            annotation for annotation in _ANNO_STORE.list()
            if len(annotation.points or []) >= 3
            and point_in_polygon(x, y, annotation.points)
        ]
    rooms = sorted(
        (annotation for annotation in containing if annotation.kind == "room"),
        key=lambda annotation: (_polygon_area(annotation.points), annotation.name),
    )
    room = rooms[0] if rooms else None
    containing.sort(key=lambda annotation: (annotation.kind, annotation.name))
    nearby = []
    for item in objects.values():
        if item is robot or item.missing:
            continue
        distance = math.hypot(float(item.pose.x) - x, float(item.pose.y) - y)
        if distance <= 3.0:
            nearby.append((distance, item))
    nearby.sort(key=lambda entry: (entry[0], entry[1].object_id))

    observed_at = float(robot.last_seen or 0.0)
    stale = observed_at <= 0.0 or snapshot_at - observed_at > 2.0
    reason = "current Scene spatial snapshot"
    if stale:
        reason = "Scene robot pose is older than 2 seconds"
    elif room is None:
        reason = "robot pose is current but outside every registered room"
    return GetRobotContext_Response(
        pose_known=True, map_id=map_id, x=x, y=y, z=float(robot.pose.z),
        yaw=float(robot.pose.yaw),
        room_id=_annotation_object_id(room) if room is not None else "",
        room_name=str(room.name) if room is not None else "",
        containing_area_ids=[_annotation_object_id(item) for item in containing],
        containing_area_names=[str(item.name) for item in containing],
        nearby_objects=[_to_idl(item) for _, item in nearby[:12]],
        observed_at_unix=observed_at, snapshot_at_unix=snapshot_at,
        stale=stale, reason=reason,
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


def _cells_nearest_to(
    *,
    min_gx: int,
    max_gx: int,
    min_gy: int,
    max_gy: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    target_x: float,
    target_y: float,
) -> Iterable[tuple[float, float, int, int]]:
    """Yield grid cells in increasing distance from a target point.

    The priority flood retains only its explored frontier, allowing room-goal
    search to stop at the nearest safe cell instead of scanning the full room.
    """
    start_gx = min(
        max(math.floor((target_x - origin_x) / resolution), min_gx),
        max_gx,
    )
    start_gy = min(
        max(math.floor((target_y - origin_y) / resolution), min_gy),
        max_gy,
    )
    pending: list[tuple[float, float, float, int, int]] = []
    visited: set[tuple[int, int]] = set()

    def push(gx: int, gy: int) -> None:
        """Add one unseen in-bounds cell to the distance frontier."""
        if not (min_gx <= gx <= max_gx and min_gy <= gy <= max_gy):
            return
        if (gx, gy) in visited:
            return
        visited.add((gx, gy))
        x = origin_x + (gx + 0.5) * resolution
        y = origin_y + (gy + 0.5) * resolution
        distance_sq = (x - target_x) ** 2 + (y - target_y) ** 2
        heapq.heappush(pending, (distance_sq, x, y, gx, gy))

    push(start_gx, start_gy)
    while pending:
        _distance_sq, x, y, gx, gy = heapq.heappop(pending)
        yield x, y, gx, gy
        push(gx - 1, gy)
        push(gx + 1, gy)
        push(gx, gy - 1)
        push(gx, gy + 1)


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
    cells = _cells_nearest_to(
        min_gx=min_x,
        max_gx=max_x,
        min_gy=min_y,
        max_gy=max_y,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        target_x=centroid_x,
        target_y=centroid_y,
    )
    for wx, wy, gx, gy in cells:
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
            return wx, wy
    return None


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
    "get_robot_context",
    "list_objects",
    "goal_near",
    "goal_room",
    "get_scene_graph",
    "get_object_context",
    "list_relations",
]
