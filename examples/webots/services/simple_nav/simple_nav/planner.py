# SPDX-License-Identifier: MulanPSL-2.0
"""A* path planner over a nav_msgs/OccupancyGrid.

8-connectivity, Euclidean heuristic, with an inflation buffer so paths
keep clear of walls. Returns a list of (x, y) world points; consumers
(the RPP follower) treat it as a polyline to follow.

Not Nav2 — but mathematically the same algorithm (A* + grid + costmap
inflation) the community has been using since the early 2010s.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class GridMap:
    """Numpy view of an OccupancyGrid: (data, info_resolution, origin_x,
    origin_y, width, height). Cells: -1 unknown, 0 free, 100 occupied
    (anything > occ_thresh is treated as obstacle).
    """
    data: np.ndarray            # shape (height, width), int8
    resolution: float           # m / cell
    origin_x: float             # world coord of cell (0,0) center
    origin_y: float
    width: int
    height: int

    @classmethod
    def from_msg(cls, msg) -> "GridMap":
        h, w = int(msg.info.height), int(msg.info.width)
        arr = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(h, w)
        return cls(
            data=arr,
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            width=w, height=h,
        )

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        x = self.origin_x + (cx + 0.5) * self.resolution
        y = self.origin_y + (cy + 0.5) * self.resolution
        return x, y

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height


def build_costmap(gm: GridMap, *,
                    inscribed_m: float,
                    inflation_m: float,
                    occ_thresh: int = 50,
                    allow_unknown: bool = True
                    ) -> Tuple[np.ndarray, np.ndarray]:
    """Build a nav2-style 2-layer costmap from an OccupancyGrid.

    Returns ``(passable, cost)``:

    * ``passable``: bool array, True iff the robot CENTER may occupy
      that cell. Cells within ``inscribed_m`` of an obstacle are
      forbidden (lethal layer) — that radius is the robot's footprint
      half-width, so a robot center inside this halo would be in
      collision.
    * ``cost``: float array in [0, 1]. 1 right at the lethal boundary
      and decays exponentially over ``inflation_m`` to ~0 in open
      space. A* adds this to its move cost so paths prefer corridor
      centers over wall-hugging.

    This matches nav2 costmap_2d's INSCRIBED + INFLATION layer split:
    binary inflation alone (the old behaviour) caused PP tracking
    overshoot to clip walls because a "passable" cell next to a
    "lethal" cell had identical cost to one in the middle of an
    open room.

    ``allow_unknown=True`` lets A* traverse unknown cells (-1) — same
    trade-off as nav2's ``track_unknown_space: true``: required for
    exploration past the current map edge.
    """
    if gm.resolution <= 0:
        raise ValueError("resolution must be positive")
    if allow_unknown:
        obstacles = (gm.data >= occ_thresh)
    else:
        obstacles = (gm.data >= occ_thresh) | (gm.data < 0)

    res = gm.resolution
    r_inscribed_cells = max(1, int(math.ceil(inscribed_m / res)))
    r_inflation_cells = max(r_inscribed_cells,
                            int(math.ceil((inscribed_m + inflation_m) / res)))

    h, w = obstacles.shape
    # Distance transform via outward dilation: track the min distance
    # from each cell to the nearest obstacle. Cheaper than scipy's EDT
    # for the grid sizes we work with (≤ 500x500).
    INF = 1e9
    dist = np.full((h, w), INF, dtype=np.float32)
    dist[obstacles] = 0.0
    yy, xx = np.where(obstacles)
    for dy in range(-r_inflation_cells, r_inflation_cells + 1):
        for dx in range(-r_inflation_cells, r_inflation_cells + 1):
            d2 = dx * dx + dy * dy
            if d2 == 0 or d2 > r_inflation_cells * r_inflation_cells:
                continue
            d_m = math.sqrt(d2) * res
            ny = yy + dy
            nx = xx + dx
            valid = (ny >= 0) & (ny < h) & (nx >= 0) & (nx < w)
            ny, nx = ny[valid], nx[valid]
            cur = dist[ny, nx]
            new = np.minimum(cur, d_m)
            dist[ny, nx] = new

    # Lethal: distance to obstacle ≤ inscribed_m (robot center inside
    # body of an obstacle).
    passable = dist > inscribed_m

    # Cost: exp-decay from 1 at inscribed boundary to ~0 over the
    # inflation tail. nav2 uses cost = 253 * exp(-k·(d - inscribed))
    # with k tuned by `cost_scaling_factor`. We use the same shape but
    # in [0,1]. k=2.5 makes cost halve every ~28cm at default settings.
    k = 2.5
    cost = np.zeros((h, w), dtype=np.float32)
    in_band = passable & (dist <= inscribed_m + inflation_m)
    cost[in_band] = np.exp(-k * (dist[in_band] - inscribed_m))
    # Cap to [0, 1] (numerical safety; expression already ≤ 1 for d≥inscribed).
    np.clip(cost, 0.0, 1.0, out=cost)
    return passable, cost


def inflate_obstacles(gm: GridMap, radius_m: float, occ_thresh: int = 50,
                       allow_unknown: bool = True) -> np.ndarray:
    """Legacy binary-inflation API — kept for back-compat with anything
    that just wants a passable mask. New code should call
    ``build_costmap`` and use both layers."""
    passable, _ = build_costmap(
        gm, inscribed_m=radius_m, inflation_m=0.0,
        occ_thresh=occ_thresh, allow_unknown=allow_unknown,
    )
    return passable


# 8-connectivity neighbours: (dx, dy, cost-multiplier)
_NEIGHBOURS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
]


def astar(passable: np.ndarray, start: Tuple[int, int],
           goal: Tuple[int, int], *,
           cost: Optional[np.ndarray] = None,
           cost_weight: float = 4.0,
           max_iter: int = 100_000) -> Optional[List[Tuple[int, int]]]:
    """A* on a passable grid with optional gradient cost layer.

    When ``cost`` is given (matching ``passable.shape``, values in
    [0,1]), the move cost from a→b is ``base + cost_weight * cost[b]``.
    A* will then prefer paths through low-cost cells (corridor centers)
    over wall-hugging ones. ``cost_weight=4.0`` is in the same scale as
    the unit move cost (1.0), so a fully-saturated cost cell costs 5x
    a free cell to traverse — strong enough to bias the path away from
    walls without making the robot detour wildly.

    Returns the cell path (list of (cx, cy)) including endpoints, or
    None when no path exists / search exhausted.
    """
    h, w = passable.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return None
    if not passable[gy, gx]:
        # Goal occluded — try widening: pick the nearest passable cell.
        # Prevents tiny localisation errors from making valid goals
        # unreachable when the goal point is just inside an inflation
        # bubble.
        best, best_d = None, float("inf")
        rr = 6  # 6 cells ≈ 30cm at 5cm resolution
        for dy in range(-rr, rr + 1):
            for dx in range(-rr, rr + 1):
                ny, nx = gy + dy, gx + dx
                if 0 <= ny < h and 0 <= nx < w and passable[ny, nx]:
                    d = dx * dx + dy * dy
                    if d < best_d:
                        best, best_d = (nx, ny), d
        if best is None:
            return None
        gx, gy = best
    if not passable[sy, sx]:
        return None

    def heuristic(cx: int, cy: int) -> float:
        return math.hypot(cx - gx, cy - gy)

    open_heap: list = []
    heapq.heappush(open_heap, (heuristic(sx, sy), 0.0, (sx, sy)))
    came_from: dict = {}
    g_cost: dict = {(sx, sy): 0.0}
    seen = 0

    while open_heap:
        seen += 1
        if seen > max_iter:
            return None
        _, g, cur = heapq.heappop(open_heap)
        if cur == (gx, gy):
            # Reconstruct.
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path
        cx, cy = cur
        for dx, dy, mult in _NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            if not passable[ny, nx]:
                continue
            step = mult
            if cost is not None:
                step += cost_weight * float(cost[ny, nx]) * mult
            ng = g + step
            nk = (nx, ny)
            if ng < g_cost.get(nk, float("inf")):
                g_cost[nk] = ng
                came_from[nk] = cur
                f = ng + heuristic(nx, ny)
                heapq.heappush(open_heap, (f, ng, nk))
    return None


def smooth_path(cells: List[Tuple[int, int]], passable: np.ndarray,
                 max_skip: int = 12) -> List[Tuple[int, int]]:
    """Line-of-sight shortcutting: drop intermediate waypoints when a
    direct line through `passable` is collision-free. Reduces the
    A*-grid-staircase artefact and gives the follower a smoother
    path. `max_skip` caps the look-ahead so we don't try expensive
    LoS checks across the whole map."""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j_max = min(len(cells) - 1, i + max_skip)
        # Furthest j with direct LoS from cells[i].
        chosen = i + 1
        for j in range(j_max, i, -1):
            if _line_clear(cells[i], cells[j], passable):
                chosen = j
                break
        out.append(cells[chosen])
        i = chosen
    return out


def _line_clear(a: Tuple[int, int], b: Tuple[int, int],
                  passable: np.ndarray) -> bool:
    """Bresenham-ish line between two grid cells; True iff every cell
    on the line is passable."""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0); dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        if not passable[y0, x0]:
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x0 += sx
        if e2 <= dx:
            err += dx; y0 += sy


@dataclass
class PlanResult:
    """Output of ``plan_world``: world-coord path + the costmap layers
    used to produce it. The follower keeps a reference to the cost
    layer so it can sample its current pose for "am I close to a
    wall?" decisions instead of duplicating obstacle awareness."""
    path: List[Tuple[float, float]]
    gm: GridMap
    passable: np.ndarray
    cost: np.ndarray


def plan_world(gm: GridMap, start_xy: Tuple[float, float],
                goal_xy: Tuple[float, float], *,
                inscribed_m: float = 0.25,
                inflation_m: float = 0.20,
                cost_weight: float = 4.0) -> PlanResult:
    """Top-level: world (x,y) start + goal → ``PlanResult`` with the
    waypoint list and the costmap layers.

    Defaults sized for Tiago: 25cm inscribed (its actual footprint
    half-width) + 20cm inflation tail (decay zone). Total clearance
    from obstacle to robot center = 25cm hard, 45cm preferred.

    Fallback: if the robot or the goal sits OUTSIDE the current map
    grid, or A* cannot find a path, return a 3-point straight-line
    plan so the RPP follower at least drives in the right general
    direction. The follower's costmap-aware regulator + lidar still
    enforce safety in that degenerate case.
    """
    passable, cost = build_costmap(gm,
                                   inscribed_m=inscribed_m,
                                   inflation_m=inflation_m)
    s = gm.world_to_cell(*start_xy)
    g = gm.world_to_cell(*goal_xy)

    def _straight() -> List[Tuple[float, float]]:
        sx, sy = start_xy
        gx_, gy_ = goal_xy
        return [(sx, sy), ((sx + gx_) * 0.5, (sy + gy_) * 0.5), (gx_, gy_)]

    def _result(path: List[Tuple[float, float]]) -> PlanResult:
        return PlanResult(path=path, gm=gm, passable=passable, cost=cost)

    if not gm.in_bounds(*s) or not gm.in_bounds(*g):
        return _result(_straight())
    if not passable[s[1], s[0]]:
        # Robot center already inside the lethal halo (typical right
        # after a collision shoves us against a wall). Return a
        # fallback plan; follower will see high cost at current pose
        # and back off before pushing further.
        return _result(_straight())
    cells = astar(passable, s, g, cost=cost, cost_weight=cost_weight)
    if cells is None:
        return _result(_straight())
    cells = smooth_path(cells, passable)
    return _result([gm.cell_to_world(cx, cy) for cx, cy in cells])
