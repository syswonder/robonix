# SPDX-License-Identifier: MulanPSL-2.0
"""Room-region suggestion from an occupancy grid (hover assist for /user).

Given a SLAM occupancy grid, a seed cell (mouse hover) and a tolerance level
(scroll wheel), suggest the room region the user most likely wants to select.
The map is NOT assumed to be well-formed: broken walls, missing patches and
speckle noise are expected. The pipeline therefore anchors on the robust
feature "a room is a wide cavity in free space, its openings (doors — or wall
gaps left by imperfect SLAM) are narrow necks":

1. binarize (unknown counts as boundary) + despeckle,
2. distance transform (DT) of free space,
3. hill-climb from the seed to the cavity core (a DT local maximum),
4. sweep a DT threshold downward; a single-step surge of the seed component's
   area growth (delta ratio > `r_jump`, absolute delta > `a_room_min`, neck
   half-width <= `t_neck_max`) means the component leaked through a neck —
   retreat one notch; the wheel `level` continues past `level` leaks,
5. expand the thresholded core wall-ward: absorb adjacent steepest-ascent DT
   basins whose peak is below the stop threshold (sub-room pockets), never
   rival cavities (other rooms) — the region hugs real walls where they
   exist and stops on the DT ridge across wall gaps,
6. polygonize (outer contour trace + Ramer-Douglas-Peucker).

Pure functions over numpy arrays; `scipy.ndimage` only. Grids use the raw
`nav_msgs/OccupancyGrid` conventions: int8 values (-1 unknown / 0 free /
100 occupied), row-major `[row, col]` with row 0 at world `origin` and +row
= +y (no image-style vertical flip anywhere in this module). DT thresholds
and areas are metric; `k_speckle` and the RDP epsilon are in cells. Defaults
come from the Phase 0 parameter study.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

log = logging.getLogger(__name__)

Cell = tuple[int, int]  # (row, col)

_N8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
_BOX3 = np.ones((3, 3), dtype=bool)


@dataclass(frozen=True)
class SuggestParams:
    """Tunables for the suggestion pipeline (Phase 0 validated defaults)."""

    free_max: int = 0        # grid values in [0, free_max] are free
    k_speckle: int = 3       # cells; despeckle component-size threshold
    t_min: float = 0.3       # m; downsweep lower bound (thinner => not a room)
    t_step: float = 0.05     # m; downsweep notch (wheel level<0 shrinks by
    #                           notches; level>0 counts leaks, see downsweep)
    r_jump: float = 3.0      # area-delta ratio surge that flags a neck leak
    t_neck_max: float = 0.65  # m; leaks only detected at neck half-width <=
    #                           this (door-scale prior: wider saddles are
    #                           internal to one room, not openings)
    a_room_min: float = 1.0  # m^2; minimum room area (leak co-criterion +
    #                           hover suppression)
    rdp_eps_cells: float = 2.0  # cells; polygon simplification tolerance


@dataclass(frozen=True)
class SuggestResult:
    """Outcome of one suggestion query.

    `polygon_cells` vertices are float `(row, col)` grid coordinates;
    `mask` is the suggested region as a READ-ONLY bool grid (None when not
    ok). `resolution`/`origin_xy` are stamped from the grid the result was
    computed on, so `to_json()` can never pair a stale result with a newer
    map's frame.
    """

    ok: bool
    reason: str = ""
    level: int = 0
    leaked: bool = False
    t_star: float = 0.0
    area_m2: float = 0.0
    resolution: float = 0.0
    origin_xy: tuple[float, float] = (0.0, 0.0)
    polygon_cells: tuple[tuple[float, float], ...] = ()
    mask: np.ndarray | None = field(default=None, repr=False, compare=False)

    def to_json(self) -> dict:
        """Serialize for the suggest endpoint: polygon in map-frame meters."""
        return {
            "ok": self.ok,
            "reason": self.reason,
            "level": self.level,
            "leaked": self.leaked,
            "area_m2": round(self.area_m2, 3),
            "polygon": [
                list(cell_to_world(r, c, self.resolution, self.origin_xy))
                for r, c in self.polygon_cells
            ],
        }


# --------------------------------------------------------------- grid <-> world

def cell_to_world(row: float, col: float, resolution: float,
                  origin_xy: tuple[float, float]) -> tuple[float, float]:
    """Center of grid cell -> map-frame meters (row 0 sits AT origin, +row = +y)."""
    return (
        origin_xy[0] + (col + 0.5) * resolution,
        origin_xy[1] + (row + 0.5) * resolution,
    )


def world_to_cell(x: float, y: float, resolution: float,
                  origin_xy: tuple[float, float]) -> Cell:
    """Map-frame meters -> containing grid cell (no bounds check)."""
    return (
        int(np.floor((y - origin_xy[1]) / resolution)),
        int(np.floor((x - origin_xy[0]) / resolution)),
    )


# ------------------------------------------------------------------ preprocess

def binarize_occupancy(grid: np.ndarray, free_max: int = 0) -> np.ndarray:
    """OccupancyGrid int8 values -> free mask.

    Values in `[0, free_max]` are free; everything else — occupied AND
    unknown (-1) — is boundary. Unmapped space must block suggestions:
    annotations should never cover cells no sensor has seen (beyond speckle
    scale — sub-`k_speckle` unknown specks are healed by `despeckle`).
    """
    return (grid >= 0) & (grid <= free_max)


def despeckle(free: np.ndarray, k_cells: int) -> np.ndarray:
    """Remove sensor speckle smaller than `k_cells` from a free mask.

    Non-free components below `k_cells` are isolated islands inside free
    space by construction (8-connected labelling merges touching obstacle
    and unknown cells, so wall fragments anchored in unknown territory sit
    in a component far larger than any speckle) — heal them to free. Free
    components below `k_cells` (pinholes in walls, stray rays) are flipped
    to boundary.
    """
    out = free.copy()
    lab, n = ndimage.label(~free, structure=_BOX3)
    if n:
        sizes = ndimage.sum_labels(np.ones(lab.shape), lab, np.arange(1, n + 1))
        small = np.nonzero(sizes < k_cells)[0] + 1
        if small.size:
            out[np.isin(lab, small)] = True
    lab, n = ndimage.label(out, structure=_BOX3)
    if n:
        sizes = ndimage.sum_labels(np.ones(lab.shape), lab, np.arange(1, n + 1))
        small = np.nonzero(sizes < k_cells)[0] + 1
        if small.size:
            out[np.isin(lab, small)] = False
    return out


def preprocess(grid: np.ndarray, resolution: float,
               params: SuggestParams) -> tuple[np.ndarray, np.ndarray]:
    """Occupancy grid -> (despeckled free mask, metric distance transform)."""
    free = despeckle(binarize_occupancy(grid, params.free_max), params.k_speckle)
    dt = ndimage.distance_transform_edt(free) * resolution
    return free, dt


# ------------------------------------------------------------ cavity anchoring

def anchor(dt: np.ndarray, seed: Cell) -> Cell:
    """Hill-climb from the seed to a DT local maximum (the cavity core).

    Hovering next to a wall must still anchor the room's wide interior, not
    the thin band under the cursor. Path length is bounded by the grid size.
    """
    h, w = dt.shape
    cur = seed
    for _ in range(h * w):
        best, best_v = cur, dt[cur]
        for dy, dx in _N8:
            y, x = cur[0] + dy, cur[1] + dx
            if 0 <= y < h and 0 <= x < w and dt[y, x] > best_v:
                best, best_v = (y, x), dt[y, x]
        if best == cur:
            return cur
        cur = best
    return cur


# ------------------------------------------------------- threshold downsweep

def downsweep(dt: np.ndarray, core: Cell, resolution: float,
              params: SuggestParams, level: int = 0,
              ) -> tuple[float, np.ndarray, bool]:
    """Find the stop threshold t* and the core's thresholded component.

    Sweep t from DT(core) down to `t_min`; at each notch take the connected
    component of `{DT >= t}` containing the core. A single-notch surge of
    the area GROWTH (delta > previous delta * `r_jump`, delta >
    `a_room_min`, and t <= `t_neck_max`) means the component leaked through
    a neck — a door or a wall gap, indistinguishable and treated alike —
    so retreat one notch. Without a surge the sweep runs to the bottom
    (open space bounded by real walls). Positive `level` (wheel) continues
    through that many leaks (merging across necks); negative `level` raises
    the stop threshold notch-wise (shrinking). Returns (t*, component mask,
    leaked); a core thinner than `t_min` yields an empty mask — thresholding
    it would select the background component, i.e. the entire rest of the
    map.
    """
    if float(dt[core]) < params.t_min:
        return float(params.t_min), np.zeros_like(dt, dtype=bool), False
    ts = np.arange(dt[core], params.t_min - 1e-9, -params.t_step)
    if ts.size == 0:
        ts = np.array([params.t_min])
    prev_area = prev_t = prev_delta = None
    leaked, owed, stop_t = False, max(level, 0), None
    for t in ts:
        lab, _ = ndimage.label(dt >= t, structure=_BOX3)
        area = float((lab == lab[core]).sum()) * resolution * resolution
        if prev_area is not None:
            delta = area - prev_area
            surge = (prev_delta is not None and prev_delta > 0
                     and delta > prev_delta * params.r_jump)
            if surge and delta > params.a_room_min and t <= params.t_neck_max:
                if owed == 0:
                    stop_t, leaked = prev_t, True
                    break
                owed -= 1
                leaked = True
            prev_delta = delta
        prev_area, prev_t = area, t
    if stop_t is None:
        stop_t = prev_t if prev_t is not None else float(params.t_min)
    if level < 0:
        stop_t = min(stop_t + (-level) * params.t_step, float(dt[core]))
    lab, _ = ndimage.label(dt >= stop_t, structure=_BOX3)
    return float(stop_t), lab == lab[core], leaked


# ------------------------------------------------------- watershed expansion

def ascent_basins(dt: np.ndarray, free: np.ndarray,
                  ) -> tuple[np.ndarray, np.ndarray]:
    """Label every free cell with its steepest-ascent DT basin.

    Cells are flooded in descending-DT order: each takes the label of its
    highest already-labeled 8-neighbour (necessarily at least as high, given
    the ordering), else it founds a new basin (a regional maximum — or an
    equal-DT plateau segment claimed first in processing order). Returns
    `(labels, peak_dt)`: `labels` is int32 with 0 = non-free and basins
    numbered from 1; `peak_dt[k]` is the DT of basin k's founding maximum
    (`peak_dt[0]` = 0).

    Deliberately the plain ordered flood, NOT a vectorized watershed:
    IFT/pointer-forest variants resolve equal-DT plateau ties differently,
    which shifts basin boundaries and measurably changes `expand()` output
    versus the Phase 0-validated behaviour. The per-cell loop costs
    ~0.1-0.3s on deployed map sizes and runs once per grid (memoized by
    `RoomSuggester`), which the latency budget absorbs.
    """
    h, w = dt.shape
    labels = np.zeros((h, w), dtype=np.int32)
    order = np.argsort(-dt[free], kind="stable")
    ys, xs = np.nonzero(free)
    ys, xs = ys[order], xs[order]
    peaks = [0.0]
    nxt = 0
    for y, x in zip(ys.tolist(), xs.tolist()):
        best_lab, best_v = 0, -1.0
        for dy, dx in _N8:
            ny, nx_ = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx_ < w and labels[ny, nx_] and dt[ny, nx_] > best_v:
                best_lab, best_v = labels[ny, nx_], dt[ny, nx_]
        if best_lab and best_v >= dt[y, x]:
            labels[y, x] = best_lab
        else:
            nxt += 1
            labels[y, x] = nxt
            peaks.append(float(dt[y, x]))
    return labels, np.array(peaks)


def expand(free: np.ndarray, region0: np.ndarray, t_star: float,
           basins: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """Grow the thresholded core out to the walls without crossing necks.

    Take every basin the core touches, then iteratively absorb adjacent
    basins whose peak DT < `t_star` — sub-room-scale pockets (corners
    behind furniture, wall nooks). Basins peaking at or above `t_star` are
    rival cavities (other rooms); they are never absorbed, so the region
    stops on the DT ridge toward them — the virtual boundary across a
    wall gap.
    """
    labels, peak_dt = basins
    mine = np.zeros(len(peak_dt), dtype=bool)
    mine[np.unique(labels[region0])] = True
    mine[0] = False
    while True:
        cur = mine[labels]
        border = ndimage.binary_dilation(cur, structure=_BOX3) & free & ~cur
        cand = np.unique(labels[border])
        grow = cand[(cand > 0) & ~mine[cand] & (peak_dt[cand] < t_star)]
        if grow.size == 0:
            return cur
        mine[grow] = True


# --------------------------------------------------------------- polygonization

def trace_outer_contour(mask: np.ndarray) -> np.ndarray:
    """Moore-neighbour trace of the hole-filled mask's outer contour.

    Returns an (N, 2) float array of (row, col) cell coordinates, implicitly
    closed. Termination follows Jacob's criterion — stop only on revisiting
    the start cell in the same tracing state, never on merely reaching it:
    an 8-connected region pinched at the start cell passes through it once
    per lobe, and stopping early would drop whole lobes from the polygon.
    A pinch cell therefore appears more than once (a degenerate vertex the
    even-odd fill handles). Single-cell masks yield that one cell; empty
    masks an empty array.
    """
    m = ndimage.binary_fill_holes(mask)
    ys, xs = np.nonzero(m)
    if ys.size == 0:
        return np.empty((0, 2), dtype=float)
    start = (int(ys[0]), int(xs[0]))  # topmost row, then leftmost col
    dirs = ((-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1))
    h, w = m.shape
    contour = [start]
    prev_dir = 6  # backtrack points west of a topmost-leftmost start
    cur = start
    seen = {(start, prev_dir)}
    for _ in range(8 * int(m.sum()) + 8):
        for k in range(8):
            i = (prev_dir + 1 + k) % 8  # clockwise from the backtrack direction
            y, x = cur[0] + dirs[i][0], cur[1] + dirs[i][1]
            if 0 <= y < h and 0 <= x < w and m[y, x]:
                contour.append((y, x))
                prev_dir = (i + 4) % 8
                cur = (y, x)
                break
        else:
            break  # isolated cell: no neighbours
        state = (cur, prev_dir)
        if state in seen:
            break
        seen.add(state)
    if len(contour) > 1 and contour[-1] == contour[0]:
        contour = contour[:-1]
    return np.array(contour, dtype=float)


def rdp(points: np.ndarray, eps: float) -> np.ndarray:
    """Ramer-Douglas-Peucker polyline simplification (iterative, closed input).

    Keeps every vertex whose perpendicular distance from the simplified
    chord exceeds `eps` cells, so the polygon never strays more than `eps`
    from the traced contour.
    """
    if len(points) < 3:
        return points
    keep = np.zeros(len(points), dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        seg = points[b] - points[a]
        norm = float(np.hypot(*seg))
        rel = points[a + 1:b] - points[a]
        if norm < 1e-12:
            d = np.hypot(rel[:, 0], rel[:, 1])
        else:
            d = np.abs(seg[0] * rel[:, 1] - seg[1] * rel[:, 0]) / norm
        i = int(np.argmax(d))
        if d[i] > eps:
            keep[a + 1 + i] = True
            stack += [(a, a + 1 + i), (a + 1 + i, b)]
    return points[keep]


def rasterize_polygon(polygon: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Even-odd scanline fill of a (row, col) polygon into a bool mask.

    Inverse of polygonization for coverage checks (tests, IoU, previews);
    parts outside `shape` paint nothing (clipped, never wrapped). Edge cells
    are painted explicitly — the half-open scanline rule alone would drop
    cells whose centers lie ON the boundary (e.g. a rectangle's bottom row).
    """
    mask = np.zeros(shape, dtype=bool)
    if len(polygon) < 3:
        return mask
    lo = max(int(np.floor(polygon[:, 0].min())), 0)
    hi = min(int(np.ceil(polygon[:, 0].max())), shape[0] - 1)
    n = len(polygon)
    for row in range(lo, hi + 1):
        y = float(row)
        xs: list[float] = []
        for i in range(n):
            y1, x1 = polygon[i]
            y2, x2 = polygon[(i + 1) % n]
            if (y1 <= y < y2) or (y2 <= y < y1):
                xs.append(x1 + (y - y1) / (y2 - y1) * (x2 - x1))
        xs.sort()
        for a, b in zip(xs[::2], xs[1::2]):
            left = max(int(np.ceil(a)), 0)
            right = min(int(np.floor(b)), shape[1] - 1)
            if left <= right:
                mask[row, left:right + 1] = True
    for i in range(n):
        y1, x1 = polygon[i]
        y2, x2 = polygon[(i + 1) % n]
        steps = int(2 * max(abs(y2 - y1), abs(x2 - x1))) + 2
        ys = np.rint(np.linspace(y1, y2, steps)).astype(int)
        cs = np.rint(np.linspace(x1, x2, steps)).astype(int)
        ok = (ys >= 0) & (ys < shape[0]) & (cs >= 0) & (cs < shape[1])
        mask[ys[ok], cs[ok]] = True
    return mask


# ----------------------------------------------------------------- the facade

def suggest(grid: np.ndarray, resolution: float, seed: Cell, level: int = 0,
            params: SuggestParams | None = None,
            origin_xy: tuple[float, float] = (0.0, 0.0),
            pre: tuple[np.ndarray, np.ndarray] | None = None,
            basins: tuple[np.ndarray, np.ndarray] | None = None,
            ) -> SuggestResult:
    """Run the full pipeline: occupancy grid + seed cell (+ wheel level).

    `pre` / `basins` accept cached `preprocess()` / `ascent_basins()`
    results (both depend only on grid, resolution and params, not on
    seed/level) — `RoomSuggester` supplies them for repeated hovers.
    Out-of-bounds, non-free or too-thin seeds and sub-`a_room_min` regions
    return `ok=False` (the hover-suppression / fall-back-to-manual case).
    The returned mask is marked read-only: it aliases pipeline state that
    later queries on the same cached stages must not see mutated.
    """
    p = params or SuggestParams()

    def fail(reason: str, **kw) -> SuggestResult:
        return SuggestResult(False, reason, level=level, resolution=resolution,
                             origin_xy=origin_xy, **kw)

    free, dt = pre if pre is not None else preprocess(grid, resolution, p)
    h, w = free.shape
    if not (0 <= seed[0] < h and 0 <= seed[1] < w):
        return fail("seed outside the grid")
    if not free[seed]:
        return fail("seed not in free space")
    core = anchor(dt, seed)
    if float(dt[core]) < p.t_min:
        return fail("cavity below t_min", t_star=float(p.t_min))
    t_star, region0, leaked = downsweep(dt, core, resolution, p, level)
    region = expand(free, region0, t_star,
                    basins if basins is not None else ascent_basins(dt, free))
    area = float(region.sum()) * resolution * resolution
    if area < p.a_room_min:
        return fail("region below a_room_min", leaked=leaked, t_star=t_star,
                    area_m2=area)
    polygon = rdp(trace_outer_contour(region), p.rdp_eps_cells)
    region.setflags(write=False)
    return SuggestResult(True, level=level, leaked=leaked, t_star=t_star,
                         area_m2=area, resolution=resolution,
                         origin_xy=origin_xy,
                         polygon_cells=tuple(map(tuple, polygon.tolist())),
                         mask=region)


class RoomSuggester:
    """Per-grid facade that memoizes the seed-independent stages.

    Holds one occupancy grid (plus its resolution/origin) and computes the
    free mask, distance transform and basin labelling lazily, once; repeated
    hover queries then cost only the seed-dependent stages (~ms). The web
    endpoint keeps one instance per occupancy stamp and rebuilds on a fresh
    grid.
    """

    def __init__(self, grid: np.ndarray, resolution: float,
                 origin_xy: tuple[float, float] = (0.0, 0.0),
                 params: SuggestParams | None = None):
        """Bind one grid; heavy stages build lazily. The grid is copied so
        a caller reusing its buffer cannot silently stale the memoized
        stages."""
        if resolution <= 0:
            raise ValueError(f"resolution must be positive, got {resolution}")
        self._grid = np.array(grid, copy=True)
        self._resolution = float(resolution)
        self._origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self._params = params or SuggestParams()
        self._pre: tuple[np.ndarray, np.ndarray] | None = None
        self._basins: tuple[np.ndarray, np.ndarray] | None = None

    def _ensure(self) -> None:
        """Build the memoized preprocess + basin stages on first use."""
        if self._pre is None:
            self._pre = preprocess(self._grid, self._resolution, self._params)
            self._basins = ascent_basins(self._pre[1], self._pre[0])

    def suggest(self, seed: Cell, level: int = 0) -> SuggestResult:
        """Suggestion for a grid-cell seed, using the memoized stages."""
        self._ensure()
        return suggest(self._grid, self._resolution, seed, level=level,
                       params=self._params, origin_xy=self._origin_xy,
                       pre=self._pre, basins=self._basins)

    def suggest_world(self, x: float, y: float, level: int = 0) -> SuggestResult:
        """Suggestion for a map-frame (x, y) meters seed (endpoint entry)."""
        seed = world_to_cell(x, y, self._resolution, self._origin_xy)
        return self.suggest(seed, level=level)
