# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for room suggestion (scene_service.room_suggest).

Pure numpy/scipy over synthetic OccupancyGrid-semantics arrays (-1 unknown /
0 free / 100 occupied) — no ROS, no milvus. Skips loudly when scipy is
unavailable (e.g. a bare mac checkout).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:  # probe ONLY the third-party deps — absent on a bare mac checkout.
    import numpy as np
    from scipy import ndimage

    _IMPORT_ERR = None
except ModuleNotFoundError as e:  # pragma: no cover - environment-dependent
    _IMPORT_ERR = e

if _IMPORT_ERR is None:
    # Deliberately OUTSIDE the try: an ImportError raised by the module
    # itself (typo'd symbol, broken internal import) must fail the run
    # loudly, not masquerade as "scipy unavailable" and skip forever.
    from scene_service.room_suggest import (  # noqa: E402
        RoomSuggester,
        SuggestParams,
        ascent_basins,
        binarize_occupancy,
        cell_to_world,
        despeckle,
        downsweep,
        expand,
        preprocess,
        rasterize_polygon,
        rdp,
        suggest,
        trace_outer_contour,
        world_to_cell,
    )

RES = 0.05  # m/cell, matches the deployed maps

# Small rooms need gentler minimums than the production defaults.
P = None if _IMPORT_ERR else SuggestParams(a_room_min=0.5, t_min=0.15)


def _unavailable() -> bool:
    """Skip (pytest) or announce-and-return (main runner) when deps miss."""
    if _IMPORT_ERR is None:
        return False
    if os.environ.get("PYTEST_CURRENT_TEST"):
        import pytest

        pytest.skip(f"scipy unavailable: {_IMPORT_ERR}")
    print(f"[SKIP] scipy unavailable: {_IMPORT_ERR}")
    return True


# ------------------------------------------------------------------ fixtures

def two_rooms(door_cols=(28, 31), gap_cols=(), unknown_outside=False):
    """Two rooms (~2.9x1.95m and ~2.9x0.9m) split by a wall, joined by a door.

    60x60 grid: room A rows 1..39, room B rows 41..58, wall row 40 with a
    free door at `door_cols` (and optional extra `gap_cols` — a wall defect).
    `unknown_outside` replaces the outer border wall with unknown space.
    """
    g = np.full((60, 60), 100, dtype=np.int8)
    g[1:40, 1:59] = 0
    g[41:59, 1:59] = 0
    if door_cols:
        g[40, door_cols[0]:door_cols[1] + 1] = 0
    for a, b in ([gap_cols] if gap_cols else []):
        g[40, a:b + 1] = 0
    if unknown_outside:
        g[0, :] = -1
        g[-1, :] = -1
        g[:, 0] = -1
        g[:, -1] = -1
    return g


def _run(g, seed, level=0, params=None):
    return suggest(g, RES, seed, level=level, params=params or P)


# ---------------------------------------------------------------- binarize

def test_binarize_occupancy_semantics():
    """0 is free; unknown (-1) and occupied (100) are both boundary."""
    if _unavailable():
        return
    g = np.array([[-1, 0], [100, 42]], dtype=np.int8)
    assert binarize_occupancy(g).tolist() == [[False, True], [False, False]]
    assert binarize_occupancy(g, free_max=50).tolist() == [[False, True], [False, True]]
    print("[PASS] test_binarize_occupancy_semantics")


def test_despeckle_keeps_wall_fragments_touching_unknown():
    """Free/obstacle specks vanish; a wall stub anchored in unknown survives."""
    if _unavailable():
        return
    g = np.zeros((20, 20), dtype=np.int8)
    g[10, 10] = 100          # 1-cell obstacle speck inside free space
    g[0:3, 5] = -1           # unknown finger
    g[3, 5] = 100            # wall fragment touching it
    g2 = g.copy()
    g2[14:17, 14:17] = 100
    g2[15, 15] = 0           # 1-cell free speck inside an obstacle block
    out = despeckle(binarize_occupancy(g2), k_cells=3)
    assert out[10, 10]            # obstacle speck healed to free
    assert not out[3, 5]          # wall fragment kept (merges with unknown)
    assert not out[15, 15]        # free speck removed
    # size-threshold boundary is strict: == k_cells survives, < k_cells goes
    g3 = np.zeros((20, 20), dtype=np.int8)
    g3[5, 3:6] = 100              # 3-cell obstacle run
    g3[12, 3:5] = 100             # 2-cell obstacle run
    out3 = despeckle(binarize_occupancy(g3), k_cells=3)
    assert not out3[5, 4], "3-cell component (== k_cells) was wrongly healed"
    assert out3[12, 3], "2-cell component (< k_cells) survived"
    print("[PASS] test_despeckle_keeps_wall_fragments_touching_unknown")


# ------------------------------------------------------------- leak detection

def test_hover_stays_inside_own_room():
    """Hovering each room suggests that room only — not through the door."""
    if _unavailable():
        return
    g = two_rooms()
    a = _run(g, (20, 30))
    b = _run(g, (50, 30))
    assert a.ok and b.ok
    assert a.mask[20, 30] and not a.mask[50, 30]
    assert b.mask[50, 30] and not b.mask[20, 30]
    assert not (a.mask & b.mask).any()
    print("[PASS] test_hover_stays_inside_own_room")


def test_wall_gap_treated_like_a_door():
    """A 0.5m wall defect is a neck too: the suggestion still stops at it."""
    if _unavailable():
        return
    g = two_rooms(gap_cols=(8, 17))  # extra 0.5m hole in the dividing wall
    a = _run(g, (20, 30))
    assert a.ok
    assert not a.mask[50, 30], "suggestion leaked through the wall gap"
    print("[PASS] test_wall_gap_treated_like_a_door")


def test_wide_internal_saddle_does_not_false_leak():
    """A furniture-pinched saddle wider than t_neck_max stays one room."""
    if _unavailable():
        return
    g = np.full((60, 90), 100, dtype=np.int8)
    g[1:59, 1:89] = 0
    # two blocks pinching the middle to a 1.5m-wide (30-cell) passage:
    # half-width 0.75m > t_neck_max 0.65m -> internal saddle, not a neck
    g[1:15, 40:50] = 100
    g[45:59, 40:50] = 100
    r = _run(g, (30, 20))
    assert r.ok
    assert r.mask[30, 70], "wide saddle wrongly split the room"
    print("[PASS] test_wide_internal_saddle_does_not_false_leak")


# -------------------------------------------------------- unknown as boundary

def test_unknown_blocks_expansion():
    """The suggestion never covers unknown cells."""
    if _unavailable():
        return
    g = two_rooms()
    g[10:20, 20:30] = -1  # unknown hole inside room A
    r = _run(g, (30, 40))
    assert r.ok
    assert not (r.mask & (g == -1)).any()
    print("[PASS] test_unknown_blocks_expansion")


def test_exterior_gap_to_unknown_does_not_spill():
    """A hole in the outer wall opens to unknown; the region stays inside."""
    if _unavailable():
        return
    g = two_rooms(unknown_outside=True)
    g[30:36, 59] = 0  # breach the right outer wall of room A
    g[30:36, 58] = 0
    r = _run(g, (20, 30))
    assert r.ok
    assert not (r.mask & (g == -1)).any()
    print("[PASS] test_exterior_gap_to_unknown_does_not_spill")


# ------------------------------------------------------------ expansion rules

def test_expand_absorbs_pockets_but_not_rival_rooms():
    """Enclosed pockets (peak < t*) merge in; the neighbour room never does."""
    if _unavailable():
        return
    g = two_rooms()
    # a genuinely enclosed nook: obstacle block with a cavity inside,
    # connected to room A through a 2-cell mouth. Interior DT peak 0.1m,
    # well below the 0.15m stop threshold -> must be absorbed by expansion.
    g[1:10, 1:13] = 100
    g[3:7, 3:11] = 0     # the nook cavity
    g[4:6, 11:13] = 0    # its mouth into room A
    free, dt = preprocess(g, RES, P)
    r = _run(g, (20, 30))
    assert r.ok
    assert r.mask[5, 5], "enclosed pocket was not absorbed"
    assert not r.mask[50, 30], "rival cavity absorbed"
    # never a cell outside free space
    assert not (r.mask & ~free).any()
    print("[PASS] test_expand_absorbs_pockets_but_not_rival_rooms")


# -------------------------------------------------------------- polygonization

def test_polygon_hugs_the_region():
    """RDP output stays within eps of the region (both directions)."""
    if _unavailable():
        return
    g = two_rooms()
    r = _run(g, (20, 30))
    assert r.ok and len(r.polygon_cells) >= 4
    poly_mask = rasterize_polygon(np.array(r.polygon_cells), g.shape)
    eps = int(np.ceil(P.rdp_eps_cells)) + 1
    grown_region = ndimage.binary_dilation(r.mask, iterations=eps)
    grown_poly = ndimage.binary_dilation(poly_mask, iterations=eps)
    assert (poly_mask & ~grown_region).sum() == 0, "polygon strays off the region"
    assert (r.mask & ~grown_poly).sum() == 0, "polygon misses part of the region"
    print("[PASS] test_polygon_hugs_the_region")


def test_rdp_simplifies_straight_runs():
    """Collinear contour runs collapse to their endpoints."""
    if _unavailable():
        return
    pts = np.array([[0.0, float(i)] for i in range(10)] + [[5.0, 9.0]])
    out = rdp(pts, eps=1.0)
    assert len(out) == 3
    print("[PASS] test_rdp_simplifies_straight_runs")


def test_contour_roundtrip_small_rectangle():
    """trace + rasterize reproduces a filled rectangle exactly."""
    if _unavailable():
        return
    m = np.zeros((12, 12), dtype=bool)
    m[3:9, 2:10] = True
    poly = trace_outer_contour(m)
    back = rasterize_polygon(poly, m.shape)
    assert (back == m).all()
    print("[PASS] test_contour_roundtrip_small_rectangle")


def test_contour_covers_diagonal_pinch_lobes():
    """A region pinched diagonally AT the start cell keeps all its lobes:
    the trace must pass through the pinch once per lobe (Jacob's stopping
    criterion), not stop on first return to the start."""
    if _unavailable():
        return
    m = np.zeros((8, 8), dtype=bool)
    m[0, 1:4] = True          # top lobe; (0,1) is the trace start
    m[1:4, 0] = True          # bottom lobe, joined only diagonally at (0,1)
    poly = trace_outer_contour(m)
    visited = {(int(r), int(c)) for r, c in poly}
    missing = {tuple(c) for c in np.argwhere(m)} - visited
    assert not missing, f"contour dropped lobe cells: {missing}"
    # degenerate/empty masks stay well-defined
    assert trace_outer_contour(np.zeros((4, 4), bool)).shape == (0, 2)
    one = np.zeros((4, 4), bool)
    one[2, 2] = True
    assert trace_outer_contour(one).tolist() == [[2.0, 2.0]]
    print("[PASS] test_contour_covers_diagonal_pinch_lobes")


def test_rasterize_clips_out_of_grid_polygons():
    """Polygon parts left of / above the grid paint nothing (no wraparound)."""
    if _unavailable():
        return
    off = np.array([[2.0, -6.0], [2.0, -2.0], [8.0, -2.0], [8.0, -6.0]])
    assert rasterize_polygon(off, (12, 12)).sum() == 0
    straddle = np.array([[-2.0, -2.0], [-2.0, 4.0], [3.0, 4.0], [3.0, -2.0]])
    m = rasterize_polygon(straddle, (12, 12))
    assert m[:4, :5].any() and not m[:, 5:].any() and not m[4:, :].any()
    print("[PASS] test_rasterize_clips_out_of_grid_polygons")


# ------------------------------------------------------------- wheel levels

def test_level_monotonicity_and_merge():
    """region(level+1) ⊇ region(level); one notch up merges through the door.

    Needs a door whose HALF-width (0.25m here) exceeds t_min — a narrower
    throat never enters `{DT >= t}` during the sweep, so there is no leak
    to pass.
    """
    if _unavailable():
        return
    g = two_rooms(door_cols=(24, 33))
    prev = None
    for level in range(-2, 3):
        r = _run(g, (20, 30), level=level)
        assert r.ok, f"level {level} gave no suggestion"
        if prev is not None:
            assert not (prev & ~r.mask).any(), f"level {level} lost cells"
        prev = r.mask
    merged = _run(g, (20, 30), level=1)
    assert merged.mask[50, 30], "level +1 did not merge through the door"
    base = _run(g, (20, 30), level=0)
    assert base.leaked and not base.mask[50, 30]
    # negative levels raise the stop threshold (expansion is basin-granular,
    # so the mask may not shrink — the branch is pinned via t_star)
    shrunk = _run(g, (20, 30), level=-2)
    assert shrunk.t_star > base.t_star + 1e-9, "level<0 did not raise t*"
    assert not (shrunk.mask & ~base.mask).any()
    print("[PASS] test_level_monotonicity_and_merge")


def test_level_two_crosses_two_necks():
    """In a three-room chain, level counts leaks: +1 reaches the middle
    room only, +2 reaches the far room too."""
    if _unavailable():
        return
    g = np.full((60, 100), 100, dtype=np.int8)
    for c0, c1 in ((1, 30), (33, 64), (67, 98)):  # three rooms in a row
        g[1:59, c0:c1 + 1] = 0
    # door widths differ so the two necks open at different thresholds —
    # equal doors open in the same notch and merge as ONE surge
    g[22:36, 31:33] = 0    # door A<->B, 0.7m
    g[25:33, 65:67] = 0    # door B<->C, 0.4m
    r0 = _run(g, (30, 15))
    r1 = _run(g, (30, 15), level=1)
    r2 = _run(g, (30, 15), level=2)
    assert not r0.mask[30, 50] and not r0.mask[30, 80]
    assert r1.mask[30, 50] and not r1.mask[30, 80], "level 1 must stop at B"
    assert r2.mask[30, 50] and r2.mask[30, 80], "level 2 must reach C"
    print("[PASS] test_level_two_crosses_two_necks")


# ----------------------------------------------------------- fallback semantics

def test_bad_seeds_and_tiny_regions_give_no_suggestion():
    """Seeds on walls/unknown/outside and sub-minimum cavities -> ok=False."""
    if _unavailable():
        return
    g = two_rooms()
    assert not _run(g, (40, 5)).ok          # on the dividing wall
    g2 = g.copy()
    g2[18:23, 28:33] = -1                   # unknown patch > k_speckle
    assert not _run(g2, (20, 30)).ok        # on unknown
    g3 = g.copy()
    g3[20, 30] = -1                         # single-cell speck: despeckle
    assert _run(g3, (20, 30)).ok            # heals it, hover still works
    assert not _run(g, (-3, 30)).ok         # outside the grid
    assert not _run(g, (20, 300)).ok
    tiny = np.full((20, 20), 100, dtype=np.int8)
    tiny[8:12, 8:12] = 0                     # 0.04 m^2 pocket
    assert not suggest(tiny, RES, (10, 10), params=P).ok
    print("[PASS] test_bad_seeds_and_tiny_regions_give_no_suggestion")


def test_thin_cavity_never_suggests_the_background():
    """Hovering free space thinner than t_min must give ok=False — never the
    inverted background component spanning unrelated rooms (the degenerate
    `dt[core] < t_min` case)."""
    if _unavailable():
        return
    g = np.full((120, 120), 100, dtype=np.int8)
    g[10:50, 10:50] = 0        # a real room
    g[70:110, 70:110] = 0      # a second, disconnected room
    g[60:65, 20:25] = 0        # 0.25m-wide pocket, thinner than t_min 0.3
    r = suggest(g, RES, (62, 22))  # production defaults
    assert not r.ok, f"thin cavity suggested area={r.area_m2}"
    assert r.reason == "cavity below t_min"
    # the guarded downsweep itself returns an empty component
    free, dt = preprocess(g, RES, SuggestParams())
    t_star, region0, leaked = downsweep(dt, (62, 22), RES, SuggestParams())
    assert not region0.any() and not leaked
    print("[PASS] test_thin_cavity_never_suggests_the_background")


def test_a_room_min_gates_leak_detection():
    """The surge RATIO alone is not a leak — the absolute delta must also
    exceed a_room_min. On a hand-built DT staircase (each notch exposes 8
    cells) a 0.075 m^2 blob fires the ratio (~4.7x) but is under the 0.5 m^2
    gate, so the sweep continues through it; the 1.0 m^2 blob then triggers
    the actual leak and the sweep retreats above it."""
    if _unavailable():
        return
    dt = np.zeros((40, 60))
    for i, c0 in enumerate(range(0, 40, 4)):  # 10 blocks, dt 0.60 -> 0.15
        dt[5:7, c0:c0 + 4] = 0.60 - 0.05 * i
    dt[7:37, 24] = 0.30        # mid blob: 30 cells = 0.075 m^2 < a_room_min
    dt[7:27, 32:52] = 0.20     # big blob: 400 cells = 1.0 m^2 > a_room_min
    t_star, region0, leaked = downsweep(dt, (5, 0), RES, P)
    assert leaked and abs(t_star - 0.25) < 1e-9
    assert region0[20, 24], "sub-a_room_min blob was cut off as a leak"
    assert not region0[10, 40], "big blob must be cut off as the real leak"
    print("[PASS] test_a_room_min_gates_leak_detection")


def test_anchor_from_near_wall_seed():
    """A seed one cell off the wall anchors the room's wide interior and
    yields the same suggestion as a center hover."""
    if _unavailable():
        return
    g = two_rooms()
    center = _run(g, (20, 30))
    near_wall = _run(g, (2, 30))
    assert near_wall.ok
    assert near_wall.t_star == center.t_star
    assert (near_wall.mask == center.mask).all()
    print("[PASS] test_anchor_from_near_wall_seed")


# ------------------------------------------------------------------- basins

def test_basin_invariants_on_random_layouts():
    """Basin labelling invariants hold on randomized clutter (fixed seeds):
    exactly the free cells are labelled, each basin's recorded peak equals
    the max DT over its cells, the expansion built on the basins contains
    the core and stays free, and relabelling is deterministic."""
    if _unavailable():
        return
    for seed_val in (3, 11, 27):
        rng = np.random.RandomState(seed_val)
        g = np.full((40, 40), 100, dtype=np.int8)
        g[1:39, 1:39] = 0
        for _ in range(5):  # scatter obstacle blocks
            y, x = rng.randint(3, 33, size=2)
            g[y:y + rng.randint(2, 6), x:x + rng.randint(2, 6)] = 100
        free, dt = preprocess(g, RES, P)
        labels, peaks = ascent_basins(dt, free)
        assert (labels[free] > 0).all() and (labels[~free] == 0).all()
        for k in range(1, len(peaks)):
            sel = labels == k
            assert sel.any(), f"basin {k} has no cells"
            assert abs(dt[sel].max() - peaks[k]) < 1e-9
        ys, xs = np.nonzero(free)
        cell = (int(ys[len(ys) // 2]), int(xs[len(xs) // 2]))
        t_star, region0, _ = downsweep(dt, cell, RES, P)
        region = expand(free, region0, t_star, (labels, peaks))
        assert (region0 & ~region).sum() == 0      # contains the core
        assert not (region & ~free).any()           # never leaves free space
        # determinism: a second run reproduces the labelling exactly
        labels2, peaks2 = ascent_basins(dt, free)
        assert (labels2 == labels).all() and (peaks2 == peaks).all()
    print("[PASS] test_basin_invariants_on_random_layouts")


# --------------------------------------------------------------- world coords

def test_cell_world_roundtrip_and_orientation():
    """Row 0 sits at origin, +row = +y; conversions round-trip."""
    if _unavailable():
        return
    res, org = 0.05, (-2.0, 3.0)
    assert cell_to_world(0, 0, res, org) == (-2.0 + 0.025, 3.0 + 0.025)
    x, y = cell_to_world(10, 4, res, org)
    assert (y - 3.0) > (x + 2.0), "+row must move +y faster than col moved +x"
    assert world_to_cell(x, y, res, org) == (10, 4)
    print("[PASS] test_cell_world_roundtrip_and_orientation")


def test_result_to_json_shape():
    """Endpoint payload carries metric polygon + flags, no numpy types;
    the frame (resolution/origin) is stamped on the result itself, and a
    not-ok result serializes to an empty polygon."""
    if _unavailable():
        return
    import json

    g = two_rooms()
    s = RoomSuggester(g, RES, origin_xy=(1.0, -1.0), params=P)
    r = s.suggest((20, 30))
    payload = r.to_json()
    json.dumps(payload)  # must be JSON-serializable as-is
    assert payload["ok"] and payload["area_m2"] > 0
    assert all(len(p) == 2 for p in payload["polygon"])
    xs = [p[0] for p in payload["polygon"]]
    assert min(xs) >= 1.0, "polygon not in map-frame meters"
    bad = s.suggest((40, 5)).to_json()  # seed on the dividing wall
    json.dumps(bad)
    assert not bad["ok"] and bad["polygon"] == [] and bad["reason"]
    print("[PASS] test_result_to_json_shape")


# -------------------------------------------------------------- memoization

def test_suggester_memoizes_per_grid_stages():
    """Preprocess/basins are built once; world-seed entry uses the origin."""
    if _unavailable():
        return
    g = two_rooms()
    s = RoomSuggester(g, RES, origin_xy=(0.0, 0.0), params=P)
    r1 = s.suggest((20, 30))
    pre1, basins1 = s._pre, s._basins
    r2 = s.suggest((50, 30))
    assert s._pre is pre1 and s._basins is basins1, "stages were rebuilt"
    assert r1.ok and r2.ok
    rw = s.suggest_world(*cell_to_world(20, 30, RES, (0.0, 0.0)))
    assert rw.ok and rw.mask[20, 30]
    print("[PASS] test_suggester_memoizes_per_grid_stages")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
