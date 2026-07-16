# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for room suggestion (scene_service.room_suggest).

Pure numpy/scipy over synthetic OccupancyGrid-semantics arrays (-1 unknown /
0 free / 100 occupied) — no ROS, no milvus. Skips loudly when scipy is
unavailable (e.g. a bare mac checkout).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:  # room_suggest needs scipy — absent on a bare mac checkout
    import numpy as np

    from scene_service.room_suggest import (  # noqa: E402
        RoomSuggester,
        SuggestParams,
        ascent_basins,
        binarize_occupancy,
        cell_to_world,
        despeckle,
        expand,
        preprocess,
        rasterize_polygon,
        rdp,
        suggest,
        trace_outer_contour,
        world_to_cell,
    )

    _IMPORT_ERR = None
except ImportError as e:  # pragma: no cover - environment-dependent
    _IMPORT_ERR = e

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
    """Two 3x2m rooms split by a wall at row 40, joined by a door.

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
    g[15, 15] = 0
    free = binarize_occupancy(g)
    free[15, 15] = False     # carve a free pinhole in a wall for the test
    g2 = g.copy()
    g2[14:17, 14:17] = 100
    g2[15, 15] = 0           # 1-cell free speck inside an obstacle block
    out = despeckle(binarize_occupancy(g2), k_cells=3)
    assert out[10, 10]            # obstacle speck healed to free
    assert not out[3, 5]          # wall fragment kept (borders unknown)
    assert not out[15, 15]        # free speck removed
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
    g[41:59, 0] = -1
    g[30:36, 59] = 0  # breach the right outer wall of room A
    g[30:36, 58] = 0
    r = _run(g, (20, 30))
    assert r.ok
    assert not (r.mask & (g == -1)).any()
    print("[PASS] test_exterior_gap_to_unknown_does_not_spill")


# ------------------------------------------------------------ expansion rules

def test_expand_absorbs_pockets_but_not_rival_rooms():
    """Corner pockets (peak < t*) merge in; the neighbour room never does."""
    if _unavailable():
        return
    g = two_rooms()
    g[1:6, 1:10] = 100
    g[6, 1:4] = 0    # carve a shallow nook behind the block (pocket)
    free, dt = preprocess(g, RES, P)
    basins = ascent_basins(dt, free)
    r = _run(g, (20, 30))
    assert r.ok
    assert r.mask[6, 2], "sub-room pocket was not absorbed"
    assert not r.mask[50, 30], "rival cavity absorbed"
    # never a cell outside free space
    assert not (r.mask & ~free).any()
    print("[PASS] test_expand_absorbs_pockets_but_not_rival_rooms")


# -------------------------------------------------------------- polygonization

def test_polygon_hugs_the_region():
    """RDP output stays within eps of the region (both directions)."""
    if _unavailable():
        return
    from scipy import ndimage

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


# ------------------------------------------------------------- wheel levels

def test_level_monotonicity_and_merge():
    """region(level+1) ⊇ region(level); one notch up merges through the door.

    Needs a door wider than t_min (0.5m here) — a narrower throat never
    enters `{DT >= t}` during the sweep, so there is no leak to pass.
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
    print("[PASS] test_level_monotonicity_and_merge")


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


# ------------------------------------------------------------------- basins

def test_basin_invariants_on_random_layouts():
    """Basin labelling invariants hold on randomized clutter (fixed seeds):
    exactly the free cells are labelled, each basin's recorded peak is the
    max DT over its cells, no basin outranks the DT of its own peak, and
    the expansion built on the basins contains the core and stays free."""
    if _unavailable():
        return
    from scene_service.room_suggest import downsweep

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
    """Endpoint payload carries metric polygon + flags, no numpy types."""
    if _unavailable():
        return
    import json

    g = two_rooms()
    s = RoomSuggester(g, RES, origin_xy=(1.0, -1.0), params=P)
    r = s.suggest((20, 30))
    payload = s.to_json(r)
    json.dumps(payload)  # must be JSON-serializable as-is
    assert payload["ok"] and payload["area_m2"] > 0
    assert all(len(p) == 2 for p in payload["polygon"])
    xs = [p[0] for p in payload["polygon"]]
    assert min(xs) >= 1.0, "polygon not in map-frame meters"
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
