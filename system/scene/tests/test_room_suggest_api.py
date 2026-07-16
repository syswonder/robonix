# SPDX-License-Identifier: MulanPSL-2.0
"""Endpoint tests for GET /api/rooms/suggest (scene_service.web).

Drives the ASGI app directly with a fake hub serving a synthetic
OccupancyGrid — no ROS, no milvus. Skips loudly when the web chain
(starlette) or scipy is unavailable (e.g. a bare mac checkout).
"""
import asyncio
import json
import os
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:  # probe the third-party deps — absent on a bare mac checkout
    import numpy as np
    import scipy  # noqa: F401 - probe only
    import starlette  # noqa: F401 - probe only

    _IMPORT_ERR = None
except ModuleNotFoundError as e:  # pragma: no cover - environment-dependent
    _IMPORT_ERR = e

if _IMPORT_ERR is None:
    try:
        # web pulls the whole service chain (robonix_api etc.) — also an
        # environment dependency, unlike room_suggest module bugs which
        # must fail loudly (covered by test_room_suggest.py's guard).
        from scene_service import web as web_mod
        from scene_service.state.object_registry import ObjectRegistry
    except ImportError as e:  # pragma: no cover - environment-dependent
        _IMPORT_ERR = e


def _unavailable() -> bool:
    """Skip (pytest) or announce-and-return (main runner) when deps miss."""
    if _IMPORT_ERR is None:
        return False
    if os.environ.get("PYTEST_CURRENT_TEST"):
        import pytest

        pytest.skip(f"web/scipy deps unavailable: {_IMPORT_ERR}")
    print(f"[SKIP] web/scipy deps unavailable: {_IMPORT_ERR}")
    return True


RES = 0.05
ORIGIN = (-1.0, -2.0)


def _two_room_grid():
    """80x60 int8 grid: two ~2m-deep rooms split at row 40 by a 0.8m door.

    Sized for the PRODUCTION defaults: the door half-width (0.4m) must
    clear t_min (0.3m) for the neck to be sweepable, and both rooms must
    be much wider than the door for the leak surge to be detectable
    (design: "a room is wider than its opening").
    """
    g = np.full((80, 60), 100, dtype=np.int8)
    g[1:40, 1:59] = 0
    g[41:79, 1:59] = 0
    g[40, 22:38] = 0
    return g


class GridHub:
    """Latched occupancy hub: serves one fixed grid message."""

    def __init__(self, grid=None, count=1):
        self._grid = grid
        self._count = count

    def has(self, key):
        return key == "occupancy_grid" and self._grid is not None

    def latest(self, key):
        if self._grid is None:
            return None, 0.0, 0
        h, w = self._grid.shape
        msg = SimpleNamespace(
            info=SimpleNamespace(
                width=w, height=h, resolution=RES,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=ORIGIN[0], y=ORIGIN[1])),
            ),
            data=self._grid.tobytes(),
        )
        return msg, time.time(), self._count


def _app(hub):
    return web_mod.make_app(registry=ObjectRegistry(), hub=hub)


def _get(app, path_qs):
    """One GET straight into the ASGI app on a private loop."""
    path, _, qs = path_qs.partition("?")
    scope = {
        "type": "http", "http_version": "1.1", "method": "GET",
        "scheme": "http", "path": path, "raw_path": path_qs.encode(),
        "root_path": "", "query_string": qs.encode(),
        "client": ("test", 0), "server": ("test", 80), "headers": [],
    }
    sent = []

    async def receive():
        return {"type": "http.request", "body": b"", "more_body": False}

    async def send(message):
        sent.append(message)

    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(app(scope, receive, send))
    finally:
        loop.close()
    status = next(m["status"] for m in sent if m["type"] == "http.response.start")
    body = b"".join(m.get("body", b"") for m in sent if m["type"] == "http.response.body")
    return status, json.loads(body or b"{}")


def _world(row, col):
    """Cell center -> map-frame meters under the fixture's origin."""
    return ORIGIN[0] + (col + 0.5) * RES, ORIGIN[1] + (row + 0.5) * RES


def test_suggest_happy_path_returns_metric_polygon():
    """Hovering room A yields ok + a polygon in map-frame meters that stays
    inside room A's world-coordinate bounds, plus the grid stamp."""
    if _unavailable():
        return
    app = _app(GridHub(_two_room_grid()))
    x, y = _world(20, 30)
    status, out = _get(app, f"/api/rooms/suggest?x={x}&y={y}")
    assert status == 200 and out["ok"], out
    assert out["area_m2"] > 1.0 and out["stamp_ms"] > 0
    xs = [p[0] for p in out["polygon"]]
    ys = [p[1] for p in out["polygon"]]
    x0, y0 = _world(1, 1)
    x1, y1 = _world(39, 58)
    assert min(xs) >= x0 - RES and max(xs) <= x1 + RES
    assert min(ys) >= y0 - RES and max(ys) <= y1 + RES, "polygon left room A"
    print("[PASS] test_suggest_happy_path_returns_metric_polygon")


def test_suggest_level_merges_through_the_door():
    """level=1 passes the door neck: the region grows past both rooms'
    combined threshold and reaches room B's side of the map."""
    if _unavailable():
        return
    app = _app(GridHub(_two_room_grid()))
    x, y = _world(20, 30)
    _, base = _get(app, f"/api/rooms/suggest?x={x}&y={y}&level=0")
    _, merged = _get(app, f"/api/rooms/suggest?x={x}&y={y}&level=1")
    assert base["ok"] and merged["ok"]
    assert merged["area_m2"] > base["area_m2"] * 1.3
    _, y_b = _world(50, 30)
    assert max(p[1] for p in merged["polygon"]) > y_b - 0.3, "no room B reach"
    print("[PASS] test_suggest_level_merges_through_the_door")


def test_suggest_wall_seed_is_ok_false_not_error():
    """A seed on the dividing wall is a clean ok:false with a reason."""
    if _unavailable():
        return
    app = _app(GridHub(_two_room_grid()))
    x, y = _world(40, 5)
    status, out = _get(app, f"/api/rooms/suggest?x={x}&y={y}")
    assert status == 200 and not out["ok"] and out["reason"]
    assert out["polygon"] == []
    print("[PASS] test_suggest_wall_seed_is_ok_false_not_error")


def test_suggest_param_validation():
    """Missing/garbage x/y/level answer 400 with ok:false."""
    if _unavailable():
        return
    app = _app(GridHub(_two_room_grid()))
    for qs in ("", "x=1", "x=abc&y=0", "x=0&y=0&level=abc", "x=nan&y=0"):
        status, out = _get(app, f"/api/rooms/suggest?{qs}")
        assert status == 400 and not out["ok"], (qs, status, out)
    print("[PASS] test_suggest_param_validation")


def test_suggest_without_grid_is_503():
    """No occupancy grid yet (or no hub at all) -> 503, not a crash."""
    if _unavailable():
        return
    for app in (_app(GridHub(None)), _app(None)):
        status, out = _get(app, "/api/rooms/suggest?x=0&y=0")
        assert status == 503 and not out["ok"]
    print("[PASS] test_suggest_without_grid_is_503")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
