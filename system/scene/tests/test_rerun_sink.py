# SPDX-License-Identifier: MulanPSL-2.0
"""Focused regressions for the rerun sink's pure helpers.

The sink's logging methods need a running rerun, which no test environment is
guaranteed to have. What is testable without it is everything that decides
*what* gets drawn: the colour an object keeps, where a map coordinate lands on
the grid, which end of a relation is which, and that a deployment with no rerun
installed stays silent instead of raising.
"""

import base64
import io
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.rerun_sink import (  # noqa: E402
    RerunSink,
    _relation_end,
    _to_pixels,
    instance_colour,
)


def test_an_object_keeps_its_colour_across_restarts():
    """Two screenshots of the same map are only comparable if it does.

    Asserted across processes on purpose. Python salts string hashing per
    process, so a colour derived from `hash()` is stable within one run and
    different on the next — which is exactly the bug this guards, and exactly
    the bug a same-process assertion cannot see.
    """
    first = instance_colour("scene.object.chair_003")
    assert first != instance_colour("scene.object.chair_004")
    assert all(0 <= channel <= 255 for channel in first)

    package = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    program = (
        f"import sys; sys.path.insert(0, {package!r});"
        "from scene_service.rerun_sink import instance_colour;"
        "print(instance_colour('scene.object.chair_003'))"
    )
    for seed in ("0", "1", "2"):
        result = subprocess.run(
            [sys.executable, "-c", program],
            capture_output=True, text=True, check=True,
            env=dict(os.environ, PYTHONHASHSEED=seed),
        )
        assert result.stdout.strip() == str(first)


def _grid(width=80, height=60, resolution=0.05, origin=(-2.0, -1.5)):
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": origin[0],
        "origin_y": origin[1],
    }


def test_the_grid_origin_is_the_bottom_left_of_a_top_down_image():
    """The PNG runs top-down while the grid's y runs up from its origin.

    Getting this backwards mirrors every annotation about the map's centre
    line, which still looks like a plausible map and is entirely wrong.
    """
    occupancy = _grid()
    convert = _to_pixels(occupancy)
    # The image spans [0, width] x [0, height], the same extent as the floor
    # plane the 3D page draws from the same grid, so the two cannot disagree.
    assert convert(-2.0, -1.5) == pytest.approx([0.0, 60.0])
    assert convert(-2.0, 1.5) == pytest.approx([0.0, 0.0])
    assert convert(2.0, -1.5) == pytest.approx([80.0, 60.0])


def test_a_grid_without_a_resolution_has_no_pixel_mapping():
    """Dividing by it would place every annotation at infinity."""
    assert _to_pixels(_grid(resolution=0.0)) is None


def test_a_relation_end_is_read_under_either_name():
    """The web payload and the registry name the ends differently.

    Reading only one name drew no edges at all, which looks exactly like a
    scene with no relations in it.
    """
    assert _relation_end({"subject": "a"}, "subject", "subject_id") == "a"
    assert _relation_end({"subject_id": "b"}, "subject", "subject_id") == "b"
    assert _relation_end({"target": ""}, "target", "object_id") is None


def test_a_deployment_without_rerun_logs_nothing_and_does_not_raise():
    """Native installs do not ship rerun and must keep working.

    Every logging method is called on a sink that never started. A raise here
    would take the perception loop down on a machine that only ever wanted the
    built-in page.
    """
    sink = RerunSink()
    assert not sink.ready
    assert sink.detail
    sink.set_time(1.0)
    sink.log_occupancy(_grid())
    sink.log_objects([], {})
    sink.log_relations([], {})
    sink.log_robot((0.0, 0.0, 0.0), [])
    sink.log_map2d(_grid(), [], {}, [], None, None)


def test_a_grid_that_is_not_a_png_is_reported_and_skipped(caplog):
    """A decode failure must not take the tick down with it."""
    from scene_service.rerun_sink import _grid_texture

    occupancy = dict(_grid(), png_b64=base64.b64encode(b"not a png").decode())
    assert _grid_texture(occupancy) is None
    assert "occupancy grid" in caplog.text


def test_the_three_cell_meanings_get_three_colours():
    """Occupied, free and never-observed have to be told apart by eye."""
    numpy = pytest.importorskip("numpy")
    PIL = pytest.importorskip("PIL.Image")
    from scene_service.rerun_sink import _grid_texture

    cells = numpy.array([[20, 128, 240]], dtype=numpy.uint8)
    buffer = io.BytesIO()
    PIL.fromarray(cells, mode="L").save(buffer, format="PNG")
    texture = _grid_texture(
        dict(_grid(), png_b64=base64.b64encode(buffer.getvalue()).decode()))
    occupied, unknown, free = (tuple(int(c) for c in texture[0][i])
                              for i in range(3))
    assert len({occupied, unknown, free}) == 3
    # The map is drawn on a dark ground, so the brightness order is the
    # inverse of a printed floor plan's: ground the robot has never seen
    # recedes into the background, floor it has seen lifts off it, and the
    # walls are the brightest thing because they are what bounds the room.
    assert sum(unknown) < sum(free) < sum(occupied)


class _FakeEdge:
    def __init__(self, source, relation, target):
        self.source_id = source
        self.relation = relation
        self.target_id = target


class _FakeSnapshot:
    def __init__(self, edges):
        self.edges = edges


class _FakeStore:
    """The two edge layers a scene graph exposes, as the sink reads them."""

    def __init__(self, geometric, snapshot_edges):
        self._geometric = geometric
        self._snapshot = (
            None if snapshot_edges is None else _FakeSnapshot(snapshot_edges))

    def get_geometric_edges(self):
        return self._geometric

    def get_snapshot(self):
        return self._snapshot


def test_an_edge_present_in_both_layers_is_drawn_once():
    """The geometric layer and the scene graph overlap by design."""
    from scene_service.rerun_sink import relation_edges

    shared = ("a", "near", "b")
    edges = relation_edges(_FakeStore(
        [_FakeEdge(*shared)],
        [_FakeEdge(*shared), _FakeEdge("b", "on", "c")],
    ))
    assert edges == [
        {"subject": "a", "target": "b", "predicate": "near"},
        {"subject": "b", "target": "c", "predicate": "on"},
    ]


def test_a_scene_with_no_graph_store_has_no_edges():
    """Relations are optional; the map still draws without them."""
    from scene_service.rerun_sink import relation_edges

    assert relation_edges(None) == []
    assert relation_edges(_FakeStore([], None)) == []


class _FakeRerun:
    """Records what would be logged, in place of a running rerun."""

    def __init__(self):
        self.logged = []
        self.static = []

    def log(self, path, entity, recording=None, static=False):
        self.logged.append((path, type(entity).__name__))
        if static:
            self.static.append(path)

    def __getattr__(self, name):
        def archetype(*args, **kwargs):
            return type(name, (), {})()
        return archetype


class _FakePose:
    def __init__(self, x, y):
        self.x, self.y, self.z, self.yaw = x, y, 0.3, 0.0


class _FakeBBox:
    size_x = size_y = size_z = 0.4
    yaw = 0.0


class _FakeObject:
    def __init__(self, object_id, cls, x=0.0, y=0.0):
        self.object_id, self.cls = object_id, cls
        self.pose = _FakePose(x, y)
        self.bbox = _FakeBBox()


def _started_sink():
    """A sink wired to a fake rerun, so the logging bodies can be exercised."""
    sink = RerunSink()
    fake = _FakeRerun()
    sink._rr = fake
    sink._ready = True
    return sink, fake


def test_an_object_that_leaves_the_registry_stops_being_drawn():
    """Each object owns its entity paths, so a dropped one lingers forever.

    Without an explicit clear the viewer shows every object the registry has
    ever held, and a stale map is indistinguishable from a live one.
    """
    sink, fake = _started_sink()
    first = _FakeObject("scene.object.chair_001", "chair")
    second = _FakeObject("scene.object.lamp_002", "lamp")
    sink.log_objects([first, second], {})
    fake.logged.clear()
    sink.log_objects([first], {})

    cleared = [path for path, entity in fake.logged if entity == "Clear"]
    assert any("lamp_002" in path for path in cleared)
    assert not any("chair_001" in path for path in cleared)


def test_the_last_relation_disappearing_clears_the_edges():
    """An entity that stops being logged keeps its last value on screen."""
    sink, fake = _started_sink()
    positions = {"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0)}
    sink.log_relations([{"subject": "a", "target": "b", "predicate": "near"}],
                       positions)
    fake.logged.clear()
    sink.log_relations([], positions)
    assert ("/map/relations", "LineStrips3D") in fake.logged


def test_a_viewer_whose_ports_are_taken_declines_instead_of_hanging():
    """`serve_grpc` blocks on a busy port rather than raising.

    A blocked call inside lifecycle activation is Scene hanging on startup
    with nothing in the log, which is how a previous instance that has not
    finished exiting presents. The viewer is a debugging aid and must lose
    that race, not the service.
    """
    import socket

    from scene_service.rerun_sink import RerunSink

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as held:
        held.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        held.bind(("0.0.0.0", 0))
        held.listen(1)
        port = held.getsockname()[1]

        sink = RerunSink(grpc_port=port)
        assert sink.start() is False
        assert not sink.ready
        assert str(port) in sink.detail


# ── The viewer's cost is how often the map is sent, not how big it is ───────
# Every publish used to append a full copy of the grid, of every point cloud
# and of every box to the timeline whether or not anything had moved. The
# browser keeps what it is sent, so a stationary robot still filled the
# viewer's store until it hit its limit and the page stopped responding.


def _sink_with(fake, *, history="changes", monkeypatch=None):
    """A sink wired to the fake, without starting any server."""
    from scene_service.rerun_sink import RerunSink

    sink = RerunSink()
    sink._rr = fake
    sink._ready = True
    sink._history = history
    sink._map3d.recording = object()
    sink._map2d.recording = object()
    return sink


def test_an_unchanged_map_is_not_sent_twice():
    """A tick that changed nothing must put nothing on the wire."""
    fake = _FakeRerun()
    sink = _sink_with(fake)
    objects = [_FakeObject("a", "chair", 1.0, 1.0)]
    clouds = {"a": [[1.0, 1.0, 0.1], [1.0, 1.1, 0.2]]}

    sink.log_objects(objects, clouds)
    first = len(fake.logged)
    assert first, "the first tick has to draw the object"

    sink.log_objects(objects, clouds)
    assert len(fake.logged) == first, (
        "the second tick logged again although nothing moved")


def test_a_moved_object_is_sent_again():
    """Change detection must not swallow a real update."""
    fake = _FakeRerun()
    sink = _sink_with(fake)
    objects = [_FakeObject("a", "chair", 1.0, 1.0)]
    sink.log_objects(objects, {"a": [[1.0, 1.0, 0.1]]})
    before = len(fake.logged)
    sink.log_objects(objects, {"a": [[2.0, 1.0, 0.1]]})
    assert len(fake.logged) > before


def test_an_object_redrawn_after_being_cleared_is_sent_again():
    """Clearing has to forget what the entity carried.

    The entity is empty on screen after a clear, so suppressing the next
    identical value would leave it empty for the rest of the session.
    """
    fake = _FakeRerun()
    sink = _sink_with(fake)
    objects = [_FakeObject("a", "chair", 1.0, 1.0)]
    clouds = {"a": [[1.0, 1.0, 0.1]]}
    sink.log_objects(objects, clouds)
    sink.log_objects([], {})            # the registry drops it: cleared
    before = len(fake.logged)
    sink.log_objects(objects, clouds)   # and it comes back unchanged
    assert len(fake.logged) > before


def test_the_grid_is_static_so_its_history_is_never_kept():
    """The floor plan is a current-state artefact.

    SLAM rewrites it continuously and every superseded version is weight the
    browser carries for the rest of the session.
    """
    numpy = pytest.importorskip("numpy")
    PIL = pytest.importorskip("PIL.Image")
    buffer = io.BytesIO()
    PIL.fromarray(numpy.full((4, 4), 240, dtype=numpy.uint8),
                  mode="L").save(buffer, format="PNG")
    fake = _FakeRerun()
    sink = _sink_with(fake)
    sink.log_occupancy(dict(
        _grid(), png_b64=base64.b64encode(buffer.getvalue()).decode()))
    assert "/map/floor" in fake.static


def test_latest_history_logs_everything_static():
    """The mode a forwarded or weak client needs: one value per entity."""
    fake = _FakeRerun()
    sink = _sink_with(fake, history="latest")
    sink.log_objects([_FakeObject("a", "chair", 1.0, 1.0)],
                     {"a": [[1.0, 1.0, 0.1]]})
    assert fake.static, "nothing was logged static in `latest` mode"


def test_latest_history_pins_the_timeline():
    """With one value per entity there is no timeline to place it on."""
    fake = _FakeRerun()
    sink = _sink_with(fake, history="latest")
    calls = []
    sink._map3d.recording = type("R", (), {
        "set_time": lambda self, *a, **k: calls.append(a)})()
    sink._map2d.recording = type("R", (), {
        "set_time": lambda self, *a, **k: calls.append(a)})()
    sink.set_time(12.0)
    assert calls == []


def test_the_viewer_link_asks_for_the_dark_theme():
    """The map is drawn for a dark ground; the chrome must not be light."""
    fake = _FakeRerun()
    sink = _sink_with(fake)
    sink._map3d.grpc_url = "rerun+http://127.0.0.1:9876/proxy"
    assert "theme=dark" in sink.viewer_url("robot.local")
