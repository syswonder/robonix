# SPDX-License-Identifier: MulanPSL-2.0
"""The landing page must offer the viewer before the viewer is running.

Bring-up moved off the boot path: rerun now starts when someone opens
`/rerun`, not when scene activates. That turned one question into two, and
the landing page was left asking the wrong one.

It rendered the viewer frame only when `rerun_sink.ready`. But `ready` turns
true only after `ensure_started`, which only `/rerun` calls, and `/rerun` is
only fetched by the frame the page refused to render. A deployment with
rerun installed served the built-in canvas for ever, and nothing in the log
said why -- the fallback is the same one a native install legitimately takes.

So layout asks `available` (is there a viewer here at all) and only the
routes that forward to a live server ask `ready`.
"""
from starlette.testclient import TestClient

from scene_service import web
from scene_service.state import BBox3D, Pose3D
from scene_service.state.object_registry import ObjectRegistry


class _Sink:
    """A sink that has rerun but has not been started -- the deadlocked case."""

    def __init__(self, *, available=True, ready=False):
        self.available = available
        self.ready = ready
        self.started = 0

    def ensure_started(self):
        self.started += 1
        self.ready = True
        return True


def _client(sink):
    reg = ObjectRegistry()
    reg.insert_object(
        cls="chair",
        pose=Pose3D(x=1.0, y=1.0, z=0.0, yaw=0.0, frame_id="map"),
        bbox=BBox3D(size_x=0.4, size_y=0.4, size_z=0.5, yaw=0.0,
                    frame_id="map"),
        confidence=0.9, now=1000.0)
    return TestClient(web.make_app(
        registry=reg, hub=None, map_binding={"map_id": "lab"},
        rerun_sink=sink))


def test_landing_page_offers_the_viewer_before_it_has_started():
    """The regression: `ready` here can never become true on its own."""
    client = _client(_Sink(available=True, ready=False))
    body = client.get("/").text
    assert "/rerun?view=3d" in body, (
        "a deployment that has rerun must be handed the viewer frame even "
        "though nothing has started it yet -- that frame is what starts it")


def test_opening_the_frame_is_what_starts_the_viewer():
    sink = _Sink(available=True, ready=False)
    client = _client(sink)
    client.get("/")
    assert sink.started == 0, "rendering the page must not start the viewer"
    client.get("/rerun")
    assert sink.started == 1, "opening the frame must start it"


def test_a_deployment_without_rerun_keeps_the_built_in_page():
    """Native installs ship no rerun and must not get a frame that cannot load."""
    client = _client(_Sink(available=False, ready=False))
    body = client.get("/").text
    assert "/rerun?view=3d" not in body
    assert "canvas" in body.lower() or "iframe" in body.lower()
