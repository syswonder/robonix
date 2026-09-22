# SPDX-License-Identifier: MulanPSL-2.0
"""Which pages carry the object panel.

The landing page exists to answer "is the perception any good", and the
panel that answers it -- the objects scene holds, which are unsure, which
need renaming or deleting -- was reachable from exactly one route. Both
`index` and `index2d` ask for it; only the branch taken when rerun is
absent dropped it, because `_framed` wrapped the page without ever passing
the flag along. On a built-in-viewer deployment that is the branch the
landing page takes, so the one page a person opens was the one page with no
object list.

Nobody chose that. These pin the panel to the pages that ask for it, so a
future branch cannot silently lose it again.
"""
import pytest


def _client(**kwargs):
    from starlette.testclient import TestClient

    from scene_service import web
    from scene_service.state import BBox3D, Pose3D
    from scene_service.state.object_registry import ObjectRegistry

    reg = ObjectRegistry()
    reg.insert_object(
        cls="chair",
        pose=Pose3D(x=1.0, y=1.0, z=0.0, yaw=0.0, frame_id="map"),
        bbox=BBox3D(size_x=0.4, size_y=0.4, size_z=0.5, yaw=0.0,
                    frame_id="map"),
        confidence=0.9, now=1000.0)
    return TestClient(web.make_app(
        registry=reg, hub=None, map_binding={"map_id": "lab"}, **kwargs))


@pytest.mark.parametrize("path", ["/", "/2d", "/3d"])
def test_every_map_page_carries_the_object_panel(path):
    """Including the landing page with no rerun sink attached, which is the
    deployment the built-in viewer runs and the case that lost it."""
    body = _client().get(path).text
    assert 'id="dock"' in body, f"{path} has no object panel"
    # The controls the panel exists for, not merely the container.
    for element in ("dock-objs", "dock-detail", "dock-flush", "dock-split",
                    "dock-gone"):
        assert element in body, f"{path} is missing {element}"


def test_a_bare_page_stays_bare():
    """`?bare=1` is what stops an embedded copy drawing a second sidebar
    and a second panel inside the frame."""
    body = _client().get("/2d?bare=1").text
    assert 'id="dock"' not in body


@pytest.mark.parametrize("path", ["/cam", "/regions"])
def test_pages_that_are_not_the_map_do_not_get_it(path):
    """The panel annotates the map. The camera feed and the region editor
    are their own subjects, and docking it there would only take width."""
    assert 'id="dock"' not in _client().get(path).text
