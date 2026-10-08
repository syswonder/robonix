# SPDX-License-Identifier: MulanPSL-2.0
"""Focused regressions for Scene's passive live-view pages."""

import asyncio
import os
import re
import shutil
import subprocess
import sys
import threading

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _web_module():
    """Import production code so its own ImportError can never become a skip."""
    from scene_service import web

    return web


class _FakeHub:
    def __init__(self, channels):
        self.channels = channels

    def has(self, kind):
        return kind in self.channels

    def latest(self, kind):
        return self.channels[kind]


def _empty_camera_cache():
    """Return isolated mutable cache state for one focused test."""
    return {
        "rgb": {"hub": None, "count": -1, "payload": None},
        "depth": {"hub": None, "count": -1, "payload": None},
    }


def test_camera_preview_is_encoded_once_per_hub_channel_message(monkeypatch):
    """Reuse completed encodings but never cross hub or message identity."""
    web = _web_module()
    monkeypatch.setattr(web, "_CAMERA_CACHE", _empty_camera_cache())
    calls = []

    def fake_encode(msg, *, kind):
        """Return a deterministic encoded record while tracking invocations."""
        calls.append((msg, kind))
        return {
            "width": 2,
            "height": 1,
            "encoding": "rgb8",
            "stamp_ms": 0,
            "png_b64": f"encoded-{msg}",
        }

    monkeypatch.setattr(web, "_image_to_png_b64", fake_encode)
    hub = _FakeHub({"rgb": ("frame-1", 12.5, 1)})

    first = web._camera_payload(hub)
    second = web._camera_payload(hub)
    assert first == second
    assert first["rgb"]["stamp_ms"] == 12500
    assert calls == [("frame-1", "rgb")]

    hub.channels["rgb"] = ("frame-2", 13.0, 2)
    assert web._camera_payload(hub)["rgb"]["png_b64"] == "encoded-frame-2"
    assert calls == [("frame-1", "rgb"), ("frame-2", "rgb")]

    # A new app/hub may start its counter at the same value. Hub identity is
    # therefore part of the cache key, preventing a stale cross-app frame.
    other_hub = _FakeHub({"rgb": ("other-frame-2", 14.0, 2)})
    assert web._camera_payload(other_hub)["rgb"]["png_b64"] == "encoded-other-frame-2"
    assert calls[-1] == ("other-frame-2", "rgb")


def test_unsupported_camera_frame_is_cached_until_message_changes(monkeypatch):
    """Cache an unsupported frame only until that channel count advances."""
    web = _web_module()
    monkeypatch.setattr(web, "_CAMERA_CACHE", _empty_camera_cache())
    calls = []

    def unsupported(msg, *, kind):
        calls.append((msg, kind))
        return None

    monkeypatch.setattr(web, "_image_to_png_b64", unsupported)
    hub = _FakeHub({"depth": ("unsupported-depth", 1.0, 7)})
    assert web._camera_payload(hub)["depth"] is None
    assert web._camera_payload(hub)["depth"] is None
    assert calls == [("unsupported-depth", "depth")]

    hub.channels["depth"] = ("next-depth", 2.0, 8)
    assert web._camera_payload(hub)["depth"] is None
    assert calls == [
        ("unsupported-depth", "depth"),
        ("next-depth", "depth"),
    ]


def test_camera_requests_are_offloaded_single_flight_and_rate_limited(monkeypatch):
    """Keep a slow preview worker from blocking or duplicating ASGI requests."""
    web = _web_module()
    hub = object()
    calls = []
    entered = threading.Event()
    release = threading.Event()

    def slow_payload(request_hub):
        """Hold the worker until the test proves the event loop is responsive."""
        calls.append(request_hub)
        entered.set()
        if not release.wait(timeout=2.0):
            raise AssertionError("camera encoding blocked the ASGI event loop")
        return {"rgb": {"stamp_ms": 1, "png_b64": "frame"}, "depth": None}

    monkeypatch.setattr(web, "_camera_payload", slow_payload)
    monkeypatch.setattr(web, "_CAMERA_PREVIEW_MIN_INTERVAL_S", 0.01)
    app = web.make_app(registry=object(), hub=hub)

    async def scenario():
        """Issue overlapping requests and one immediate cache-hit request."""
        import httpx

        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(
            transport=transport, base_url="http://scene.test"
        ) as client:
            first = asyncio.create_task(client.get("/api/camera"))
            second = asyncio.create_task(client.get("/api/camera"))
            deadline = asyncio.get_running_loop().time() + 0.5
            while not entered.is_set() and asyncio.get_running_loop().time() < deadline:
                await asyncio.sleep(0.001)
            assert entered.is_set(), "slow encoder did not start off the event loop"
            assert not first.done()
            release.set()
            responses = await asyncio.gather(first, second)
            responses.append(await client.get("/api/camera"))
            await asyncio.sleep(0.02)
            responses.append(await client.get("/api/camera"))
            return responses

    responses = asyncio.run(scenario())
    assert [response.status_code for response in responses] == [200, 200, 200, 200]
    assert all(
        response.json()["rgb"]["png_b64"] == "frame" for response in responses
    )
    assert calls == [hub, hub]


def test_live_map_pages_expose_first_paint_readiness_and_errors():
    """Expose visible first-paint state and guard both image callbacks."""
    web = _web_module()
    for html in (web._REGIONS_HTML,):
        # `data-ready` is the contract -- the first-paint state this test
        # is about. The tag also names the page it renders now, so the
        # attribute is read rather than the whole tag matched.
        assert 'data-ready="loading"' in html
        assert 'role="status"' in html
        assert "document.body.dataset.ready = value" in html
        assert "c.clientWidth > 0 && c.clientHeight > 0" in html
        assert "visibilitychange" in html
        assert "regions.unavailable" in html
        assert "regions.badImage" in html
        assert html.count("if (!isCurrentOccupancyLoad(loadToken, meta.stamp_ms)) return;") == 2

    assert (
        "occStamp = meta.stamp_ms;\n            occLoading = 0;\n            draw();"
        in web._REGIONS_HTML
    )
    assert "no depth frame available" in web._INDEX_CAM_HTML


def test_occupancy_generation_guard_rejects_late_success_and_error_callbacks():
    """Execute the shared JS guard against stale token and stamp generations."""
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    web = _web_module()
    pattern = re.compile(
        r"function isCurrentOccupancyLoad\(token, stamp\) \{\s*"
        r"return token === occLoadToken && stamp === occLoading;\s*\}"
    )
    for html in (web._REGIONS_HTML,):
        match = pattern.search(html)
        assert match is not None
        program = "\n".join(
            (
                "let occLoadToken = 2;",
                "let occLoading = 200;",
                match.group(0),
                "if (isCurrentOccupancyLoad(1, 100)) process.exit(1);",
                "if (isCurrentOccupancyLoad(2, 100)) process.exit(2);",
                "if (!isCurrentOccupancyLoad(2, 200)) process.exit(3);",
            )
        )
        subprocess.run([node], input=program, text=True, check=True)


@pytest.mark.parametrize(
    "html_name",
    ["_REGIONS_HTML", "_INDEX_CAM_HTML"],
)
def test_live_view_inline_javascript_is_valid(html_name):
    """Parse each changed inline script with the host JavaScript engine."""
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    web = _web_module()
    html = getattr(web, html_name)
    script = html.rsplit("<script>", 1)[1].split("</script>", 1)[0]
    subprocess.run([node, "--check"], input=script, text=True, check=True)


def test_every_navigation_target_is_a_page_with_the_sidebar():
    """The sidebar links must all resolve, and each page must carry it back."""
    from starlette.testclient import TestClient

    web = _web_module()
    app = web.make_app(registry=_registry_with_no_objects(), hub=None)
    client = TestClient(app)
    for href, label, _key in web._NAV_LINKS:
        response = client.get(href)
        assert response.status_code == 200, href
        assert label in response.text, href
        # Every link is present on every page, so any page reaches any other.
        for other_href, _label, _k in web._NAV_LINKS:
            assert f'href="{other_href}"' in response.text, (href, other_href)


def test_the_viewer_endpoint_says_why_there_is_no_viewer():
    """A blank frame cannot distinguish "no data" from "never started"."""
    from starlette.testclient import TestClient

    web = _web_module()
    app = web.make_app(registry=_registry_with_no_objects(), hub=None)
    payload = TestClient(app).get("/api/viewer").json()
    assert payload["url"] == ""
    assert payload["detail"]


def test_a_cross_site_form_post_is_refused():
    """A page on another site can POST text/plain without a preflight."""
    from starlette.testclient import TestClient

    web = _web_module()
    client = TestClient(web.make_app(registry=_registry_with_no_objects(), hub=None))
    forged = client.post("/api/objects/flush", content=b"{}",
                         headers={"content-type": "text/plain"})
    assert forged.status_code == 415
    assert client.post("/api/objects/flush", json={}).status_code != 415


def _registry_with_no_objects():
    """An empty registry, enough for the page-shape assertions above."""
    from scene_service.state import ObjectRegistry

    return ObjectRegistry()


# ── One port for the whole UI ───────────────────────────────────────────────
# rerun serves the viewer application on one port and each page's stream on
# another. The frame used to point straight at them, so reading the map from
# a laptop meant forwarding four ports and forgetting one produced a blank
# frame with no error. Scene proxies all of them under its own port.


class _FakeSink:
    """A sink that is up, with recognisable ports and nothing behind them."""

    ready = True
    # Layout asks `available` (does this deployment have a viewer at all) and
    # only the forwarding routes ask `ready` (is one running). A sink that is
    # up is both. See `RerunSink.available`.
    available = True
    detail = ""

    def data_port(self, page):
        return 55552 if page == "2d" else 55551


def _proxy_client():
    from starlette.testclient import TestClient

    web = _web_module()
    return TestClient(web.make_app(registry=_registry_with_no_objects(),
                                   hub=None, rerun_sink=_FakeSink()))


def test_the_data_source_is_on_scenes_own_origin():
    """rerun takes an absolute source URL whose path must be exactly `/proxy`;
    it has to name the host the reader reached, not a port scene bound."""
    client = _proxy_client()
    payload = client.get("/api/viewer").json()
    assert payload["url"] == "rerun+http://testserver/proxy"


def test_each_page_reaches_its_own_feed():
    """The two feeds share a path, so the asking page decides which one."""
    client = _proxy_client()
    two_d = client.get("/proxy",
                       headers={"referer": "http://h/rerun?view=2d"})
    assert "55552" in two_d.text, two_d.text
    three_d = client.get("/proxy",
                         headers={"referer": "http://h/rerun?view=3d"})
    assert "55551" in three_d.text, three_d.text
    bare = client.get("/proxy")
    assert "55551" in bare.text, "an unmarked request must get the 3D feed"


def test_the_proxy_says_so_when_there_is_no_viewer():
    """With no viewer the data path answers at once rather than hanging."""
    from starlette.testclient import TestClient

    web = _web_module()
    client = TestClient(web.make_app(registry=_registry_with_no_objects(),
                                     hub=None))
    assert client.get("/proxy").status_code == 404
    assert client.get("/api/viewer").json()["url"] == ""


def test_the_map_pages_carry_the_object_and_relation_list():
    """rerun draws the map; it knows nothing about the registry behind it."""
    client = _proxy_client()
    for href in ("/", "/2d"):
        page = client.get(href).text
        # The lists moved from the floating info panel into the docked
        # one when the landing page became the viewer's; the question they
        # answer is the same, so this asks for either.
        assert ('id="dock-objs"' in page or 'id="info-objs"' in page), href
        assert ('id="dock-rels"' in page or 'id="info-rels"' in page), href
        assert "/api/state" in page, href


def test_renamed_routes_and_fields_remain_as_deprecated_aliases():
    from starlette.testclient import TestClient
    from scene_service.state import BBox3D, Pose3D

    web = _web_module()
    reg = _registry_with_no_objects()
    reg.insert_object(
        label="chair", pose=Pose3D(x=1.0, y=1.0, z=0.0, yaw=0.0, frame_id="map"),
        bbox=BBox3D(size_x=.4, size_y=.4, size_z=.5, yaw=0.0, frame_id="map"),
        confidence=0.9, now=1000.0)
    client = TestClient(web.make_app(registry=reg, hub=None))
    for old, new in (("/user", "/regions"), ("/api/annotations", "/api/regions")):
        assert client.get(old).status_code == client.get(new).status_code
    assert client.get("/3d", follow_redirects=False).headers["location"] == "/"
    obj = client.get("/api/state").json()["objects"][0]
    assert obj["cls"] == obj["label"]
