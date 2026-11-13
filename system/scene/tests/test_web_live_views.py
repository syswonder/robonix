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

    def fake_encode(msg, *, kind, width=0, quality=0):
        """Return a deterministic encoded record while tracking invocations."""
        calls.append((msg, kind))
        return {
            "width": 2,
            "height": 1,
            "encoding": "rgb8",
            "stamp_ms": 0,
            "png_b64": f"encoded-{msg}",
        }

    monkeypatch.setattr(web, "_image_to_base64", fake_encode)
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

    def unsupported(msg, *, kind, width=0, quality=0):
        calls.append((msg, kind))
        return None

    monkeypatch.setattr(web, "_image_to_base64", unsupported)
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

    def slow_payload(request_hub, *, width=0, quality=0):
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


def test_camera_preview_refresh_interval_includes_encoding_time(monkeypatch):
    import time
    import httpx

    web = _web_module()
    calls = []

    def encode(hub, *, width=0, quality=0):
        calls.append(hub)
        time.sleep(.04)
        return {"rgb": {"stamp_ms": len(calls)}, "depth": None}

    monkeypatch.setattr(web, "_camera_payload", encode)
    monkeypatch.setattr(web, "_CAMERA_PREVIEW_MIN_INTERVAL_S", .1)
    app = web.make_app(registry=object(), hub=object())

    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),
                                     base_url="http://scene.test") as client:
            first = await client.get("/api/camera")
            await asyncio.sleep(.07)
            second = await client.get("/api/camera")
            return first.json(), second.json()

    first, second = asyncio.run(scenario())
    assert first["rgb"]["stamp_ms"] == 1
    assert second["rgb"]["stamp_ms"] == 2


def test_camera_preview_controls_resize_encode_and_invalidate_same_frame_cache():
    import base64
    from io import BytesIO
    from types import SimpleNamespace

    from PIL import Image
    from starlette.testclient import TestClient

    web = _web_module()
    msg = SimpleNamespace(width=1024, height=576, encoding="bgr8",
                          data=bytes([20, 60, 200]) * (1024 * 576))
    hub = _FakeHub({"rgb": (msg, 12.5, 1)})
    client = TestClient(web.make_app(registry=object(), hub=hub))

    def decode(query, field):
        response = client.get("/api/camera" + query)
        assert response.status_code == 200
        payload = response.json()["rgb"]
        assert payload["source_width"] == 1024 and payload["source_height"] == 576
        return payload, Image.open(BytesIO(base64.b64decode(payload[field])))

    low, low_image = decode("?width=320&quality=40&fps=5", "jpeg_b64")
    assert (low["width"], low["height"]) == low_image.size == (320, 180)
    assert low_image.format == "JPEG"
    assert all(abs(actual - expected) <= 4
               for actual, expected in zip(low_image.getpixel((0, 0)), (200, 60, 20)))
    _, high_image = decode("?width=320&quality=95&fps=1", "jpeg_b64")
    assert sum(high_image.quantization[0]) < sum(low_image.quantization[0])
    resized, _ = decode("?width=640&quality=75&fps=2", "jpeg_b64")
    assert (resized["width"], resized["height"]) == (640, 360)
    _, original = decode("", "png_b64")
    assert original.format == "PNG" and original.size == (1024, 576)
    _, unscaled = decode("?width=1920&quality=75", "jpeg_b64")
    assert unscaled.size == (1024, 576)


@pytest.mark.parametrize("query", ["width=99999", "quality=5", "fps=nan", "fps=0"])
def test_camera_preview_rejects_invalid_settings(query):
    from starlette.testclient import TestClient

    web = _web_module()
    client = TestClient(web.make_app(registry=object(), hub=None))
    assert client.get("/api/camera?" + query).status_code == 400


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


def test_camera_battery_display_handles_fresh_stale_and_failed_readings():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    html = _web_module()._INDEX_CAM_HTML
    function = html.split('    async function updateBattery()', 1)[1].split(
        "    if ('__BATTERY_ENABLED__'", 1)[0]
    program = """
const assert = require('node:assert/strict');
const badge = {};
const document = {getElementById: () => badge};
const t = key => key;
const tv = (key, values) => key + ':' + values.percent;
const setTimeout = () => 1, clearTimeout = () => {};
let reading;
const fetch = async () => {
  if (reading instanceof Error) throw reading;
  return {ok: true, json: async () => reading};
};
async function updateBattery()""" + function + """
(async () => {
  for (const [value, text, style] of [
    [{available:true,percent:71}, 'camera.battery:71', 'fresh'],
    [{available:true,percent:0,charging:true}, 'camera.battery:0 · camera.charging', 'low'],
    [{available:false,percent:71}, 'camera.batteryStale:71', ''],
    [{available:true,percent:null}, 'camera.batteryUnavailable', ''],
    [new Error('offline'), 'camera.batteryUnavailable', '']
  ]) {
    reading = value;
    await updateBattery();
    assert.equal(badge.textContent, text);
    assert.equal(badge.className, style);
    assert.equal(badge.hidden, false);
  }
})().catch(error => { console.error(error); process.exit(1); });
"""
    subprocess.run([node], input=program, text=True, check=True)


def test_every_navigation_target_preserves_its_map_type():
    from html.parser import HTMLParser
    from starlette.testclient import TestClient

    class Navigation(HTMLParser):
        def __init__(self, page):
            super().__init__()
            self.inside = False
            self.links = []
            self.feed(page)

        def handle_starttag(self, tag, attrs):
            if tag == "nav":
                self.inside = True
            if tag == "a" and self.inside:
                values = dict(attrs)
                if "id" not in values:
                    self.links.append(values["href"])

        def handle_endtag(self, tag):
            if tag == "nav":
                self.inside = False

    web = _web_module()
    client = TestClient(web.make_app(registry=_registry_with_no_objects(), hub=None))
    expected_outdoor = {"/outdoor", "/maps?map_type=outdoor", "/regions?map_type=outdoor",
                        "/cam?map_type=outdoor", "/logs?map_type=outdoor"}
    for href, _label, _key in web._NAV_LINKS:
        response = client.get(href)
        assert response.status_code == 200
        links = set(Navigation(response.text).links)
        if href == "/outdoor":
            assert links == expected_outdoor
        else:
            assert links == {entry[0] for entry in web._NAV_LINKS}
    for target in expected_outdoor:
        response = client.get(target)
        assert response.status_code == 200
        assert set(Navigation(response.text).links) == expected_outdoor


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


def test_camera_viewer_uses_scene_origin_and_limits_forwarded_operations():
    import json
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from starlette.testclient import TestClient

    calls = []

    class CameraViewer(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def do_GET(self):
            calls.append((self.command, self.path))
            payload = b'camera-preview' if self.path.startswith('/frame') else b'{"cameras":{"front":{"available":true}}}'
            if self.path == '/api/battery':
                payload = b'{"percent":71,"available":true,"charging":false}'
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg' if self.path.startswith('/frame') else 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_POST(self):
            calls.append((self.command,self.path))
            payload = self.rfile.read(int(self.headers['Content-Length']))
            self.send_response(200)
            self.send_header('Content-Type','application/json')
            self.end_headers()
            self.wfile.write(payload)

    server = ThreadingHTTPServer(('127.0.0.1', 0), CameraViewer)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    web = _web_module()
    try:
        client = TestClient(web.make_app(registry=_registry_with_no_objects(),camera_viewer_port=server.server_port))
        response = client.get('/camera-viewer/api/cameras')
        assert response.json()['cameras']['front']['available']
        frame = client.get('/camera-viewer/frame?cam=front&t=1')
        assert frame.content == b'camera-preview' and frame.headers['content-type'] == 'image/jpeg'
        settings = client.post('/camera-viewer/api/settings',json={'level':55})
        assert settings.json() == {'level':55}
        assert client.post('/camera-viewer/frame',json={}).status_code == 404
        assert client.get('/camera-viewer/admin').status_code == 404
        assert client.post('/camera-viewer/api/settings',json={'x':'x'*4096}).status_code == 413
        assert calls == [('GET','/api/cameras'),('GET','/frame?cam=front&t=1'),('POST','/api/settings')]
        battery_only = TestClient(web.make_app(
            registry=_registry_with_no_objects(), camera_battery_port=server.server_port))
        assert battery_only.get('/camera-viewer/api/battery').json() == {
            'percent': 71, 'available': True, 'charging': False}
        assert battery_only.get('/camera-viewer/').status_code == 404
        assert battery_only.get('/camera-viewer/frame').status_code == 404
        assert battery_only.post('/camera-viewer/api/battery', json={}).status_code == 404
        disabled = TestClient(web.make_app(registry=_registry_with_no_objects()))
        assert disabled.get('/camera-viewer/api/cameras').status_code == 404
        assert disabled.get('/camera-viewer/api/battery').status_code == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
