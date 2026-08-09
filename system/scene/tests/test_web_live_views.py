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
    for html in (web._INDEX_HTML, web._USER_HTML):
        assert '<body data-ready="loading">' in html
        assert 'role="status"' in html
        assert "document.body.dataset.ready = value" in html
        assert "c.clientWidth > 0 && c.clientHeight > 0" in html
        assert "visibilitychange" in html
        assert "Scene state unavailable" in html
        assert "Scene map image could not be decoded" in html
        assert html.count("if (!isCurrentOccupancyLoad(loadToken, meta.stamp_ms)) return;") == 2

    assert "if (currentState) draw(currentState);" in web._INDEX_HTML
    assert (
        "occStamp = meta.stamp_ms;\n            occLoading = 0;\n            draw();"
        in web._USER_HTML
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
    for html in (web._INDEX_HTML, web._USER_HTML):
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
    ["_INDEX_HTML", "_USER_HTML", "_INDEX_CAM_HTML", "_INDEX_3D_HTML"],
)
def test_live_view_inline_javascript_is_valid(html_name):
    """Parse each changed inline script with the host JavaScript engine."""
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    web = _web_module()
    html = getattr(web, html_name)
    if html_name == "_INDEX_3D_HTML":
        script = html.split('<script type="module">', 1)[1].split("</script>", 1)[0]
        args = [node, "--input-type=module", "--check"]
    else:
        script = html.rsplit("<script>", 1)[1].split("</script>", 1)[0]
        args = [node, "--check"]
    subprocess.run(args, input=script, text=True, check=True)
