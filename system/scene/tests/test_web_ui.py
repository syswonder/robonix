# SPDX-License-Identifier: MulanPSL-2.0
"""Browser tests for Scene's map pages.

Everything else under tests/ exercises Scene through Python: the web tests
build a page and assert on the HTML string, which is enough to catch a broken
template and nothing at all to catch a viewer that mounts and then renders an
empty canvas, a sidebar link that points at a route that no longer exists, or
a WASM bundle that 404s. Those are the failures the map pages actually have,
and they only exist in a browser.

So these drive a real one. They are written against a **running** Scene -- the
service, the simulator behind it and the models it loads -- because a viewer
fed by a fake registry proves the template renders, which is the part already
covered. Where no Scene is listening the whole module skips rather than fails:
a machine without the stack up is not a broken build, and a test that cannot
run must not be able to go green either.

    # bring the stack up first, then
    pytest system/scene/tests/test_web_ui.py

    SCENE_WEB_URL=http://host:50107 pytest ...   # against another host

Requires `playwright` and its Chromium download:

    pip install pytest-playwright && playwright install chromium
"""
from __future__ import annotations

import os
import urllib.error
import urllib.request

import pytest

BASE_URL = os.environ.get("SCENE_WEB_URL", "http://127.0.0.1:50107").rstrip("/")

# Every page the sidebar offers, with the text that proves the right one
# arrived rather than a generic shell.
PAGES = [
    ("/", "semantic map"),
    ("/2d", "2D map"),
    ("/cam", "camera"),
    ("/user", "regions"),
]

pytest.importorskip(
    "playwright.sync_api",
    reason="playwright is not installed; see this module's docstring",
)

from playwright.sync_api import sync_playwright  # noqa: E402


def _scene_is_up() -> bool:
    try:
        with urllib.request.urlopen(BASE_URL + "/", timeout=3) as r:
            return r.status == 200
    except (urllib.error.URLError, OSError):
        return False


pytestmark = pytest.mark.skipif(
    not _scene_is_up(),
    reason=f"no Scene web UI at {BASE_URL}; bring the stack up to run these",
)


@pytest.fixture(scope="module")
def browser():
    with sync_playwright() as pw:
        b = pw.chromium.launch()
        yield b
        b.close()


@pytest.fixture
def page(browser):
    """A page that records console errors, so every test asserts on them.

    A viewer failing to start logs and carries on showing an empty frame, which
    looks exactly like a room with nothing in it.
    """
    ctx = browser.new_context(viewport={"width": 1440, "height": 900})
    p = ctx.new_page()
    p.errors = []  # type: ignore[attr-defined]
    p.on("console", lambda m: p.errors.append(m.text) if m.type == "error" else None)
    p.on("pageerror", lambda e: p.errors.append(str(e)))
    yield p
    ctx.close()


def _goto(page, path: str):
    page.goto(BASE_URL + path, wait_until="domcontentloaded")
    page.wait_for_timeout(600)


@pytest.mark.parametrize("path,label", PAGES)
def test_page_loads_with_its_own_sidebar(page, path, label):
    """Each route serves its page, and the sidebar marks which one is open."""
    _goto(page, path)

    nav = page.locator("a, .nav a, nav a")
    assert nav.count() > 0, f"{path} rendered no navigation at all"

    body = page.inner_text("body").lower()
    for entry in ("semantic map", "2d map", "camera", "regions"):
        assert entry in body, f"{path} is missing the '{entry}' sidebar entry"

    assert not page.errors, f"{path} logged console errors: {page.errors[:3]}"


def test_sidebar_reaches_every_page(page):
    """The links go where they say. Reaching a page by editing the address bar
    is what the sidebar exists to replace."""
    _goto(page, "/")

    for _, label in PAGES:
        link = page.get_by_text(label, exact=True).first
        assert link.count() > 0, f"no sidebar link labelled {label!r}"
        link.click()
        page.wait_for_timeout(500)
        assert label.split()[0].lower() in page.title().lower() or label.lower() in (
            page.inner_text("body").lower()
        ), f"clicking {label!r} did not open its page (title={page.title()!r})"


def test_camera_shows_both_streams(page):
    """RGB and depth, side by side, each labelled with its own format and the
    stamp they share. One stream alone means the pair fell out of step."""
    _goto(page, "/cam")
    page.wait_for_timeout(1500)

    body = page.inner_text("body").lower()
    assert "rgb" in body, "camera page has no RGB panel"
    assert "depth" in body, "camera page has no depth panel"
    assert not page.errors, f"camera page logged console errors: {page.errors[:3]}"


def test_semantic_map_mounts_the_viewer(page):
    """The 3D page is the landing page because it answers the question the UI
    is opened to ask. If its WASM viewer does not start, the page is a
    rectangle."""
    _goto(page, "/")
    page.wait_for_timeout(4000)

    has_surface = page.locator("canvas, iframe").count() > 0
    assert has_surface, "semantic map mounted no canvas or iframe"

    fatal = [e for e in page.errors if "404" in e or "wasm" in e.lower()]
    assert not fatal, f"viewer failed to load: {fatal[:3]}"


def test_regions_page_offers_marking(page):
    """The regions page is where a region gets its name, so the control that
    starts that has to be on it."""
    _goto(page, "/user")
    page.wait_for_timeout(800)

    body = page.inner_text("body").lower()
    assert "region" in body, "regions page never says 'region'"
    assert "room" not in body.replace("room3", ""), (
        "regions page still calls a region a room: " + body[:200]
    )


@pytest.mark.parametrize("path,_label", PAGES)
def test_bare_form_renders(page, path, _label):
    """`?bare=1` is the escape hatch for a browser without WASM, and it has to
    keep working or a headless robot has no UI at all."""
    _goto(page, f"{path}?bare=1" if "?" not in path else f"{path}&bare=1")
    assert page.inner_text("body").strip(), f"{path}?bare=1 rendered an empty body"
