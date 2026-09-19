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
import re
import urllib.error
import urllib.request

import pytest

BASE_URL = os.environ.get("SCENE_WEB_URL", "http://127.0.0.1:50107").rstrip("/")

# Every page the sidebar offers, with the text that proves the right one
# arrived rather than a generic shell.
PAGES = [
    ("/maps", "maps"),
    ("/", "semantic map"),
    ("/2d", "2D map"),
    ("/cam", "camera"),
    ("/regions", "regions"),
]

pytest.importorskip(
    "playwright.sync_api",
    reason="playwright is not installed; see this module's docstring",
)

from playwright.sync_api import expect, sync_playwright  # noqa: E402


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
    for entry in ("maps", "semantic map", "2d map", "camera", "regions"):
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

    # The pages are a sidebar shell around an iframe, so the panels are in the
    # child document: page.inner_text("body") sees the shell and reports an
    # empty camera page while the camera is plainly running. The panels also
    # label themselves only once a frame lands, so this waits for the data
    # rather than for a guessed delay, which would test the simulator's frame
    # rate instead of the page.
    inner = page.frame_locator("iframe").locator("body")
    expect(inner).to_contain_text(re.compile("rgb", re.I), timeout=20000)
    expect(inner).to_contain_text(re.compile("depth", re.I), timeout=20000)
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
    _goto(page, "/regions")
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


# ── The dock ───────────────────────────────────────────────────────────────
# Objects, relations and robot pose, in a column beside the view rather than
# over it. The overlay it replaces had to be moved twice because it covered
# something; the first test here is the one that would have caught both.


def test_dock_docks_rather_than_covers(page):
    """The dock is in the layout, not over it.

    Its predecessor floated, and twice ended up on top of something meant to
    be clicked -- the sidebar once, the view under it once. An overlay is
    always over something, so the property worth testing is geometric: no
    overlap with the view, and none with the sidebar.
    """
    _goto(page, "/")
    page.wait_for_timeout(1200)

    dock = page.locator("#dock").bounding_box()
    main = page.locator("main").bounding_box()
    nav = page.locator("nav").bounding_box()
    assert dock, "no dock on the semantic map page"

    assert dock["x"] >= main["x"] + main["width"] - 1, (
        f"dock at x={dock['x']} overlaps the view ending at "
        f"{main['x'] + main['width']}"
    )
    assert dock["x"] >= nav["x"] + nav["width"], "dock overlaps the sidebar"


def test_dock_tabs_switch_panes(page):
    """One pane at a time, and the tab strip is what switches them."""
    _goto(page, "/")
    page.wait_for_timeout(1000)

    for tab in ("objects", "relations", "robot"):
        page.locator(f'.tabs button[data-tab="{tab}"]').click()
        page.wait_for_timeout(200)
        shown = page.locator(".dock .pane.on")
        assert shown.count() == 1, f"{tab}: {shown.count()} panes visible, want 1"
        assert shown.get_attribute("data-pane") == tab, (
            f"clicking {tab} showed {shown.get_attribute('data-pane')}"
        )


def test_dock_collapses_and_the_tabs_bring_it_back(page):
    """Collapsing keeps the tab strip, so reopening is the same click that
    switches panes. A separate 'show' button parked in a corner is what the
    old panel needed, and it was easy to lose."""
    _goto(page, "/")
    page.wait_for_timeout(1000)

    wide = page.locator("#dock").bounding_box()["width"]
    page.locator("#dock-shut").click()
    page.wait_for_timeout(250)
    shut = page.locator("#dock").bounding_box()["width"]
    assert shut < wide, f"collapse did not narrow the dock ({shut} vs {wide})"
    assert page.locator('.tabs button[data-tab="objects"]').is_visible(), (
        "collapsing hid the tabs, leaving no way back"
    )

    page.locator('.tabs button[data-tab="relations"]').click()
    page.wait_for_timeout(250)
    assert page.locator("#dock").bounding_box()["width"] > shut, (
        "clicking a tab on the collapsed rail did not reopen the dock"
    )
    assert page.locator(".dock .pane.on").get_attribute("data-pane") == "relations"


def test_dock_lists_the_registry(page):
    """The dock exists to say what scene actually holds. The map is drawn by
    rerun, which knows nothing about the registry behind it."""
    _goto(page, "/")
    expect(page.locator("#dock-objs")).to_contain_text(
        re.compile(r"robot|_\d", re.I), timeout=20000)
    expect(page.locator("#dock-stamp")).to_contain_text(
        re.compile(r"\d+ obj"), timeout=20000)


def test_uncertain_objects_look_uncertain(page):
    """Scene's perception is not accurate enough to present every hit flatly.
    A row below the confidence threshold is marked, so going back for a second
    look reads as the next step rather than as doubt about the whole list."""
    _goto(page, "/")
    page.wait_for_timeout(1500)

    marked = page.evaluate("""() => {
      const rows = [...document.querySelectorAll('#dock-objs tr')];
      return rows.map(r => {
        const pp = r.querySelector('td.pp');
        const c = pp ? parseFloat((pp.textContent.split('·')[1] || '1')) : null;
        return {c: c, unsure: r.classList.contains('unsure')};
      }).filter(r => r.c !== null && !Number.isNaN(r.c));
    }""")
    for row in marked:
        assert row["unsure"] == (row["c"] < 0.55), (
            f"confidence {row['c']} marked unsure={row['unsure']}"
        )


def test_sidebar_entries_carry_an_icon(page):
    """Four words in a column give the eye nothing to aim at."""
    _goto(page, "/")
    links = page.locator("nav a")
    assert links.count() >= 4, "sidebar lost its links"
    for i in range(links.count()):
        assert links.nth(i).locator("svg.ico").count() == 1, (
            f"sidebar link {i} has no icon"
        )


def test_the_shell_is_not_monospace(page):
    """Monospace is for data whose columns line up. Navigation, headings and
    buttons on a terminal face read as a terminal, not as an interface."""
    _goto(page, "/regions")
    face = page.evaluate(
        "getComputedStyle(document.querySelector('nav a')).fontFamily")
    assert "mono" not in face.lower(), f"sidebar is still monospace: {face}"


# ── Map binding ────────────────────────────────────────────────────────────
# Everything on the other pages is about one map. Which one is never implicit.


def test_maps_page_owns_map_management(page):
    """Naming and saving a map, loading one, and re-estimating the pose are
    map operations and live on the maps page. They used to sit above the
    region drawing button, which said they were the same kind of thing."""
    _goto(page, "/maps")
    page.wait_for_timeout(900)

    frame = page.frame_locator("iframe")
    for control in ("#map-id", "#btn-save-map", "#btn-pose-estimate"):
        assert frame.locator(control).is_visible(), (
            f"maps page is missing {control}"
        )
    assert not frame.locator("#btn-draw").is_visible(), (
        "the maps page still offers region drawing"
    )


def test_regions_page_keeps_only_region_marking(page):
    """And the reverse: the regions page does not manage maps."""
    _goto(page, "/regions")
    page.wait_for_timeout(900)

    frame = page.frame_locator("iframe")
    assert not frame.locator("#map-id").is_visible(), (
        "the regions page still carries the map form"
    )


def test_marking_is_gated_on_a_bound_map(page):
    """A region belongs to a map. With nothing bound the control is absent,
    not disabled, and what stands in its place says where to go -- failing at
    save time is how orphan regions were made."""
    _goto(page, "/regions")
    page.wait_for_timeout(2500)

    frame = page.frame_locator("iframe")
    unbound = frame.locator("body.unbound").count() > 0

    if unbound:
        gate = frame.locator("#map-gate")
        assert gate.is_visible(), "nothing is bound and no gate was shown"
        assert "maps" in gate.inner_text().lower(), (
            "the gate does not say where to go"
        )
        assert not frame.locator("#btn-draw").is_visible(), (
            "marking is offered with no map bound"
        )
    else:
        assert frame.locator("#btn-draw").is_visible(), (
            "a map is bound but marking is not offered"
        )


def test_the_bound_map_is_named_on_the_page(page):
    """Which map the thing in front of you describes is never left to be
    inferred from the map drawing itself."""
    for path in ("/maps", "/regions"):
        _goto(page, path)
        page.wait_for_timeout(2000)
        pill = page.frame_locator("iframe").locator("#bound-pill")
        assert pill.is_visible(), f"{path} does not name the bound map"
        assert pill.inner_text().strip(), f"{path} binding pill is empty"


# ── The 2D map ─────────────────────────────────────────────────────────────


def test_2d_map_draws_no_point_cloud(page):
    """The 2D page is a floor plan, not the 3D recording seen from above.

    It used to be rerun's top-down view, which draws the point clouds; from
    above a point cloud is a smear over exactly the thing a plan is opened
    for. The built-in renderer draws the occupancy grid and one dot per
    object, so the check is that the page is the canvas and not the viewer.
    """
    _goto(page, "/2d")
    page.wait_for_timeout(2500)
    frame = page.frame_locator("iframe")
    assert frame.locator("canvas#c").count() == 1, (
        "the 2D page is not the built-in canvas renderer"
    )


def test_2d_labels_do_not_overlap(page):
    """Two objects a few centimetres apart -- a cup and a monitor on one table
    -- printed their names over each other, and neither could be read. The
    placement pass is annealing over candidate positions (Christensen, Marks &
    Shieber 1995); this asserts the property it exists to provide."""
    _goto(page, "/2d")
    page.wait_for_timeout(4000)

    boxes = page.frame_locator("iframe").locator("canvas#c").evaluate(
        "() => (window.lblCache && window.lblCache.boxes) || []")
    if len(boxes) < 2:
        pytest.skip(f"only {len(boxes)} labels on the map; nothing to collide")

    worst = 0.0
    for i, a in enumerate(boxes):
        for b in boxes[i + 1:]:
            dx = min(a["x"] + a["w"], b["x"] + b["w"]) - max(a["x"], b["x"])
            dy = min(a["y"] + a["h"], b["y"] + b["h"]) - max(a["y"], b["y"])
            if dx > 0 and dy > 0:
                worst = max(worst, dx * dy / (a["w"] * a["h"]))
    assert worst < 0.12, (
        f"labels overlap by {worst:.0%} of a label's area; placement failed"
    )

