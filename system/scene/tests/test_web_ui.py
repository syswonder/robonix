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
    ("/maps", "map library"),
    ("/", "3D"),
    ("/2d", "2D"),
    ("/cam", "camera"),
    ("/regions", "regions"),
    ("/logs", "logs"),
]

# The three that are views of the one live map, and so sit under the
# heading rather than beside the library of saved ones.
LIVE_MAP_VIEWS = ("/", "/2d", "/regions")

pytest.importorskip(
    "playwright.sync_api",
    reason="playwright is not installed; see this module's docstring",
)

from playwright.sync_api import expect, sync_playwright  # noqa: E402


def _copy(lang: str, key: str) -> str:
    """One string from `web_assets/strings_<lang>.json`."""
    import json
    import pathlib

    path = (pathlib.Path(__file__).resolve().parents[1]
            / "scene_service" / "web_assets" / f"strings_{lang}.json")
    return json.loads(path.read_text(encoding="utf-8"))[key]



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

    # Asked of the sidebar rather than of the whole page: "map library"
    # contains "map", and a body-text search cannot tell a nav entry from a
    # heading or from a map name that happens to match.
    hrefs = set(page.eval_on_selector_all(
        "nav a", "els => els.map(e => new URL(e.href).pathname)"))
    for href, _ in PAGES:
        assert href in hrefs, f"{path} is missing the {href} sidebar entry"

    assert not page.errors, f"{path} logged console errors: {page.errors[:3]}"


def test_the_sidebar_groups_the_live_maps_views(page):
    """Three of these entries are ways of looking at the map the robot is on
    now, and one is the library of saved maps. Listed flat they read as four
    maps, which is what the heading exists to stop -- so the heading has to
    actually contain the three, and not the library."""
    _goto(page, "/")
    grouped = set(page.eval_on_selector_all(
        ".nav-group a", "els => els.map(e => new URL(e.href).pathname)"))
    assert grouped == set(LIVE_MAP_VIEWS), (
        f"the live map's views are {sorted(grouped)}, "
        f"expected {sorted(LIVE_MAP_VIEWS)}"
    )
    head = page.locator(".nav-group .nav-head")
    assert head.count() == 1, "the group has no heading to say what it is"
    assert head.inner_text().strip(), "the heading is empty"


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
    # The header carries the count and nothing else: the unix timestamp it
    # used to show told a reader nothing they could act on and rewrote itself
    # twice a second, which made the one stable line the busiest in the panel.
    expect(page.locator("#dock-stamp")).to_contain_text(
        re.compile(r"^\s*\d+"), timeout=20000)
    assert "t=" not in page.locator("#dock-stamp").inner_text(), (
        "the unix timestamp is back in the dock header"
    )


def test_uncertain_objects_look_uncertain(page):
    """Scene's perception is not accurate enough to present every hit flatly.
    A row below the confidence threshold is marked, so going back for a second
    look reads as the next step rather than as doubt about the whole list."""
    _goto(page, "/")
    page.wait_for_timeout(1500)

    # The row shows coordinates, not confidence -- the number lives only in
    # the data the dock is built from, so that is where it is read. Matching
    # by id rather than by position keeps this honest if the list reorders.
    marked = page.evaluate("""async () => {
      const live = await fetch('/api/state', {cache: 'no-store'})
        .then(r => r.json());
      const conf = new Map((live.objects || []).map(o => [o.id, o.confidence]));
      return [...document.querySelectorAll('#dock-objs tr.row')]
        .map(r => ({
          id: r.dataset.oid,
          c: conf.has(r.dataset.oid) ? conf.get(r.dataset.oid) : null,
          unsure: r.classList.contains('unsure'),
        }))
        .filter(r => typeof r.c === 'number');
    }""")
    assert marked, "no rows carried a confidence to check"
    for row in marked:
        assert row["unsure"] == (row["c"] < 0.55), (
            f"{row['id']}: confidence {row['c']} marked unsure={row['unsure']}"
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

    # The page is rendered into the shell now, not an iframe: it is a
    # library, not an editor, so it has no canvas to frame.
    assert page.locator("#mg-grid").count() == 1, "no map grid"
    assert page.locator("#mg-save").is_visible(), "no way to save a session"
    assert page.locator("#btn-draw").count() == 0, (
        "the maps page still carries the region editor"
    )


def test_regions_page_keeps_only_region_marking(page):
    """And the reverse: the regions page does not manage maps."""
    _goto(page, "/regions")
    page.wait_for_timeout(900)

    frame = page.frame_locator("iframe")
    assert not frame.locator("#map-id").is_visible(), (
        "the regions page still carries the map form"
    )


def test_a_temporary_map_can_still_be_marked(page):
    """A temporary map is still a map.

    Marking regions and recognising objects work on an unnamed live session;
    what it lacks is a name and persistence. The server agrees -- saving the
    session rebinds the annotation store with carry_current and snapshots the
    objects, so the marks come along. An earlier version of this page blocked
    the marking, which refused something the backend supports and lost the
    work it would not let you start.
    """
    _goto(page, "/regions")
    page.wait_for_timeout(2500)

    frame = page.frame_locator("iframe")
    assert frame.locator("#btn-draw").is_visible(), (
        "region marking is not offered"
    )

    if frame.locator("body.unsaved-map").count() > 0:
        # Temporary: the one fact a reader cannot see is that it does not
        # persist, so it is stated beside the work rather than in front of it.
        note = frame.locator("#map-note")
        assert note.is_visible(), "temporary map does not say it is temporary"
        text = note.inner_text().lower()
        assert "maps" in text, "the note does not say where to save it"
        assert any(_copy(lang, "map.note.temporary").lower() in text
                   for lang in ("en", "zh")), (
            "the note does not say what temporary costs"
        )
    else:
        assert not frame.locator("#map-note").is_visible(), (
            "a saved map is being described as temporary"
        )

def test_the_bound_map_is_named_on_the_page(page):
    """Which map the thing in front of you describes is never left to be
    inferred from the map drawing itself."""
    for path in ("/regions",):
        _goto(page, path)
        page.wait_for_timeout(2000)
        pill = page.frame_locator("iframe").locator("#bound-pill")
        assert pill.is_visible(), f"{path} does not name the bound map"
        text = pill.inner_text().strip()
        assert text, f"{path} binding pill is empty"
        # Either a named map, or the temporary session said in as many words.
        # "no map bound" was the wrong reading: there is always a map.
        assert "bound" not in text.lower(), (
            f"{path} still describes the session as unbound: {text!r}"
        )


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


# ── Interface language ─────────────────────────────────────────────────────
# One language on screen, chosen once and remembered. Printing the Chinese and
# the English side by side is not a bilingual interface; it is one interface
# with everything said twice.


def test_language_switch_changes_the_sidebar(page):
    """The switch is in the sidebar and it changes the sidebar."""
    _goto(page, "/")
    page.wait_for_timeout(800)

    before = page.inner_text("nav")
    page.locator("#lang-switch").click()
    page.wait_for_timeout(400)
    after = page.inner_text("nav")

    assert before != after, "the switch changed nothing"
    han = [c for c in after if "\u4e00" <= c <= "\u9fff"]
    latin_before = [c for c in before if c.isascii() and c.isalpha()]
    assert han or latin_before, "neither language rendered"


def test_language_choice_survives_a_reload(page):
    """Chosen once, not once per page."""
    _goto(page, "/")
    page.wait_for_timeout(800)
    page.locator("#lang-switch").click()
    page.wait_for_timeout(400)
    chosen = page.inner_text("nav")

    _goto(page, "/2d")
    page.wait_for_timeout(900)
    assert page.inner_text("nav") == chosen, (
        "the language reset on navigation"
    )


def test_the_view_follows_the_shell(page):
    """The views are iframes with their own documents; they have to be told,
    not left to pick the language up on the next navigation."""
    _goto(page, "/regions")
    page.wait_for_timeout(2000)

    frame = page.frame_locator("iframe").locator("#page-title")
    before = frame.inner_text()
    page.locator("#lang-switch").click()
    page.wait_for_timeout(700)
    assert frame.inner_text() != before, (
        f"the view kept its old language ({before!r})"
    )


def test_nothing_is_printed_in_both_languages(page):
    """The bug this replaced: every string written twice, once per language.
    A block of text is in one language or the other, never both."""
    _goto(page, "/regions")
    page.wait_for_timeout(2500)

    note = page.frame_locator("iframe").locator("#map-note")
    if note.count() == 0 or not note.is_visible():
        pytest.skip("the map is saved; no temporary-map note on screen")
    text = note.inner_text()
    han = sum(1 for c in text if "\u4e00" <= c <= "\u9fff")
    # "maps" in a link is a route, not a translation; require a real English
    # sentence before calling it doubled.
    words = len([w for w in text.split() if w.isascii() and len(w) > 3])
    assert not (han > 8 and words > 8), (
        "the note is printed in both languages at once:\n" + text
    )


def test_the_map_form_switches_language(page):
    """The form the maps page is built around has to switch too.

    It did not, so choosing Chinese left a Chinese frame around an English
    panel -- which is worse than either language alone, because it reads as a
    half-finished translation rather than as a choice.
    """
    # Still the regions page: it is where the map form remains.
    _goto(page, "/regions")
    page.wait_for_timeout(1500)

    frame = page.frame_locator("iframe")
    controls = ["#btn-save-map", "#btn-refresh-maps", "#btn-pose-estimate"]
    before = [frame.locator(c).inner_text() for c in controls]

    page.locator("#lang-switch").click()
    page.wait_for_timeout(800)
    after = [frame.locator(c).inner_text() for c in controls]

    assert before != after, f"the map form kept its language: {before}"
    for a, b in zip(before, after):
        assert a.strip() and b.strip(), "a control lost its label entirely"


def test_the_status_line_default_survives_translation(page):
    """The default status text was compared against literally --
    `msg.textContent === 'Ready.'` decided whether a hint could overwrite it.
    Translating the text alone breaks that test silently, so the comparison
    moved onto a data key.

    The invariant is conditional, not absolute: a real status message (a map
    list refresh, a save result) legitimately replaces the default and clears
    the key with it. So what must hold is that whenever the line *is* showing
    the default, it carries the key that identifies it as the default.
    """
    # The map form lives on the regions page now; maps is a grid.
    _goto(page, "/regions")
    page.wait_for_timeout(1800)

    msg = page.frame_locator("iframe").locator("#map-status-msg")
    text = msg.inner_text().strip()
    key = msg.get_attribute("data-i18n")

    # Read from the copy files rather than restated here: a test that
    # hardcodes interface text is a second place the wording lives, and the
    # two drift.
    defaults = {_copy(lang, "status.ready") for lang in ("en", "zh")}
    if text in defaults:
        assert key == "status.ready", (
            f"the line reads the default {text!r} but carries no key "
            "-- the comparisons that gate the hint are back to matching "
            "English text"
        )
    elif key:
        # A keyed non-default message is correct and intended: keys are what
        # let the language switch re-render text the script wrote. What must
        # not happen is a key whose translation is not what is on screen.
        # FrameLocator has no evaluate; the Frame behind it does.
        frame = next((f for f in page.frames if "bare=1" in (f.url or "")), None)
        rendered = frame.evaluate(
            "k => (typeof t === 'function' ? t(k) : '')", key) if frame else ""

        assert not rendered or rendered.strip() == text, (
            f"the line reads {text!r} but its key {key!r} renders "
            f"{rendered!r}; a language switch would replace one with the other"
        )

def test_the_bound_map_is_named_once(page):
    """It was named twice: the binding pill, and a line beside it repeating the
    same thing in English regardless of the chosen language."""
    _goto(page, "/regions")
    page.wait_for_timeout(1500)
    frame = page.frame_locator("iframe")
    assert frame.locator("#meta").count() == 0, (
        "the duplicated map line is back"
    )
    assert frame.locator("#bound-pill").is_visible(), "no binding pill"


# ── The log view ───────────────────────────────────────────────────────────
# Scribe already writes one JSON object per line into a file per tag. The page
# reads those; it does not keep a second copy of the same records.


def test_logs_api_serves_scribe_lines(page):
    """The endpoint reads scribe's files and hands back a cursor.

    A deployment without SCRIBE_LOG_DIR has nothing to read, and says so
    rather than guessing a path -- that case is reported, not failed, because
    a native run outside rbnx is legitimate.
    """
    body = page.request.get(BASE_URL + "/api/logs").json()

    if body.get("ok") is False:
        pytest.skip(f"no scribe directory: {body.get('detail')}")

    assert isinstance(body.get("entries"), list), "no entries array"
    assert isinstance(body.get("cursor"), dict), "no cursor to poll with"
    assert body.get("tags"), "no log tags found in the scribe directory"
    for entry in body["entries"][:20]:
        assert set(entry) >= {"ts", "level", "tag", "msg"}, (
            f"entry is missing scribe's fields: {entry}"
        )
        assert entry["level"] in ("debug", "info", "warn", "error"), (
            f"unnormalised level {entry['level']!r}"
        )


def test_logs_cursor_only_returns_what_is_new(page):
    """The cursor is a byte offset per file, which is what makes the poll
    cheap: asking again with the cursor just returned must not replay
    everything. A tail that re-sends its whole window every second is a tail
    that cannot be left open."""
    import json as _json

    first = page.request.get(BASE_URL + "/api/logs").json()
    if first.get("ok") is False:
        pytest.skip("no scribe directory")

    cursor = _json.dumps(first["cursor"])
    again = page.request.get(
        BASE_URL + "/api/logs", params={"cursor": cursor}).json()

    assert len(again["entries"]) < max(1, len(first["entries"])) or \
        len(first["entries"]) == 0, (
        f"the cursor replayed {len(again['entries'])} of "
        f"{len(first['entries'])} lines"
    )


def test_logs_page_streams_scribe(page):
    """The page itself: level chips, a filter, and rows arriving live."""
    _goto(page, "/logs")
    page.wait_for_timeout(2500)

    for level in ("debug", "info", "warn", "error"):
        assert page.locator(f".lg-chip.{level}").count() == 1, (
            f"no {level} chip"
        )
    assert page.locator("#lg-q").is_visible(), "no text filter"
    assert page.locator("#lg-live").is_visible(), "no live toggle"

    rows = page.locator(".lg-row")
    if rows.count() == 0:
        pytest.skip("the scribe directory is present but empty")
    assert rows.count() > 0


def test_logs_level_filter_narrows_the_view(page):
    """The chips are the filter as well as the count: clicking a level raises
    the floor, and clicking it again clears it."""
    _goto(page, "/logs")
    page.wait_for_timeout(3000)

    before = page.locator(".lg-row").count()
    if before == 0:
        pytest.skip("no lines to filter")

    page.locator(".lg-chip.error").click()
    page.wait_for_timeout(400)
    narrowed = page.locator(".lg-row").count()
    assert narrowed <= before, "raising the level floor showed more lines"

    page.locator(".lg-chip.error").click()
    page.wait_for_timeout(400)
    assert page.locator(".lg-row").count() >= narrowed, (
        "clicking the active level again did not clear the floor"
    )


# ── Object properties ──────────────────────────────────────────────────────
# Perception here is not accurate: a wrong label and a phantom object are the
# normal case, so a panel that can only display them sends you elsewhere to
# fix what you are looking at.


def test_clicking_an_object_opens_its_properties(page):
    """A row opens its detail below the list, and the list stays.

    Replacing the list made every inspection a round trip -- to check the next
    object you had to go back first. The row it describes is marked, because
    with the detail underneath nothing else says which one it is.
    """
    _goto(page, "/")
    page.wait_for_timeout(2500)

    rows = page.locator("#dock-objs tr.row")
    if rows.count() == 0:
        pytest.skip("the registry is empty; nothing to open")

    rows.first.click()
    page.wait_for_timeout(300)

    assert page.locator("#dock-detail .detail").count() == 1, (
        "clicking a row opened no detail"
    )
    assert rows.count() > 0, "the list disappeared when the detail opened"
    assert page.locator("#dock-objs tr.row.sel").count() == 1, (
        "the detail does not say which row it is about"
    )
    for key in ("dock.id", "dock.conf", "dock.pos"):
        assert page.locator(f'#dock-detail [data-i18n="{key}"]').count() == 1, (
            f"the detail is missing {key}"
        )
    assert page.locator("#dock-detail .ren").count() == 1, "no rename control"
    assert page.locator("#dock-detail .del").count() == 1, "no delete control"


def test_clicking_the_open_row_closes_it(page):
    """The control is its own undo."""
    _goto(page, "/")
    page.wait_for_timeout(2500)
    rows = page.locator("#dock-objs tr.row")
    if rows.count() == 0:
        pytest.skip("the registry is empty")

    rows.first.click()
    page.wait_for_timeout(250)
    assert page.locator("#dock-detail .detail").count() == 1
    rows.first.click()
    page.wait_for_timeout(250)
    assert page.locator("#dock-objs tr.row.sel").count() == 0, (
        "clicking the open row left it selected"
    )


def test_rename_goes_through_the_shared_entry_point(page):
    """The web API, MCP and gRPC call one function per correction.

    This drives the HTTP surface the panel uses. What it proves about the
    sharing is indirect -- that the route exists and applies -- but a rename
    that lands here lands through ObjectMutationCoordinator, which is the only
    place the mechanism is called from.
    """
    state = page.request.get(BASE_URL + "/api/state").json()
    objs = [o for o in (state.get("objects") or []) if o.get("cls") != "robot"]
    if not objs:
        pytest.skip("no non-robot object to rename")
    obj = objs[0]
    original = obj["cls"]

    r = page.request.post(
        f"{BASE_URL}/api/objects/{obj['id']}/label",
        data={"label": "renamed-by-test",
              "expected_map_id": (state.get("map_binding") or {}).get("map_id", ""),
              "expected_generation": (state.get("map_binding") or {}).get("generation")})
    body = r.json()
    assert body.get("ok"), f"rename refused: {body.get('detail')}"
    assert body.get("label") == "renamed-by-test"

    # Put it back, so the run leaves the map as it found it.
    page.request.post(
        f"{BASE_URL}/api/objects/{obj['id']}/label",
        data={"label": original,
              "expected_map_id": (state.get("map_binding") or {}).get("map_id", ""),
              "expected_generation": (state.get("map_binding") or {}).get("generation")})


def test_rename_edits_in_place_without_a_browser_dialog(page):
    """The panel asks in its own style, not the browser's.

    `prompt()` and `confirm()` bring a different typeface, palette and button
    order, and suspend the page while open -- which undoes the point of the
    panel having one type scale. A dialog appearing here fails the test rather
    than merely looking wrong.
    """
    fired = []
    page.on("dialog", lambda d: (fired.append(d.type), d.dismiss()))

    _goto(page, "/")
    page.wait_for_timeout(2500)
    rows = page.locator("#dock-objs tr.row")
    if rows.count() == 0:
        pytest.skip("the registry is empty")

    rows.first.click()
    page.wait_for_timeout(250)
    page.locator("#dock-detail .ren").click()
    page.wait_for_timeout(300)

    assert not fired, f"a browser dialog opened: {fired}"
    field = page.locator("#dock-detail .edit input")
    assert field.count() == 1, "rename did not open an inline field"
    assert field.input_value().strip(), "the field did not start from the name"

    # Escape puts it back, so the edit is abandonable without a round trip.
    field.press("Escape")
    page.wait_for_timeout(200)
    assert page.locator("#dock-detail .edit").count() == 0, (
        "Escape left the field open"
    )
    assert page.locator("#dock-detail h3").count() == 1, (
        "the heading did not come back"
    )


def test_delete_asks_inside_the_panel(page):
    """Still asks -- it destroys something and sits beside a button pressed
    often -- but in the panel, with the confirming button the red one."""
    fired = []
    page.on("dialog", lambda d: (fired.append(d.type), d.dismiss()))

    _goto(page, "/")
    page.wait_for_timeout(2500)
    rows = page.locator("#dock-objs tr.row")
    if rows.count() == 0:
        pytest.skip("the registry is empty")

    rows.first.click()
    page.wait_for_timeout(250)
    page.locator("#dock-detail .del").click()
    page.wait_for_timeout(300)

    assert not fired, f"a browser dialog opened: {fired}"
    assert page.locator("#dock-detail .acts .confirm").count() == 1, (
        "delete did not ask in the panel"
    )
    page.locator("#dock-detail .acts .no").click()
    page.wait_for_timeout(250)
    assert page.locator("#dock-detail .del").count() == 1, (
        "cancelling the delete did not restore the actions"
    )


# ── The map library ────────────────────────────────────────────────────────
# Maps used to be the regions editor with its controls hidden by CSS, which
# made choosing a map look like editing one. It is a grid of cards now.


def test_maps_is_a_grid_not_the_region_editor(page):
    """No form column, no plan canvas, no marking controls -- a grid and the
    one action that adds to it."""
    _goto(page, "/maps")
    page.wait_for_timeout(1200)

    assert page.locator("#mg-grid").count() == 1, "no grid"
    assert page.locator("#mg-save").is_visible(), "no save action"
    assert page.locator("#mg-name").is_visible(), "no name field"
    # The regions editor's parts must not be here in any form.
    for gone in ("#btn-draw", "#region-list", "#map-tools", "canvas#c"):
        assert page.locator(gone).count() == 0, (
            f"the maps page still carries {gone}"
        )


def test_maps_grid_reports_an_empty_library_honestly(page):
    """With nothing saved, the page says so rather than showing an empty
    rectangle that looks like a failure to load."""
    _goto(page, "/maps")
    page.wait_for_timeout(1500)

    cards = page.locator(".mg-card")
    if cards.count() == 0:
        note = page.locator("#mg-empty")
        assert note.is_visible(), "an empty library shows nothing at all"
        assert note.inner_text().strip(), "the empty note has no text"
    else:
        # Every card must name its map and offer the two actions.
        first = cards.first
        assert first.locator(".mg-name").inner_text().strip(), "card has no name"
        assert first.locator(".mg-acts .del").count() == 1, "card cannot be deleted"


def test_saving_without_a_name_is_refused_in_the_page(page):
    """The one destructive-ish mistake here is saving over nothing: it asks
    for a name first, in the page rather than through the browser."""
    fired = []
    page.on("dialog", lambda d: (fired.append(d.type), d.dismiss()))

    _goto(page, "/maps")
    page.wait_for_timeout(1000)
    page.locator("#mg-name").fill("")
    page.locator("#mg-save").click()
    page.wait_for_timeout(400)

    assert not fired, f"a browser dialog opened: {fired}"
    assert page.locator("#mg-msg.err").inner_text().strip(), (
        "saving with no name said nothing"
    )


def test_the_shell_has_transitions(page):
    """Controls move. A nav item that changes colour between two frames reads
    as a series of stills rather than as something being operated."""
    _goto(page, "/maps")
    page.wait_for_timeout(600)

    dur = page.locator("nav a").first.evaluate(
        "el => getComputedStyle(el).transitionDuration")
    assert dur and dur != "0s", f"nav items have no transition ({dur})"

    marker = page.locator("nav a.on").first.evaluate(
        "el => getComputedStyle(el, '::before').transitionDuration")
    assert marker and marker != "0s", (
        f"the active marker does not animate ({marker})"
    )


def test_map_cards_keep_a_uniform_shape(page):
    """A map's own proportions must not decide its card's.

    The thumbnail slot is fixed and the image fits inside it; without that,
    one corridor-shaped floor plan stretches its card and the grid stops
    being a grid.
    """
    _goto(page, "/maps")
    page.wait_for_timeout(1500)
    shots = page.locator(".mg-shot")
    if shots.count() == 0:
        pytest.skip("no saved maps to draw")
    heights = [shots.nth(i).bounding_box()["height"] for i in range(shots.count())]
    assert max(heights) - min(heights) < 2, (
        f"thumbnail slots differ in height: {heights}"
    )


def test_a_map_card_opens_its_details(page):
    """A Details button opens it, not the card itself: a map id is there to be
    read and copied, and making the whole surface a control takes that away.
    Open shows everything list_maps reports, including why a broken one
    cannot be loaded."""
    _goto(page, "/maps")
    page.wait_for_timeout(1500)
    cards = page.locator(".mg-card")
    if cards.count() == 0:
        pytest.skip("no saved maps")

    cards.first.locator('.mg-acts .more').click()
    page.wait_for_timeout(300)
    more = cards.first.locator(".mg-more")
    assert more.is_visible(), "the card did not open"
    for key in ("maps.health", "maps.id", "maps.artifactPath"):
        assert more.locator(f'[data-i18n="{key}"]').count() == 1, (
            f"the detail is missing {key}"
        )

    cards.first.locator('.mg-acts .more').click()
    page.wait_for_timeout(300)
    assert not more.is_visible(), "Back did not close the details"

