# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for user annotations (scene_service.annotations) and the
annotation CRUD REST API (scene_service.web).

The store tests are pure Python + tmp dirs and always run. The web tests
drive the Starlette app through a hand-rolled ASGI call (no httpx /
TestClient dependency); they need starlette + the scene_service.web import
chain and skip loudly when that is unavailable (e.g. a bare mac checkout).
"""
import asyncio
import json
import os
import sys
import tempfile
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.annotations import (  # noqa: E402
    AnnotationStore,
    validate_annotation_fields,
)

ROOM_PTS = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.5], [0.0, 1.5]]


# ── validation ──────────────────────────────────────────────────────────────

def test_validation_rules():
    ok = validate_annotation_fields
    assert ok("room", "kitchen", ROOM_PTS, None) is None
    assert ok("poi", "dock", [[1.0, 2.0]], 1.57) is None
    assert ok("no_go", "x", ROOM_PTS, None)           # unknown kind
    assert ok("room", 7, ROOM_PTS, None)              # non-string name
    assert ok("room", "x" * 200, ROOM_PTS, None)      # name too long
    assert ok("room", "x", ROOM_PTS[:2], None)        # polygon < 3 points
    assert ok("poi", "x", ROOM_PTS[:2], None)         # poi != 1 point
    assert ok("room", "x", [[0.0, float("nan")]] * 3, None)   # NaN point
    assert ok("room", "x", [[0.0], [1.0], [2.0]], None)       # not pairs
    assert ok("room", "x", "not-a-list", None)
    assert ok("poi", "x", [[0.0, 0.0]], float("inf"))         # inf theta
    assert ok("poi", "x", [[0.0, 0.0]], "north")              # non-number theta
    assert ok("poi", "x", [[True, 0.0]], None)                # bool is not a coord
    assert ok("room", "x", ROOM_PTS, 1.57)                    # heading on a room
    print("  [PASS] test_validation_rules")


# ── store ───────────────────────────────────────────────────────────────────

def test_store_roundtrip_across_reopen():
    with tempfile.TemporaryDirectory() as base:
        s1 = AnnotationStore(base, map_id="lab", generation=2)
        room = s1.create(kind="room", name="kitchen", points=ROOM_PTS)
        poi = s1.create(kind="poi", name="dock", points=[[1.0, 2.0]], theta=0.5)
        assert room.annotation_id.startswith("anno.")
        assert room.created_at > 0 and room.updated_at >= room.created_at

        s2 = AnnotationStore(base, map_id="lab", generation=2)
        got = {a.annotation_id: a for a in s2.list()}
        assert got[room.annotation_id].points == ROOM_PTS
        assert got[room.annotation_id].name == "kitchen"
        assert got[poi.annotation_id].theta == 0.5
        assert not any(a.stale for a in got.values())
    print("  [PASS] test_store_roundtrip_across_reopen")


def test_room_create_is_idempotent_by_name():
    with tempfile.TemporaryDirectory() as base:
        s = AnnotationStore(base, map_id="lab", generation=1)
        a = s.create(kind="room", name="Bathroom", points=ROOM_PTS)
        b = s.create(kind="room", name="  bathroom  ",
                     points=[[1.0, 1.0], [2.0, 1.0], [2.0, 2.0]])
        assert a.annotation_id == b.annotation_id
        assert len(s.list()) == 1
        assert s.list()[0].points == [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0]]
        raw = json.loads(open(s.path).read())
        assert len(raw["annotations"]) == 1
    print("  [PASS] test_room_create_is_idempotent_by_name")


def test_update_and_delete():
    with tempfile.TemporaryDirectory() as base:
        s = AnnotationStore(base, map_id="lab", generation=1)
        a = s.create(kind="room", name="old", points=ROOM_PTS)
        u = s.update(a.annotation_id, name="new")
        assert u.name == "new" and u.updated_at >= a.updated_at
        assert s.update("anno.missing", name="x") is None
        assert s.delete(a.annotation_id) is True
        assert s.delete(a.annotation_id) is False
        assert s.list() == []
    print("  [PASS] test_update_and_delete")


def test_map_id_partition_isolation_and_sanitize():
    with tempfile.TemporaryDirectory() as base:
        s_a = AnnotationStore(base, map_id="lab_a", generation=1)
        s_b = AnnotationStore(base, map_id="lab_b", generation=1)
        s_a.create(kind="room", name="only-in-a", points=ROOM_PTS)
        assert s_b.list() == []
        assert AnnotationStore(base, map_id="lab_b", generation=1).list() == []
        assert len(AnnotationStore(base, map_id="lab_a", generation=1).list()) == 1

        # Hostile map_id chars are squashed to "_" — same rule as the
        # object store, so the two partitions always agree on the key.
        s_evil = AnnotationStore(base, map_id="we ird/../id", generation=1)
        assert s_evil.map_id == "we_ird_.._id"
        assert s_evil.path.parent == s_a.path.parent  # no path escape
    print("  [PASS] test_map_id_partition_isolation_and_sanitize")


def test_delete_map_removes_only_target_partition():
    with tempfile.TemporaryDirectory() as base:
        target = AnnotationStore(base, map_id="target", generation=1)
        other = AnnotationStore(base, map_id="other", generation=1)
        target.create(kind="room", name="target room", points=ROOM_PTS)
        other.create(kind="room", name="other room", points=ROOM_PTS)

        assert target.delete_map("target") is True
        assert target.list() == []
        assert AnnotationStore(base, map_id="target", generation=1).list() == []
        assert len(AnnotationStore(base, map_id="other", generation=1).list()) == 1
        assert target.delete_map("target") is False
    print("  [PASS] test_delete_map_removes_only_target_partition")


def test_atomic_write_leaves_no_tmp():
    with tempfile.TemporaryDirectory() as base:
        s = AnnotationStore(base, map_id="lab", generation=1)
        s.create(kind="room", name="r", points=ROOM_PTS)
        s.mark_all_stale("test", new_generation=2)
        leftovers = [p for p in os.listdir(base) if p.endswith(".tmp")]
        assert leftovers == []
        data = json.loads(open(os.path.join(base, "lab.json")).read())
        assert set(data) == {"map_id", "generation", "annotations"}
    print("  [PASS] test_atomic_write_leaves_no_tmp")


def test_generation_mismatch_marks_stale_on_load():
    with tempfile.TemporaryDirectory() as base:
        s1 = AnnotationStore(base, map_id="lab", generation=2)
        a = s1.create(kind="room", name="kitchen", points=ROOM_PTS)

        # Same generation → untouched.
        assert not AnnotationStore(base, map_id="lab", generation=2).list()[0].stale

        # Bumped generation → stale + reason, and the flag is persisted.
        s3 = AnnotationStore(base, map_id="lab", generation=3)
        got = s3.get(a.annotation_id)
        assert got.stale and "2→3" in got.stale_reason
        raw = json.loads(open(s3.path).read())
        assert raw["annotations"][0]["stale"] is True

        # User confirms it is still valid → stale cleared, persisted.
        s3.update(a.annotation_id, clear_stale=True)
        assert not AnnotationStore(base, map_id="lab", generation=3).get(
            a.annotation_id).stale
    print("  [PASS] test_generation_mismatch_marks_stale_on_load")


def test_unknown_generation_preserves_stored_epoch():
    with tempfile.TemporaryDirectory() as base:
        AnnotationStore(base, map_id="lab", generation=5).create(
            kind="room", name="r", points=ROOM_PTS)
        # A static-binding boot (no broadcast → generation None) cannot
        # judge staleness: nothing is flagged AND the stored epoch must
        # survive the session so a later bound boot still can.
        s = AnnotationStore(base, map_id="lab", generation=None)
        assert not s.list()[0].stale
        s.update(s.list()[0].annotation_id, name="renamed")   # forces a save
        assert json.loads(open(s.path).read())["generation"] == 5
        assert AnnotationStore(base, map_id="lab", generation=6).list()[0].stale
    print("  [PASS] test_unknown_generation_preserves_stored_epoch")


def test_reconcile_generation_learns_and_flags():
    with tempfile.TemporaryDirectory() as base:
        AnnotationStore(base, map_id="lab", generation=None).create(
            kind="room", name="r", points=ROOM_PTS)
        # A file written under an unknown epoch adopts the confirmed one
        # without flagging anything (there is nothing to disagree with).
        s = AnnotationStore(base, map_id="lab", generation=None)
        assert s.reconcile_generation(5) == 0
        assert not s.list()[0].stale
        assert json.loads(open(s.path).read())["generation"] == 5
        assert s.reconcile_generation(5) == 0        # same epoch: no-op

        # A recorded epoch that DIFFERS from the confirmed one means the
        # map frame moved since the annotations were drawn → all stale.
        s2 = AnnotationStore(base, map_id="lab", generation=None)
        assert s2.reconcile_generation(7) == 1
        got = s2.list()[0]
        assert got.stale and "5→7" in got.stale_reason
    print("  [PASS] test_reconcile_generation_learns_and_flags")


def test_redraw_clears_stale_and_mark_all_stale_counts():
    with tempfile.TemporaryDirectory() as base:
        s = AnnotationStore(base, map_id="lab", generation=1)
        a = s.create(kind="room", name="r", points=ROOM_PTS)
        b = s.create(kind="poi", name="p", points=[[0.0, 0.0]])
        assert s.mark_all_stale("epoch flip", new_generation=2) == 2
        assert s.mark_all_stale("epoch flip", new_generation=2) == 0  # idempotent
        # Redrawing geometry re-authors it against the CURRENT frame →
        # staleness no longer applies.
        assert s.update(a.annotation_id, points=ROOM_PTS).stale is False
        assert s.get(b.annotation_id).stale is True
    print("  [PASS] test_redraw_clears_stale_and_mark_all_stale_counts")


def test_corrupt_file_set_aside_not_destroyed():
    # Every bad-file shape must take the same quarantine path: parse
    # error, valid-JSON-but-not-an-object, non-object annotation entry,
    # non-numeric generation, and a record that fails field validation
    # (per-map files are user-visible JSON, hand edits happen).
    bad_files = [
        "{not json",
        "[1, 2, 3]",
        '"just a string"',
        '{"map_id": "lab", "generation": 1, "annotations": [3]}',
        '{"map_id": "lab", "generation": "three", "annotations": []}',
        '{"map_id": "lab", "generation": 1, "annotations": '
        '[{"annotation_id": "anno.x", "kind": "castle", "name": "x", '
        '"points": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]}]}',
    ]
    for content in bad_files:
        with tempfile.TemporaryDirectory() as base:
            path = os.path.join(base, "lab.json")
            with open(path, "w") as f:
                f.write(content)
            s = AnnotationStore(base, map_id="lab", generation=1)
            assert s.list() == [], content
            aside = [p for p in os.listdir(base) if ".corrupt-" in p]
            assert len(aside) == 1, content
            assert open(os.path.join(base, aside[0])).read() == content
    print("  [PASS] test_corrupt_file_set_aside_not_destroyed")


def test_from_json_drops_unknown_keys():
    from scene_service.annotations import Annotation
    a = Annotation.from_json({
        "annotation_id": "anno.x", "kind": "room", "name": "r",
        "points": [[0.0, 0.0]], "written_by_future_scene": True,
    })
    assert a.annotation_id == "anno.x" and not hasattr(a, "written_by_future_scene")
    print("  [PASS] test_from_json_drops_unknown_keys")


# ── REST API (hand-rolled ASGI, no httpx needed) ────────────────────────────

def _make_web_app(anno_store, **overrides):
    """Import scene_service.web lazily and build an app with only the
    pieces the annotation routes need. When the web import chain is
    unavailable (bare environment): under pytest the caller's test is
    SKIPPED (visible in the report, not a hollow pass); standalone it
    prints and returns None."""
    try:
        from scene_service import web as web_mod
    except ImportError as e:
        if os.environ.get("PYTEST_CURRENT_TEST"):
            import pytest
            pytest.skip(f"web deps unavailable: {e}")
        print(f"  [SKIP] web tests unavailable: {e}")
        return None

    class _StubRegistry:
        _objects: dict = {}
        _surfaces: dict = {}

        @staticmethod
        def clear_objects():
            return 0

    options = {
        "registry": _StubRegistry(),
        "anno_store": anno_store,
        "map_binding": {"map_id": "lab", "mode": "mapping",
                        "generation": 1, "source": "lifecycle"},
    }
    options.update(overrides)
    return web_mod.make_app(**options)


def _call(app, method: str, path: str, body=None):
    """Drive the ASGI app directly with one JSON request; return
    (status, parsed-body). Avoids the TestClient→httpx dependency.

    Runs on a PRIVATE event loop (not asyncio.run, which unsets the
    thread's current loop on exit and would starve later test files
    that still use the ambient-loop pattern — the exact cross-file
    pollution test_scene_graph._ensure_loop guards against)."""
    raw = json.dumps(body).encode() if body is not None else b""
    scope = {
        "type": "http", "http_version": "1.1", "method": method,
        "scheme": "http", "path": path, "raw_path": path.encode(),
        "root_path": "", "query_string": b"", "client": ("test", 0),
        "server": ("test", 80),
        "headers": [(b"content-type", b"application/json"),
                    (b"content-length", str(len(raw)).encode())],
    }
    sent, done = [], {"body": False}

    async def receive():
        if done["body"]:
            return {"type": "http.disconnect"}
        done["body"] = True
        return {"type": "http.request", "body": raw, "more_body": False}

    async def send(message):
        sent.append(message)

    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(app(scope, receive, send))
    finally:
        loop.close()
    status = next(m["status"] for m in sent
                  if m["type"] == "http.response.start")
    payload = b"".join(m.get("body", b"") for m in sent
                       if m["type"] == "http.response.body")
    return status, (json.loads(payload) if payload else None)


def test_rest_crud_roundtrip():
    with tempfile.TemporaryDirectory() as base:
        store = AnnotationStore(base, map_id="lab", generation=1)
        app = _make_web_app(store)
        if app is None:
            return

        st, out = _call(app, "GET", "/api/annotations")
        assert (st, out) == (200, {"ok": True, "annotations": []})

        st, out = _call(app, "POST", "/api/annotations",
                        {"kind": "room", "name": "kitchen-draft", "points": ROOM_PTS})
        assert st == 200 and out["ok"]
        ann_id = out["annotation"]["annotation_id"]

        st, out = _call(app, "PUT", f"/api/annotations/{ann_id}",
                        {"name": "kitchen"})
        assert st == 200 and out["annotation"]["name"] == "kitchen"

        # stale confirm flow: flag via the store (what the lifecycle
        # watcher does), clear via the API (what the UI button does).
        store.mark_all_stale("epoch flip", new_generation=2)
        st, out = _call(app, "PUT", f"/api/annotations/{ann_id}",
                        {"stale": False})
        assert st == 200 and out["annotation"]["stale"] is False

        st, out = _call(app, "DELETE", f"/api/annotations/{ann_id}")
        assert (st, out) == (200, {"ok": True})
        assert store.list() == []
    print("  [PASS] test_rest_crud_roundtrip")


def test_map_load_waits_for_new_occupancy_before_rebinding(monkeypatch):
    try:
        from scene_service import web as web_mod
    except ImportError as e:
        import pytest
        pytest.skip(f"web deps unavailable: {e}")

    class Hub:
        loaded = False

        @staticmethod
        def has(_name):
            return True

        def latest(self, _name):
            count = 2 if self.loaded else 1
            msg = SimpleNamespace(info=SimpleNamespace(width=20, height=12))
            return msg, float(count), count

    hub = Hub()

    def map_rpc(op, payload, _timeout=30.0):
        assert op == "load_map"
        assert payload["map_id"] == "saved_lab"
        hub.loaded = True
        return {"ok": True, "detail": "loaded once"}

    monkeypatch.setattr(web_mod, "_map_rpc", map_rpc)
    with tempfile.TemporaryDirectory() as base:
        AnnotationStore(base, map_id="saved_lab", generation=1).create(
            kind="room", name="restored", points=ROOM_PTS,
        )
        store = AnnotationStore(base, map_id="live", generation=1)
        binding = {"map_id": "live", "mode": "mapping",
                   "generation": 1, "source": "lifecycle"}
        app = _make_web_app(store, hub=hub, map_binding=binding)

        status, out = _call(
            app, "POST", "/api/maps/load",
            {"map_id": "saved_lab", "mode": "localization"},
        )

        assert status == 200 and out["ok"] is True
        assert out["occupancy"]["count"] == 2
        assert binding["map_id"] == "saved_lab"
        assert binding["mode"] == "localization"
        assert [room.name for room in store.list()] == ["restored"]
    print("  [PASS] test_map_load_waits_for_new_occupancy_before_rebinding")


def test_rest_validation_and_errors():
    with tempfile.TemporaryDirectory() as base:
        store = AnnotationStore(base, map_id="lab", generation=1)
        app = _make_web_app(store)
        if app is None:
            return

        st, out = _call(app, "POST", "/api/annotations",
                        {"kind": "castle", "name": "x", "points": ROOM_PTS})
        assert st == 400 and not out["ok"] and "kind" in out["detail"]

        st, _ = _call(app, "POST", "/api/annotations",
                      {"kind": "room", "name": "x", "points": ROOM_PTS[:2]})
        assert st == 400

        st, out = _call(app, "POST", "/api/annotations",
                        {"kind": "room", "name": "x", "points": ROOM_PTS,
                         "theta": 1.57})
        assert st == 400 and "poi" in out["detail"]  # headings are poi-only

        st, _ = _call(app, "PUT", "/api/annotations/anno.nope", {"name": "x"})
        assert st == 404
        st, _ = _call(app, "DELETE", "/api/annotations/anno.nope")
        assert st == 404

        # A partial update may not sneak in an invalid merged state.
        _, created = _call(app, "POST", "/api/annotations",
                           {"kind": "room", "name": "x", "points": ROOM_PTS})
        st, _ = _call(app, "PUT",
                      f"/api/annotations/{created['annotation']['annotation_id']}",
                      {"points": [[0.0, 0.0]]})
        assert st == 400

        # Store unavailable → 503 on every verb.
        app_off = _make_web_app(None)
        for method, path in [("GET", "/api/annotations"),
                             ("POST", "/api/annotations"),
                             ("PUT", "/api/annotations/x"),
                             ("DELETE", "/api/annotations/x")]:
            st, out = _call(app_off, method, path, {})
            assert st == 503 and not out["ok"]
    print("  [PASS] test_rest_validation_and_errors")


def test_user_page_served():
    with tempfile.TemporaryDirectory() as base:
        app = _make_web_app(AnnotationStore(base, map_id="lab", generation=1))
        if app is None:
            return
        sent = []

        async def receive():
            return {"type": "http.request", "body": b"", "more_body": False}

        async def send(message):
            sent.append(message)

        scope = {"type": "http", "http_version": "1.1", "method": "GET",
                 "scheme": "http", "path": "/user", "raw_path": b"/user",
                 "root_path": "", "query_string": b"", "headers": [],
                 "client": ("test", 0), "server": ("test", 80)}
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(app(scope, receive, send))
        finally:
            loop.close()
        status = next(m["status"] for m in sent
                      if m["type"] == "http.response.start")
        body = b"".join(m.get("body", b"") for m in sent
                        if m["type"] == "http.response.body")
        assert status == 200
        assert b"Annotate room" in body and b"/api/annotations" in body
    print("  [PASS] test_user_page_served")


def test_state_payload_carries_annotations_and_binding():
    with tempfile.TemporaryDirectory() as base:
        store = AnnotationStore(base, map_id="lab", generation=1)
        store.create(kind="room", name="kitchen", points=ROOM_PTS)
        app = _make_web_app(store)
        if app is None:
            return
        st, out = _call(app, "GET", "/api/state")
        assert st == 200
        assert out["map_binding"]["map_id"] == "lab"
        assert len(out["annotations"]) == 1
        assert out["annotations"][0]["name"] == "kitchen"
    print("  [PASS] test_state_payload_carries_annotations_and_binding")


if __name__ == "__main__":
    test_validation_rules()
    test_store_roundtrip_across_reopen()
    test_room_create_is_idempotent_by_name()
    test_update_and_delete()
    test_map_id_partition_isolation_and_sanitize()
    test_delete_map_removes_only_target_partition()
    test_atomic_write_leaves_no_tmp()
    test_generation_mismatch_marks_stale_on_load()
    test_unknown_generation_preserves_stored_epoch()
    test_reconcile_generation_learns_and_flags()
    test_redraw_clears_stale_and_mark_all_stale_counts()
    test_corrupt_file_set_aside_not_destroyed()
    test_from_json_drops_unknown_keys()
    test_rest_crud_roundtrip()
    test_rest_validation_and_errors()
    test_user_page_served()
    test_state_payload_carries_annotations_and_binding()
    print("test_annotations: all tests passed")
