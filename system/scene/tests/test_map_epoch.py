# SPDX-License-Identifier: MulanPSL-2.0
"""Epoch-safety tests for the web map facade (Save/Load/Delete).

Drives scene_service.web's /api/maps routes through a hand-rolled ASGI call
with mapping RPCs monkeypatched and a fake (in-memory) object store, so the
commit-ordering rules are testable without milvus or ROS:

- Save writes the object snapshot into a FRESH partition token, commits the
  sidecar only after a complete write, then purges the previous snapshot.
- Save refuses: mapping-mode semantic-only re-save (frame drift), and
  overwriting a map's saved annotations from a session that never loaded
  them (bound-is-not-loaded).
- Load restores exactly the sidecar's partition; no sidecar → restores
  NOTHING (the legacy mixed-epoch partition stays dead).
- Delete removes the sidecar and both partitions.

Skips loudly when the web import chain is unavailable (bare mac checkout).
"""
import asyncio
import json
import os
import sys
import tempfile
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.annotations import AnnotationStore  # noqa: E402
from scene_service.map_meta import MapMetaStore, make_meta  # noqa: E402

try:  # the state package needs scipy — absent on a bare mac checkout
    from scene_service.state.object_registry import (
        BBox3D,
        ObjectRegistry,
        Pose3D,
        SceneObject,
    )
    _IMPORT_ERR = None
except ImportError as e:  # pragma: no cover - environment-dependent
    _IMPORT_ERR = e
    BBox3D = ObjectRegistry = Pose3D = SceneObject = None

ROOM_PTS = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.5], [0.0, 1.5]]



def _unavailable() -> bool:
    """Skip guard shared by every test — the fakes construct real
    SceneObjects, so the state import must exist before any argument
    expression runs."""
    err = _IMPORT_ERR
    if err is None:
        try:
            import scene_service.web  # noqa: F401
            return False
        except ImportError as e:
            err = e
    if os.environ.get("PYTEST_CURRENT_TEST"):
        import pytest
        pytest.skip(f"map facade deps unavailable: {err}")
    print(f"  [SKIP] map facade tests unavailable: {err}")
    return True

def _obj(oid: str, cls: str = "cup", *, missing: bool = False,
         attributes: "dict | None" = None) -> SceneObject:
    return SceneObject(
        object_id=oid, cls=cls,
        pose=Pose3D(x=1.0, y=2.0, z=0.0, yaw=0.0, frame_id="map"),
        bbox=BBox3D(size_x=0.1, size_y=0.1, size_z=0.1, yaw=0.0, frame_id="map"),
        confidence=0.9, first_seen=1.0, last_seen=2.0,
        observation_count=3, missing=missing, attributes=attributes or {},
    )


class FakeObjectStore:
    """Partition-faithful in-memory stand-in for persistence.ObjectStore."""

    def __init__(self):
        self.rows: dict[str, dict[str, SceneObject]] = {}

    def persist(self, pairs, *, partition=None):
        part = self.rows.setdefault(partition, {})
        for obj, _cap in pairs:
            part[obj.object_id] = obj
        return len(pairs)

    def load_all(self, *, partition=None, limit=16384, strict=False):
        return list(self.rows.get(partition, {}).values())

    def delete_map(self, map_id):
        return len(self.rows.pop(map_id, {}))


class FakeHub:
    """Serves a fresh occupancy grid (Load's readiness gate) and, when set,
    a latched lifecycle sample."""

    def __init__(self, lifecycle=None):
        self._count = 0
        self.lifecycle = lifecycle

    def has(self, key):
        if key == "occupancy_grid":
            return True
        return key == "map_lifecycle" and self.lifecycle is not None

    def latest(self, key):
        if key == "map_lifecycle":
            return self.lifecycle, time.time(), 1
        self._count += 1
        grid = SimpleNamespace(info=SimpleNamespace(width=10, height=8))
        return grid, time.time(), self._count


def _registry_with(objs) -> ObjectRegistry:
    reg = ObjectRegistry()
    reg._objects.update({o.object_id: o for o in objs})
    return reg


_KEEP = object()  # sentinel: "build the real MapMetaStore"


def _make_env(tmp, *, binding, objs=(), anno_map_id=".live",
              lifecycle=None, map_meta=_KEEP, derived_state_reset=None):
    """(web_mod, app, pieces) or (None, None, None) when deps can't import."""
    err = _IMPORT_ERR
    web_mod = None
    if err is None:
        try:
            from scene_service import web as web_mod
        except ImportError as e:
            err = e
    if err is not None:
        if os.environ.get("PYTEST_CURRENT_TEST"):
            import pytest
            pytest.skip(f"map facade deps unavailable: {err}")
        print(f"  [SKIP] map facade tests unavailable: {err}")
        return None, None, None

    registry = _registry_with(objs)
    anno = AnnotationStore(os.path.join(tmp, "anno"), map_id=anno_map_id)
    meta = MapMetaStore(os.path.join(tmp, "maps")) if map_meta is _KEEP else map_meta
    store = FakeObjectStore()
    hub = FakeHub(lifecycle=lifecycle)
    bind = dict(binding)
    app = web_mod.make_app(
        registry=registry, hub=hub, anno_store=anno,
        object_store=store, map_meta=meta, map_binding=bind,
        derived_state_reset=derived_state_reset,
    )
    return web_mod, app, SimpleNamespace(
        registry=registry, anno=anno, meta=meta, store=store, hub=hub,
        binding=bind,
    )


def _call(app, method, path, body=None):
    """One JSON request straight into the ASGI app on a private loop."""
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
    status = next(m["status"] for m in sent if m["type"] == "http.response.start")
    payload = b"".join(m.get("body", b"") for m in sent if m["type"] == "http.response.body")
    return status, json.loads(payload or b"{}")


class _patched:
    """Temporarily replace module-level mapping RPC helpers on web."""

    def __init__(self, web_mod, *, rpc=None, maps=None):
        self._mod, self._rpc, self._maps = web_mod, rpc, maps

    def __enter__(self):
        self._old = (self._mod._map_rpc, self._mod._maps_payload)
        if self._rpc is not None:
            self._mod._map_rpc = self._rpc
        if self._maps is not None:
            self._mod._maps_payload = self._maps
        return self

    def __exit__(self, *exc):
        self._mod._map_rpc, self._mod._maps_payload = self._old
        return False


def _spatial(*ids):
    return lambda: {"ok": True, "detail": "", "maps": [
        {"map_id": i, "has_spatial_artifact": True, "spatial_ok": True}
        for i in ids
    ]}


def _rpc_ok(op, payload=None, timeout_s=None):
    return {"ok": True, "map_id": (payload or {}).get("map_id", ""), "detail": "ok"}


# ── Save ────────────────────────────────────────────────────────────────────

def test_save_commits_fresh_partition_sidecar_then_purges():
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
            objs=[_obj("o1"), _obj("o2"), _obj("gone", missing=True)],
        )
        if app is None:
            return
        # Legacy rows under the bare id must be gone after the save.
        env.store.rows["labx"] = {"legacy": _obj("legacy")}
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert body["snapshot_partition"] == "labx__s1"
        # `missing` objects (known, not currently seen — e.g. everything
        # right after a Load) are part of the snapshot: filtering them out
        # would let a Load→Save round-trip commit an empty snapshot.
        assert body["objects"] == 3
        assert set(env.store.rows.get("labx__s1", {})) == {"o1", "o2", "gone"}
        assert "labx" not in env.store.rows              # legacy purged
        meta = env.meta.read("labx")
        assert meta and meta.object_partition == "labx__s1"
    print("  [PASS] test_save_commits_fresh_partition_sidecar_then_purges")


def test_resave_rotates_partition_and_purges_previous():
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "labx", "mode": "localization",
                          "generation": 5, "source": "ui_load"},
            objs=[_obj("o3")], anno_map_id="labx",
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s1", 1))
        env.store.rows["labx__s1"] = {"old": _obj("old")}
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert body["snapshot_partition"] == "labx__s2"
        assert body.get("spatial_unchanged") is True
        assert set(env.store.rows.get("labx__s2", {})) == {"o3"}
        assert "labx__s1" not in env.store.rows          # previous snapshot purged
        assert env.meta.read("labx").object_partition == "labx__s2"
    print("  [PASS] test_resave_rotates_partition_and_purges_previous")


def test_mapping_mode_resave_refused():
    """The artifact froze at the original Save while the live frame kept
    drifting — a semantic-only re-save would mis-anchor everything."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "labx", "mode": "mapping",
                          "generation": 5, "source": "ui_save"},
            objs=[_obj("o1")], anno_map_id="labx",
        )
        if app is None:
            return
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 409, (status, body)
        assert env.meta.read("labx") is None             # nothing committed
    print("  [PASS] test_mapping_mode_resave_refused")


def test_save_refuses_overwriting_unloaded_annotations():
    """Bound-is-not-loaded: a live session must not carry its (possibly
    empty) annotation partition over a map's previously saved rooms."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        # Pre-save rooms under labx from an earlier "session".
        earlier = AnnotationStore(os.path.join(tmp, "anno"), map_id="labx")
        earlier.create(kind="room", name="kitchen", points=ROOM_PTS)
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "labx", "mode": "", "generation": None,
                          "source": "lifecycle"},
            objs=[_obj("o1")], anno_map_id=".live",
        )
        if app is None:
            return
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 409, (status, body)
        # The saved rooms survived untouched.
        kept = AnnotationStore(os.path.join(tmp, "anno"), map_id="labx")
        assert [a.name for a in kept.list()] == ["kitchen"]
    print("  [PASS] test_save_refuses_overwriting_unloaded_annotations")


def test_save_excludes_self_robot_object():
    """The self-object must stay out of snapshots: the pose tracker
    re-creates it from the live pose stream every session, so restoring a
    saved copy would leave a second, frozen robot marker after a Load."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
            objs=[_obj("o1"),
                  _obj("selfbot", cls="robot", attributes={"is_robot": True})],
        )
        if app is None:
            return
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert body["objects"] == 1, body
        assert set(env.store.rows.get("labx__s1", {})) == {"o1"}
    print("  [PASS] test_save_excludes_self_robot_object")


def test_failed_snapshot_keeps_previous_sidecar_and_fails_the_save():
    """An incomplete object write must leave the sidecar at the previous,
    still-consistent snapshot (and not purge it) — and the Save must report
    an explicit failure, not 200/ok: geometry + rooms + objects commit as
    one unit, so an operator must retry rather than trust the artifact."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "labx", "mode": "localization",
                          "generation": None, "source": "ui_load"},
            objs=[_obj("o1")], anno_map_id="labx",
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s1", 1))
        env.store.rows["labx__s1"] = {"old": _obj("old")}

        def broken_persist(pairs, *, partition=None):
            return 0  # milvus upsert failed (persist swallows + returns 0)

        env.store.persist = broken_persist
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 502 and not body["ok"], body
        assert "object_persist_error" in body, body
        assert body.get("partial") == "spatial_saved_object_snapshot_failed", body
        assert "OBJECT SNAPSHOT FAILED" in body.get("detail", ""), body
        assert env.meta.read("labx").object_partition == "labx__s1"
        assert "old" in env.store.rows.get("labx__s1", {})
    print("  [PASS] test_failed_snapshot_keeps_previous_sidecar_and_fails_the_save")


# ── Load ────────────────────────────────────────────────────────────────────

def test_load_without_sidecar_restores_nothing():
    """The mis-anchored-restore regression: a legacy partition (rows under
    the bare map id, written across unknown epochs) must NOT be restored."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return
        env.store.rows["labx"] = {"ghost": _obj("ghost")}   # mixed-epoch legacy
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
            status, body = _call(app, "POST", "/api/maps/load", {"map_id": "labx"})
            assert status == 200 and body["ok"], body
            assert body["objects_restored"] == 0
            assert "semantic_snapshot" in body
            assert env.registry._objects == {}           # nothing snuck in
            # The no-sidecar path still COMPLETES the load — the save hold
            # is lifted, and this Save creates the map's first sidecar.
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
            assert status == 200 and body["ok"], body
            assert env.meta.read("labx") is not None
    print("  [PASS] test_load_without_sidecar_restores_nothing")


def test_load_restores_exactly_the_sidecar_partition():
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s2", 2))
        env.store.rows["labx__s2"] = {"o1": _obj("o1"), "o2": _obj("o2")}
        env.store.rows["labx"] = {"ghost": _obj("ghost")}   # must stay dead
        with _patched(web_mod, rpc=_rpc_ok):
            status, body = _call(app, "POST", "/api/maps/load", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert body["objects_restored"] == 2
        assert body["snapshot_partition"] == "labx__s2"
        assert set(env.registry._objects) == {"o1", "o2"}
    print("  [PASS] test_load_restores_exactly_the_sidecar_partition")


# ── Delete ──────────────────────────────────────────────────────────────────

def test_delete_removes_sidecar_and_both_partitions():
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s3", 3))
        env.store.rows["labx__s3"] = {"o1": _obj("o1")}
        env.store.rows["labx"] = {"legacy": _obj("legacy")}
        with _patched(web_mod, rpc=_rpc_ok):
            status, body = _call(app, "POST", "/api/maps/delete", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert body["scene_cleanup"]["objects_deleted"] == 2
        assert "labx__s3" not in env.store.rows and "labx" not in env.store.rows
        assert env.meta.read("labx") is None
    print("  [PASS] test_delete_removes_sidecar_and_both_partitions")


def test_reserved_map_ids_rejected():
    """`.live*` (purged at boot) and `*__s<N>` (aliases another map's
    snapshot partition) are scene-internal namespaces — Save must 400."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            for bad in (".livelab", ".live", "foo__s1", "labx__s12"):
                status, body = _call(app, "POST", "/api/maps/save",
                                     {"map_id": bad})
                assert status == 400, (bad, status, body)
            status, _body = _call(app, "POST", "/api/maps/save",
                                  {"map_id": "fine__sx"})  # not the reserved shape
            assert status == 200
    print("  [PASS] test_reserved_map_ids_rejected")


def test_generation_threading_from_latched_broadcast():
    """A latched sample naming the SAME map feeds its generation into the
    sidecar and live binding; a stale sample naming another map — or a
    malformed one — degrades to None instead of poisoning the epoch."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        sample = SimpleNamespace(map_id="labx", generation=4, mode="mapping")
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
            objs=[_obj("o1")], lifecycle=sample,
        )
        if app is None:
            return
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert env.meta.read("labx").mapping_generation == 4
        assert env.binding["generation"] == 4

        # Stale latch: names a different map than the one being saved.
        env2_sample = SimpleNamespace(map_id="other", generation=9, mode="mapping")
        web_mod, app, env = _make_env(
            tmp + "/b", binding={"map_id": "default", "mode": "",
                                 "generation": None, "source": "default"},
            objs=[_obj("o1")], lifecycle=env2_sample,
        )
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert env.meta.read("labx").mapping_generation is None
        assert env.binding["generation"] is None

        # Malformed sample: generation isn't an int.
        junk = SimpleNamespace(map_id="labx", generation="junk", mode="mapping")
        web_mod, app, env = _make_env(
            tmp + "/c", binding={"map_id": "default", "mode": "",
                                 "generation": None, "source": "default"},
            objs=[_obj("o1")], lifecycle=junk,
        )
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert env.meta.read("labx").mapping_generation is None
    print("  [PASS] test_generation_threading_from_latched_broadcast")


def test_save_without_sidecar_store_fails_the_save():
    """map_meta init failure (None) while an object store exists: the
    objects cannot be snapshotted, so the Save reports failure (annotations
    were still written; retrying after the sidecar store heals completes
    the unit) and writes no orphan rows."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
            objs=[_obj("o1")], map_meta=None,
        )
        if app is None:
            return
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 502 and not body["ok"], body
        assert "object_persist_error" in body, body
        assert body.get("partial") == "spatial_saved_object_snapshot_failed", body
        assert env.store.rows == {}                      # no orphan write
    print("  [PASS] test_save_without_sidecar_store_fails_the_save")


def test_load_flushes_preload_live_objects():
    """Objects observed before a Load are anchored to the dead pre-load
    frame — the registry must hold only the restored snapshot afterwards."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        resets = []

        async def reset_derived_state():
            resets.append("reset")

        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
            objs=[_obj("preload1"), _obj("preload2")],
            derived_state_reset=reset_derived_state,
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s1", 1))
        env.store.rows["labx__s1"] = {"saved": _obj("saved")}
        with _patched(web_mod, rpc=_rpc_ok):
            status, body = _call(app, "POST", "/api/maps/load", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert body["objects_flushed"] == 2
        assert body["objects_restored"] == 1
        assert set(env.registry._objects) == {"saved"}
        assert resets == ["reset"]
    print("  [PASS] test_load_flushes_preload_live_objects")


def test_load_restore_failure_fails_load_and_keeps_previous_binding():
    """A DB read error during restore fails the Load outright: the previous
    scene binding (and annotation partition) stay in place, so the flushed
    registry cannot be committed against the half-loaded map."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s1", 1))

        def broken_load_all(*, partition=None, limit=16384, strict=False):
            raise RuntimeError("milvus went away")

        env.store.load_all = broken_load_all
        with _patched(web_mod, rpc=_rpc_ok):
            status, body = _call(app, "POST", "/api/maps/load", {"map_id": "labx"})
        assert status == 502 and not body["ok"], body
        assert "milvus went away" in body.get("object_restore_error", ""), body
        assert "RESTORE FAILED" in body.get("detail", ""), body
        assert env.binding["map_id"] == "default", env.binding  # binding kept
        assert env.anno.map_id == ".live", env.anno.map_id      # anno not rebound
    print("  [PASS] test_load_restore_failure_fails_load_and_keeps_previous_binding")


def test_load_flush_failure_aborts_before_any_rebind():
    """A registry flush failure aborts the Load before annotations or
    objects are rebound: continuing would restore the new map's snapshot
    into a registry that may still hold objects anchored to the previous
    frame — the mixed-epoch state this ordering exists to prevent."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
            objs=[_obj("preload")],
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s1", 1))
        env.store.rows["labx__s1"] = {"saved": _obj("saved")}

        def broken_clear():
            raise RuntimeError("registry wedged")

        env.registry.clear_objects = broken_clear
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
            status, body = _call(app, "POST", "/api/maps/load", {"map_id": "labx"})
            assert status == 502 and not body["ok"], body
            assert "registry wedged" in body.get("object_flush_error", ""), body
            assert set(env.registry._objects) == {"preload"}  # nothing restored
            assert env.binding["map_id"] == "default", env.binding
            assert env.anno.map_id == ".live", env.anno.map_id
            # The hold from this failure blocks Save too.
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
            assert status == 409 and "save blocked" in body.get("detail", ""), body
    print("  [PASS] test_load_flush_failure_aborts_before_any_rebind")


def test_occupancy_timeout_fails_load_and_blocks_save():
    """When the loaded map's grid never reaches scene, the Load fails and
    the divergent window (mapping runs the new DB, scene still bound to
    the old map) blocks Save until a Load completes."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return

        def frozen_latest(_key):  # grid count/stamp never advance
            grid = SimpleNamespace(info=SimpleNamespace(width=10, height=8))
            return grid, 5.0, 1

        env.hub.latest = frozen_latest
        os.environ["SCENE_MAP_READY_TIMEOUT_S"] = "0.2"
        try:
            with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
                status, body = _call(app, "POST", "/api/maps/load",
                                     {"map_id": "labx"})
                assert status == 502 and not body["ok"], body
                assert "did not observe" in body.get("detail", ""), body
                status, body = _call(app, "POST", "/api/maps/save",
                                     {"map_id": "labx"})
                assert status == 409 and "save blocked" in body.get("detail", ""), body
        finally:
            del os.environ["SCENE_MAP_READY_TIMEOUT_S"]
    print("  [PASS] test_occupancy_timeout_fails_load_and_blocks_save")


def test_fresh_save_snapshot_failure_names_workable_recovery():
    """A FIRST save from a mapping session latches the immutable-artifact
    state, so after a failed object snapshot a plain 'retry Save' would
    only bounce off the mapping-mode 409 — the failure detail must point
    at the recovery that actually works (delete and save anew)."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
            objs=[_obj("o1")],
        )
        if app is None:
            return

        def broken_persist(pairs, *, partition=None):
            raise RuntimeError("milvus down")

        env.store.persist = broken_persist
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial()):
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
        assert status == 502 and not body["ok"], body
        assert body.get("partial") == "spatial_saved_object_snapshot_failed", body
        assert "delete the map and Save anew" in body.get("detail", ""), body
        assert env.meta.read("labx") is None            # sidecar not committed
    print("  [PASS] test_fresh_save_snapshot_failure_names_workable_recovery")


def test_failed_load_blocks_save_until_successful_retry():
    """The load-failure-then-save sequence: after a failed Load the
    registry/binding may not match the map mapping runs, so Save is refused
    (409) until a Load completes; a successful retry lifts the hold."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s1", 1))
        env.store.rows["labx__s1"] = {"saved": _obj("saved")}
        real_load_all = env.store.load_all

        def broken_load_all(*, partition=None, limit=16384, strict=False):
            raise RuntimeError("milvus went away")

        env.store.load_all = broken_load_all
        with _patched(web_mod, rpc=_rpc_ok, maps=_spatial("labx")):
            status, body = _call(app, "POST", "/api/maps/load", {"map_id": "labx"})
            assert status == 502 and not body["ok"], body
            # Save is now held — even one that would otherwise pass.
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
            assert status == 409 and not body["ok"], body
            assert "save blocked" in body.get("detail", ""), body
            # A successful Load retry lifts the hold …
            env.store.load_all = real_load_all
            status, body = _call(app, "POST", "/api/maps/load", {"map_id": "labx"})
            assert status == 200 and body["ok"], body
            assert body["objects_restored"] == 1, body
            # … and the same Save now proceeds (semantic-only re-save).
            status, body = _call(app, "POST", "/api/maps/save", {"map_id": "labx"})
            assert status == 200 and body["ok"], body
            assert env.meta.read("labx").object_partition == "labx__s2"
    print("  [PASS] test_failed_load_blocks_save_until_successful_retry")


def test_delete_keeps_sidecar_when_snapshot_delete_fails():
    """The sidecar is the only pointer to the snapshot rows; deleting it
    while the rows survived would orphan them beyond any retry."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web_mod, app, env = _make_env(
            tmp, binding={"map_id": "default", "mode": "", "generation": None,
                          "source": "default"},
        )
        if app is None:
            return
        env.meta.write(make_meta("labx", "labx__s1", 1))
        env.store.rows["labx__s1"] = {"o1": _obj("o1")}
        real_delete = env.store.delete_map

        def failing_delete(map_id):
            if map_id == "labx__s1":
                raise RuntimeError("db locked")
            return real_delete(map_id)

        env.store.delete_map = failing_delete
        with _patched(web_mod, rpc=_rpc_ok):
            status, body = _call(app, "POST", "/api/maps/delete", {"map_id": "labx"})
        assert status == 200 and body["ok"], body
        assert "sidecar kept" in body.get("scene_cleanup_error", ""), body
        assert env.meta.read("labx") is not None         # pointer preserved
        assert "labx__s1" in env.store.rows              # rows still reachable
    print("  [PASS] test_delete_keeps_sidecar_when_snapshot_delete_fails")


if __name__ == "__main__":
    print("Running map facade epoch tests...\n")
    test_save_commits_fresh_partition_sidecar_then_purges()
    test_resave_rotates_partition_and_purges_previous()
    test_mapping_mode_resave_refused()
    test_save_refuses_overwriting_unloaded_annotations()
    test_save_excludes_self_robot_object()
    test_failed_snapshot_keeps_previous_sidecar_and_fails_the_save()
    test_load_without_sidecar_restores_nothing()
    test_load_restores_exactly_the_sidecar_partition()
    test_delete_removes_sidecar_and_both_partitions()
    test_reserved_map_ids_rejected()
    test_generation_threading_from_latched_broadcast()
    test_save_without_sidecar_store_fails_the_save()
    test_load_flushes_preload_live_objects()
    test_load_restore_failure_fails_load_and_keeps_previous_binding()
    test_load_flush_failure_aborts_before_any_rebind()
    test_occupancy_timeout_fails_load_and_blocks_save()
    test_fresh_save_snapshot_failure_names_workable_recovery()
    test_failed_load_blocks_save_until_successful_retry()
    test_delete_keeps_sidecar_when_snapshot_delete_fails()
    print("\nAll tests passed!")
