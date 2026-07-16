# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for object persistence (warm restore).

The ObjectStore round-trip needs pymilvus + milvus-lite installed; it skips
cleanly when they aren't. The skip guards import BOTH `pymilvus` and
`milvus_lite`: the backend (`milvus_lite`) is Linux-only, so a dev mac can
have `pymilvus` importable yet no working store — guarding on `pymilvus`
alone passes the gate then crashes mid-test. The restore_object counter test
is pure Python and always runs.
"""
import os
import sys
import tempfile

# Ensure scene_service is importable without rclpy / codegen.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.state.object_registry import (  # noqa: E402
    BBox3D,
    ObjectRegistry,
    Pose3D,
    SceneObject,
)


def _make_obj(oid: str, cls: str, *, n: int = 3) -> SceneObject:
    return SceneObject(
        object_id=oid,
        cls=cls,
        pose=Pose3D(x=1.25, y=-2.5, z=0.75, yaw=0.5, frame_id="map"),
        bbox=BBox3D(size_x=0.2, size_y=0.3, size_z=0.4, yaw=0.7, frame_id="map"),
        confidence=0.8,
        first_seen=100.0,
        last_seen=200.0,
        observation_count=n,
        missing=False,
        attributes={"graspable": True, "movable": True, "source": "perception"},
    )


def _close(a: float, b: float, tol: float = 1e-3) -> bool:
    return abs(a - b) <= tol


def test_restore_object_counter_collision():
    """A new object of a restored class numbers after the highest restored id,
    never reusing or colliding with it."""
    reg = ObjectRegistry()
    reg.restore_object(_make_obj("scene.object.cup_005", "cup"))
    reg.restore_object(_make_obj("scene.object.cup_002", "cup"))

    new = reg.insert_object(
        "cup", Pose3D(0, 0, 0), BBox3D(), 0.5, now=1.0
    )
    assert new.object_id == "scene.object.cup_006", new.object_id
    # Restored objects are still present and addressable.
    assert reg.get_object("scene.object.cup_005") is not None
    print("  [PASS] test_restore_object_counter_collision")


def test_restore_object_unparseable_id():
    """An id without a numeric suffix is stored but doesn't touch the counter."""
    reg = ObjectRegistry()
    reg.restore_object(_make_obj("scene.object.weird", "thing"))
    new = reg.insert_object("thing", Pose3D(0, 0, 0), BBox3D(), 0.5, now=1.0)
    assert new.object_id == "scene.object.thing_001", new.object_id
    print("  [PASS] test_restore_object_unparseable_id")


def test_restore_object_clears_cg_uuid_and_flags_restored():
    """Restore drops the stale per-process concept-graphs uuid and flags the
    record `restored`, so the perception reconcile won't evict it before a live
    detection re-binds it."""
    reg = ObjectRegistry()
    obj = _make_obj("scene.object.cup_005", "cup")
    obj.attributes["cg_uuid"] = "stale-uuid-from-last-process"
    reg.restore_object(obj)
    got = reg.get_object("scene.object.cup_005")
    assert got.attributes.get("restored") is True
    assert "cg_uuid" not in got.attributes
    print("  [PASS] test_restore_object_clears_cg_uuid_and_flags_restored")


# ── perception reconcile (warm-restore re-binding) ───────────────────────────
def _make_detector(reg):
    """A ConceptGraphsDetector built via __new__ (bypassing __init__, which
    loads YOLO/SAM/CLIP), wired with only the state `_apply_snapshot` touches."""
    from scene_service.ingest.perception_concept_graphs import ConceptGraphsDetector

    p = ConceptGraphsDetector.__new__(ConceptGraphsDetector)
    p._registry = reg
    p._uuid_to_oid = {}
    p._world_frame_fn = lambda: "map"
    p.cfg = {"max_merge_dist_m": 1.5}
    # State added by the cross-tick re-bind / soft-eviction work that
    # _apply_snapshot now also touches.
    p._object_ttl_s = 30.0
    p._merge_class_group = {}
    return p


def _snap(uuid_s, cls, x, y, z=0.0, *, obs=4, conf=0.9):
    return {
        "uuid": uuid_s, "cls": cls, "confidence": conf, "obs": obs,
        "x": x, "y": y, "z": z, "yaw": 0.0,
        "size_x": 0.2, "size_y": 0.2, "size_z": 0.2,
    }


def _restore(reg, oid, cls, x, y, *, n=7, source="concept_graphs"):
    """Restore an object the way the persistence layer does (missing=True) at a
    known pose, then flag its source so the eviction path actually considers it."""
    obj = _make_obj(oid, cls, n=n)
    obj.pose = Pose3D(x, y, 0.0)
    obj.missing = True
    obj.attributes["source"] = source
    reg.restore_object(obj)
    return reg.get_object(oid)


def test_apply_snapshot_rebinds_restored_object():
    """A restored object re-observed within the merge gate keeps its id (no
    churn, no duplicate), flips out of `missing`, and binds the fresh uuid."""
    import asyncio
    try:
        from scene_service.ingest.perception_concept_graphs import ConceptGraphsDetector  # noqa: F401
    except Exception as e:  # numpy / perception deps absent
        print(f"  [SKIP] test_apply_snapshot_rebinds_restored_object ({e})")
        return

    reg = ObjectRegistry()
    _restore(reg, "scene.object.cup_005", "cup", 1.0, 1.0, n=7)
    p = _make_detector(reg)

    asyncio.run(p._apply_snapshot([_snap("u-new", "cup", 1.1, 1.0, obs=8)]))

    cups = [o for o in reg.all_objects() if o.cls == "cup"]
    assert len(cups) == 1, [o.object_id for o in cups]      # no duplicate minted
    cup = cups[0]
    assert cup.object_id == "scene.object.cup_005"          # stable id preserved
    assert cup.missing is False                             # re-observed
    assert cup.observation_count == 8
    assert "restored" not in cup.attributes                # flag consumed on bind
    assert cup.attributes.get("cg_uuid") == "u-new"
    assert p._uuid_to_oid["u-new"] == "scene.object.cup_005"
    print("  [PASS] test_apply_snapshot_rebinds_restored_object")


def test_apply_snapshot_restored_survives_and_far_detection_is_new():
    """An unseen restored object is exempt from uuid-membership eviction, and a
    same-class detection beyond the merge gate spawns a new id (numbered after
    the restored one) rather than stealing it."""
    import asyncio
    try:
        from scene_service.ingest.perception_concept_graphs import ConceptGraphsDetector  # noqa: F401
    except Exception as e:
        print(f"  [SKIP] test_apply_snapshot_restored_survives_and_far_detection_is_new ({e})")
        return

    reg = ObjectRegistry()
    _restore(reg, "scene.object.cup_005", "cup", 1.0, 1.0, n=7)
    p = _make_detector(reg)

    # A cup detected 5–6 m away — outside the 1.5 m gate — must not adopt cup_005.
    asyncio.run(p._apply_snapshot([_snap("u-far", "cup", 5.0, 5.0)]))

    survivor = reg.get_object("scene.object.cup_005")
    assert survivor is not None                             # not evicted
    assert survivor.attributes.get("restored") is True      # still awaiting re-bind
    assert survivor.missing is True
    new = [o for o in reg.all_objects()
           if o.cls == "cup" and o.object_id != "scene.object.cup_005"]
    assert len(new) == 1 and new[0].object_id == "scene.object.cup_006", \
        [o.object_id for o in new]
    print("  [PASS] test_apply_snapshot_restored_survives_and_far_detection_is_new")


def test_object_store_roundtrip():
    """persist → fresh ObjectStore → load_all preserves scalar state."""
    try:
        import milvus_lite  # noqa: F401
        import pymilvus  # noqa: F401
    except Exception:
        print("  [SKIP] test_object_store_roundtrip (pymilvus/milvus-lite not installed)")
        return

    from scene_service.persistence import ObjectStore

    objs = [_make_obj("scene.object.cup_001", "cup", n=4),
            _make_obj("scene.object.table_001", "table", n=9)]

    with tempfile.TemporaryDirectory() as d:
        db_path = os.path.join(d, "objects.db")
        store = ObjectStore(db_path)
        # No embedder wired → placeholder vectors; scalar state is what matters.
        written = store.persist([(o, f"a {o.cls}") for o in objs])
        assert written == 2, written
        store.close()

        store2 = ObjectStore(db_path)
        loaded = {o.object_id: o for o in store2.load_all()}
        store2.close()

    assert set(loaded) == {"scene.object.cup_001", "scene.object.table_001"}
    cup = loaded["scene.object.cup_001"]
    src = objs[0]
    assert cup.cls == "cup"
    assert cup.observation_count == 4
    assert cup.missing is True  # restored objects come back not-currently-seen
    assert _close(cup.pose.x, src.pose.x) and _close(cup.pose.y, src.pose.y)
    assert _close(cup.pose.yaw, src.pose.yaw)
    assert cup.pose.frame_id == "map"
    assert _close(cup.bbox.size_z, src.bbox.size_z)
    assert _close(cup.bbox.yaw, src.bbox.yaw)        # bbox orientation survives
    assert cup.bbox.frame_id == "map"                # reuses the pose frame
    assert _close(cup.confidence, src.confidence)
    assert cup.attributes.get("graspable") is True
    print("  [PASS] test_object_store_roundtrip")


def test_object_store_upsert_latest_wins():
    """Re-persisting the same object_id overwrites (no version history in v1)."""
    try:
        import milvus_lite  # noqa: F401
        import pymilvus  # noqa: F401
    except Exception:
        print("  [SKIP] test_object_store_upsert_latest_wins (pymilvus/milvus-lite not installed)")
        return

    from scene_service.persistence import ObjectStore

    with tempfile.TemporaryDirectory() as d:
        store = ObjectStore(os.path.join(d, "objects.db"))
        o = _make_obj("scene.object.cup_001", "cup", n=2)
        store.persist([(o, "a cup")])
        o.observation_count = 7
        o.confidence = 0.95
        store.persist([(o, "a cup")])
        loaded = store.load_all()
        store.close()

    assert len(loaded) == 1, len(loaded)
    assert loaded[0].observation_count == 7
    print("  [PASS] test_object_store_upsert_latest_wins")


def test_object_store_map_id_isolation():
    """Objects of one map_id are invisible to another, and the same object_id
    on two maps coexists without overwriting (composite primary key)."""
    try:
        import milvus_lite  # noqa: F401
        import pymilvus  # noqa: F401
    except Exception:
        print("  [SKIP] test_object_store_map_id_isolation (pymilvus/milvus-lite not installed)")
        return

    from scene_service.persistence import ObjectStore

    with tempfile.TemporaryDirectory() as d:
        db_path = os.path.join(d, "objects.db")
        # Same object_id "scene.object.cup_001" lives on two different maps.
        store_a = ObjectStore(db_path, map_id="kitchen")
        store_a.persist([(_make_obj("scene.object.cup_001", "cup", n=3), "kitchen cup")])
        store_a.close()

        store_b = ObjectStore(db_path, map_id="office")
        store_b.persist([(_make_obj("scene.object.cup_001", "cup", n=9), "office cup")])
        store_b.close()

        # Each map sees only its own object, with its own state.
        store_a2 = ObjectStore(db_path, map_id="kitchen")
        a_loaded = store_a2.load_all()
        store_a2.close()

        store_b2 = ObjectStore(db_path, map_id="office")
        b_loaded = store_b2.load_all()
        store_b2.close()

        # An unknown map is empty, not a cold-start error.
        store_c = ObjectStore(db_path, map_id="garage")
        c_loaded = store_c.load_all()
        store_c.close()

    assert len(a_loaded) == 1 and a_loaded[0].observation_count == 3, a_loaded
    assert len(b_loaded) == 1 and b_loaded[0].observation_count == 9, b_loaded
    assert c_loaded == [], c_loaded
    print("  [PASS] test_object_store_map_id_isolation")


def test_legacy_schema_recreated():
    """A collection left by an earlier draft (object_id primary key, no pk/
    map_id) is dropped + recreated with the composite pk on open, so a stale
    host-mounted DB can't silently defeat per-map isolation."""
    try:
        import milvus_lite  # noqa: F401
        from pymilvus import DataType, MilvusClient
    except Exception:
        print("  [SKIP] test_legacy_schema_recreated (pymilvus/milvus-lite not installed)")
        return

    from scene_service import persistence
    from scene_service.persistence import ObjectStore

    with tempfile.TemporaryDirectory() as d:
        db_path = os.path.join(d, "objects.db")
        # Hand-build the legacy schema: object_id is the primary key.
        client = MilvusClient(uri=db_path)
        schema = client.create_schema(enable_dynamic_field=True)
        schema.add_field("object_id", DataType.VARCHAR, max_length=128, is_primary=True)
        schema.add_field("embedding", DataType.FLOAT_VECTOR, dim=512)
        ip = client.prepare_index_params()
        ip.add_index(field_name="embedding", index_type="FLAT", metric_type="COSINE")
        client.create_collection(
            collection_name=persistence._COLLECTION, schema=schema, index_params=ip
        )
        client.close()
        from milvus_lite.server_manager import server_manager_instance
        server_manager_instance.release_server(db_path)

        # Opening via ObjectStore must recreate with the composite pk and work.
        store = ObjectStore(db_path, map_id="kitchen")
        assert store._has_composite_pk(), "expected recreated collection to have pk primary key"
        assert store._has_field("bbox_yaw"), "expected recreated collection to be schema-current"
        store.persist([(_make_obj("scene.object.cup_001", "cup", n=2), "cup")])
        loaded = store.load_all()
        store.close()

    assert len(loaded) == 1 and loaded[0].object_id == "scene.object.cup_001", loaded
    print("  [PASS] test_legacy_schema_recreated")


def test_map_id_sanitized():
    """A map_id with unsafe characters is squashed to a safe identifier so it
    can't break (or inject into) the milvus filter expression."""
    import re

    from scene_service.persistence import _sanitize_map_id

    inject = _sanitize_map_id('a" or "1"=="1')
    assert re.fullmatch(r"[A-Za-z0-9._\-]+", inject), inject  # no quotes/spaces survive
    assert _sanitize_map_id("  ") == "default"
    assert _sanitize_map_id(None) == "default"
    assert _sanitize_map_id("kitchen-2.floor_1") == "kitchen-2.floor_1"  # safe id untouched
    print("  [PASS] test_map_id_sanitized")


if __name__ == "__main__":
    print("Running persistence unit tests...\n")
    test_restore_object_counter_collision()
    test_restore_object_unparseable_id()
    test_restore_object_clears_cg_uuid_and_flags_restored()
    test_apply_snapshot_rebinds_restored_object()
    test_apply_snapshot_restored_survives_and_far_detection_is_new()
    test_map_id_sanitized()
    test_object_store_roundtrip()
    test_object_store_upsert_latest_wins()
    test_object_store_map_id_isolation()
    test_legacy_schema_recreated()
    print("\nAll tests passed!")
