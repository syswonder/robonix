# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for object-identity stability (cross-tick re-bind / soft eviction).

Runs without ROS2, Docker, atlas, LLM, or perception models — pure Python over
ObjectRegistry and ConceptGraphsDetector._apply_snapshot. Mirrors the plain
`assert` + `__main__` style of test_scene_graph.py.
"""
import asyncio
import os
import sys
import time

# Ensure scene_service is importable without rclpy / codegen.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.state.object_registry import BBox3D, ObjectRegistry, Pose3D


# ── registry-level helpers ────────────────────────────────────────────────

def _insert(reg: ObjectRegistry, cls: str, xyz, *, now: float):
    """Insert a perception object at `xyz` and return it (test helper).

    Bypasses the asyncio lock — single-threaded test, no contention."""
    return reg.insert_object(
        cls,
        Pose3D(*xyz),
        BBox3D(0.2, 0.2, 0.2),
        confidence=0.9,
        now=now,
        source="concept_graphs",
    )


def test_find_rebindable():
    """Nearest same-class match within max_d, honoring only_missing + excludes."""
    reg = ObjectRegistry()
    now = time.time()
    kb = _insert(reg, "keyboard", (1.0, 0.0, 0.7), now=now)
    _insert(reg, "monitor", (1.0, 0.0, 0.9), now=now)  # wrong class
    far = _insert(reg, "keyboard", (5.0, 0.0, 0.7), now=now)  # out of gate

    # Live (not missing): only_missing=True must skip it.
    assert reg.find_rebindable("keyboard", Pose3D(1.05, 0.0, 0.7), 1.5,
                               only_missing=True) is None
    # Without only_missing it matches the near keyboard, not the far one.
    m = reg.find_rebindable("keyboard", Pose3D(1.05, 0.0, 0.7), 1.5)
    assert m is kb, "expected nearest in-gate same-class match"
    # Distance gate excludes the far keyboard even when it's the only candidate.
    assert reg.find_rebindable("keyboard", Pose3D(5.0, 0.0, 0.7), 1.5) is far
    # A 0.5 m gate around the origin reaches neither keyboard (1.0 / 5.0 away).
    assert reg.find_rebindable("keyboard", Pose3D(0.0, 0.0, 0.7), 0.5) is None
    # exclude_oids skips an already-claimed record.
    assert reg.find_rebindable("keyboard", Pose3D(1.05, 0.0, 0.7), 1.5,
                               exclude_oids={kb.object_id}) is None
    print("  [PASS] test_find_rebindable")


def test_soft_evict():
    """soft_evict marks missing + releases cg_uuid but keeps the record."""
    reg = ObjectRegistry()
    now = time.time()
    kb = _insert(reg, "keyboard", (1.0, 0.0, 0.7), now=now)
    kb.attributes["cg_uuid"] = "u1"
    reg.soft_evict(kb)
    assert kb.missing is True
    assert "cg_uuid" not in kb.attributes
    assert reg.get_object(kb.object_id) is kb, "record must be retained, not deleted"
    # Now visible to only_missing re-bind.
    assert reg.find_rebindable("keyboard", Pose3D(1.0, 0.0, 0.7), 1.5,
                               only_missing=True) is kb
    print("  [PASS] test_soft_evict")


def test_prune_expired():
    """prune_expired deletes stale missing records; spares fresh/restored/robot."""
    reg = ObjectRegistry()
    now = time.time()
    stale = _insert(reg, "keyboard", (1.0, 0.0, 0.7), now=now - 100)
    stale.missing = True
    fresh = _insert(reg, "keyboard", (2.0, 0.0, 0.7), now=now)
    fresh.missing = True  # missing but recently seen
    restored = _insert(reg, "table", (3.0, 0.0, 0.4), now=now - 100)
    restored.missing = True
    restored.attributes["restored"] = True
    robot = _insert(reg, "robot", (0.0, 0.0, 0.0), now=now - 100)
    robot.missing = True
    robot.attributes["is_robot"] = True

    deleted = reg.prune_expired(now, ttl_s=30.0)
    assert stale.object_id in deleted, "stale missing record should be pruned"
    assert reg.get_object(fresh.object_id) is fresh, "fresh missing record spared"
    assert reg.get_object(restored.object_id) is restored, "restored record spared"
    assert reg.get_object(robot.object_id) is robot, "robot self spared"
    print("  [PASS] test_prune_expired")


def test_parse_merge_class_groups():
    """SCENE_CG_MERGE_CLASS_GROUPS parsing into a class->bucket map."""
    from scene_service.ingest.perception_concept_graphs import (
        _parse_merge_class_groups,
    )

    assert _parse_merge_class_groups("") == {}
    g = _parse_merge_class_groups("chair, table ,desk; sofa,couch")
    assert g["chair"] == g["table"] == g["desk"], "group members share a bucket"
    assert g["sofa"] == g["couch"]
    assert g["chair"] != g["sofa"], "distinct groups, distinct buckets"
    # Single-member / empty groups are ignored (cannot reclassify alone).
    assert _parse_merge_class_groups("lonely") == {}
    # Order within a group does not change the bucket key.
    assert (_parse_merge_class_groups("a,b")["a"]
            == _parse_merge_class_groups("b,a")["a"])
    print("  [PASS] test_parse_merge_class_groups")


# ── _apply_snapshot integration (no models) ───────────────────────────────

def _make_detector(registry):
    """Construct a ConceptGraphsDetector with stub I/O callables. Only the
    registry + reconcile state are exercised; no models are loaded."""
    from scene_service.ingest.perception_concept_graphs import ConceptGraphsDetector

    async def _noop(_):
        return None

    return ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None,
        depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None,
        on_detections=_noop,
        registry=registry,
        world_frame_fn=lambda: "map",
    )


def _snap(uuid, cls, xyz, conf=0.9):
    x, y, z = xyz
    return {
        "uuid": uuid, "cls": cls,
        "x": x, "y": y, "z": z, "yaw": 0.0,
        "size_x": 0.2, "size_y": 0.2, "size_z": 0.2,
        "confidence": conf,
    }


def test_apply_snapshot_cross_tick_rebind():
    """A culled object re-detected under a NEW uuid keeps its id + obs count
    instead of resetting to obs=1 — the core 9→1-collapse fix."""
    reg = ObjectRegistry()
    det = _make_detector(reg)
    run = asyncio.new_event_loop().run_until_complete

    # Tick 1 + 2: same uuid u1 seen twice → one record, obs climbs to 2.
    run(det._apply_snapshot([_snap("u1", "keyboard", (1.0, 0.0, 0.7))]))
    run(det._apply_snapshot([_snap("u1", "keyboard", (1.0, 0.0, 0.7))]))
    objs = [o for o in reg.all_objects() if o.cls == "keyboard"]
    assert len(objs) == 1, f"expected 1 keyboard, got {len(objs)}"
    oid, obs = objs[0].object_id, objs[0].observation_count
    assert obs == 2, f"expected obs=2, got {obs}"

    # Tick 3: u1 vanishes (cull/uuid-churn) → soft-evicted, NOT deleted.
    run(det._apply_snapshot([]))
    survivor = reg.get_object(oid)
    assert survivor is not None, "soft-evicted record must survive"
    assert survivor.missing is True
    assert survivor.observation_count == 2, "obs preserved across soft eviction"

    # Tick 4: re-detected under a fresh uuid u2 → re-binds the SAME id, obs→3.
    run(det._apply_snapshot([_snap("u2", "keyboard", (1.02, 0.0, 0.7))]))
    objs = [o for o in reg.all_objects() if o.cls == "keyboard"]
    assert len(objs) == 1, f"re-detect must not duplicate; got {len(objs)}"
    assert objs[0].object_id == oid, "id must be stable across the uuid gap"
    assert objs[0].observation_count == 3, "obs count must continue, not reset"
    assert objs[0].missing is False
    assert objs[0].attributes.get("cg_uuid") == "u2", "rebound to the new uuid"
    print("  [PASS] test_apply_snapshot_cross_tick_rebind")


def test_apply_snapshot_ttl_prune():
    """A soft-evicted object not re-detected within the TTL is hard-pruned."""
    reg = ObjectRegistry()
    det = _make_detector(reg)
    det._object_ttl_s = 30.0
    run = asyncio.new_event_loop().run_until_complete

    run(det._apply_snapshot([_snap("u1", "keyboard", (1.0, 0.0, 0.7))]))
    oid = next(o.object_id for o in reg.all_objects() if o.cls == "keyboard")

    # Soft-evict, then backdate last_seen past the TTL.
    run(det._apply_snapshot([]))
    reg.get_object(oid).last_seen = time.time() - 100.0

    # Next reconcile runs prune_expired → record is gone.
    run(det._apply_snapshot([]))
    assert reg.get_object(oid) is None, "stale missing record should be pruned"
    print("  [PASS] test_apply_snapshot_ttl_prune")


if __name__ == "__main__":
    print("Running object-identity unit tests...\n")
    test_find_rebindable()
    test_soft_evict()
    test_prune_expired()
    test_parse_merge_class_groups()
    test_apply_snapshot_cross_tick_rebind()
    test_apply_snapshot_ttl_prune()
    print("\nAll tests passed!")
