# SPDX-License-Identifier: MulanPSL-2.0
"""Operator corrections are immune to staleness eviction, and the graph
store invalidates narrowly by object."""
from __future__ import annotations

import asyncio

from scene_service.state import BBox3D, ObjectRegistry, Pose3D


def _seed(registry: ObjectRegistry, cls: str = "chair"):
    async def go():
        async with registry.lock():
            return registry.insert_object(
                cls=cls,
                pose=Pose3D(1.0, 1.0, 0.1, 0.0, "map"),
                bbox=BBox3D(0.5, 0.5, 0.9, 0.0, "map"),
                confidence=0.9,
                now=100.0,
            )
    return asyncio.run(go())


def test_operator_geometry_skips_mark_stale():
    registry = ObjectRegistry()
    plain = _seed(registry, "chair")
    corrected = _seed(registry, "box")
    corrected.attributes["operator_geometry"] = True
    # Both silent far beyond the grace period.
    registry.mark_stale(now=100.0 + registry.grace_period_s + 60.0)
    assert plain.missing is True
    assert corrected.missing is False


def test_operator_records_skip_ttl_prune():
    registry = ObjectRegistry()
    labeled = _seed(registry, "chair")
    moved = _seed(registry, "box")
    victim = _seed(registry, "mug")
    labeled.attributes["operator_label"] = "desk"
    moved.attributes["operator_geometry"] = True
    for obj in (labeled, moved, victim):
        obj.missing = True
    doomed = registry.prune_expired(now=100.0 + 3600.0, ttl_s=30.0)
    assert doomed == [victim.object_id]
    assert registry.get_object(labeled.object_id) is not None
    assert registry.get_object(moved.object_id) is not None


def test_store_invalidates_one_object_only():
    from scene_service.scene_graph.store import SceneGraphStore

    store = SceneGraphStore.__new__(SceneGraphStore)
    store._caption_cache = {
        "a:chair:0": "a chair",
        "b:mug:0": "a mug",
    }
    store._relation_cache = {
        "a__b__sig__f": "edge-ab",
        "b__c__sig__f": "edge-bc",
        "c__d__sig__f": "edge-cd",
    }
    dropped = store.invalidate_object("b")
    assert dropped == 3
    assert store._caption_cache == {"a:chair:0": "a chair"}
    assert store._relation_cache == {"c__d__sig__f": "edge-cd"}
