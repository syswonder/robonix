# SPDX-License-Identifier: MulanPSL-2.0
"""ObjectMutationCoordinator: epoch checks, guards, narrow invalidation.

Covers the decisions this feature encodes, not the plumbing:
  * an edit read under a stale map epoch is rejected, not applied;
  * the robot self-record is untouchable;
  * operator labels are stored verbatim (no silent canonicalization);
  * one edit invalidates only that object's graph-cache entries;
  * delete removes the record and notifies the detector; flush clears
    everything derived and only flush uses the full cache clear.
"""
from __future__ import annotations

import asyncio

import pytest

from scene_service.object_mutations import ObjectMutationCoordinator
from scene_service.state import BBox3D, ObjectRegistry, Pose3D


class StubStore:
    def __init__(self):
        self.invalidated: list[str] = []
        self.cleared = 0

    def invalidate_object(self, object_id: str) -> int:
        self.invalidated.append(object_id)
        return 1

    def clear_derived_state(self) -> None:
        self.cleared += 1


class StubDetector:
    def __init__(self):
        self.deleted: list[str] = []
        self.resets = 0

    async def delete_object(self, object_id: str) -> None:
        self.deleted.append(object_id)

    async def reset_derived_state(self) -> None:
        self.resets += 1


def make_coordinator(registry: ObjectRegistry, store: StubStore,
                     detector: StubDetector) -> ObjectMutationCoordinator:
    return ObjectMutationCoordinator(
        registry=registry,
        detector=detector,
        scene_graph_store=store,
        live_binding={"map_id": "m1", "generation": 3},
        ops_lock=asyncio.Lock(),
        semantic_hold={},
    )


def seed(registry: ObjectRegistry, cls: str = "chair") -> str:
    async def _seed() -> str:
        async with registry.lock():
            obj = registry.insert_object(
                cls=cls,
                pose=Pose3D(1.0, 2.0, 0.1, 0.0, "map"),
                bbox=BBox3D(0.5, 0.5, 0.9, 0.0, "map"),
                confidence=0.9,
                now=100.0,
            )
            return obj.object_id
    return asyncio.run(_seed())


def test_stale_epoch_is_rejected():
    registry = ObjectRegistry()
    store, detector = StubStore(), StubDetector()
    coord = make_coordinator(registry, store, detector)
    oid = seed(registry)
    with pytest.raises(RuntimeError, match="epoch changed"):
        asyncio.run(coord.update_label(
            object_id=oid, label="stool",
            expected_map_id="m1", expected_generation=2,   # stale
            persist_to_snapshot=False,
        ))
    assert store.invalidated == [] and store.cleared == 0


def test_operator_label_is_verbatim():
    registry = ObjectRegistry()
    store, detector = StubStore(), StubDetector()
    coord = make_coordinator(registry, store, detector)
    oid = seed(registry)
    updated, persisted, map_id, generation = asyncio.run(coord.update_label(
        object_id=oid, label="desk",
        expected_map_id="m1", expected_generation=3,
        persist_to_snapshot=False,
    ))
    # "desk" must not be canonicalized into "table" or anything else.
    assert updated.cls == "desk"
    assert updated.attributes["operator_label"] == "desk"
    assert (persisted, map_id, generation) == (False, "m1", 3)
    # Narrow invalidation: this object only, no full clear.
    assert store.invalidated == [oid] and store.cleared == 0


def test_robot_record_is_untouchable():
    registry = ObjectRegistry()
    store, detector = StubStore(), StubDetector()
    coord = make_coordinator(registry, store, detector)
    oid = seed(registry)

    async def _mark_robot():
        async with registry.lock():
            registry.get_object(oid).attributes["is_robot"] = True
    asyncio.run(_mark_robot())

    with pytest.raises(ValueError, match="robot"):
        asyncio.run(coord.delete_object(
            object_id=oid,
            expected_map_id="m1", expected_generation=3,
            persist_to_snapshot=False,
        ))


def test_geometry_override_marks_provenance_and_survives_frame_check():
    registry = ObjectRegistry()
    store, detector = StubStore(), StubDetector()
    coord = make_coordinator(registry, store, detector)
    oid = seed(registry)
    with pytest.raises(ValueError, match="frame mismatch"):
        asyncio.run(coord.update_geometry(
            object_id=oid, x=2.0, y=2.0, z=0.2, yaw=0.0,
            size_x=0.4, size_y=0.4, size_z=0.8, frame_id="odom",
            expected_map_id="m1", expected_generation=3,
            persist_to_snapshot=False,
        ))
    updated, _, _, _ = asyncio.run(coord.update_geometry(
        object_id=oid, x=2.0, y=2.0, z=0.2, yaw=0.0,
        size_x=0.4, size_y=0.4, size_z=0.8, frame_id="map",
        expected_map_id="m1", expected_generation=3,
        persist_to_snapshot=False,
    ))
    assert updated.attributes["operator_geometry"] is True
    assert updated.attributes["navigation_grade"] is False
    assert updated.pose.x == 2.0
    assert store.invalidated == [oid] and store.cleared == 0


def test_delete_notifies_detector_and_removes_record():
    registry = ObjectRegistry()
    store, detector = StubStore(), StubDetector()
    coord = make_coordinator(registry, store, detector)
    oid = seed(registry)
    deleted_id, persisted, _, _ = asyncio.run(coord.delete_object(
        object_id=oid,
        expected_map_id="m1", expected_generation=3,
        persist_to_snapshot=False,
    ))
    assert deleted_id == oid and persisted is False
    assert detector.deleted == [oid]

    async def _lookup():
        async with registry.lock():
            return registry.get_object(oid)
    assert asyncio.run(_lookup()) is None
    assert store.invalidated == [oid] and store.cleared == 0


def test_flush_clears_derived_state_and_uses_full_cache_clear():
    registry = ObjectRegistry()
    store, detector = StubStore(), StubDetector()
    coord = make_coordinator(registry, store, detector)
    seed(registry, "chair")
    seed(registry, "mug")
    count, persisted, _, _ = asyncio.run(coord.flush_objects(
        expected_map_id="m1", expected_generation=3,
        persist_to_snapshot=False,
    ))
    assert count == 2 and persisted is False
    assert detector.resets == 1
    # Flush is the one path allowed to wipe the whole derived cache.
    assert store.cleared == 1 and store.invalidated == []


def test_semantic_hold_blocks_mutations():
    registry = ObjectRegistry()
    store, detector = StubStore(), StubDetector()
    coord = make_coordinator(registry, store, detector)
    oid = seed(registry)
    coord.semantic_hold["reason"] = "map swap in progress"
    with pytest.raises(RuntimeError, match="not mutation-safe"):
        asyncio.run(coord.update_label(
            object_id=oid, label="stool",
            expected_map_id="m1", expected_generation=3,
            persist_to_snapshot=False,
        ))
