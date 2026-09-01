# SPDX-License-Identifier: MulanPSL-2.0
"""The v1 correction contract has to be callable from its own read side.

UpdateObjectGeometry demands extents and a frame and rejects a frame that
differs from the object's own, so ListObjects must publish both; without them
an operator can only guess. These tests pin that round trip, plus the two
epoch/provenance fields that let a caller tell what it is editing and say why.
"""
from __future__ import annotations

import asyncio

import pytest

from scene_service.object_mutations import ObjectMutationCoordinator
from scene_service.state import BBox3D, ObjectRegistry, Pose3D


class StubStore:
    def invalidate_object(self, object_id: str) -> int:
        return 1

    def clear_derived_state(self) -> None:
        pass


def make_registry() -> tuple[ObjectRegistry, str]:
    registry = ObjectRegistry()

    async def _seed() -> str:
        async with registry.lock():
            obj = registry.insert_object(
                cls="chair",
                pose=Pose3D(1.0, 2.0, 0.1, 0.25, "map"),
                bbox=BBox3D(0.4, 0.6, 0.9, 0.25, "map"),
                confidence=0.9,
                now=100.0,
            )
            return obj.object_id

    return registry, asyncio.run(_seed())


def make_coordinator(registry, generation=3) -> ObjectMutationCoordinator:
    return ObjectMutationCoordinator(
        registry=registry,
        detector=object(),
        scene_graph_store=StubStore(),
        live_binding={"map_id": "m1", "generation": generation},
        ops_lock=asyncio.Lock(),
        semantic_hold={},
    )


def test_read_side_publishes_what_the_write_side_demands():
    """The extents and frame UpdateObjectGeometry requires are readable."""
    from scene_service.mcp_tools import _to_idl

    registry, oid = make_registry()
    obj = asyncio.run(_get(registry, oid))
    idl = _to_idl(obj)
    assert (idl.size_x, idl.size_y, idl.size_z) == (0.4, 0.6, 0.9)
    assert idl.frame_id == "map"
    # Resending exactly what was read must satisfy the frame check.
    coord = make_coordinator(registry)
    updated, _persisted, _m, _g = asyncio.run(coord.update_geometry(
        object_id=oid,
        x=idl.x, y=idl.y, z=idl.z, yaw=idl.yaw,
        size_x=idl.size_x, size_y=idl.size_y, size_z=idl.size_z,
        frame_id=idl.frame_id,
        expected_map_id="m1", expected_generation=3,
        persist_to_snapshot=False,
    ))
    assert updated.bbox.size_y == pytest.approx(0.6)


def test_observation_count_is_published():
    from scene_service.mcp_tools import _to_idl

    registry, oid = make_registry()
    idl = _to_idl(asyncio.run(_get(registry, oid)))
    assert idl.observation_count >= 1


def test_generation_supported_reports_a_weak_epoch():
    registry, _oid = make_registry()
    assert make_coordinator(registry).generation_supported() is True
    assert make_coordinator(registry, generation=None).generation_supported() is False


def test_note_is_stored_and_cleared_with_the_override():
    registry, oid = make_registry()
    coord = make_coordinator(registry)
    updated, *_ = asyncio.run(coord.update_label(
        object_id=oid, label="stool",
        expected_map_id="m1", expected_generation=3,
        persist_to_snapshot=False, note="  detector keeps calling it a table  ",
    ))
    assert updated.attributes["operator_note"] == "detector keeps calling it a table"
    cleared, *_ = asyncio.run(coord.update_label(
        object_id=oid, label="", clear_override=True,
        expected_map_id="m1", expected_generation=3,
        persist_to_snapshot=False,
    ))
    assert "operator_note" not in cleared.attributes


def test_label_length_is_checked_even_when_ignored():
    """A label that is going to be dropped still must not be unbounded."""
    registry, oid = make_registry()
    coord = make_coordinator(registry)
    with pytest.raises(ValueError, match="128 characters"):
        asyncio.run(coord.update_label(
            object_id=oid, label="x" * 200, clear_override=True,
            expected_map_id="m1", expected_generation=3,
            persist_to_snapshot=False,
        ))


async def _get(registry, oid):
    async with registry.lock():
        return registry.get_object(oid)
