import asyncio
from types import SimpleNamespace

import pytest

from scene_service import mcp_tools


class _Store:
    map_id = "3f_demo"

    def list(self):
        return [
            SimpleNamespace(
                annotation_id="anno.315",
                kind="room",
                name="room 315",
                points=[(0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (0.0, 3.0)],
                theta=0.5,
                stale=False,
                stale_reason="",
                updated_at=123.0,
            ),
            SimpleNamespace(
                annotation_id="anno.poi",
                kind="poi",
                name="charging spot",
                points=[(1.0, 1.0)],
                theta=None,
                stale=False,
                stale_reason="",
                updated_at=124.0,
            ),
        ]


def test_list_regions_returns_rooms_with_goal_room_ids_and_full_geometry():
    mcp_tools.attach_annotation_store(_Store())
    response = asyncio.run(mcp_tools.list_regions(mcp_tools.ListRegions_Request()))

    assert response.map_id == "3f_demo"
    assert len(response.regions) == 1
    region = response.regions[0]
    assert region.id == "scene.room.anno.315"
    assert region.kind == "room"
    assert region.name == "room 315"
    assert region.points_xy == [0.0, 0.0, 2.0, 0.0, 2.0, 3.0, 0.0, 3.0]
    assert region.stale is False


def test_list_regions_reports_annotation_store_unavailable():
    mcp_tools.attach_annotation_store(None)
    with pytest.raises(RuntimeError, match="annotation store is unavailable"):
        asyncio.run(mcp_tools.list_regions(mcp_tools.ListRegions_Request()))


class _Registry:
    async def snapshot(self):
        return {}, {}


def test_list_objects_keeps_room_entries_for_v1_compatibility():
    mcp_tools.attach_state(registry=_Registry())
    mcp_tools.attach_annotation_store(_Store())
    response = asyncio.run(mcp_tools.list_objects(mcp_tools.ListObjects_Request()))
    assert [obj.id for obj in response.objects] == ["scene.room.anno.315"]


class _StaleStore(_Store):
    def list(self):
        rooms = super().list()
        rooms[0].stale = True
        rooms[0].stale_reason = "map epoch changed"
        return rooms


def test_goal_room_rejects_stale_geometry_before_navigation():
    mcp_tools.attach_annotation_store(_StaleStore())
    response = asyncio.run(
        mcp_tools.goal_room(mcp_tools.GoalRoom_Request(room_id="scene.room.anno.315"))
    )
    assert response.reachable is False
    assert "is stale" in response.reason
    assert "map epoch changed" in response.reason
