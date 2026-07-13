# SPDX-License-Identifier: MulanPSL-2.0
import time
from types import SimpleNamespace

import asyncio

from scene_service import mcp_tools
from scene_service.state.object_registry import BBox3D, Pose3D, SceneObject


class Registry:
    def __init__(self, objects):
        self.objects = objects

    async def snapshot(self):
        return self.objects, {}


class Annotations:
    map_id = "floor-3"

    def list(self):
        return [
            SimpleNamespace(
                annotation_id="315",
                kind="room",
                name="room 315",
                points=[[-1.0, -1.0], [2.0, -1.0], [2.0, 2.0], [-1.0, 2.0]],
            ),
            SimpleNamespace(
                annotation_id="open",
                kind="area",
                name="open area",
                points=[[-2.0, -2.0], [3.0, -2.0], [3.0, 3.0], [-2.0, 3.0]],
            ),
        ]


def scene_object(object_id, label, x, y, last_seen):
    return SceneObject(
        object_id=object_id,
        cls=label,
        pose=Pose3D(x, y, 0.0, yaw=0.4),
        bbox=BBox3D(),
        confidence=1.0,
        first_seen=last_seen,
        last_seen=last_seen,
    )


def test_robot_context_combines_pose_room_area_and_nearby_objects():
    now = time.time()
    objects = {
        "robot": scene_object("scene.object.robot_001", "robot", 0.5, 0.5, now),
        "banana": scene_object("scene.object.banana_001", "banana", 1.0, 0.5, now),
        "far": scene_object("scene.object.chair_001", "chair", 10.0, 10.0, now),
    }
    mcp_tools.attach_state(registry=Registry(objects))
    mcp_tools.attach_annotation_store(Annotations())

    response = asyncio.run(
        mcp_tools.get_robot_context(mcp_tools.GetRobotContext_Request())
    )

    assert response.pose_known and not response.stale
    assert response.map_id == "floor-3"
    assert response.room_id == "scene.room.315"
    assert response.room_name == "room 315"
    assert response.containing_area_names == ["open area", "room 315"]
    assert [item.id for item in response.nearby_objects] == [
        "scene.object.banana_001"
    ]
