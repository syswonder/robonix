# SPDX-License-Identifier: MulanPSL-2.0
"""The tick decides, per frame, whether to spend a vision-model call.

`test_perception_gating.py` pins the decision itself; these tests pin that the
loop acts on it — that a skipped view costs nothing downstream, and that a view
is only remembered once a call has actually described it.
"""
import asyncio

import numpy as np

from scene_service.ingest.perception_vlm import VLMObjectDetector


def _pose(x=0.0, y=0.0, yaw=0.0):
    transform = np.eye(4)
    transform[:3, 3] = [x, y, 0.0]
    transform[0, 0] = np.cos(yaw)
    transform[0, 1] = -np.sin(yaw)
    transform[1, 0] = np.sin(yaw)
    transform[1, 1] = np.cos(yaw)
    return transform


def _detector(view, *, frames):
    """A detector whose only side effect is counting frame fetches."""
    detector = VLMObjectDetector(
        rgb_fetcher=lambda: frames.append(1) or None,
        camera_to_world_fn=lambda: (view(), "map"),
        on_detections=None,
    )
    return detector


def test_an_unchanged_view_does_not_even_fetch_a_frame():
    frames = []
    detector = _detector(lambda: _pose(), frames=frames)
    detector._last_view = _pose()
    detector._last_view_at = 1e9

    asyncio.run(detector._tick())

    # The gate runs before the image is fetched, so a skipped view costs
    # neither an encode nor a request.
    assert frames == []


def test_a_moved_view_proceeds_to_the_frame():
    frames = []
    detector = _detector(lambda: _pose(x=2.0), frames=frames)
    detector._last_view = _pose()
    detector._last_view_at = 1e9

    asyncio.run(detector._tick())

    assert frames == [1]


def test_a_view_is_not_remembered_when_no_frame_arrived():
    detector = _detector(lambda: _pose(x=2.0), frames=[])
    detector._last_view = _pose()
    detector._last_view_at = 1e9

    asyncio.run(detector._tick())

    # `rgb_fetcher` returned None, so nothing described this viewpoint and the
    # next tick must be free to try it again.
    assert np.allclose(np.asarray(detector._last_view), _pose())


def test_chassis_pose_is_used_when_no_transform_is_available():
    detector = VLMObjectDetector(
        rgb_fetcher=lambda: None,
        camera_to_world_fn=lambda: None,
        on_detections=None,
    )
    detector.chassis_pose_fn = lambda: (1.0, 2.0, 0.5, 0.0)

    assert detector._viewpoint() == (1.0, 2.0, 0.5, 0.0)


def test_a_failing_pose_source_does_not_stop_perception():
    def explode():
        raise RuntimeError("tf lookup failed")

    detector = VLMObjectDetector(
        rgb_fetcher=lambda: None,
        camera_to_world_fn=explode,
        on_detections=None,
    )

    assert detector._viewpoint() is None
    # An unknown viewpoint is a reason to perceive, never to go quiet.
    assert detector._should_perceive(None) is True
