# SPDX-License-Identifier: MulanPSL-2.0
"""Perception is gated on viewpoint change, not on a clock.

A detection costs one round-trip to a vision model, so re-describing a view
the detector has already described is the single largest avoidable cost in an
episode. These tests pin when the loop decides a call is worth making.
"""
import numpy as np

from scene_service.ingest.perception_vlm import VLMObjectDetector


def _pose(x=0.0, y=0.0, yaw=0.0):
    """A camera-to-world transform at (x, y) rotated `yaw` about z."""
    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    transform[:3, 3] = [x, y, 0.0]
    return transform


async def _ignore(_detections):
    """Accept projected detections without side effects."""


def _detector(**env):
    return VLMObjectDetector(
        rgb_fetcher=lambda: None,
        camera_to_world_fn=lambda: None,
        on_detections=_ignore,
    )


def test_first_view_is_always_perceived():
    detector = _detector()

    assert detector._should_perceive(_pose()) is True


def test_an_unchanged_view_is_skipped():
    detector = _detector()
    detector._last_view = _pose()
    detector._last_view_at = 1e9  # far in the future: staleness cannot fire

    assert detector._should_perceive(_pose()) is False


def test_translation_past_the_epsilon_is_perceived():
    detector = _detector()
    detector._last_view = _pose()
    detector._last_view_at = 1e9

    assert detector._should_perceive(_pose(x=0.01)) is False
    assert detector._should_perceive(_pose(x=1.0)) is True


def test_rotation_past_the_epsilon_is_perceived():
    detector = _detector()
    detector._last_view = _pose()
    detector._last_view_at = 1e9

    assert detector._should_perceive(_pose(yaw=0.001)) is False
    # A robot that turns in place sees an entirely different scene without
    # moving, so rotation must count on its own.
    assert detector._should_perceive(_pose(yaw=0.6)) is True


def test_a_stale_description_is_refreshed_from_the_same_place():
    detector = _detector()
    detector._last_view = _pose()
    detector._last_view_at = 0.0  # long ago against a monotonic clock

    # The world can change while the robot stands still, so an old
    # description is refreshed even from an identical viewpoint.
    assert detector._should_perceive(_pose()) is True


def test_unknown_geometry_does_not_suppress_perception():
    detector = _detector()
    detector._last_view = _pose()
    detector._last_view_at = 1e9

    assert detector._should_perceive(None) is True
    # A transform of the wrong shape is not a basis for skipping either.
    assert detector._should_perceive(np.eye(3)) is True
    assert detector._should_perceive("not a transform") is True


def test_thresholds_are_configurable(monkeypatch):
    monkeypatch.setenv("SCENE_DETECT_MIN_MOVE_M", "2.0")
    detector = _detector()
    detector._last_view = _pose()
    detector._last_view_at = 1e9

    # One metre no longer counts as movement under a two-metre epsilon.
    assert detector._should_perceive(_pose(x=1.0)) is False
    assert detector._should_perceive(_pose(x=3.0)) is True


def test_an_unusable_threshold_falls_back_to_the_default(monkeypatch):
    monkeypatch.setenv("SCENE_DETECT_MIN_MOVE_M", "-1")
    monkeypatch.setenv("SCENE_DETECT_MAX_STALE_S", "not-a-number")
    detector = _detector()

    assert detector._move_epsilon_m == 0.15
    assert detector._max_stale_s == 30.0
