# SPDX-License-Identifier: MulanPSL-2.0
"""Regression coverage for Scene's Atlas channel reconciliation."""

from unittest.mock import patch

from robonix_api.atlas_types import Transport

from scene_service import service
from scene_service.ingest.ros_subscribers import TopicSpec


def test_auto_discovery_does_not_reconnect_known_kinds():
    """Resolve only missing kinds so reconciliation cannot leak channels."""
    contracts = [
        ("rgb", "robonix/primitive/camera/rgb", "Image"),
        ("pose", "robonix/service/map/pose", "PoseWithCovarianceStamped"),
    ]
    pose_spec = TopicSpec("pose", "/pose", "PoseWithCovarianceStamped", "default")

    with (
        patch.object(service, "_SCENE_CONTRACTS", contracts),
        patch.object(service, "_DEFAULT_DISABLED_KINDS", frozenset()),
        patch.object(
            service,
            "_resolve_one_contract",
            return_value=pose_spec,
        ) as resolve,
    ):
        specs = service._resolve_auto(
            None,
            int(Transport.ROS2),
            known_kinds={"rgb"},
        )

    assert specs == [pose_spec]
    resolve.assert_called_once_with(
        Transport.ROS2,
        "pose",
        "robonix/service/map/pose",
        "PoseWithCovarianceStamped",
        provider_id="",
    )
