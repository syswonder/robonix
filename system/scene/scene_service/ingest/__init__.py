# SPDX-License-Identifier: MulanPSL-2.0
"""Ingest layer — async tasks that pull observations into the registry.

v2 (current): scene runs in its own docker container with rclpy on
the host DDS bus. ROS2 subscribers are the primary ingest path; VLM
perception consumes the latest RGB frame each tick and produces
Detections that go through state/data_assoc.

v1 (removed): MCP-based polling of primitive caps via atlas. MCP is
for pilot only; scene's own data fetching uses the fast direct path."""

from .perception_concept_graphs import ConceptGraphsDetector
from .perception_vlm import VLMObjectDetector
from .ros_subscribers import (
    DEFAULT_WEBOTS_TIAGO_TOPICS,
    SubscribersHub,
    TopicSpec,
)

__all__ = [
    "ConceptGraphsDetector",
    "DEFAULT_WEBOTS_TIAGO_TOPICS",
    "SubscribersHub",
    "TopicSpec",
    "VLMObjectDetector",
]
