# SPDX-License-Identifier: MulanPSL-2.0
"""Ingest layer — async tasks that pull observations into the registry.
v1 design: poll existing primitive caps via atlas-mediated channels
(MCP for image/scan tools, gRPC for chassis state) instead of native
ROS subscription. Reasoning: scene runs on host, primitives run inside
the Webots docker container; ROS DDS over host network is fragile,
while atlas's transport-aware Connect already gives us a portable
connection. When we eventually run scene on a real robot with rclpy
on host, native subscribers can be added alongside.

The Soma adapter (see service.py) reads `config.observations[]` to
decide which of these tasks to launch. Missing caps → task is silently
skipped (not an error)."""

from .poll_primitive import PrimitivePoller, ChassisStatePoller
from .perception_vlm import VLMObjectDetector

__all__ = ["PrimitivePoller", "ChassisStatePoller", "VLMObjectDetector"]
