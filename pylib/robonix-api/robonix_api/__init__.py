# SPDX-License-Identifier: MulanPSL-2.0
"""robonix-api — Python helpers for writing robonix capabilities.

Two layers:

  Layer 1 — atlas client + lifecycle gRPC + subprocess + sentinel helpers.
            What every package needs regardless of which middleware it uses.

  Layer 2 — opt-in convenience wrappers over rclpy / FastMCP / grpcio.
            Skip them if you'd rather use those libs directly; just call
            `cap.declare_ros2 / declare_grpc / declare_mcp` to register with
            atlas after spinning your own server.

Typical usage:

    from robonix_api import Capability, Ok, Err, Deferred

    cap = Capability(id="mid360_lidar",
                     namespace="robonix/primitive/lidar")

    @cap.on_init
    def init(cfg: dict):
        topic = cfg.get("lidar_topic", "/scanner/cloud")
        if not cap.wait_for_topic(topic, "PointCloud2", 30.0):
            return Deferred(f"no PointCloud2 on {topic} yet")
        cap.declare_ros2("robonix/primitive/lidar/lidar3d", topic)
        return Ok()

    if __name__ == "__main__":
        cap.run()
"""
from __future__ import annotations

from .atlas_types import (
    CapabilityRecord,
    CapabilityState,
    Channel,
    ContractDescriptor,
    InterfaceMetadata,
    Transport,
)
from .capability import Capability
from .result import Deferred, Err, Ok, Result
from .tool import mcp_contract

__all__ = [
    "Capability",
    # Lifecycle return type — every @cap.on_* handler returns one of these.
    "Ok", "Err", "Deferred", "Result",
    # Atlas dataclass mirrors — return values of cap.find / find_one / connect.
    "Transport", "CapabilityState",
    "InterfaceMetadata", "CapabilityRecord", "ContractDescriptor", "Channel",
    # MCP decorator (standalone form — `@cap.mcp` is sugar over this).
    "mcp_contract",
]
