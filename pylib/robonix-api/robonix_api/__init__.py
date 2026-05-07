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

    from robonix_api import Capability

    cap = Capability(id="com.robonix.ranger.mid360_lidar",
                     namespace="primitive/lidar")

    @cap.on_init
    def init(cfg):
        cap.spawn(["ros2", "launch", "livox_ros_driver2",
                   "msg_MID360_launch.py"], log="livox.log")
        topic = cfg.get("lidar_topic", "/scanner/cloud")
        if not cap.wait_for_topic(topic, "PointCloud2", 30.0):
            return cap.error(f"no PointCloud2 on {topic}")
        cap.declare_ros2("primitive/lidar/lidar3d", topic)
        return cap.ready()

    if __name__ == "__main__":
        cap.run()
"""
from __future__ import annotations

from .capability import Capability
from .tool import mcp_contract

__all__ = ["Capability", "mcp_contract"]
