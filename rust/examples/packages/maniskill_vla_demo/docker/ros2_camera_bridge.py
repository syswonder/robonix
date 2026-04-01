#!/usr/bin/env python3
"""ROS2 camera bridge: polls env_node gRPC and publishes to ROS2 sensor_msgs/Image.

Published topics (on /robonix/camera namespace):
  /robonix/camera/rgb     — sensor_msgs/Image  (rgb8)
  /robonix/camera/depth   — sensor_msgs/Image  (32FC1, meters)

Environment variables:
  ROBONIX_SERVER   grpc endpoint of env_node  (default: host.docker.internal:50052)
  BRIDGE_FPS       polling rate               (default: 10)
  FRAME_ID         camera frame_id            (default: camera_optical_frame)
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import grpc
import numpy as np

# proto stubs are in /proto_stubs (mounted or on PYTHONPATH)
import maniskill_env_pb2 as env_pb
import maniskill_env_pb2_grpc as env_pb_grpc


class CameraBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("robonix_camera_bridge")

        fps = float(os.environ.get("BRIDGE_FPS", "10"))
        server = os.environ.get("ROBONIX_SERVER", "host.docker.internal:50052")
        self._frame_id = os.environ.get("FRAME_ID", "camera_optical_frame")

        self.get_logger().info(f"Connecting to env gRPC at {server}  fps={fps}")
        channel = grpc.insecure_channel(server)
        self._stub = env_pb_grpc.EnvDataServiceStub(channel)

        self._pub_rgb = self.create_publisher(Image, "/robonix/camera/rgb", 5)
        self._pub_depth = self.create_publisher(Image, "/robonix/camera/depth", 5)

        self._seq = 0
        period = 1.0 / fps
        self.create_timer(period, self._poll)
        self.get_logger().info("ROS2 camera bridge ready")

    def _poll(self) -> None:
        try:
            obs = self._stub.GetObs(env_pb.Empty())
        except grpc.RpcError as e:
            self.get_logger().warn(f"GetObs failed: {e.details()}", throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()
        self._seq += 1

        # ── RGB image ────────────────────────────────────────────────────────
        if obs.rgb:
            rgb_msg = Image()
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = self._frame_id
            rgb_msg.height = obs.height
            rgb_msg.width = obs.width
            rgb_msg.encoding = "rgb8"
            rgb_msg.step = obs.width * 3
            rgb_msg.data = bytes(obs.rgb)
            self._pub_rgb.publish(rgb_msg)

        # ── Depth image ───────────────────────────────────────────────────────
        if obs.depth:
            depth_arr = np.frombuffer(obs.depth, dtype=np.float32)
            depth_msg = Image()
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self._frame_id
            depth_msg.height = obs.height
            depth_msg.width = obs.width
            depth_msg.encoding = "32FC1"
            depth_msg.step = obs.width * 4
            depth_msg.data = depth_arr.tobytes()
            self._pub_depth.publish(depth_msg)


def main() -> None:
    rclpy.init()
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
