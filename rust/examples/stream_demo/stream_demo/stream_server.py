# SPDX-License-Identifier: MulanPSL-2.0
"""Stream demo: pose output server. Registers with robonix-server and publishes pose.
Must be run via rbnx start."""

import grpc
import rclpy

from robonix.prm.base import create_pose_cov_publisher
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    node_id = "stream_demo_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    publisher = create_pose_cov_publisher(runtime_client, node_id=node_id)

    def publish_once() -> None:
        msg = publisher._msg_type()
        msg.header.frame_id = "map"
        msg.header.stamp = publisher.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.25
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0
        publisher.publish(msg)

    timer = publisher.create_timer(0.5, publish_once)

    try:
        rclpy.spin(publisher)
    finally:
        timer.cancel()
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
