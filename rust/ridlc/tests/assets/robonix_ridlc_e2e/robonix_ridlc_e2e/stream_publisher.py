import os

import grpc
import rclpy

from robonix.hal.localization import create_pose_publisher
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    node_id = os.environ.get("ROBONIX_TEST_STREAM_SERVER_ID", "odom_provider_1")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    publisher = create_pose_publisher(runtime_client, node_id=node_id)

    def publish_once() -> None:
        msg = publisher._msg_type()
        msg.header.frame_id = "map"
        msg.header.stamp = publisher.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.25
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0
        publisher.publish(msg)

    timer = publisher.create_timer(0.2, publish_once)

    try:
        rclpy.spin(publisher)
    finally:
        timer.cancel()
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
