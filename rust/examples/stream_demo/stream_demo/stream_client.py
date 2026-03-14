# SPDX-License-Identifier: MulanPSL-2.0
"""Stream demo: pose subscriber. Resolves channel from robonix-server and connects to stream_server.
Must be run via rbnx start."""

import sys
import time

import grpc
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from robonix.prm.base import create_pose_cov_subscriber
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    requester_id = "stream_demo_client"
    target = "stream_demo_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    subscriber = create_pose_cov_subscriber(runtime_client, requester_id=requester_id, target=target)

    received = []

    def on_msg(msg: PoseWithCovarianceStamped) -> None:
        received.append(msg)
        print(f"pose: x={msg.pose.pose.position.x:.2f} y={msg.pose.pose.position.y:.2f}", flush=True)

    subscriber.start(on_msg)

    deadline = time.time() + 15.0
    try:
        while time.time() < deadline:
            rclpy.spin_once(subscriber, timeout_sec=0.5)
            if received:
                break
    finally:
        subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not received:
        print("No message received (timeout). Start stream_server first.", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
