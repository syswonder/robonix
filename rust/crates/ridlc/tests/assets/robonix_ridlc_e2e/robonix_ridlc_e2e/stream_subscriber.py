import os
import time

import grpc
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from robonix_runtime_pb2 import ResolveStreamRequest
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def resolve_topic(runtime_client, requester_id: str, target: str) -> str:
    deadline = time.time() + 10.0
    last_error = None
    while time.time() < deadline:
        try:
            response = runtime_client.ResolveStream(
                ResolveStreamRequest(
                    requester_id=requester_id,
                    target=target,
                    namespace="robonix/hal/localization",
                    stream_name="pose",
                    role="consumer",
                )
            )
            return response.channel_name
        except grpc.RpcError as exc:
            last_error = exc
            time.sleep(0.2)
    raise RuntimeError(f"failed to resolve stream topic: {last_error}")


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    requester_id = os.environ.get("ROBONIX_TEST_STREAM_REQUESTER_ID", "odom_subscriber_1")
    target = os.environ.get("ROBONIX_TEST_STREAM_SERVER_ID", "odom_provider_1")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    topic_name = resolve_topic(runtime_client, requester_id=requester_id, target=target)

    rclpy.init()
    node = Node("odom_stream_subscriber")
    received = []

    def on_msg(msg: PoseWithCovarianceStamped) -> None:
        received.append(msg)
        print(f"{msg.pose.pose.position.x:.2f},{msg.pose.pose.position.y:.2f}", flush=True)

    sub = node.create_subscription(PoseWithCovarianceStamped, topic_name, on_msg, 10)
    deadline = time.time() + 10.0
    try:
        while time.time() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not received:
        raise RuntimeError(f"timed out waiting for stream message on {topic_name!r}")


if __name__ == "__main__":
    main()
