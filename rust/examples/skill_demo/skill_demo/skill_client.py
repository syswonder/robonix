# SPDX-License-Identifier: MulanPSL-2.0
"""Skill demo: greet command client. Resolves and sends greet goal.
Must be run via rbnx start."""

import sys

import grpc
import rclpy

from skill_demo.skill.greet_command import create_greet_client
from skill_demo_msgs.msg import GreetRequest
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    requester_id = "skill_client"
    target = "skill_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    client = create_greet_client(
        runtime_client,
        requester_id=requester_id,
        target=target,
    )

    request = client._action_type.Goal()
    request.request = GreetRequest()
    request.request.name = "world"

    goal_handle = client.send(request)
    if goal_handle is None or not goal_handle.accepted:
        print("Greet goal not accepted", flush=True)
        sys.exit(1)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client, result_future, timeout_sec=10.0)
    wrapped = result_future.result()
    if wrapped is None:
        print("Timeout waiting for result", flush=True)
        sys.exit(1)

    print("result:", wrapped.result.response.message, "success:", wrapped.result.response.success)

    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
