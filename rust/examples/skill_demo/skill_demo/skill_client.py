# SPDX-License-Identifier: MulanPSL-2.0
"""Skill demo: greet command client. Resolves and sends greet goal.
Must be run via rbnx start."""

from __future__ import annotations

import sys

import grpc
import rclpy

from skill_demo.skill.greet_command import create_greet_client
from skill_demo_msgs.msg import GreetRequest
from skill_demo_interfaces_ros2.action import SkillDemoSkillGreet
from rclpy.action.client import ClientGoalHandle
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

    # input: goal (request)
    request: SkillDemoSkillGreet.Goal = client._action_type.Goal()
    request.request = GreetRequest()
    request.request.name = "world"

    # output (feedback): callback for progress during execution
    feedback_received: list[str] = []

    def on_feedback(fb_msg: SkillDemoSkillGreet.Impl.FeedbackMessage) -> None:
        # fb_msg is FeedbackMessage (goal_id, feedback); feedback is SkillDemoSkillGreet.Feedback
        feedback_received.append(fb_msg.feedback.feedback.progress)
        print(f"  feedback: {fb_msg.feedback.feedback.progress}", flush=True)

    goal_future = client._client.send_goal_async(request, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(client, goal_future, timeout_sec=10.0)
    goal_handle: ClientGoalHandle | None = goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print("Greet goal not accepted", flush=True)
        sys.exit(1)

    # result: final outcome
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client, result_future, timeout_sec=10.0)
    wrapped = result_future.result()
    if wrapped is None:
        print("Timeout waiting for result", flush=True)
        sys.exit(1)

    print("result:", wrapped.result.response.message, "success:", wrapped.result.response.success)
    if feedback_received:
        print("  (received", len(feedback_received), "feedback messages)", flush=True)

    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
