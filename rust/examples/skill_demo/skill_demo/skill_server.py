# SPDX-License-Identifier: MulanPSL-2.0
"""Skill demo: greet command server. Example skill implementation.
Must be run via rbnx start."""

from __future__ import annotations

import grpc
import rclpy

from skill_demo.skill.greet_command import create_greet_server
from skill_demo_interfaces_ros2.action import SkillDemoSkillGreet
from rclpy.action.server import ServerGoalHandle
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    node_id = "skill_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    server = create_greet_server(runtime_client, node_id=node_id)

    def execute(
        request: SkillDemoSkillGreet.Goal,
        goal_handle: ServerGoalHandle | None = None,
    ) -> SkillDemoSkillGreet.Result:
        name = request.request.name
        # output (feedback): publish progress during execution
        if goal_handle is not None:
            fb = server._action_type.Feedback()
            fb.feedback.progress = "processing..."
            goal_handle.publish_feedback(fb)
            fb.feedback.progress = f"greeting {name}..."
            goal_handle.publish_feedback(fb)
        # result: final outcome
        result = server._action_type.Result()
        result.response.message = f"Hello, {name}!"
        result.response.success = True
        server.get_logger().info(f"greet request name={name!r}")
        return result

    server.execute = execute
    server.start()

    try:
        rclpy.spin(server)
    finally:
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
