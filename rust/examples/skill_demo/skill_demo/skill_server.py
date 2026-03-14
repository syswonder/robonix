# SPDX-License-Identifier: MulanPSL-2.0
"""Skill demo: greet command server. Example skill implementation.
Must be run via rbnx start."""

import grpc
import rclpy

from skill_demo.skill.greet_command import create_greet_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    node_id = "skill_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    server = create_greet_server(runtime_client, node_id=node_id)

    def execute(request, goal_handle=None):
        result = server._action_type.Result()
        name = request.request.name
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
