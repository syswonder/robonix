# SPDX-License-Identifier: MulanPSL-2.0
"""Skill demo: execute command server. Example skill implementation."""

import os
import sys

def _setup_path():
    try:
        import robonix_runtime_pb2_grpc  # noqa: F401
    except ImportError:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(os.path.dirname(script_dir))
        rust_dir = os.path.dirname(pkg_root)
        python_pkg = os.path.join(
            rust_dir, "robonix-server", "target", "rclrs_interfaces_ws", "python_pkg"
        )
        if os.path.exists(python_pkg) and python_pkg not in sys.path:
            sys.path.insert(0, python_pkg)


_setup_path()

import grpc
import rclpy

from robonix.system.skill.execute_command import create_execute_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    node_id = os.environ.get("ROBONIX_NODE_ID", "skill_server")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    server = create_execute_server(runtime_client, node_id=node_id)

    def execute(request, goal_handle=None):
        # Example skill: echo request_json in result
        result = server._action_type.Result()
        req_json = request.request_json.data if hasattr(request.request_json, "data") else "{}"
        result.response_json.data = f'{{"status":"ok","echo":{req_json}}}'
        server.get_logger().info(f"execute request_json={req_json!r}")
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
