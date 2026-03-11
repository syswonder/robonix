# SPDX-License-Identifier: MulanPSL-2.0
"""Skill demo: execute command client. Resolves and sends execute goal."""

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

from robonix.system.skill.execute_command import create_execute_client
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    requester_id = os.environ.get("ROBONIX_NODE_ID", "skill_client")
    target = os.environ.get("ROBONIX_COMMAND_TARGET", "skill_server")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    client = create_execute_client(
        runtime_client,
        requester_id=requester_id,
        target=target,
    )

    request = client._action_type.Goal()
    request.request_json.data = '{"skill":"greet","params":{}}'

    goal_handle = client.send(request)
    if goal_handle is None or not goal_handle.accepted:
        print("Execute goal not accepted", flush=True)
        sys.exit(1)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client, result_future, timeout_sec=10.0)
    wrapped = result_future.result()
    if wrapped is None:
        print("Timeout waiting for result", flush=True)
        sys.exit(1)

    print("result:", wrapped.result.response_json.data)

    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
