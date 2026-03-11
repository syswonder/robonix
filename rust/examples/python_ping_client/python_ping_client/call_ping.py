# SPDX-License-Identifier: MulanPSL-2.0
"""
Python client for robonix/system/debug/ping query.
Calls the ping service via robonix-server meta API + ROS2 (Zenoh).
"""

import os
import sys

import grpc

# Add generated Python stubs when not running from colcon install
def _setup_path():
    try:
        import robonix_runtime_pb2_grpc  # noqa: F401
    except ImportError:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # python_ping_client/call_ping.py -> examples/python_ping_client -> rust
        pkg_root = os.path.dirname(os.path.dirname(script_dir))
        rust_dir = os.path.dirname(pkg_root)
        python_pkg = os.path.join(
            rust_dir, "robonix-server", "target", "rclrs_interfaces_ws", "python_pkg"
        )
        if os.path.exists(python_pkg) and python_pkg not in sys.path:
            sys.path.insert(0, python_pkg)


_setup_path()

from robonix.system.debug.ping_query import create_ping_client
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix_interfaces_ros2.srv import SystemDebugPing
from std_msgs.msg import String


def main():
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    target = os.environ.get("ROBONIX_PING_TARGET", "robonix-server")
    requester_id = os.environ.get("ROBONIX_QUERY_REQUESTER_ID", "python_ping_client")
    payload = sys.argv[1] if len(sys.argv) > 1 else "hello"

    channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(channel)

    client = create_ping_client(runtime_client, requester_id, target)

    req = SystemDebugPing.Request()
    req.data = String()
    req.data.data = payload

    response = client.call(req, timeout_sec=10.0)
    print(response.data.data)


if __name__ == "__main__":
    main()
