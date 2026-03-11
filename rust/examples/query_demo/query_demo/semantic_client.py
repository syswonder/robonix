# SPDX-License-Identifier: MulanPSL-2.0
"""Query demo: semantic_query client. Resolves and calls semantic_query server."""

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

from robonix.system.map.semantic_query_query import create_semantic_query_client
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    requester_id = os.environ.get("ROBONIX_NODE_ID", "semantic_client")
    target = os.environ.get("ROBONIX_QUERY_TARGET", "semantic_map_server")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    client = create_semantic_query_client(
        runtime_client,
        requester_id=requester_id,
        target=target,
    )

    request = client._srv_type.Request()
    request.filter.data = "room"

    response = client.call(request)
    for obj in response.objects:
        print(f"  id={obj.id} label={obj.label}")

    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
