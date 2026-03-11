# SPDX-License-Identifier: MulanPSL-2.0
"""Query demo: semantic_query server. Example semantic map service implementation."""

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

from robonix_msgs.msg import Object
from robonix.system.map.semantic_query_query import create_semantic_query_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_META_GRPC_ENDPOINT", "127.0.0.1:50051")
    node_id = os.environ.get("ROBONIX_NODE_ID", "semantic_map_server")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    server = create_semantic_query_server(runtime_client, node_id=node_id)

    def handler(request, response):
        filter_str = request.filter.data if hasattr(request.filter, "data") else ""
        # Example: return Object[] with id, label (minimal fields)
        obj1 = Object()
        obj1.id = "obj1"
        obj1.label = "table"
        obj2 = Object()
        obj2.id = "obj2"
        obj2.label = "cup"
        response.objects = [obj1, obj2]
        server.get_logger().info(f"semantic_query filter={filter_str!r} -> {len(response.objects)} objects")
        return response

    server.start(handler)

    try:
        rclpy.spin(server)
    finally:
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
