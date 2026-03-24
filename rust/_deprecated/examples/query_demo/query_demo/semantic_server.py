# SPDX-License-Identifier: MulanPSL-2.0
"""Query demo: semantic_query server. Example semantic map service implementation.
Must be run via rbnx start."""

from __future__ import annotations

import grpc
import rclpy

from robonix_msgs.msg import Object
from robonix.system.map.semantic_query_query import create_semantic_query_server
from robonix_interfaces_ros2.srv import SystemMapSemanticQuery
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    node_id = "semantic_map_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    server = create_semantic_query_server(runtime_client, node_id=node_id)

    def handler(
        request: SystemMapSemanticQuery.Request,
        response: SystemMapSemanticQuery.Response,
    ) -> SystemMapSemanticQuery.Response:
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
