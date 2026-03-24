# SPDX-License-Identifier: MulanPSL-2.0
"""Map semantic service: implements robonix/system/map/semantic_query.

Code structure:
- [Generated] ridlc generates: create_semantic_query_server, RobonixRuntimeStub
- [You write] In main(): connect to meta, create server, implement handler, start(handler) + spin
- [Hook] Query server: pass handler via start(handler); invoked when request arrives
"""

from __future__ import annotations

import grpc
import rclpy

from robonix.system.map.semantic_query_query import create_semantic_query_server
from robonix_msgs.msg import Object
from robonix_interfaces_ros2.srv import SystemMapSemanticQuery
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    node_id = "com.robonix.example.map_semantic"

    # [Required] 1) Init ROS2  2) Connect to meta gRPC  3) Get runtime_client (for channel registration)
    rclpy.init()
    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)

    server = create_semantic_query_server(runtime_client, node_id=node_id)

    # [You write] handler: request has filter, fill response.objects; passed via start(handler)
    def handler(
        request: SystemMapSemanticQuery.Request,
        response: SystemMapSemanticQuery.Response,
    ) -> SystemMapSemanticQuery.Response:
        filter_str = request.filter.data if hasattr(request.filter, "data") else ""
        # Real impl: query map backend with filter_str
        obj1 = Object()
        obj1.id = "obj1"
        obj1.label = "table"
        obj2 = Object()
        obj2.id = "obj2"
        obj2.label = "cup"
        response.objects = [obj1, obj2]
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
