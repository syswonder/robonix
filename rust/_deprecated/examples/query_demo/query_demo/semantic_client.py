# SPDX-License-Identifier: MulanPSL-2.0
"""Query demo: semantic_query client. Resolves and calls semantic_query server.
Must be run via rbnx start."""

from __future__ import annotations

import grpc
import rclpy

from robonix.system.map.semantic_query_query import create_semantic_query_client
from robonix_interfaces_ros2.srv import SystemMapSemanticQuery
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = "127.0.0.1:50051"
    requester_id = "semantic_client"
    target = "semantic_map_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    client = create_semantic_query_client(
        runtime_client,
        requester_id=requester_id,
        target=target,
    )

    request: SystemMapSemanticQuery.Request = client._srv_type.Request()
    request.filter.data = "room"

    response: SystemMapSemanticQuery.Response | None = client.call(request)
    if response is None:
        raise RuntimeError("Semantic query call failed or timed out")
    for obj in response.objects:
        print(f"  id={obj.id} label={obj.label}")

    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
