import os

import grpc
import rclpy

from robonix.hal.base import create_status_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    node_id = os.environ.get("ROBONIX_TEST_SERVER_ID", "status_server_1")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    server = create_status_server(runtime_client, node_id=node_id)

    def handler(request, response):
        response.res.data = f"ok:{request.req.data}"
        server.get_logger().info(
            f"handled status request={request.req.data!r} response={response.res.data!r}"
        )
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
