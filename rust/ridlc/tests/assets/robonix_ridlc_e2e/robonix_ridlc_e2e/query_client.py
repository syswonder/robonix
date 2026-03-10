import os

import grpc
import rclpy

from robonix.hal.base import create_status_client
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    requester_id = os.environ.get("ROBONIX_TEST_REQUESTER_ID", "query_client_1")
    target = os.environ.get("ROBONIX_TEST_SERVER_ID", "status_server_1")
    request_payload = os.environ.get("ROBONIX_TEST_REQUEST", "ping")
    expected_payload = os.environ.get("ROBONIX_TEST_EXPECTED", f"ok:{request_payload}")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    client = create_status_client(
        runtime_client,
        requester_id=requester_id,
        target=target,
    )

    request = client._srv_type.Request()
    request.req.data = request_payload
    response = client.call(request)
    print(response.res.data, flush=True)

    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if response.res.data != expected_payload:
        raise SystemExit(
            f"unexpected response payload: expected {expected_payload!r}, got {response.res.data!r}"
        )


if __name__ == "__main__":
    main()
