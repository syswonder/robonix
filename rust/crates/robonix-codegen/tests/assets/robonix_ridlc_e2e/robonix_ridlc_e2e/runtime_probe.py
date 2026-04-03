import os

import grpc

from robonix_runtime_pb2 import ResolveQueryRequest
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    requester_id = os.environ.get("ROBONIX_TEST_REQUESTER_ID", "runtime_probe_1")
    target = os.environ.get("ROBONIX_TEST_SERVER_ID", "status_server_1")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    response = runtime_client.ResolveQuery(
        ResolveQueryRequest(
            requester_id=requester_id,
            target=target,
            namespace="robonix/hal/base",
            query_name="status",
        )
    )

    print(response.channel_name, flush=True)
    if not response.channel_name.startswith("/rbnx/ch/q/"):
        raise SystemExit(
            f"unexpected runtime channel name: {response.channel_name!r}"
        )


if __name__ == "__main__":
    main()
