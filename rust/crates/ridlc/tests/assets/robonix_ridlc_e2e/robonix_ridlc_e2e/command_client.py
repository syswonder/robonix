import os
import time

import grpc
import rclpy

from robonix.hal.base import create_motion_cmd_client
from robonix_runtime_pb2 import ResolveCommandRequest
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def wait_for_command(runtime_client, requester_id: str, target: str) -> None:
    deadline = time.time() + 10.0
    last_error = None
    while time.time() < deadline:
        try:
            runtime_client.ResolveCommand(
                ResolveCommandRequest(
                    requester_id=requester_id,
                    target=target,
                    namespace="robonix/hal/base",
                    command_name="motion_cmd",
                )
            )
            return
        except grpc.RpcError as exc:
            last_error = exc
            time.sleep(0.2)
    raise RuntimeError(f"failed to resolve command action: {last_error}")


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    requester_id = os.environ.get("ROBONIX_TEST_COMMAND_REQUESTER_ID", "move_client_1")
    target = os.environ.get("ROBONIX_TEST_COMMAND_SERVER_ID", "move_server_1")
    expected = os.environ.get("ROBONIX_TEST_COMMAND_EXPECTED", "ok:1.25")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    wait_for_command(runtime_client, requester_id=requester_id, target=target)

    client = create_motion_cmd_client(runtime_client, requester_id=requester_id, target=target)
    request = client._action_type.Goal()
    request.cmd.linear.x = 1.25
    request.cmd.angular.z = 0.3

    try:
        goal_handle = client.send(request)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("action goal was not accepted")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(client, result_future, timeout_sec=10.0)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("timed out waiting for action result")
        result = wrapped_result.result
        print(result.status.message, flush=True)
        if result.status.message != expected:
            raise RuntimeError(
                f"unexpected action result: expected {expected!r}, got {result.status.message!r}"
            )
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
