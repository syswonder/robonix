import os

import grpc
import rclpy

from robonix.hal.base import create_motion_cmd_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    node_id = os.environ.get("ROBONIX_TEST_COMMAND_SERVER_ID", "move_server_1")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    server = create_motion_cmd_server(runtime_client, node_id=node_id)

    def execute(request):
        result = server._action_type.Result()
        result.status.success = True
        result.status.message = f"ok:{request.cmd.linear.x:.2f}"
        return result

    server.execute = execute
    server.start()

    try:
        rclpy.spin(server)
    finally:
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
