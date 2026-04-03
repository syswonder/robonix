import os

import grpc
import rclpy
from rclpy.executors import MultiThreadedExecutor

from robonix.hal.base import create_motion_cmd_server, create_status_server
from robonix.hal.localization import create_pose_publisher
from robonix_runtime_pb2_grpc import RobonixRuntimeStub


def main() -> None:
    endpoint = os.environ.get("ROBONIX_RUNTIME_ENDPOINT", "127.0.0.1:50051")
    query_node_id = os.environ.get("ROBONIX_TEST_SERVER_ID", "status_server_1")
    stream_node_id = os.environ.get("ROBONIX_TEST_STREAM_SERVER_ID", "odom_provider_1")
    command_node_id = os.environ.get("ROBONIX_TEST_COMMAND_SERVER_ID", "move_server_1")

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)

    query_server = create_status_server(runtime_client, node_id=query_node_id)
    move_server = create_motion_cmd_server(runtime_client, node_id=command_node_id)
    odom_publisher = create_pose_publisher(runtime_client, node_id=stream_node_id)

    def query_handler(request, response):
        response.res.data = f"ok:{request.req.data}"
        return response

    def execute_move(request):
        result = move_server._action_type.Result()
        result.status.success = True
        result.status.message = f"ok:{request.cmd.linear.x:.2f}"
        return result

    def publish_odom() -> None:
        msg = odom_publisher._msg_type()
        msg.header.frame_id = "map"
        msg.header.stamp = odom_publisher.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.25
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0
        odom_publisher.publish(msg)

    query_server.start(query_handler)
    move_server.execute = execute_move
    move_server.start()
    timer = odom_publisher.create_timer(0.2, publish_odom)

    executor = MultiThreadedExecutor()
    executor.add_node(query_server)
    executor.add_node(move_server)
    executor.add_node(odom_publisher)

    try:
        executor.spin()
    finally:
        timer.cancel()
        executor.shutdown()
        odom_publisher.destroy_node()
        move_server.destroy_node()
        query_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
