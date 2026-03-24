# SPDX-License-Identifier: MulanPSL-2.0
"""ROS2 service echo server for latency benchmark (typical RMW: rmw_fastrtps_cpp)."""

import rclpy
from rclpy.node import Node
from latency_bench_msgs.srv import Echo


class EchoServer(Node):
    def __init__(self):
        super().__init__("echo_server")
        self.srv = self.create_service(Echo, "/latency_bench/echo", self.echo_cb)

    def echo_cb(self, request, response):
        response.data = request.data
        return response


def main():
    rclpy.init()
    node = EchoServer()
    print("ROS2 echo server listening on /latency_bench/echo", flush=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
