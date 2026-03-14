# SPDX-License-Identifier: MulanPSL-2.0
"""ROS2 service echo client for latency benchmark."""

import rclpy
from rclpy.node import Node
from latency_bench_msgs.srv import Echo


class EchoClient(Node):
    def __init__(self):
        super().__init__("echo_client")
        self.client = self.create_client(Echo, "/latency_bench/echo")

    def wait_for_service(self, timeout_sec: float = 10.0) -> bool:
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def echo(self, data: bytes) -> bytes:
        req = Echo.Request()
        req.data = list(data)  # list of uint8
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is None:
            raise RuntimeError("ROS2 service call failed or timed out")
        return bytes(future.result().data)


def create_client() -> EchoClient:
    rclpy.init()
    return EchoClient()
