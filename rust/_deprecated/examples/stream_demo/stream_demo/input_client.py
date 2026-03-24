# SPDX-License-Identifier: MulanPSL-2.0
"""Stream demo: alerts input client. Continuously publishes alerts to input_server.
Must be run via rbnx start."""

from __future__ import annotations

import time

import grpc
import rclpy
from std_msgs.msg import String

from robonix.prm.debug import create_alerts_publisher
from robonix_runtime_pb2_grpc import RobonixRuntimeStub

NUM_MESSAGES = 5
INTERVAL_SEC = 0.5


def main() -> None:
    endpoint = "127.0.0.1:50051"
    requester_id = "input_demo_client"
    target = "input_demo_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    publisher = create_alerts_publisher(runtime_client, requester_id=requester_id, target=target)

    try:
        for i in range(NUM_MESSAGES):
            msg = String()
            msg.data = f"alert #{i + 1}"
            publisher.publish(msg)
            print(f"[{i + 1}/{NUM_MESSAGES}] sent: {msg.data}", flush=True)
            if i < NUM_MESSAGES - 1:
                time.sleep(INTERVAL_SEC)
        print(f"Sent {NUM_MESSAGES} alerts (input stream test OK)", flush=True)
        time.sleep(1.0)  # allow messages to propagate before shutdown
    finally:
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
