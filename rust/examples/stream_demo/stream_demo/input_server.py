# SPDX-License-Identifier: MulanPSL-2.0
"""Stream demo: alerts input server. Subscribes and continuously receives alerts from input_client.
Must be run via rbnx start."""

from __future__ import annotations

import sys
import time

import grpc
import rclpy
from std_msgs.msg import String

from robonix.prm.debug import create_alerts_subscriber
from robonix_runtime_pb2_grpc import RobonixRuntimeStub

NUM_MESSAGES = 5
TIMEOUT_SEC = 15.0


def main() -> None:
    endpoint = "127.0.0.1:50051"
    node_id = "input_demo_server"

    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)
    subscriber = create_alerts_subscriber(runtime_client, node_id=node_id)

    received: list[String] = []

    def on_msg(msg: String) -> None:
        received.append(msg)
        print(f"[{len(received)}] alert: {msg.data}", flush=True)

    subscriber.start(on_msg)

    deadline = time.time() + TIMEOUT_SEC
    try:
        while time.time() < deadline and len(received) < NUM_MESSAGES:
            rclpy.spin_once(subscriber, timeout_sec=0.5)
    finally:
        subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not received:
        print("No alert received (timeout). Start input_client first.", flush=True)
        sys.exit(1)
    print(f"Received {len(received)} alerts (input stream test OK)", flush=True)
    if len(received) < NUM_MESSAGES:
        print(f"Warning: expected {NUM_MESSAGES} messages, got {len(received)}", flush=True)


if __name__ == "__main__":
    main()
