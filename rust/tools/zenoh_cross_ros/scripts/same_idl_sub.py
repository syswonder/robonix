#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Same-IDL test: subscribe to /point (geometry_msgs/Point), expect 10 messages (Jazzy side).

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


def main():
    rclpy.init()
    node = Node("same_idl_sub")
    received = []

    def cb(msg: Point):
        received.append((msg.x, msg.y, msg.z))
        n = len(received)
        node.get_logger().info(
            f"Received #{n}/10 Point: x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}"
        )

    node.create_subscription(Point, "/point", cb, 10)
    node.get_logger().info("Subscribed to /point (waiting for 10 messages, up to 15s)...")

    deadline = time.monotonic() + 15.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(received) >= 10:
            break

    node.destroy_node()
    rclpy.shutdown()

    if len(received) < 10:
        print(f"FAIL: expected 10 messages, received {len(received)} on /point", file=sys.stderr)
        sys.exit(1)
    print(f"PASS: received all {len(received)} Point message(s); same-IDL Zenoh bridge works.")
    sys.exit(0)


if __name__ == "__main__":
    main()
