#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Test subscriber: subscribe to /point_jazzy (Jazzy Point3D) and verify (zenoh_cross_ros test).
# Waits for all 10 messages from humble_pub; exits 0 if all received, non-zero otherwise.

import sys
import time

import rclpy
from rclpy.node import Node
from cross_ros_jazzy_msgs.msg import Point3D as JazzyPoint3D


def main():
    rclpy.init()
    node = Node("jazzy_sub")
    received = []

    def cb(msg: JazzyPoint3D):
        received.append((msg.position_x, msg.position_y, msg.position_z))
        n = len(received)
        node.get_logger().info(
            f"Received #{n}/10 Jazzy Point3D: position_x={msg.position_x}, position_y={msg.position_y}, position_z={msg.position_z}"
        )
        print(f"Jazzy received #{n}/10: x={msg.position_x}, y={msg.position_y}, z={msg.position_z}", flush=True)

    sub = node.create_subscription(JazzyPoint3D, "/point_jazzy", cb, 10)
    node.get_logger().info("Subscribed to /point_jazzy (waiting for 10 messages, up to 15s)...")

    deadline = time.monotonic() + 15.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(received) >= 10:
            break

    node.destroy_node()
    rclpy.shutdown()

    if len(received) < 10:
        print(
            f"FAIL: expected 10 messages, received {len(received)} on /point_jazzy",
            file=sys.stderr,
        )
        sys.exit(1)
    print(f"PASS: received all {len(received)} Jazzy Point3D message(s); cross-version bridge works.")
    sys.exit(0)


if __name__ == "__main__":
    main()
