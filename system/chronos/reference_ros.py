# SPDX-License-Identifier: MulanPSL-2.0
"""Chronos-owned ROS reference reader; stdout carries only clock input."""

import json
import sys
import os
import select
import time


def main():
    import rclpy
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node("robonix_chronos_reference")
    latest = None

    def receive(message):
        nonlocal latest
        stamp = message.clock
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("/clock requires nonnegative seconds and valid nanoseconds")
        latest = stamp.sec * 1_000_000_000 + stamp.nanosec
        print(json.dumps({"timestamp_ns": latest, "available": True}), flush=True)

    subscription = node.create_subscription(
        Clock, "/clock", receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    print(json.dumps({"ready": True}), flush=True)
    next_health = time.monotonic()
    try:
        while True:
            if select.select([sys.stdin], [], [], 0)[0] and not os.read(sys.stdin.fileno(), 1):
                break
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now >= next_health:
                # Equal stamps are valid during pause; graph presence is separate evidence.
                present = node.count_publishers("/clock") > 0
                if not present:
                    latest = None
                print(json.dumps({"timestamp_ns": latest, "available": latest is not None and present}), flush=True)
                next_health = now + 0.5
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
