#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Observe a full turn through odometry, with a bounded wait and collision checks."""
from __future__ import annotations

import json
import math
import signal
import time


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import UInt32

    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init()
    node = rclpy.create_node("ci_look_around")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    previous = None
    turn = 0.0
    last_odom = 0.0
    count = None

    def odometry(message):
        nonlocal previous, turn, last_odom
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if not math.isfinite(yaw) or q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < .5:
            raise RuntimeError("invalid odometry orientation")
        if previous is not None:
            turn += math.atan2(math.sin(yaw - previous), math.cos(yaw - previous))
        previous = yaw
        last_odom = time.monotonic()

    def collision(message):
        nonlocal count
        count = message.data

    node.create_subscription(Odometry, "/odom", odometry,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(UInt32, "/webots/collision_count", collision,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    started = time.monotonic()
    try:
        while previous is None or count is None:
            if time.monotonic() - started > 30:
                raise RuntimeError("odometry or collision counter did not arrive")
            rclpy.spin_once(node, timeout_sec=.05)
        baseline = count
        command = Twist()
        command.angular.z = .4
        while turn < 2 * math.pi:
            if time.monotonic() - started > 180 or time.monotonic() - last_odom > 3:
                raise RuntimeError("full observation turn timed out or lost odometry")
            if count != baseline:
                raise RuntimeError("simulator reported a collision during observation turn")
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=.05)
        publisher.publish(Twist())
        until = time.monotonic() + 1
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if count != baseline:
            raise RuntimeError("simulator reported a collision while stopping the observation turn")
        print(json.dumps({"rotation_rad": turn, "wall_seconds": time.monotonic() - started,
                          "collision_count_before": baseline, "collision_count_after": count}), flush=True)
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
