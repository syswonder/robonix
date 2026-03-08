#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Test publisher: publish Humble Point3D on /point_humble (for zenoh_cross_ros test).

import time

import rclpy
from rclpy.node import Node
from cross_ros_humble_msgs.msg import Point3D as HumblePoint3D


def main():
    rclpy.init()
    node = Node("humble_pub")
    pub = node.create_publisher(HumblePoint3D, "/point_humble", 10)
    time.sleep(1.0)

    for i in range(10):
        msg = HumblePoint3D()
        msg.x = 1.0 + i * 0.1
        msg.y = 2.0 + i * 0.1
        msg.z = 3.0 + i * 0.1
        pub.publish(msg)
        node.get_logger().info(f"Published Humble Point3D: x={msg.x}, y={msg.y}, z={msg.z}")
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
