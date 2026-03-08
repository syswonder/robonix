#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Same-IDL test: publish 10 x geometry_msgs/Point to /point (Humble side).

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


def main():
    rclpy.init()
    node = Node("same_idl_pub")
    pub = node.create_publisher(Point, "/point", 10)
    time.sleep(1.0)

    for i in range(10):
        msg = Point()
        msg.x = 1.0 + i * 0.1
        msg.y = 2.0 + i * 0.1
        msg.z = 3.0 + i * 0.1
        pub.publish(msg)
        node.get_logger().info(f"Published Point: x={msg.x}, y={msg.y}, z={msg.z}")
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
