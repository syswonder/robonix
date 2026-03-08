#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Humble bridge: subscribe to ROS /point_humble (cross_ros_humble_msgs/Point3D),
# publish canonical JSON to Zenoh key robonix/cross_ros/point3d.

import json
import os
import sys

import rclpy
from rclpy.node import Node
from cross_ros_humble_msgs.msg import Point3D as HumblePoint3D

try:
    import zenoh
except ImportError:
    zenoh = None

ZENOH_KEY = "robonix/cross_ros/point3d"
ROS_TOPIC = "/point_humble"


def main():
    if zenoh is None:
        print("zenoh-python not installed, install with: pip install eclipse-zenoh", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Node("humble_bridge")

    conf = zenoh.Config()
    connect = os.environ.get("ZENOH_CONNECT")
    if connect:
        conf.insert_json5("connect/endpoints", f'["{connect}"]')
    session = zenoh.open(conf)
    publisher = session.declare_publisher(ZENOH_KEY)

    def cb(msg: HumblePoint3D):
        payload = {"x": msg.x, "y": msg.y, "z": msg.z}
        # Use string payload to avoid Zenoh buffer/encoding issues
        publisher.put(json.dumps(payload))

    sub = node.create_subscription(HumblePoint3D, ROS_TOPIC, cb, 10)
    node.get_logger().info(f"Humble bridge: ROS {ROS_TOPIC} -> Zenoh {ZENOH_KEY}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
