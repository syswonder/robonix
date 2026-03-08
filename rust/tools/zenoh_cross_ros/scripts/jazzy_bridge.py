#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Jazzy bridge: subscribe to Zenoh key robonix/cross_ros/point3d (canonical JSON),
# publish to ROS /point_jazzy (cross_ros_jazzy_msgs/Point3D) with header + position_*.

import json
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cross_ros_jazzy_msgs.msg import Point3D as JazzyPoint3D

try:
    import zenoh
except ImportError:
    zenoh = None

ZENOH_KEY = "robonix/cross_ros/point3d"
ROS_TOPIC = "/point_jazzy"


def main():
    if zenoh is None:
        print("zenoh-python not installed, install with: pip install eclipse-zenoh", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Node("jazzy_bridge")

    conf = zenoh.Config()
    listen = os.environ.get("ZENOH_LISTEN")
    if listen:
        conf.insert_json5("listen/endpoints", f'["{listen}"]')
    connect = os.environ.get("ZENOH_CONNECT")
    if connect:
        conf.insert_json5("connect/endpoints", f'["{connect}"]')
    session = zenoh.open(conf)
    pub = node.create_publisher(JazzyPoint3D, ROS_TOPIC, 10)

    def listener(sample):
        try:
            # Payload may be string or bytes from Zenoh
            raw = sample.payload
            s = raw.to_string() if hasattr(raw, "to_string") else (raw.decode("utf-8") if isinstance(raw, bytes) else str(raw))
            payload = json.loads(s)
            x = float(payload.get("x", 0))
            y = float(payload.get("y", 0))
            z = float(payload.get("z", 0))
        except (json.JSONDecodeError, KeyError, TypeError) as e:
            node.get_logger().warn(f"Invalid payload: {e}")
            return
        stamp = node.get_clock().now().to_msg()
        msg = JazzyPoint3D()
        msg.header = Header(stamp=stamp, frame_id="zenoh_bridge")
        msg.position_x = x
        msg.position_y = y
        msg.position_z = z
        pub.publish(msg)

    sub = session.declare_subscriber(ZENOH_KEY, listener)
    node.get_logger().info(f"Jazzy bridge: Zenoh {ZENOH_KEY} -> ROS {ROS_TOPIC}")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
