#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Normalize a Webots LaserScan to standard ROS conventions.
Webots publishes /scanner with reversed angle direction:
    angle_min > angle_max, angle_increment < 0
which triggers known scan-matching bugs in slam_toolbox / cartographer
(github.com/cyberbotics/webots/issues/5540 — "every frame of map is
rotating about the center of the robot").
This relay subscribes to a Webots scan, flips it to:
    angle_min < angle_max, angle_increment > 0, ranges reversed
and republishes it without changing the scan timestamp or timing fields.
Webots renders the range image at one simulation timestamp, so consumers must
treat it as an instantaneous scan instead of applying per-ray compensation.
Usage:
    python3 scan_normalize.py --in /scanner --out /scan_normalized
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class Normalizer(Node):
    def __init__(self, in_topic: str, out_topic: str):
        super().__init__("scan_normalizer")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        self.pub = self.create_publisher(LaserScan, out_topic, qos)
        self.create_subscription(LaserScan, in_topic, self._cb, qos)
        self.get_logger().info(f"normalizing {in_topic} -> {out_topic}")
        self._warned = False

    def _cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        if msg.angle_increment < 0.0 or msg.angle_min > msg.angle_max:
            # Standard webots case: flip orientation.
            out.angle_min = float(msg.angle_max)
            out.angle_max = float(msg.angle_min)
            out.angle_increment = float(-msg.angle_increment)
            out.ranges = list(msg.ranges)[::-1]
            out.intensities = list(msg.intensities)[::-1] if msg.intensities else []
            if not self._warned:
                self.get_logger().info(
                    f"flipping scan: angle_min {msg.angle_min:.3f}->{out.angle_min:.3f}, "
                    f"increment {msg.angle_increment:.5f}->{out.angle_increment:.5f}")
                self._warned = True
        else:
            # Already standard — pass through.
            out.angle_min = msg.angle_min
            out.angle_max = msg.angle_max
            out.angle_increment = msg.angle_increment
            out.ranges = msg.ranges
            out.intensities = msg.intensities
        # A Webots range image is sampled at one simulation timestamp. Preserve
        # that contract exactly: inventing per-ray timing makes RTAB-Map apply
        # motion compensation to a scan that did not move during acquisition.
        out.scan_time = msg.scan_time
        out.time_increment = msg.time_increment
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_topic", default="/scanner")
    ap.add_argument("--out", dest="out_topic", default="/scan_normalized")
    args = ap.parse_args()
    rclpy.init()
    node = Normalizer(args.in_topic, args.out_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
