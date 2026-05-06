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
and republishes. SLAM then sees a normal scan and walls stop drifting.
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
        # Track inter-scan period so we can synthesise scan_time when
        # the upstream LaserScan has it as 0 (Webots does — it never
        # populates scan_time / time_increment, which makes SLAM treat
        # every scan as instantaneous and ghost-walls form on rotation).
        self._last_stamp_s: float | None = None
        self._scan_period_s: float = 0.118    # 8.5 Hz fallback for tiago hokuyo

    def _cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        # Webots's lidar plugin stamps scans at the END of the sim
        # step that produced them; the rays were actually sampled at
        # the START. Pulling the stamp back by ~half a scan period
        # makes SLAM's tf lookup land near the true sample time
        # (combined with `use_scan_barycenter`, which then adds
        # scan_time/2 to recover sample-center). On combined linear+
        # angular motion this removes the residual ~degree-of-tens
        # ghost we still saw after the time_increment / angle fixes.
        offset_s = self._scan_period_s * 0.5
        sec_total = (msg.header.stamp.sec
                     + msg.header.stamp.nanosec * 1e-9
                     - offset_s)
        if sec_total > 0:
            out.header.stamp.sec = int(sec_total)
            out.header.stamp.nanosec = int((sec_total - int(sec_total)) * 1e9)
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
        # Estimate inter-scan period from the stamp deltas (low-pass).
        cur_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_stamp_s is not None:
            dt = cur_s - self._last_stamp_s
            if 0.02 < dt < 1.0:   # ignore obvious outliers / time resets
                self._scan_period_s = 0.85 * self._scan_period_s + 0.15 * dt
        self._last_stamp_s = cur_s

        n = len(out.ranges) if out.ranges else 1
        # Patch scan_time / time_increment when the upstream sets them
        # to 0 (Webots lidar plugin does). Without these fields SLAM
        # cannot motion-compensate across the scan duration; with the
        # robot rotating at 0.5 rad/s and scan_period ~0.12s the
        # robot turns ~3.5° per scan, which is exactly the ghost-wall
        # halo we see in rviz.
        out.scan_time = float(msg.scan_time) if msg.scan_time > 1e-6 else float(self._scan_period_s)
        out.time_increment = float(msg.time_increment) if msg.time_increment > 1e-9 else float(self._scan_period_s / max(1, n))
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
