# SPDX-License-Identifier: MulanPSL-2.0
"""Stationary synthetic sensor for the real ROS/Atlas/Chronos process smoke test."""
import sys
import time
import signal

import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy

rclpy.init(args=[])
node = rclpy.create_node("chronos_collector_test_camera")
native = len(sys.argv) > 2 and sys.argv[2] == "native"
publisher = node.create_publisher(Image, sys.argv[1], QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if native else 1)
start = time.monotonic_ns()
sequence = 0
paused = False

def pause(_signum, _frame):
    global paused
    paused = True

def resume(_signum, _frame):
    global paused
    paused = False

signal.signal(signal.SIGUSR1, pause)
signal.signal(signal.SIGUSR2, resume)
try:
    while rclpy.ok():
        if paused:
            rclpy.spin_once(node, timeout_sec=0.05)
            continue
        stamp = 5_000_000_000 + time.monotonic_ns() - start
        image = Image()
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 10**9)
        image.header.frame_id = "camera-session"
        image.height = image.width = image.step = 1
        sequence += 1
        if native:
            image.width = sequence
        image.encoding = "mono8"
        image.data = [0]
        publisher.publish(image)
        rclpy.spin_once(node, timeout_sec=0.05)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
