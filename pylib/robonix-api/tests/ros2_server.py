# SPDX-License-Identifier: MulanPSL-2.0
"""ROS 2 provider for execution, QoS, and entity-lifetime tests."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from navigation.srv import Navigate, GetNavigationStatus, CancelNavigation, GetNavigationSpeedLimit
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy

parser = argparse.ArgumentParser()
parser.add_argument("--prefix", required=True)
parser.add_argument("--control", type=Path, required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node("channel_test_provider")
runs = {}


def config():
    return json.loads(args.control.read_text())


def speed_limit(request, response):
    time.sleep(config().get("delay", 0))
    response.available = True
    response.effective_percentage = 100.0
    return response


def navigate(request, response):
    runs["ros-run"] = False
    response.accepted = True
    response.run_id = "ros-run"
    return response


def status(request, response):
    response.known = request.run_id in runs
    response.state = "SUCCEEDED" if config().get("finish") else "CANCELED" if runs.get(request.run_id) else "RUNNING"
    return response


def cancel(request, response):
    runs[request.run_id] = True
    response.accepted = True
    return response


def receive(message):
    args.control.with_suffix(".received").write_text(str(message.linear.x))


for cls, suffix, callback in ((GetNavigationSpeedLimit, "speed_limit", speed_limit), (Navigate, "navigate", navigate),
                               (GetNavigationStatus, "status", status), (CancelNavigation, "cancel", cancel)):
    node.create_service(cls, f"/{args.prefix}/{suffix}", callback)
node.create_subscription(Twist, f"/{args.prefix}/command", receive, 10)
publisher = node.create_publisher(Odometry, f"/{args.prefix}/odom", QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
message = Odometry()
message.pose.pose.position.x = 42.0
node.create_timer(0.05, lambda: publisher.publish(message))
args.control.with_suffix(".ready").touch()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
