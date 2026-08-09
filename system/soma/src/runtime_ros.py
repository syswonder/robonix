#!/usr/bin/env python3
"""Internal Soma ROS 2 reader. Endpoints come exclusively from Atlas."""
from __future__ import annotations

import json
import sys
import time


def emit(payload: dict) -> None:
    print(json.dumps(payload, separators=(",", ":")), flush=True)


def main() -> int:
    config = json.loads(open(sys.argv[1], encoding="utf-8").read())
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
    except Exception as exc:
        emit({"kind": "warning", "message": f"ROS 2 runtime reader unavailable: {exc}"})
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node("robonix_soma_runtime_state")
    subscriptions = []

    def qos(source: dict):
        profile = QoSProfile(depth=10)
        if str(source.get("qos", "")).lower() == "best_effort":
            profile.reliability = ReliabilityPolicy.BEST_EFFORT
        return profile

    for source in config.get("sources", []):
        provider = str(source["provider_id"])
        topic = str(source["topic"])
        kind = str(source["kind"])
        if kind == "joint_state":
            def on_joint(msg, provider_id=provider):
                emit({
                    "kind": "joint_state",
                    "provider_id": provider_id,
                    "received_at_unix": time.time(),
                    "names": list(msg.name),
                    "positions": list(msg.position),
                })
            subscriptions.append(node.create_subscription(JointState, topic, on_joint, qos(source)))
        elif kind == "odom":
            def on_odom(msg, provider_id=provider):
                linear = msg.twist.twist.linear
                angular = msg.twist.twist.angular
                emit({
                    "kind": "odom",
                    "provider_id": provider_id,
                    "received_at_unix": time.time(),
                    "linear": [linear.x, linear.y, linear.z],
                    "angular": [angular.x, angular.y, angular.z],
                })
            subscriptions.append(node.create_subscription(Odometry, topic, on_odom, qos(source)))

    if not subscriptions:
        emit({"kind": "warning", "message": "runtime reader created no subscriptions"})
        return 3
    emit({"kind": "ready", "subscriptions": len(subscriptions)})
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
