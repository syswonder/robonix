#!/usr/bin/env python3

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robonix_sdk.srv import QueryPrimitive


SERVICE_WAIT_TIMEOUT = 30.0


def query_schema(node, client, name, is_input):
    req = QueryPrimitive.Request()
    req.name = name
    req.filter = "{}"
    if not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT):
        raise RuntimeError(
            "prm service unavailable: ensure PRM node is running and "
            "service /rbnx/prm/query is available (e.g. source workspace and launch PRM)"
        )
    future = client.call_async(req)
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError(f"query {name} timeout")
    resp = future.result()
    if not resp or not resp.instances:
        raise RuntimeError(f"no instance for {name}")
    s = resp.instances[0].input_schema if is_input else resp.instances[0].output_schema
    return json.loads(s)


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def normalize_angle_delta(delta):
    """Normalize angle difference to [-pi, pi]."""
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


POSITION_TOLERANCE = 0.15
ANGLE_TOLERANCE = 0.1
ARRIVAL_TIMEOUT = 120.0


def main():
    rclpy.init(args=sys.argv)
    node = Node("forward_2m")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )
    client = node.create_client(QueryPrimitive, "/rbnx/prm/query", qos_profile=qos)

    pose_topic = query_schema(node, client, "prm::base.pose.cov", False)["pose"]
    nav_in = query_schema(node, client, "prm::base.navigate", True)
    goal_topic = nav_in["goal"]

    current_pose = [None]

    def on_pose(msg):
        current_pose[0] = msg

    sub = node.create_subscription(PoseWithCovarianceStamped, pose_topic, on_pose, 10)
    while current_pose[0] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    p = current_pose[0].pose.pose
    x, y = p.position.x, p.position.y
    yaw = yaw_from_quat(p.orientation)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = x + 2.0 * math.cos(yaw)
    goal.pose.position.y = y + 2.0 * math.sin(yaw)
    goal.pose.position.z = 0.0
    goal.pose.orientation = p.orientation

    pub = node.create_publisher(PoseStamped, goal_topic, 10)
    pub.publish(goal)
    node.get_logger().info(
        f"published goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
    )

    gx = goal.pose.position.x
    gy = goal.pose.position.y
    goal_yaw = yaw_from_quat(goal.pose.orientation)
    deadline = time.time() + ARRIVAL_TIMEOUT
    arrived = False
    last_dist = last_yaw_delta = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_pose[0] is None:
            continue
        cp = current_pose[0].pose.pose
        cx, cy = cp.position.x, cp.position.y
        curr_yaw = yaw_from_quat(cp.orientation)
        dist = math.hypot(gx - cx, gy - cy)
        yaw_delta = abs(normalize_angle_delta(goal_yaw - curr_yaw))
        last_dist, last_yaw_delta = dist, yaw_delta
        if dist <= POSITION_TOLERANCE and yaw_delta <= ANGLE_TOLERANCE:
            node.get_logger().info(
                f"arrived: dist={dist:.3f}m, yaw_delta={math.degrees(yaw_delta):.2f}°"
            )
            arrived = True
            break
    if not arrived and last_dist is not None and last_yaw_delta is not None:
        node.get_logger().warn(
            f"timeout: dist={last_dist:.3f}m, yaw_delta={math.degrees(last_yaw_delta):.2f}°"
        )

    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
