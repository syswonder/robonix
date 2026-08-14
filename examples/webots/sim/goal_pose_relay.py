#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Relay rviz's private goal topic to Nav2's navigate_to_pose action, zero-stamped.

Why this is needed (not a hack): under use_sim_time, rtabmap publishes the
map->odom TF on the /clock timeline, but rviz2's "2D Goal Pose" / Nav2 Goal
tool stamps the goal with the WALL clock regardless of the rviz node's
use_sim_time parameter (verified: the rviz node reports use_sim_time=True
yet the goal arrives stamped at wall-epoch while the TF is at sim seconds).
The planner then can't transform the goal ("Extrapolation … into the
future") and aborts every plan, so the robot only spin-recovers in place.

This relay forwards a private RViz goal topic to the action with header.stamp = 0, which
tells tf2 to use the LATEST available transform instead of one exact
instant — the standard workaround for this rviz/Nav2 (Humble) behaviour,
and exactly what the navigation wrapper already does for its own goals.
Works the same on a real robot (where stamps line up anyway).

Run inside the sim container alongside rviz (start_rviz.sh launches it).
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from nav2_msgs.action import NavigateToPose


class GoalPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_relay")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.input_topic = self.declare_parameter(
            "input_topic", "/rviz_goal_pose").get_parameter_value().string_value
        self.sub = self.create_subscription(
            PoseStamped, self.input_topic, self._on_goal, 10)
        self.get_logger().info(
            f"goal_pose_relay: {self.input_topic} -> navigate_to_pose (stamp=0)")

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                "navigate_to_pose action server not available; goal dropped")
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.stamp = Time()  # 0 -> latest available TF
        if not goal.pose.header.frame_id:
            goal.pose.header.frame_id = "map"
        self.get_logger().info(
            "relaying goal (%.2f, %.2f) frame=%s" % (
                msg.pose.position.x, msg.pose.position.y,
                goal.pose.header.frame_id))
        self.client.send_goal_async(goal)


def main() -> None:
    rclpy.init()
    node = GoalPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
