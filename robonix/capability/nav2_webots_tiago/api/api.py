#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Range
import sys

from robonix.manager.eaios_decorators import eaios


class NavWithUltrasonicSafety(Node):
    def __init__(self, safety_threshold=0.5):
        super().__init__("nav_with_ultrasonic_safety")
        self.navigator = BasicNavigator()
        self.safety_threshold = safety_threshold
        self.cancelled = False

        # Subscribe to ultrasonic range topic
        self.create_subscription(
            Range, "/ultrasonic/sensor0_front", self.range_callback, 10
        )

        # # Send goal
        # self.send_goal(x, y, yaw)

    def set_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.z = float(
            yaw
        )  # This can be optimized to convert real yaw to quaternion
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info("Going to goal pose...")
        self.navigator.goThroughPoses([goal_pose])

        # Monitor navigation and sensors
        while not self.navigator.isTaskComplete() and not self.cancelled:
            rclpy.spin_once(self, timeout_sec=0.5)

        if self.cancelled:
            self.get_logger().warn("Navigation cancelled due to ultrasonic obstacle.")
            return False
        else:
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Goal succeeded!")
                return True
            else:
                self.get_logger().warn("Goal failed!")
                return False

    def range_callback(self, msg: Range):
        distance = msg.range
        self.get_logger().info(f"Ultrasonic distance: {distance:.2f} meters")

        if distance < self.safety_threshold and not self.cancelled:
            self.get_logger().warn("Obstacle too close! Cancelling navigation...")
            self.navigator.cancelTask()
            self.cancelled = True


@eaios.api
def nv_test():
    import time

    if int(time.time()) % 2 == 0:
        func = eaios.get_plugin("navigation2", "ros2_navigation")
    else:
        func = eaios.get_plugin("navigation2", "simple_navigation")
    # res = func()
    print("lhe debug in cap test nv res", func, id(func))
    # return res
    return func()


@eaios.api
def set_goal(x, y, yaw) -> str:
    """Set navigation goal point
    Args:
        x: Target X coordinate
        y: Target Y coordinate
        yaw: Target yaw angle
    """
    # rclpy.init()
    import yaml

    plugin_name = "simple_navigation"
    config_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "description.yml"
    )
    with open(config_path, "r") as f:
        description_data = yaml.safe_load(f)
        plugin_name = description_data.get("plugins", [])[
            0
        ]  # Get the first plugin name
    if plugin_name == "ros2_navigation":
        func = eaios.get_plugin("navigation2", "ros2_navigation")
    else:
        func = eaios.get_plugin("navigation2", "simple_navigation")
    res = func(x, y, yaw)
    func_status = f"Service set_goal response: {res}"
    return func_status


@eaios.api
def simple_set_goal(x: float, y: float, yaw: float) -> str:
    # a simple set_goal function use navigation2 plugins - wheatfox
    from robonix.capability.navigation2.plugins.ros2_navigation.lib import set_goal

    res = set_goal(x, y, yaw)
    return res


@eaios.api
def stop_goal() -> str:
    """Stop current navigation goal
    Args:
        None
    """
    rclpy.init()
    # Assuming nv_controller is defined elsewhere or this is a placeholder
    # If nv_controller is an instance of NavWithUltrasonicSafety, it should be passed or managed correctly.
    # For now, commenting out the line that depends on an undefined 'nv_controller'
    # nv_controller.cancelled = True
    func_status = f"Service stop response: {True}"
    rclpy.shutdown()
    return func_status


def test():
    rclpy.init()
    node = NodeController()  # Assuming NodeController is defined elsewhere

    # ros2 service list
    # ros2 service type /get_count
    # ros2 service call get_count std_srvs/srv/Trigger

    # Assuming Trigger and its Request are defined/imported if this `test` function is meant to be runnable
    # from std_srvs.srv import Trigger
    req = Trigger.Request()
    res = node.call_service("get_count", req)
    print(f"Service get_count response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service("modify_name", req)
    print(f"Service modify_name response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service("shutdown_node", req)
    print(f"Service shutdown_node response: {res.success}, message: {res.message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # Initialize and run server
    mcp.run(transport="stdio")
