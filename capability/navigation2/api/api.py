#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from mcp.server.fastmcp import FastMCP

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Range
import sys

from DeepEmbody.manager.eaios_decorators import eaios

class NavWithUltrasonicSafety(Node):
    def __init__(self,safety_threshold=0.5):
        super().__init__('nav_with_ultrasonic_safety')
        self.navigator = BasicNavigator()
        self.safety_threshold = safety_threshold
        self.cancelled = False

        # 订阅超声波测距话题
        self.create_subscription(Range, '/ultrasonic/sensor0_front', self.range_callback, 10)

        # # 发送目标
        # self.send_goal(x, y, yaw)

    def set_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.z = float(yaw)  # 这里可以优化为真实 yaw 转 quaternion
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info('Going to goal pose...')
        self.navigator.goThroughPoses([goal_pose])

        # 监控导航和传感器
        while not self.navigator.isTaskComplete() and not self.cancelled:
            rclpy.spin_once(self, timeout_sec=0.5)

        if self.cancelled:
            self.get_logger().warn('Navigation cancelled due to ultrasonic obstacle.')
            return False
        else:
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                return True
            else:
                self.get_logger().warn('Goal failed!')
                return False

    def range_callback(self, msg: Range):
        distance = msg.range
        self.get_logger().info(f'Ultrasonic distance: {distance:.2f} meters')

        if distance < self.safety_threshold and not self.cancelled:
            self.get_logger().warn('Obstacle too close! Cancelling navigation...')
            self.navigator.cancelTask()
            self.cancelled = True

rclpy.init()
nv_controller = NavWithUltrasonicSafety()

@eaios.api
def nv_test():
    import time
    if int(time.time())%2 == 0:
        func = eaios.get_plugin("navigation2","ros2_navigation")
    else:
        func = eaios.get_plugin("navigation2","simple_navigation")
    # res = func()
    # print("lhe debug in cap test nv res",res)
    # return res
    return func()

@eaios.api
def set_goal(x, y, yaw) -> str:
    """设置导航目标点
    Args:
        x: 目标点X坐标
        y: 目标点Y坐标
        yaw: 目标点偏航角
    """
    # rclpy.init()
    res = nv_controller.set_goal(x,y,yaw)
    func_status = f"Service set_gaol response: {res}"
    rclpy.shutdown()
    return func_status

@eaios.api
def stop_goal() -> str:
    """停止当前导航目标
    Args:
        None
    """
    rclpy.init()
    nv_controller.cancelled = True
    func_status = f"Service stop response: {True}"
    rclpy.shutdown()
    return func_status

def test():
    rclpy.init()
    node = NodeController()

    # ros2 service list
    # ros2 service type /get_count
    # ros2 service call get_count std_srvs/srv/Trigger

    req = Trigger.Request()
    res = node.call_service('get_count', req)
    print(f"Service get_count response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service('modify_name', req)
    print(f"Service modify_name response: {res.success}, message: {res.message}")

    req = Trigger.Request()
    res = node.call_service('shutdown_node', req)
    print(f"Service shutdown_node response: {res.success}, message: {res.message}")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # 初始化并运行 server
    mcp.run(transport='stdio')