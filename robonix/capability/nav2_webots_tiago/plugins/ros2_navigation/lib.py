import os
from mcp.server.fastmcp import FastMCP

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Range
import sys

from robonix.manager.eaios_decorators import eaios

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

@eaios.plugin("navigation2","ros2_navigation")
def nv_test():
    print("nv test ros2_navigation")
    return "nv test ros2_navigation"

@eaios.plugin("navigation2","ros2_navigation")
def set_goal(x, y, yaw) -> str:
    # rclpy.init()
    res = nv_controller.set_goal(x,y,yaw)
    func_status = f"Service set_goal response: {res}"
    # rclpy.shutdown()
    return func_status
    
if __name__ == "__main__":
    rclpy.init()
    nv_controller = NavWithUltrasonicSafety()

