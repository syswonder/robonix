#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from mcp.server.fastmcp import FastMCP

from tf_transformations import euler_from_quaternion

mcp = FastMCP("acml_pos")


class AmclPoseGetter(Node):
    def __init__(self):
        super().__init__('amcl_pose_getter')
        self.pose = None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.pose = msg
        self.get_logger().info("Got AMCL pose.")
        self.destroy_subscription(self.subscription)

@mcp.tool()
def get_pose(timeout_sec=2.0):
    """获取当前机器人位姿信息
    Args:
        timeout_sec: 等待位姿数据的超时时间（秒），默认2.0秒
    Returns:
        成功时返回(x, y, yaw)表示当前位置和朝向，超时返回None
    """
    rclpy.init()
    node = AmclPoseGetter()

    # 等待 pose 被接收到
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while rclpy.ok() and node.pose is None and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    result = None
    if node.pose:
        pos = node.pose.pose.pose.position
        ori = node.pose.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        result = (pos.x, pos.y, yaw)

    node.destroy_node()
    rclpy.shutdown()
    return result
