#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Pose Converter Node
#
# Converts PoseWithCovarianceStamped (from AMCL) to PoseStamped
# This node subscribes to /amcl_pose and publishes to /robot_pose

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class PoseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )
        self.get_logger().info('Pose converter node started: /amcl_pose -> /robot_pose')

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.publisher.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = PoseConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
