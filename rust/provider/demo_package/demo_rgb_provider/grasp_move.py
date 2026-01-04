#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Grasp Move Capability
#
# Demo grasp move capability package.
# Subscribes to pose goals and publishes status.
""""""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import time


class GraspMoveCapability(Node):
    """Implements cap::grasp.move capability."""

    def __init__(self):
        super().__init__('demo_grasp_move')
        
        # Subscribe to pose goal (input)
        self.pose_goal_subscriber = self.create_subscription(
            PoseStamped,
            '/demo_grasp/pose_goal',
            self.pose_goal_callback,
            10
        )
        
        # Publish status (output)
        self.status_publisher = self.create_publisher(
            Bool,
            '/demo_grasp/pose_status',
            10
        )
        
        self.current_status = False
        self.get_logger().info('Demo grasp move capability started')
        self.get_logger().info('  Subscribing to: /demo_grasp/pose_goal')
        self.get_logger().info('  Publishing to: /demo_grasp/pose_status')

    def pose_goal_callback(self, msg):
        """Handle incoming pose goal."""
        self.get_logger().info(
            f'Received pose goal: position=({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        )
        
        # Simulate movement
        self.current_status = False
        self.publish_status(False)
        
        # Simulate movement delay
        time.sleep(0.5)
        
        # Movement complete
        self.current_status = True
        self.publish_status(True)
        self.get_logger().info('Grasp movement completed')

    def publish_status(self, status):
        """Publish status."""
        msg = Bool()
        msg.data = status
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    grasp_move = GraspMoveCapability()
    
    try:
        rclpy.spin(grasp_move)
    except KeyboardInterrupt:
        pass
    finally:
        grasp_move.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

