#!/usr/bin/env python3
"""
Demo pick skill that combines vision and grasp capabilities.
Skill queries robonix core to get dynamic topic names for required capabilities.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import time


class PickSkill(Node):
    """Implements skl::pick skill by combining cap::vision.capture_rgb and cap::grasp.move."""

    def __init__(self):
        super().__init__('demo_pick_skill')
        
        # Query capabilities and get topic names
        self.vision_image_topic = None
        self.grasp_pose_goal_topic = None
        self.grasp_status_topic = None
        
        # Use RobonixSDK from robonix_core
        try:
            from robonix_core.client import RobonixSDK
            self.sdk = RobonixSDK()
            self._query_capabilities()
        except ImportError as e:
            self.get_logger().warn(f'robonix_core.client not available ({e}), using fallback topics')
            self.sdk = None
            self._use_fallback_topics()
        except Exception as e:
            import traceback
            self.get_logger().warn(f'Failed to create RobonixSDK: {e}, using fallback topics')
            self.get_logger().debug(f'Traceback: {traceback.format_exc()}')
            self.sdk = None
            self._use_fallback_topics()
        
        # Subscribe to target label (skill input)
        self.target_label_subscriber = self.create_subscription(
            String,
            '/demo_pick/target_label',
            self.target_label_callback,
            10
        )
        
        # Publish status (skill output)
        self.status_publisher = self.create_publisher(
            Bool,
            '/demo_pick/status',
            10
        )
        
        # Subscribe to vision capability output (dynamic topic)
        if self.vision_image_topic:
            self.image_subscriber = self.create_subscription(
                Image,
                self.vision_image_topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f'  Subscribing to vision output: {self.vision_image_topic}')
        else:
            self.image_subscriber = None
            self.get_logger().warn('  Vision capability not available!')
        
        # Publish to grasp capability input (dynamic topic)
        if self.grasp_pose_goal_topic:
            self.pose_goal_publisher = self.create_publisher(
                PoseStamped,
                self.grasp_pose_goal_topic,
                10
            )
            self.get_logger().info(f'  Publishing to grasp input: {self.grasp_pose_goal_topic}')
        else:
            self.pose_goal_publisher = None
            self.get_logger().warn('  Grasp capability not available!')
        
        # Subscribe to grasp capability output (dynamic topic)
        if self.grasp_status_topic:
            self.grasp_status_subscriber = self.create_subscription(
                Bool,
                self.grasp_status_topic,
                self.grasp_status_callback,
                10
            )
            self.get_logger().info(f'  Subscribing to grasp output: {self.grasp_status_topic}')
        else:
            self.grasp_status_subscriber = None
        
        self.current_target_label = None
        self.latest_image = None
        self.grasp_complete = False
        self.picking_in_progress = False
        
        self.get_logger().info('Demo pick skill initialized')
        self.get_logger().info('  Subscribing to: /demo_pick/target_label')
        self.get_logger().info('  Publishing to: /demo_pick/status')
    
    def _query_capabilities(self):
        """Query robonix core for required capabilities and get their topic names."""
        if not self.sdk:
            self._use_fallback_topics()
            return
        
        # Query cap::vision.capture_rgb
        self.get_logger().info('Querying cap::vision.capture_rgb...')
        result = self.sdk.query_cap_skl('cap::vision.capture_rgb')
        
        if result.success:
            # Get channel by parameter name from spec
            self.vision_image_topic = result.get_output_channel('image')
            if self.vision_image_topic:
                self.get_logger().info(f'  Found vision capability at: {self.vision_image_topic}')
            else:
                self.get_logger().warn('  Vision capability found but "image" output not available')
                self._use_fallback_topics()
        else:
            self.get_logger().warn(f'  Failed to query vision capability: {result.error_message}')
            self._use_fallback_topics()
        
        # Query cap::grasp.move
        self.get_logger().info('Querying cap::grasp.move...')
        result = self.sdk.query_cap_skl('cap::grasp.move')
        
        if result.success:
            # Get channels by parameter names from spec
            self.grasp_pose_goal_topic = result.get_input_channel('target_pose')
            self.grasp_status_topic = result.get_output_channel('status')
            
            if self.grasp_pose_goal_topic and self.grasp_status_topic:
                self.get_logger().info(f'  Found grasp capability')
                self.get_logger().info(f'    Input (target_pose): {self.grasp_pose_goal_topic}')
                self.get_logger().info(f'    Output (status): {self.grasp_status_topic}')
            else:
                self.get_logger().warn('  Grasp capability found but required parameters not available')
                if not self.grasp_pose_goal_topic:
                    self._use_fallback_topics()
        else:
            self.get_logger().warn(f'  Failed to query grasp capability: {result.error_message}')
            if not self.grasp_pose_goal_topic:
                self._use_fallback_topics()
    
    def _use_fallback_topics(self):
        """Use hardcoded topics as fallback when query service is not available."""
        if not self.vision_image_topic:
            self.vision_image_topic = '/demo_rgb/image'
            self.get_logger().info(f'  Using fallback vision topic: {self.vision_image_topic}')
        if not self.grasp_pose_goal_topic:
            self.grasp_pose_goal_topic = '/demo_grasp/pose_goal'
            self.get_logger().info(f'  Using fallback grasp input topic: {self.grasp_pose_goal_topic}')
        if not self.grasp_status_topic:
            self.grasp_status_topic = '/demo_grasp/pose_status'
            self.get_logger().info(f'  Using fallback grasp output topic: {self.grasp_status_topic}')

    def target_label_callback(self, msg):
        """Handle pick request with target label."""
        target_label = msg.data
        self.get_logger().info(f'Received pick request for label: {target_label}')
        
        if self.picking_in_progress:
            self.get_logger().warn('Pick operation already in progress, ignoring request')
            return
        
        # Check if required capabilities are available
        if not self.vision_image_topic or not self.grasp_pose_goal_topic:
            self.get_logger().error('Required capabilities not available!')
            status_msg = Bool()
            status_msg.data = False
            self.status_publisher.publish(status_msg)
            return
        
        self.current_target_label = target_label
        self.picking_in_progress = True
        self.grasp_complete = False
        
        # Step 1: Wait for image from vision capability
        self.get_logger().info('Step 1: Waiting for image from cap::vision.capture_rgb...')
        # In real implementation, we would process the image here
        # For demo, we simulate object detection
        
        # Step 2: Simulate finding object and calculate pose
        time.sleep(0.2)
        self.get_logger().info(f'Step 2: Simulated detection of object with label: {target_label}')
        
        # Step 3: Send pose goal to grasp capability
        pose_goal = PoseStamped()
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.header.frame_id = 'base_frame'
        # Simulated pose for the object
        pose_goal.pose.position.x = 0.5
        pose_goal.pose.position.y = 0.2
        pose_goal.pose.position.z = 0.1
        pose_goal.pose.orientation.w = 1.0
        
        self.get_logger().info('Step 3: Sending pose goal to cap::grasp.move...')
        if self.pose_goal_publisher:
            self.pose_goal_publisher.publish(pose_goal)
        else:
            self.get_logger().error('Grasp capability not available!')
            status_msg = Bool()
            status_msg.data = False
            self.status_publisher.publish(status_msg)
            self.picking_in_progress = False
        
        # Wait for grasp to complete (handled in grasp_status_callback)

    def image_callback(self, msg):
        """Handle image from vision capability (dynamic topic)."""
        self.latest_image = msg
        self.get_logger().debug(f'Received image from {self.vision_image_topic}')

    def grasp_status_callback(self, msg):
        """Handle status from grasp capability (dynamic topic)."""
        if msg.data and self.picking_in_progress:
            self.grasp_complete = True
            self.get_logger().info('Step 4: Grasp movement completed')
            
            # Publish skill status
            status_msg = Bool()
            status_msg.data = True
            self.status_publisher.publish(status_msg)
            self.get_logger().info('Pick skill completed successfully')
            
            self.picking_in_progress = False
            self.grasp_complete = False

    def publish_status(self, status):
        """Publish skill status."""
        msg = Bool()
        msg.data = status
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pick_skill = PickSkill()
    
    try:
        rclpy.spin(pick_skill)
    except KeyboardInterrupt:
        pass
    finally:
        pick_skill.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

