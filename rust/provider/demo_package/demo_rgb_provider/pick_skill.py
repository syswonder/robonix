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
import signal
from robonix_core import RobonixClient


class PickSkill(Node):
    """Implements skl::pick skill by combining cap::vision.capture_rgb and cap::grasp.move."""

    def __init__(self):
        super().__init__('demo_pick_skill')
        
        # Query capabilities and get topic names
        self.vision_image_topic = None
        self.grasp_pose_goal_topic = None
        self.grasp_status_topic = None
        
        # Create Robonix client for service calls
        try:
            self.get_logger().info('Creating RobonixClient...')
            self.robonix_client = RobonixClient(node_name='demo_pick_skill_client', max_workers=20)
            self.get_logger().info('RobonixClient created successfully, querying capabilities...')
            # Test ping service with 20 concurrent calls
            # self._test_ping_service()
            # Query capabilities
            self._query_capabilities()
        except Exception as e:
            import traceback
            error_traceback = traceback.format_exc()
            self.get_logger().error(f'Failed to create RobonixClient: {e}')
            self.get_logger().error(f'Error type: {type(e).__name__}')
            self.get_logger().error(f'Full traceback:\n{error_traceback}')
            # Also print to stderr for immediate visibility
            import sys
            print(f'[ERROR] Failed to create RobonixClient: {e}', file=sys.stderr)
            print(f'[ERROR] Error type: {type(e).__name__}', file=sys.stderr)
            print(f'[ERROR] Full traceback:\n{error_traceback}', file=sys.stderr)
            self.robonix_client = None
            self._use_fallback_topics()
        
        self.get_logger().info(f'RobonixClient created: {self.robonix_client}')
        
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
    
    def _test_ping_service(self):
        """Test ping service with 20 concurrent calls to check for concurrency issues."""
        if not self.robonix_client:
            self.get_logger().warn('RobonixClient not available, skipping ping test')
            return
        
        self.get_logger().info('Testing ping service with 20 calls...')
        success_count = 0
        
        # Use thread pool for concurrent calls
        from concurrent.futures import ThreadPoolExecutor, as_completed
        
        with ThreadPoolExecutor(max_workers=20) as executor:
            futures = {executor.submit(self.robonix_client.ping, i, timeout_sec=5.0): i for i in range(20)}
            
            for future in as_completed(futures):
                seq = futures[future]
                try:
                    response = future.result()
                    if response and response.success and response.sequence == seq:
                        success_count += 1
                        self.get_logger().debug(f'Ping {seq}: success (timestamp: {response.timestamp})')
                    else:
                        self.get_logger().warn(f'Ping {seq}: failed')
                except Exception as e:
                    self.get_logger().warn(f'Ping {seq}: exception: {e}')
        
        self.get_logger().info(f'Ping test completed: {success_count}/20 successful')
        if success_count < 20:
            self.get_logger().warn(f'Ping test FAILED: {20 - success_count} calls failed or timed out')
    
    def _query_capabilities(self):
        """Query robonix core for required capabilities and get their topic names."""
        if not self.robonix_client:
            self._use_fallback_topics()
            return
        
        # Query cap::vision.capture_rgb
        self.get_logger().info('Querying cap::vision.capture_rgb...')
        try:
            response = self.robonix_client.query_capability('cap::vision.capture_rgb', '', None, timeout_sec=5.0)
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error in query_capability for vision: {e}')
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
            import sys
            print(f'[ERROR] Error in query_capability for vision: {e}', file=sys.stderr)
            print(f'[ERROR] Traceback:\n{traceback.format_exc()}', file=sys.stderr)
            self._use_fallback_topics()
            return
        
        if response and response.success:
            try:
                idx = list(response.output_names).index('image')
                if idx < len(response.output_channels):
                    self.vision_image_topic = list(response.output_channels)[idx]
                    self.get_logger().info(f'  Found vision capability at: {self.vision_image_topic}')
                else:
                    self.get_logger().warn('  Vision capability found but "image" output not available')
                    self._use_fallback_topics()
            except ValueError:
                self.get_logger().warn('  Vision capability found but "image" output not found')
                self._use_fallback_topics()
        else:
            error_msg = response.error_message if response else 'No response'
            self.get_logger().warn(f'  Failed to query vision capability: {error_msg}')
            self._use_fallback_topics()
        
        # Query cap::grasp.move
        self.get_logger().info('Querying cap::grasp.move...')
        try:
            response = self.robonix_client.query_capability('cap::grasp.move', '', None, timeout_sec=5.0)
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error in query_capability for grasp: {e}')
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
            import sys
            print(f'[ERROR] Error in query_capability for grasp: {e}', file=sys.stderr)
            print(f'[ERROR] Traceback:\n{traceback.format_exc()}', file=sys.stderr)
            if not self.grasp_pose_goal_topic:
                self._use_fallback_topics()
            return
        
        if response and response.success:
            try:
                input_idx = list(response.input_names).index('target_pose')
                output_idx = list(response.output_names).index('status')
                
                if input_idx < len(response.input_channels):
                    self.grasp_pose_goal_topic = list(response.input_channels)[input_idx]
                if output_idx < len(response.output_channels):
                    self.grasp_status_topic = list(response.output_channels)[output_idx]
                
                if self.grasp_pose_goal_topic and self.grasp_status_topic:
                    self.get_logger().info(f'  Found grasp capability')
                    self.get_logger().info(f'    Input (target_pose): {self.grasp_pose_goal_topic}')
                    self.get_logger().info(f'    Output (status): {self.grasp_status_topic}')
                else:
                    self.get_logger().warn('  Grasp capability found but required parameters not available')
                    if not self.grasp_pose_goal_topic:
                        self._use_fallback_topics()
            except ValueError as e:
                self.get_logger().warn(f'  Grasp capability found but required parameters not found: {e}')
                if not self.grasp_pose_goal_topic:
                    self._use_fallback_topics()
        else:
            error_msg = response.error_message if response else 'No response'
            self.get_logger().warn(f'  Failed to query grasp capability: {error_msg}')
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
    
    # Flag to track if shutdown was requested
    shutdown_requested = False
    
    def signal_handler(signum, frame):
        """Handle shutdown signals (SIGTERM, SIGINT)."""
        nonlocal shutdown_requested
        shutdown_requested = True
        pick_skill.get_logger().info(f'Received signal {signum}, shutting down...')
        # Shutdown rclpy to exit spin loop
        rclpy.shutdown()
    
    # Register signal handlers
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(pick_skill)
    except KeyboardInterrupt:
        shutdown_requested = True
        pick_skill.get_logger().info('Received KeyboardInterrupt, shutting down...')
    finally:
        if pick_skill.robonix_client:
            pick_skill.robonix_client.shutdown()
        pick_skill.destroy_node()
        rclpy.shutdown()
        if shutdown_requested:
            pick_skill.get_logger().info('Pick skill shutdown complete')


if __name__ == '__main__':
    main()

