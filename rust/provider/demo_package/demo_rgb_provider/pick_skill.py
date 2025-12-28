#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Pick Skill
#
# Demo pick skill that combines vision and grasp capabilities.
# Implements EAIOS skill interface with start_topic and status_topic.
""""""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import json
import time
import signal
from robonixpy import RobonixClient


class PickSkill(Node):
    """Implements pick skill using EAIOS skill interface."""

    def __init__(self):
        super().__init__('demo_pick_skill')
        
        # EAIOS skill interface topics (from manifest)
        self.start_topic = '/robot1/skill/pick/start'
        self.status_topic = '/robot1/skill/pick/status'
        
        # Query primitives and get topic names
        self.vision_image_topic = None
        self.grasp_pose_goal_topic = None
        self.grasp_status_topic = None
        
        # Create Robonix client for service calls
        try:
            self.get_logger().info('Creating RobonixClient...')
            self.robonix_client = RobonixClient(node_name='demo_pick_skill_client', max_workers=20)
            self.get_logger().info('RobonixClient created successfully, querying primitives...')
            # Query primitives
            self._query_primitives()
        except Exception as e:
            import traceback
            error_traceback = traceback.format_exc()
            self.get_logger().error(f'Failed to create RobonixClient: {e}')
            self.get_logger().error(f'Error type: {type(e).__name__}')
            self.get_logger().error(f'Full traceback:\n{error_traceback}')
            import sys
            print(f'[ERROR] Failed to create RobonixClient: {e}', file=sys.stderr)
            self.robonix_client = None
            self._use_fallback_topics()
        
        # Subscribe to skill start topic (receives JSON with parameters)
        self.start_subscriber = self.create_subscription(
            String,
            self.start_topic,
            self.start_callback,
            10
        )
        self.get_logger().info(f'Subscribing to start topic: {self.start_topic}')
        
        # Publish to skill status topic (publishes JSON with state and result)
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        self.get_logger().info(f'Publishing to status topic: {self.status_topic}')
        
        # Subscribe to vision primitive output (dynamic topic)
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
            self.get_logger().warn('  Vision primitive not available!')
        
        # Publish to grasp primitive input (dynamic topic)
        if self.grasp_pose_goal_topic:
            self.pose_goal_publisher = self.create_publisher(
                PoseStamped,
                self.grasp_pose_goal_topic,
                10
            )
            self.get_logger().info(f'  Publishing to grasp input: {self.grasp_pose_goal_topic}')
        else:
            self.pose_goal_publisher = None
            self.get_logger().warn('  Grasp primitive not available!')
        
        # Subscribe to grasp primitive output (dynamic topic)
        if self.grasp_status_topic:
            self.grasp_status_subscriber = self.create_subscription(
                String,
                self.grasp_status_topic,
                self.grasp_status_callback,
                10
            )
            self.get_logger().info(f'  Subscribing to grasp output: {self.grasp_status_topic}')
        else:
            self.grasp_status_subscriber = None
        
        self.current_skill_id = None
        self.current_target_label = None
        self.latest_image = None
        self.grasp_complete = False
        self.picking_in_progress = False
        
        self.get_logger().info('Demo pick skill initialized')
    
    def _query_primitives(self):
        """Query robonix core for required primitives and get their topic names."""
        if not self.robonix_client:
            self._use_fallback_topics()
            return
        
        # Query prm::camera.capture
        self.get_logger().info('Querying prm::camera.capture...')
        try:
            response = self.robonix_client.query_primitive('prm::camera.capture')
            if response and response.instances:
                instance = response.instances[0]
                # Parse output_schema to get image topic
                import json
                output_schema = json.loads(instance.output_schema)
                if 'image' in output_schema:
                    self.vision_image_topic = output_schema['image']
                    self.get_logger().info(f'  Found vision primitive: {self.vision_image_topic}')
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error querying prm::camera.capture: {e}')
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
        
        # Query prm::arm.move.ee
        self.get_logger().info('Querying prm::arm.move.ee...')
        try:
            response = self.robonix_client.query_primitive('prm::arm.move.ee')
            if response and response.instances:
                instance = response.instances[0]
                # Parse input_schema and output_schema to get topics
                import json
                input_schema = json.loads(instance.input_schema)
                output_schema = json.loads(instance.output_schema)
                if 'pose' in input_schema:
                    self.grasp_pose_goal_topic = input_schema['pose']
                    self.get_logger().info(f'  Found grasp input topic: {self.grasp_pose_goal_topic}')
                if 'status' in output_schema:
                    self.grasp_status_topic = output_schema['status']
                    self.get_logger().info(f'  Found grasp output topic: {self.grasp_status_topic}')
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error querying prm::arm.move.ee: {e}')
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
        
        # Use fallback topics if query failed
        if not self.vision_image_topic or not self.grasp_pose_goal_topic:
            self.get_logger().warn('Some primitives not found, using fallback topics')
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

    def start_callback(self, msg):
        """Handle skill start request from start_topic (JSON format)."""
        try:
            data = json.loads(msg.data)
            skill_id = data.get('skill_id', 'unknown')
            params = data.get('params', {})
            target_label = params.get('target_label', '')
            
            self.get_logger().info(f'Received pick request: skill_id={skill_id}, target_label={target_label}')
            
            if self.picking_in_progress:
                self.get_logger().warn('Pick operation already in progress, ignoring request')
                self._publish_status(skill_id, 'error', {'error': 'Operation already in progress'})
                return
            
            # Check if required primitives are available
            if not self.vision_image_topic or not self.grasp_pose_goal_topic:
                self.get_logger().error('Required primitives not available!')
                self._publish_status(skill_id, 'error', {'error': 'Required primitives not available'})
                return
            
            self.current_skill_id = skill_id
            self.current_target_label = target_label
            self.picking_in_progress = True
            self.grasp_complete = False
            
            # Publish running status
            self._publish_status(skill_id, 'running', {})
            
            # Step 1: Wait for image from vision primitive
            self.get_logger().info('Step 1: Waiting for image from prm::camera.capture...')
            # In real implementation, we would process the image here
            # For demo, we simulate object detection
            
            # Step 2: Simulate finding object and calculate pose
            time.sleep(0.2)
            self.get_logger().info(f'Step 2: Simulated detection of object with label: {target_label}')
            
            # Step 3: Send pose goal to grasp primitive
            pose_goal = PoseStamped()
            pose_goal.header.stamp = self.get_clock().now().to_msg()
            pose_goal.header.frame_id = 'base_frame'
            # Simulated pose for the object
            pose_goal.pose.position.x = 0.5
            pose_goal.pose.position.y = 0.2
            pose_goal.pose.position.z = 0.1
            pose_goal.pose.orientation.w = 1.0
            
            self.get_logger().info('Step 3: Sending pose goal to prm::arm.move.ee...')
            if self.pose_goal_publisher:
                self.pose_goal_publisher.publish(pose_goal)
            else:
                self.get_logger().error('Grasp primitive not available!')
                self._publish_status(skill_id, 'error', {'error': 'Grasp primitive not available'})
                self.picking_in_progress = False
            
            # Wait for grasp to complete (handled in grasp_status_callback)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse start message JSON: {e}')
            self._publish_status('unknown', 'error', {'error': f'Invalid JSON: {e}'})
        except Exception as e:
            self.get_logger().error(f'Error in start_callback: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')

    def image_callback(self, msg):
        """Handle image from vision primitive (dynamic topic)."""
        self.latest_image = msg
        self.get_logger().debug(f'Received image from {self.vision_image_topic}')

    def grasp_status_callback(self, msg):
        """Handle status from grasp primitive (dynamic topic)."""
        # Parse status (could be JSON or Bool, depending on implementation)
        try:
            if hasattr(msg, 'data'):
                # Bool message
                success = msg.data
            else:
                # String message (JSON)
                status_data = json.loads(msg.data) if isinstance(msg.data, str) else msg.data
                success = status_data.get('success', False) if isinstance(status_data, dict) else bool(status_data)
        except:
            success = False
        
        if success and self.picking_in_progress:
            self.grasp_complete = True
            self.get_logger().info('Step 4: Grasp movement completed')
            
            # Publish skill success status
            result = {
                'target_label': self.current_target_label,
                'success': True
            }
            self._publish_status(self.current_skill_id, 'finished', result)
            self.get_logger().info('Pick skill completed successfully')
            
            self.picking_in_progress = False
            self.grasp_complete = False
            self.current_skill_id = None
            self.current_target_label = None

    def _publish_status(self, skill_id, state, result):
        """Publish skill status to status_topic (JSON format)."""
        status_msg = {
            'skill_id': skill_id,
            'state': state,
            'result': result,
            'diagnostics': {}
        }
        
        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Published status: skill_id={skill_id}, state={state}')


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
