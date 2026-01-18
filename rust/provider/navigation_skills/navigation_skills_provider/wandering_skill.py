#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Wandering Skill
#
# Random wandering skill that collects semantic objects in rooms.
# Uses semantic_map service to discover objects and navigates to them.
""""""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import json
import random
import time
import signal
from robonixpy import RobonixClient
from robonix_sdk.srv import QuerySemanticMap
from robonix_sdk.msg import Object


class WanderingSkill(Node):
    """Implements wandering skill that collects semantic objects."""

    def __init__(self):
        super().__init__('wandering_skill')
        
        # EAIOS skill interface topics (from manifest)
        self.start_topic = '/robot1/skill/wandering/start'
        self.status_topic = '/robot1/skill/wandering/status'
        
        # Query services and primitives
        self.semantic_map_service_entry = None
        self.pose_topic = None
        self.navigate_goal_topic = None
        self.navigate_status_topic = None
        
        # Collected objects registry
        self.collected_objects = {}  # {object_id: Object}
        
        # Create Robonix client for service calls
        try:
            self.get_logger().info('Creating RobonixClient...')
            self.robonix_client = RobonixClient(node_name='wandering_skill_client', max_workers=20)
            self.get_logger().info('RobonixClient created successfully, querying services and primitives...')
            # Query services and primitives
            self._query_services_and_primitives()
        except Exception as e:
            import traceback
            error_traceback = traceback.format_exc()
            self.get_logger().error(f'Failed to create RobonixClient: {e}')
            self.get_logger().error(f'Full traceback:\n{error_traceback}')
            self.robonix_client = None
        
        # Subscribe to skill start topic
        self.start_subscriber = self.create_subscription(
            String,
            self.start_topic,
            self.start_callback,
            10
        )
        self.get_logger().info(f'Subscribing to start topic: {self.start_topic}')
        
        # Publish to skill status topic
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        self.get_logger().info(f'Publishing to status topic: {self.status_topic}')
        
        # Subscribe to pose topic (if available)
        if self.pose_topic:
            self.pose_subscriber = self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                10
            )
            self.get_logger().info(f'Subscribing to pose topic: {self.pose_topic}')
        else:
            self.pose_subscriber = None
        
        # Publish to navigate goal topic (if available)
        if self.navigate_goal_topic:
            self.navigate_goal_publisher = self.create_publisher(
                PoseStamped,
                self.navigate_goal_topic,
                10
            )
            self.get_logger().info(f'Publishing to navigate goal: {self.navigate_goal_topic}')
        else:
            self.navigate_goal_publisher = None
        
        # Subscribe to navigate status topic (if available)
        if self.navigate_status_topic:
            self.navigate_status_subscriber = self.create_subscription(
                Bool,
                self.navigate_status_topic,
                self.navigate_status_callback,
                10
            )
            self.get_logger().info(f'Subscribing to navigate status: {self.navigate_status_topic}')
        else:
            self.navigate_status_subscriber = None
        
        # Create semantic map service client
        if self.semantic_map_service_entry:
            from robonix_sdk.srv import QuerySemanticMap
            self.semantic_map_client = self.create_client(
                QuerySemanticMap,
                self.semantic_map_service_entry
            )
            self.get_logger().info(f'Semantic map service client created: {self.semantic_map_service_entry}')
        else:
            self.semantic_map_client = None
        
        # State
        self.current_skill_id = None
        self.wandering_in_progress = False
        self.current_goal = None
        self.navigation_complete = False
        self.latest_pose = None
        
        self.get_logger().info('Wandering skill initialized')
    
    def _query_services_and_primitives(self):
        """Query robonix core for required services and primitives."""
        if not self.robonix_client:
            self.get_logger().error('RobonixClient not available')
            return
        
        # Query semantic_map service
        self.get_logger().info('Querying semantic_map service...')
        try:
            response = self.robonix_client.query_service('semantic_map')
            if response and response.instances:
                instance = response.instances[0]
                self.semantic_map_service_entry = instance.entry
                self.get_logger().info(f'  Found semantic_map service: {self.semantic_map_service_entry}')
        except Exception as e:
            self.get_logger().error(f'Error querying semantic_map service: {e}')
        
        # Query prm::base.pose
        self.get_logger().info('Querying prm::base.pose...')
        try:
            response = self.robonix_client.query_primitive('prm::base.pose')
            if response and response.instances:
                instance = response.instances[0]
                output_schema = json.loads(instance.output_schema)
                if 'pose' in output_schema:
                    self.pose_topic = output_schema['pose']
                    self.get_logger().info(f'  Found pose topic: {self.pose_topic}')
        except Exception as e:
            self.get_logger().error(f'Error querying prm::base.pose: {e}')
        
        # Query prm::base.navigate
        self.get_logger().info('Querying prm::base.navigate...')
        try:
            response = self.robonix_client.query_primitive('prm::base.navigate')
            if response and response.instances:
                instance = response.instances[0]
                input_schema = json.loads(instance.input_schema)
                output_schema = json.loads(instance.output_schema)
                if 'goal' in input_schema:
                    self.navigate_goal_topic = input_schema['goal']
                    self.get_logger().info(f'  Found navigate goal topic: {self.navigate_goal_topic}')
                if 'status' in output_schema:
                    self.navigate_status_topic = output_schema['status']
                    self.get_logger().info(f'  Found navigate status topic: {self.navigate_status_topic}')
        except Exception as e:
            self.get_logger().error(f'Error querying prm::base.navigate: {e}')
    
    def start_callback(self, msg):
        """Handle skill start request."""
        try:
            data = json.loads(msg.data)
            skill_id = data.get('skill_id', 'unknown')
            params = data.get('params', {})
            
            self.get_logger().info(f'Received wandering request: skill_id={skill_id}, params={params}')
            
            if self.wandering_in_progress:
                self.get_logger().warn('Wandering already in progress, ignoring request')
                self._publish_status(skill_id, 'error', {'error': 'Operation already in progress'})
                return
            
            # Check if required services/primitives are available
            if not self.semantic_map_client:
                self.get_logger().error('Semantic map service not available!')
                self._publish_status(skill_id, 'error', {'error': 'Semantic map service not available'})
                return
            
            if not self.navigate_goal_publisher:
                self.get_logger().error('Navigation primitive not available!')
                self._publish_status(skill_id, 'error', {'error': 'Navigation primitive not available'})
                return
            
            self.current_skill_id = skill_id
            self.wandering_in_progress = True
            self.collected_objects = {}  # Reset collected objects
            
            # Publish running status
            self._publish_status(skill_id, 'running', {'message': 'Starting wandering...'})
            
            # Start wandering loop in a separate thread or async
            self._start_wandering_loop()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse start message JSON: {e}')
            self._publish_status('unknown', 'error', {'error': f'Invalid JSON: {e}'})
        except Exception as e:
            self.get_logger().error(f'Error in start_callback: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
    
    def _start_wandering_loop(self):
        """Start the wandering loop to collect objects."""
        import threading
        thread = threading.Thread(target=self._wandering_loop, daemon=True)
        thread.start()
    
    def _wandering_loop(self):
        """Main wandering loop that collects objects."""
        max_iterations = 10  # Limit iterations
        iteration = 0
        
        while self.wandering_in_progress and iteration < max_iterations:
            iteration += 1
            self.get_logger().info(f'Wandering iteration {iteration}/{max_iterations}')
            
            # Query semantic map for objects
            try:
                objects = self._query_semantic_map()
                if not objects:
                    self.get_logger().info('No objects found in semantic map, waiting...')
                    time.sleep(2.0)
                    continue
                
                # Filter out already collected objects
                new_objects = [obj for obj in objects if obj.id not in self.collected_objects]
                
                if not new_objects:
                    self.get_logger().info('All objects already collected, selecting random object to revisit')
                    new_objects = objects
                
                # Randomly select an object
                target_object = random.choice(new_objects)
                self.get_logger().info(f'Selected target object: {target_object.id} ({target_object.label})')
                
                # Get object position in map frame
                object_pose = self._get_object_pose_in_map(target_object)
                if not object_pose:
                    self.get_logger().warn(f'Could not get pose for object {target_object.id}, skipping')
                    continue
                
                # Navigate to object (with offset to be "near" it)
                offset_distance = 0.5  # meters
                goal_pose = self._calculate_goal_pose_near_object(object_pose, offset_distance)
                
                self.get_logger().info(f'Navigating to near object {target_object.id} at ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
                
                # Publish navigation goal
                self.current_goal = target_object
                self.navigation_complete = False
                self.navigate_goal_publisher.publish(goal_pose)
                
                # Wait for navigation to complete (with timeout)
                timeout = 30.0  # seconds
                start_time = time.time()
                while not self.navigation_complete and (time.time() - start_time) < timeout:
                    time.sleep(0.5)
                
                if self.navigation_complete:
                    # Register collected object
                    if target_object.id not in self.collected_objects:
                        self.collected_objects[target_object.id] = target_object
                        self.get_logger().info(f'Collected object: {target_object.id} ({target_object.label})')
                        self._publish_status(
                            self.current_skill_id,
                            'running',
                            {
                                'message': f'Collected object: {target_object.label}',
                                'collected_count': len(self.collected_objects),
                                'object_id': target_object.id
                            }
                        )
                else:
                    self.get_logger().warn(f'Navigation timeout for object {target_object.id}')
                
                # Wait before next iteration
                time.sleep(1.0)
                
            except Exception as e:
                self.get_logger().error(f'Error in wandering loop: {e}')
                import traceback
                self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
                time.sleep(2.0)
        
        # Wandering complete
        self.wandering_in_progress = False
        result = {
            'message': 'Wandering completed',
            'collected_objects': len(self.collected_objects),
            'object_ids': list(self.collected_objects.keys())
        }
        self._publish_status(self.current_skill_id, 'finished', result)
        self.get_logger().info(f'Wandering completed. Collected {len(self.collected_objects)} objects.')
    
    def _query_semantic_map(self):
        """Query semantic map service for objects."""
        if not self.semantic_map_client:
            return []
        
        # Wait for service
        if not self.semantic_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Semantic map service not available')
            return []
        
        # Create request (empty types means all objects)
        request = QuerySemanticMap.Request()
        request.types = []  # Empty means all types
        
        # Call service
        future = self.semantic_map_client.call_async(request)
        
        # Wait for response
        start_time = time.time()
        timeout = 5.0
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().warn('Semantic map query timeout')
            return []
        
        try:
            response = future.result()
            return list(response.objects)
        except Exception as e:
            self.get_logger().error(f'Error querying semantic map: {e}')
            return []
    
    def _get_object_pose_in_map(self, obj):
        """Get object pose in map frame."""
        # Look for map frame mapping
        for frame_mapping in obj.frame_mapping:
            if frame_mapping.frame_id == 'map':
                # Create PoseStamped from frame mapping
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(frame_mapping.center.x)
                pose.pose.position.y = float(frame_mapping.center.y)
                pose.pose.position.z = float(frame_mapping.center.z)
                # Use default orientation (facing forward)
                pose.pose.orientation.w = 1.0
                return pose
        
        # If no map frame, try to use camera frame and transform (simplified)
        # For now, return None if no map frame
        return None
    
    def _calculate_goal_pose_near_object(self, object_pose, offset_distance):
        """Calculate goal pose near object (offset by distance)."""
        goal_pose = PoseStamped()
        goal_pose.header = object_pose.header
        
        # Simple approach: offset in negative x direction (towards object from front)
        # In real implementation, could use current robot pose to calculate better offset
        goal_pose.pose.position.x = object_pose.pose.position.x - offset_distance
        goal_pose.pose.position.y = object_pose.pose.position.y
        goal_pose.pose.position.z = object_pose.pose.position.z
        
        # Face the object
        import math
        dx = object_pose.pose.position.x - goal_pose.pose.position.x
        dy = object_pose.pose.position.y - goal_pose.pose.position.y
        yaw = math.atan2(dy, dx)
        
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal_pose
    
    def pose_callback(self, msg):
        """Handle pose updates."""
        self.latest_pose = msg
    
    def navigate_status_callback(self, msg):
        """Handle navigation status updates."""
        if hasattr(msg, 'data'):
            success = msg.data
            if success:
                self.navigation_complete = True
                self.get_logger().info('Navigation completed successfully')
    
    def _publish_status(self, skill_id, state, result):
        """Publish skill status to status_topic."""
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
    wandering_skill = WanderingSkill()
    
    shutdown_requested = False
    
    def signal_handler(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        wandering_skill.get_logger().info(f'Received signal {signum}, shutting down...')
        rclpy.shutdown()
    
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(wandering_skill)
    except KeyboardInterrupt:
        shutdown_requested = True
        wandering_skill.get_logger().info('Received KeyboardInterrupt, shutting down...')
    finally:
        if wandering_skill.robonix_client:
            wandering_skill.robonix_client.shutdown()
        wandering_skill.destroy_node()
        rclpy.shutdown()
        if shutdown_requested:
            wandering_skill.get_logger().info('Wandering skill shutdown complete')


if __name__ == '__main__':
    main()
