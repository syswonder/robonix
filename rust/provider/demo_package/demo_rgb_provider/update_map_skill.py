#!/usr/bin/env python3
"""
Demo update_map skill that adds entities to semantic map.
Generates one entity every 5 seconds and adds it to the map.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import uuid
import random
import signal
from robonix_core import RobonixClient


class UpdateMapSkill(Node):
    """Implements skl::update_map skill by adding entities to semantic map."""

    def __init__(self):
        super().__init__('demo_update_map_skill')
        
        # Create Robonix client for service calls
        try:
            self.robonix_client = RobonixClient(node_name='demo_update_map_skill_client', max_workers=20)
        except Exception as e:
            import traceback
            self.get_logger().warn(f'Failed to create RobonixClient: {e}')
            self.get_logger().debug(f'Traceback: {traceback.format_exc()}')
            self.robonix_client = None
        
        self.get_logger().info(f'RobonixClient created: {self.robonix_client}')
        
        # Publish status (skill output)
        self.status_publisher = self.create_publisher(
            Bool,
            '/demo_update_map/status',
            10
        )
        
        self.entity_counter = 0
        
        # Create timer to generate entity every 5 seconds
        self.timer = self.create_timer(10, self._timer_callback)
        
        self.get_logger().info('Demo update_map skill initialized')
        self.get_logger().info('  Publishing to: /demo_update_map/status')
        self.get_logger().info('  Generating entity every 5 seconds')
    
    def _timer_callback(self):
        """Timer callback: generate and add one entity every 5 seconds."""
        self.entity_counter += 1
        
        # Generate a simple entity
        label = f"object_{self.entity_counter}"
        # Simulated 3D pose
        x = 0.3 + random.uniform(-0.2, 0.2)
        y = 0.2 + random.uniform(-0.2, 0.2)
        z = 0.1 + random.uniform(-0.05, 0.05)
        pose = (x, y, z)
        
        self.get_logger().info(f'Generating entity: {label} at pose ({x:.2f}, {y:.2f}, {z:.2f})')
        
        # Add entity to map
        if self._add_entity_to_map(label, pose):
            self.get_logger().info(f'Successfully added entity: {label}')
            self._publish_status(True)
        else:
            self.get_logger().warn(f'Failed to add entity: {label}')
            self._publish_status(False)

    def _add_entity_to_map(self, label, pose):
        """Add an entity to the semantic map using AddEntity service."""
        if not self.robonix_client:
            self.get_logger().warn('RobonixClient not available, cannot add entity')
            return False
        
        try:
            from robonix_core.msg import Entity, FrameMapping, Point3D
            
            # Create Entity message
            entity = Entity()
            entity.id = str(uuid.uuid4())
            entity.label = label
            # Initialize list fields as proper lists (not dict-like)
            entity.relations = []  # Empty relations for now
            entity.registered_skills = []
            entity.registered_capabilities = []
            
            # Create FrameMapping
            frame_mapping = FrameMapping()
            frame_mapping.frame_id = 'camera_frame'
            
            # Set center point
            center = Point3D()
            center.x = float(pose[0])
            center.y = float(pose[1])
            center.z = float(pose[2])
            frame_mapping.center = center
            
            # Optional bounding box (empty array means no bbox)
            frame_mapping.bbox = []
            
            # Optional texture (empty array means no texture info)
            frame_mapping.texture = []
            
            # Add frame mapping to entity (as array)
            # Ensure it's a proper list, not dict-like
            entity.frame_mapping = [frame_mapping]
            
            # Call AddEntity service using client
            response = self.robonix_client.add_entity(entity, timeout_sec=5.0)
            
            if response and response.success:
                self.get_logger().info(f'  Successfully added entity {entity.id} ({label})')
                return True
            else:
                error_msg = response.error_message if response else 'No response'
                self.get_logger().warn(f'  Failed to add entity: {error_msg}')
                return False
                
        except ImportError as e:
            self.get_logger().error(f'Failed to import robonix_core messages: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error adding entity: {e}')
            return False

    def _publish_status(self, status):
        """Publish skill status."""
        msg = Bool()
        msg.data = status
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Published status: {status}')


def main(args=None):
    rclpy.init(args=args)
    update_map_skill = UpdateMapSkill()
    
    # Flag to track if shutdown was requested
    shutdown_requested = False
    
    def signal_handler(signum, frame):
        """Handle shutdown signals (SIGTERM, SIGINT)."""
        nonlocal shutdown_requested
        shutdown_requested = True
        update_map_skill.get_logger().info(f'Received signal {signum}, shutting down...')
        # Destroy timer to stop generating entities
        if hasattr(update_map_skill, 'timer'):
            update_map_skill.destroy_timer(update_map_skill.timer)
        # Shutdown rclpy to exit spin loop
        rclpy.shutdown()
    
    # Register signal handlers
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(update_map_skill)
    except KeyboardInterrupt:
        shutdown_requested = True
        update_map_skill.get_logger().info('Received KeyboardInterrupt, shutting down...')
    finally:
        if update_map_skill.robonix_client:
            update_map_skill.robonix_client.shutdown()
        update_map_skill.destroy_node()
        rclpy.shutdown()
        if shutdown_requested:
            update_map_skill.get_logger().info('Update map skill shutdown complete')


if __name__ == '__main__':
    main()

