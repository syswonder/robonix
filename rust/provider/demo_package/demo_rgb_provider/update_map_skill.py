#!/usr/bin/env python3
"""
Demo update_map skill that updates semantic and spatial maps.
Skill queries robonix core to get dynamic topic names for vision capability,
then processes images and adds entities to the semantic map.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import time
import uuid


class UpdateMapSkill(Node):
    """Implements skl::update_map skill by using cap::vision.capture_rgb and perception services."""

    def __init__(self):
        super().__init__('demo_update_map_skill')
        
        # Query capabilities and get topic names
        self.vision_image_topic = None
        
        # Use RobonixSDK for service calls
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
        
        # Publish status (skill output)
        self.status_publisher = self.create_publisher(
            Bool,
            '/demo_update_map/status',
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
        
        self.latest_image = None
        self.processing_image = False
        self.entity_counter = 0
        
        self.get_logger().info('Demo update_map skill initialized')
        self.get_logger().info('  Publishing to: /demo_update_map/status')
    
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
    
    def _use_fallback_topics(self):
        """Use hardcoded topics as fallback when query service is not available."""
        if not self.vision_image_topic:
            self.vision_image_topic = '/demo_rgb/image'
            self.get_logger().info(f'  Using fallback vision topic: {self.vision_image_topic}')

    def image_callback(self, msg):
        """Handle image from vision capability (dynamic topic)."""
        self.latest_image = msg
        self.get_logger().debug(f'Received image from {self.vision_image_topic}')
        
        # Process image and update map
        if not self.processing_image:
            self._process_image_and_update_map(msg)

    def _process_image_and_update_map(self, image_msg):
        """Process image and add detected entities to semantic map."""
        if self.processing_image:
            return
        
        self.processing_image = True
        self.get_logger().info('Processing image and updating maps...')
        
        try:
            # Simulate object detection from image
            # In a real implementation, this would use computer vision algorithms
            detected_objects = self._simulate_object_detection(image_msg)
            
            if not detected_objects:
                self.get_logger().info('No objects detected in image')
                self._publish_status(True)  # Success but no objects
                self.processing_image = False
                return
            
            # Add each detected object as an entity to semantic map
            success_count = 0
            for obj_label, obj_pose in detected_objects:
                if self._add_entity_to_map(obj_label, obj_pose):
                    success_count += 1
                    self.get_logger().info(f'  Added entity: {obj_label}')
                else:
                    self.get_logger().warn(f'  Failed to add entity: {obj_label}')
            
            # Add spatial map entry
            if self._add_spatial_map_entry(image_msg):
                self.get_logger().info('  Added spatial map entry')
            
            self.get_logger().info(f'Successfully updated map: {success_count}/{len(detected_objects)} entities added')
            self._publish_status(True)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            self._publish_status(False)
        finally:
            self.processing_image = False

    def _simulate_object_detection(self, image_msg):
        """
        Simulate object detection from image.
        In a real implementation, this would use computer vision.
        
        Returns:
            List of (label, pose) tuples where pose is (x, y, z)
        """
        # Simulate detecting some objects
        # In real implementation, this would analyze the image
        self.entity_counter += 1
        
        # Simulate detecting 1-3 objects
        import random
        num_objects = random.randint(1, 3)
        objects = []
        
        for i in range(num_objects):
            label = f"object_{self.entity_counter}_{i}"
            # Simulated 3D pose
            x = 0.3 + random.uniform(-0.2, 0.2)
            y = 0.2 + random.uniform(-0.2, 0.2)
            z = 0.1 + random.uniform(-0.05, 0.05)
            objects.append((label, (x, y, z)))
        
        return objects

    def _add_entity_to_map(self, label, pose):
        """Add an entity to the semantic map using AddEntity service."""
        if not self.sdk:
            self.get_logger().warn('SDK not available, cannot add entity')
            return False
        
        try:
            from robonix_core.msg import Entity, Relation, FrameMapping, Point3D, BoundingBox, TextureType
            from robonix_core.msg import RelationType
            
            # Create Entity message
            entity = Entity()
            entity.id = str(uuid.uuid4())
            entity.label = label
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
            entity.frame_mapping = [frame_mapping]
            
            # Call AddEntity service
            result = self.sdk.add_entity(entity)
            
            if result.success:
                self.get_logger().info(f'  Successfully added entity {entity.id} ({label})')
                return True
            else:
                self.get_logger().warn(f'  Failed to add entity: {result.error_message}')
                return False
                
        except ImportError as e:
            self.get_logger().error(f'Failed to import robonix_core messages: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error adding entity: {e}')
            return False

    def _add_spatial_map_entry(self, image_msg):
        """Add a spatial map entry using AddSpatialMapEntry service."""
        if not self.sdk:
            self.get_logger().warn('SDK not available, cannot add spatial map entry')
            return False
        
        try:
            from robonix_core.msg import SpatialMapEntry
            
            # Create SpatialMapEntry
            entry = SpatialMapEntry()
            entry.frame_id = image_msg.header.frame_id if image_msg.header.frame_id else 'camera_frame'
            # Use image timestamp
            entry.timestamp = int(image_msg.header.stamp.sec * 1e9 + image_msg.header.stamp.nanosec)
            entry.source_skill = 'skl::update_map'
            
            # Call AddSpatialMapEntry service
            result = self.sdk.add_spatial_map_entry(entry)
            
            if result.success:
                return True
            else:
                self.get_logger().warn(f'  Failed to add spatial map entry: {result.error_message}')
                return False
                
        except ImportError as e:
            self.get_logger().error(f'Failed to import robonix_core messages: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error adding spatial map entry: {e}')
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
    
    try:
        rclpy.spin(update_map_skill)
    except KeyboardInterrupt:
        pass
    finally:
        update_map_skill.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

