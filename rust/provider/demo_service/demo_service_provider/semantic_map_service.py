#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Semantic Map Service
#
# Demo semantic map service implementation.
# Simulates an object graph by constructing mock objects.
""""""

import rclpy
from rclpy.node import Node
from robonix_sdk.srv import QuerySemanticMap
from robonix_sdk.msg import Object, Relation, RelationType, FrameMapping, Point3D, BoundingBox
from builtin_interfaces.msg import Time
import random
import json


class SemanticMapService(Node):
    """Implements semantic_map service with simulated object graph."""

    def __init__(self):
        super().__init__('demo_semantic_map_service')
        
        # Create service
        self.service = self.create_service(
            QuerySemanticMap,
            '/demo_service/semantic_map/query',
            self.query_callback
        )
        
        self.get_logger().info('Semantic map service started')
        self.get_logger().info('  Service: /demo_service/semantic_map/query')

    def query_callback(self, request, response):
        """Handle semantic map query request."""
        self.get_logger().info(f'Received query with types filter: {request.types}')
        
        # Generate mock object graph
        objects = self._generate_mock_objects(request.types)
        
        # Set response - objects is Object[] as per service definition
        response.objects = objects
        response.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'Returning {len(objects)} objects')
        
        # Log objects for debugging
        objects_json = self._objects_to_json(objects)
        self.get_logger().info(f'Objects JSON: {json.dumps(objects_json, indent=2)}')
        return response
    
    def _objects_to_json(self, objects):
        """Convert Object messages to JSON-serializable format."""
        result = []
        for obj in objects:
            obj_dict = {
                'id': obj.id,
                'label': obj.label,
                'registered_skills': list(obj.registered_skills),
                'registered_primitives': list(obj.registered_primitives),
                'relations': []
            }
            
            # Convert relations
            for rel in obj.relations:
                rel_dict = {
                    'relation_type': rel.relation_type.type if hasattr(rel.relation_type, 'type') else None,
                    'target_entity_id': rel.target_entity_id
                }
                obj_dict['relations'].append(rel_dict)
            
            # Convert frame mappings
            obj_dict['frame_mapping'] = []
            for fm in obj.frame_mapping:
                fm_dict = {
                    'frame_id': fm.frame_id,
                    'center': {
                        'x': fm.center.x,
                        'y': fm.center.y,
                        'z': fm.center.z
                    },
                    'bbox': []
                }
                for bbox in fm.bbox:
                    bbox_dict = {
                        'scale_x': bbox.scale_x,
                        'scale_y': bbox.scale_y,
                        'scale_z': bbox.scale_z,
                        'yaw': bbox.yaw
                    }
                    fm_dict['bbox'].append(bbox_dict)
                obj_dict['frame_mapping'].append(fm_dict)
            
            result.append(obj_dict)
        return result

    def _generate_mock_objects(self, type_filter=None):
        """Generate a mock object graph."""
        objects = []
        
        # Define mock objects - simplified and aligned with specs
        # Primitives from specs_table.rs: prm::camera.capture, prm::arm.move.ee, prm::gripper.close
        # Note: Only robots have skills and primitives. Objects in the environment don't have skills.
        mock_data = [
            {
                'id': 'robot_001',
                'label': 'robot1',
                'type': 'robot',
                'skills': ['skl::pick'], # should be queried from OS - TODO!
                'primitives': ['prm::arm.move.ee', 'prm::gripper.close', 'prm::camera.capture'],  # Standard primitives from specs
                'position': (0.5, 0.5, 0.0),
                'size': (0.4, 0.4, 0.8)
            },
            {
                'id': 'table_001',
                'label': 'dining_table',
                'type': 'table',
                'skills': [],  # Objects don't have skills
                'primitives': [],
                'position': (1.0, 1.0, 0.4),
                'size': (1.2, 0.8, 0.4)
            },
            {
                'id': 'box_001',
                'label': 'red_box',
                'type': 'box',
                'skills': [],  # Objects don't have skills
                'primitives': [],
                'position': (1.0, 1.0, 0.8),
                'size': (0.2, 0.2, 0.2),
                'parent': 'table_001'
            }
        ]
        
        # Filter by type if specified
        if type_filter and len(type_filter) > 0:
            mock_data = [obj for obj in mock_data if obj['type'] in type_filter]
        
        # Convert to Object messages
        for data in mock_data:
            obj = Object()
            obj.id = data['id']
            obj.label = data['label']
            
            # Create relations
            obj.relations = []
            if 'parent' in data:
                relation = Relation()
                relation.relation_type = RelationType()
                relation.relation_type.type = RelationType.CHILD_OF
                relation.target_entity_id = data['parent']
                obj.relations.append(relation)
            
            # Set registered skills and primitives (aligned with Object.msg and specs_table.rs)
            obj.registered_skills = data['skills']
            obj.registered_primitives = data['primitives']  # Use standard primitive names from specs
            
            # Create frame mapping
            obj.frame_mapping = []
            frame_mapping = FrameMapping()
            frame_mapping.center = Point3D()
            frame_mapping.center.x = float(data['position'][0])
            frame_mapping.center.y = float(data['position'][1])
            frame_mapping.center.z = float(data['position'][2])
            
            # Create bounding box
            bbox = BoundingBox()
            bbox.scale_x = float(data['size'][0])
            bbox.scale_y = float(data['size'][1])
            bbox.scale_z = float(data['size'][2])
            bbox.yaw = 0.0
            frame_mapping.bbox = [bbox]
            
            frame_mapping.frame_id = 'map'
            obj.frame_mapping.append(frame_mapping)
            
            objects.append(obj)
        
        return objects


def main(args=None):
    rclpy.init(args=args)
    semantic_map_service = SemanticMapService()
    rclpy.spin(semantic_map_service)
    semantic_map_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

