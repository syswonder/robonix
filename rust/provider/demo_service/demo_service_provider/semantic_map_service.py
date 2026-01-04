#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Semantic Map Service
#
# Real semantic map service implementation using front camera and qwen3-vl VLM.
# Queries camera primitives from OS, subscribes to RGB images and camera_info,
# uses qwen3-vl to detect objects and estimate their 3D coordinates in camera frame.
""""""

import os
import json
import base64
from pathlib import Path
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from robonix_sdk.srv import QuerySemanticMap
from robonix_sdk.msg import Object, Relation, RelationType, FrameMapping, Point3D, BoundingBox
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from dotenv import load_dotenv
from openai import OpenAI
import numpy as np
from robonix_sdk.srv import QueryPrimitive


class SemanticMapService(Node):
    """Implements semantic_map service using front camera and qwen3-vl VLM."""

    def __init__(self):
        super().__init__('demo_semantic_map_service')
        
        # Load environment variables
        current_file = Path(__file__).resolve()
        package_root = current_file.parent
        while package_root != package_root.parent:
            if (package_root / 'setup.py').exists():
                break
            package_root = package_root.parent
        if not (package_root / 'setup.py').exists():
            package_root = current_file.parent.parent
        
        env_path = package_root / '.env'
        load_dotenv(env_path)
        
        # Get Qwen3-VL API key from environment
        self.qwen_api_key = os.getenv('QWEN3_VL_API_KEY')
        if not self.qwen_api_key:
            self.get_logger().error(
                'QWEN3_VL_API_KEY not found in .env file. '
                'Please configure QWEN3_VL_API_KEY in .env file.'
            )
            raise ValueError('QWEN3_VL_API_KEY not found in .env file.')
        
        # Initialize Qwen3-VL client (using OpenAI-compatible API)
        try:
            # Qwen3-VL uses DashScope API (Alibaba Cloud)
            self.qwen_client = OpenAI(
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
                api_key=self.qwen_api_key,
            )
            self.qwen_model = "qwen-vl-plus"  # Model name
            self.get_logger().info(f'Qwen3-VL API client initialized with model: {self.qwen_model}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Qwen3-VL API client: {e}')
            raise
        
        # Create service client for querying primitives with QoS matching server
        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            durability=DurabilityPolicy.VOLATILE
        )
        self.query_primitive_client = self.create_client(
            QueryPrimitive,
            '/rbnx/prm/query',
            qos_profile=service_qos
        )
        self.get_logger().info('QueryPrimitive service client created')
        
        # Initialize CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Setup cache directory for saving RGB images (for debugging)
        self.cache_dir = package_root / 'cache'
        try:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f'Cache directory: {self.cache_dir}')
        except Exception as e:
            self.get_logger().error(f'Failed to create cache directory {self.cache_dir}: {e}')
            raise
        
        # Camera topics (will be queried from primitives)
        self.rgb_image_topic = None
        self.camera_info_topic = None
        self.latest_rgb_image = None
        self.latest_camera_info = None
        self.image_counter = 0  # Counter for saved images
        
        # Query camera primitives - exit if failed
        self._query_camera_primitives()
        
        # Subscribe to camera topics
        self.rgb_subscriber = self.create_subscription(
            Image,
            self.rgb_image_topic,
            self.rgb_image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to RGB image: {self.rgb_image_topic}')
        
        # Infer camera_info topic from image topic (standard ROS2 convention)
        # If image topic is /head_front_camera/rgb/image_raw, camera_info is /head_front_camera/rgb/camera_info
        if self.rgb_image_topic.endswith('/image_raw'):
            self.camera_info_topic = self.rgb_image_topic.replace('/image_raw', '/camera_info')
        else:
            # Try to infer from common patterns
            import re
            match = re.match(r'(.+)/(rgb|depth)/image_raw', self.rgb_image_topic)
            if match:
                self.camera_info_topic = f"{match.group(1)}/{match.group(2)}/camera_info"
            else:
                self.get_logger().error(f'Cannot infer camera_info topic from image topic: {self.rgb_image_topic}')
                raise ValueError(f'Cannot infer camera_info topic from image topic: {self.rgb_image_topic}')
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.get_logger().info(f'Subscribed to camera info: {self.camera_info_topic}')
        
        # Create service
        self.service = self.create_service(
            QuerySemanticMap,
            '/demo_service/semantic_map/query',
            self.query_callback
        )
        
        self.get_logger().info('Semantic map service started')
        self.get_logger().info('  Service: /demo_service/semantic_map/query')
        self.get_logger().info('  Using Qwen3-VL for object detection')
    
    def _query_camera_primitives(self):
        """Query front camera primitives from OS with retry logic. Exits if failed."""
        # Query prm::camera.capture with filter for front camera
        # Retry a few times in case robonix-core is still starting up
        max_retries = 5
        retry_delay = 2.0  # seconds
        
        for attempt in range(max_retries):
            self.get_logger().info(f'Querying prm::camera.capture (front camera)... (attempt {attempt + 1}/{max_retries})')
            try:
                # Wait for service to be available (like ping_client: longer timeout on first attempts)
                wait_timeout = 10.0 if attempt < 2 else 5.0
                if not self.query_primitive_client.wait_for_service(timeout_sec=wait_timeout):
                    self.get_logger().warn(f'  query_primitive service not available (attempt {attempt + 1}/{max_retries})')
                    if attempt < max_retries - 1:
                        import time
                        time.sleep(retry_delay)
                        continue
                    else:
                        self.get_logger().error('  query_primitive service not available after all retries')
                        raise RuntimeError('query_primitive service not available after all retries')
                
                # Create request
                request = QueryPrimitive.Request()
                request.name = 'prm::camera.capture'
                filter_dict = {"camera": "front"}
                request.filter = json.dumps(filter_dict)
                
                # Call service
                future = self.query_primitive_client.call_async(request)
                
                # Wait for response (like ping_client: no sleep, just spin)
                import time
                start_time = time.time()
                timeout_sec = 3.0  # Same as ping_client
                while not future.done() and (time.time() - start_time) < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.01)
                
                if not future.done():
                    elapsed = time.time() - start_time
                    self.get_logger().warn(f'  Service call timeout after {elapsed:.1f}s (attempt {attempt + 1}/{max_retries})')
                    # Cancel the future to avoid resource leak
                    try:
                        future.cancel()
                    except:
                        pass
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        raise RuntimeError(f'Service call timeout after {elapsed:.1f}s after all retries')
                
                # Get response
                response = future.result()
                
                if response and response.instances:
                    instance = response.instances[0]
                    # output_schema is already a dict, not a string
                    if isinstance(instance.output_schema, str):
                        output_schema = json.loads(instance.output_schema)
                    else:
                        output_schema = instance.output_schema
                    
                    if 'image' in output_schema:
                        self.rgb_image_topic = output_schema['image']
                        self.get_logger().info(f'  Found front camera RGB topic: {self.rgb_image_topic}')
                        return  # Success, exit retry loop
                    else:
                        raise ValueError('Front camera primitive found but no "image" in output_schema')
                else:
                    self.get_logger().warn(f'  No front camera primitive found (attempt {attempt + 1}/{max_retries})')
                    if attempt < max_retries - 1:
                        import time
                        time.sleep(retry_delay)
                        continue
                    else:
                        raise RuntimeError('No front camera primitive found after all retries')
                        
            except (RuntimeError, ValueError) as e:
                # Re-raise these exceptions as they indicate fatal errors
                raise
            except Exception as e:
                self.get_logger().error(f'Error querying camera primitive: {e}')
                import traceback
                self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
                if attempt < max_retries - 1:
                    import time
                    time.sleep(retry_delay)
                    continue
                else:
                    raise RuntimeError(f'Failed to query camera primitive after all retries: {e}')
        
        # Should never reach here, but just in case
        raise RuntimeError('Failed to query camera primitive: unknown error')
    
    def rgb_image_callback(self, msg):
        """Callback for RGB image messages."""
        self.latest_rgb_image = msg
        
        # Save image to cache for debugging
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
            from PIL import Image as PILImage
            pil_image = PILImage.fromarray(cv_image)
            
            # Save with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_filename = self.cache_dir / f"rgb_{timestamp}_{self.image_counter:04d}.jpg"
            pil_image.save(image_filename, quality=95)
            self.image_counter += 1
            
            # Keep only last 20 images to avoid filling disk
            if self.image_counter > 20:
                # Remove oldest images
                image_files = sorted(self.cache_dir.glob("rgb_*.jpg"))
                if len(image_files) > 20:
                    for old_file in image_files[:-20]:
                        old_file.unlink()
                    self.image_counter = 20
            
            self.get_logger().debug(f'Saved RGB image to cache: {image_filename.name}')
        except Exception as e:
            self.get_logger().warn(f'Failed to save RGB image to cache: {e}')
    
    def camera_info_callback(self, msg):
        """Callback for camera info messages."""
        self.latest_camera_info = msg
    
    def query_callback(self, request, response):
        """Handle semantic map query request."""
        self.get_logger().info(f'Received query with types filter: {request.types}')
        
        # Check if we have latest image and camera info
        if not self.latest_rgb_image:
            self.get_logger().warn('No RGB image available yet')
            response.objects = []
            response.stamp = self.get_clock().now().to_msg()
            return response
        
        if not self.latest_camera_info:
            self.get_logger().warn('No camera info available yet')
            response.objects = []
            response.stamp = self.get_clock().now().to_msg()
            return response
        
        # Convert ROS image to OpenCV format
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_rgb_image, "rgb8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            response.objects = []
            response.stamp = self.get_clock().now().to_msg()
            return response
        
        # Convert image to base64 for API
        from PIL import Image as PILImage
        pil_image = PILImage.fromarray(cv_image)
        import io
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG')
        image_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
        
        # Prepare camera info for VLM
        camera_info_text = self._format_camera_info(self.latest_camera_info)
        
        # Call Qwen3-VL API to detect objects and estimate 3D coordinates
        objects = self._detect_objects_with_vlm(image_base64, camera_info_text, request.types)
        
        # Set response
        response.objects = objects
        response.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'Returning {len(objects)} objects detected by VLM')
        return response
    
    def _format_camera_info(self, camera_info):
        """Format camera info as text for VLM prompt."""
        # Extract key camera parameters
        fx = camera_info.k[0] if len(camera_info.k) > 0 else 0
        fy = camera_info.k[4] if len(camera_info.k) > 4 else 0
        cx = camera_info.k[2] if len(camera_info.k) > 2 else 0
        cy = camera_info.k[5] if len(camera_info.k) > 5 else 0
        width = camera_info.width
        height = camera_info.height
        
        return f"Camera parameters: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}, resolution={width}x{height}"
    
    def _detect_objects_with_vlm(self, image_base64, camera_info_text, type_filter):
        """Use Qwen3-VL to detect objects and estimate 3D coordinates in camera frame."""
        # Prepare prompt for VLM
        prompt = f"""Analyze this image and detect all visible objects. For each object, provide:
1. Object label/name
2. 2D bounding box coordinates (x_min, y_min, x_max, y_max) in pixels
3. Estimated 3D position in camera coordinate system (x, y, z in meters). 
   Camera coordinate system: x-right, y-down, z-forward (ROS convention).
   Use visual cues, object size, and perspective to estimate depth.
4. Estimated object size (width, height, depth in meters)

Camera info: {camera_info_text}

Return the results as a JSON array, where each object has:
{{
    "label": "object_name",
    "bbox_2d": [x_min, y_min, x_max, y_max],
    "position_cam": [x, y, z],
    "size": [width, height, depth]
}}

Only include objects that are clearly visible. Estimate coordinates as accurately as possible based on visual cues."""
        
        try:
            # Call Qwen3-VL API
            api_response = self.qwen_client.chat.completions.create(
                model=self.qwen_model,  # "qwen-vl-plus"
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}"
                                }
                            },
                            {
                                "type": "text",
                                "text": prompt
                            }
                        ]
                    }
                ],
                max_tokens=2000
            )
            
            # Parse response
            response_text = api_response.choices[0].message.content
            self.get_logger().info(f'VLM response: {response_text[:200]}...')
            
            # Extract JSON from response (may be wrapped in markdown code blocks)
            import re
            json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
            if json_match:
                objects_data = json.loads(json_match.group())
            else:
                # Try to parse entire response as JSON
                objects_data = json.loads(response_text)
            
            # Convert to Object messages
            objects = []
            for idx, obj_data in enumerate(objects_data):
                # Apply type filter if specified
                if type_filter and len(type_filter) > 0:
                    # Simple type matching based on label (can be improved)
                    obj_type = self._infer_object_type(obj_data.get('label', ''))
                    if obj_type not in type_filter:
                        continue
                
                obj = Object()
                obj.id = f"object_{idx:03d}"
                obj.label = obj_data.get('label', 'unknown')
                obj.registered_skills = []
                obj.registered_primitives = []
                obj.relations = []
                
                # Create frame mapping with camera frame coordinates
                frame_mapping = FrameMapping()
                frame_mapping.frame_id = 'head_front_camera_rgb_optical_frame'  # Camera frame
                
                # Set center position from VLM estimate (camera coordinates)
                pos_cam = obj_data.get('position_cam', [0.0, 0.0, 1.0])
                frame_mapping.center = Point3D()
                frame_mapping.center.x = float(pos_cam[0]) if len(pos_cam) > 0 else 0.0
                frame_mapping.center.y = float(pos_cam[1]) if len(pos_cam) > 1 else 0.0
                frame_mapping.center.z = float(pos_cam[2]) if len(pos_cam) > 2 else 1.0
                
                # Create bounding box from VLM size estimate
                size = obj_data.get('size', [0.1, 0.1, 0.1])
                bbox = BoundingBox()
                bbox.scale_x = float(size[0]) if len(size) > 0 else 0.1
                bbox.scale_y = float(size[1]) if len(size) > 1 else 0.1
                bbox.scale_z = float(size[2]) if len(size) > 2 else 0.1
                bbox.yaw = 0.0  # VLM doesn't estimate orientation yet
                frame_mapping.bbox = [bbox]
                
                obj.frame_mapping = [frame_mapping]
                objects.append(obj)
            
            return objects
            
        except Exception as e:
            self.get_logger().error(f'Error calling Qwen3-VL API: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
            return []
    
    def _infer_object_type(self, label):
        """Infer object type from label (simple heuristic)."""
        label_lower = label.lower()
        if 'table' in label_lower or 'desk' in label_lower:
            return 'table'
        elif 'box' in label_lower or 'container' in label_lower:
            return 'box'
        elif 'chair' in label_lower:
            return 'chair'
        elif 'robot' in label_lower:
            return 'robot'
        else:
            return 'object'


def main(args=None):
    rclpy.init(args=args)
    semantic_map_service = None
    try:
        semantic_map_service = SemanticMapService()
        rclpy.spin(semantic_map_service)
        semantic_map_service.destroy_node()
    except (RuntimeError, ValueError) as e:
        import sys
        print(f'FATAL: Failed to initialize semantic map service: {e}', file=sys.stderr)
        if semantic_map_service is not None:
            try:
                semantic_map_service.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass
        sys.exit(1)
    except Exception as e:
        import sys
        print(f'FATAL: Unexpected error: {e}', file=sys.stderr)
        if semantic_map_service is not None:
            try:
                semantic_map_service.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
