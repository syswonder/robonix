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
import uuid
import math
import threading
import time
from pathlib import Path
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration
from robonix_sdk.srv import QuerySemanticMap
from robonix_sdk.msg import Object, Relation, RelationType, FrameMapping, Point3D, BoundingBox
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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
            self.qwen_model = "qwen3-vl-plus"  # Model name
            self.get_logger().info(f'Qwen3-VL API client initialized with model: {self.qwen_model}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Qwen3-VL API client: {e}')
            raise
        
        # Create service client for querying primitives with QoS matching server
        # Match Rust server QoS configuration exactly:
        # - KeepLast(depth=10), Reliable, Volatile
        # - INFINITE deadline/lifespan, Automatic liveliness
        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        # Set additional QoS policies to match server (INFINITE = Duration(seconds=0))
        service_qos.deadline = Duration(seconds=0)  # INFINITE
        service_qos.lifespan = Duration(seconds=0)  # INFINITE
        service_qos.liveliness = LivelinessPolicy.AUTOMATIC
        service_qos.liveliness_lease_duration = Duration(seconds=0)  # INFINITE
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
        
        # Robot pose (will be queried from primitives)
        self.pose_topic = None
        self.latest_pose = None
        # Pose history for timestamp matching (stores up to 100 poses with timestamps)
        self.pose_history = []  # List of (stamp, pose) tuples, sorted by stamp
        self.pose_history_lock = threading.Lock()  # Thread-safe access to pose history
        
        # Semantic map memory: store objects with map coordinates for clustering
        # Key: object.id -> Object with map frame
        # Objects are never deleted once they have map coordinates
        self.semantic_map_memory = {}  # Maps object ID to Object
        self.memory_lock = threading.Lock()  # Thread-safe access to memory
        
        
        # Background update thread control
        self.update_thread = None
        self.update_thread_running = False
        self.update_interval = 5.0  # Update every 5 seconds
        
        # Query camera primitives - exit if failed
        self._query_camera_primitives()
        
        # Query robot pose primitive (prefer AMCL version)
        self._query_pose_primitive()
        
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
        
        # Subscribe to robot pose topic if available
        # prm::base.pose.amcl outputs PoseWithCovarianceStamped per spec
        # AMCL uses RELIABLE QoS with TRANSIENT_LOCAL durability
        if self.pose_topic:
            pose_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE  # Compatible with TRANSIENT_LOCAL publisher
            )
            self.pose_subscriber = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_cov_callback,
                pose_qos
            )
            self.get_logger().info(f'Subscribed to robot pose (PoseWithCovarianceStamped from prm::base.pose.amcl): {self.pose_topic} with RELIABLE QoS')
            
            # Wait a bit for first pose message to arrive (AMCL may take time to initialize)
            # But don't block - pose will be received asynchronously via callback
            import time
            wait_start = time.time()
            wait_timeout = 2.0  # Short wait to check if pose is immediately available
            while not self.latest_pose and (time.time() - wait_start) < wait_timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose:
                self.get_logger().info(f'Received initial pose: x={self.latest_pose.pose.position.x:.2f}, y={self.latest_pose.pose.position.y:.2f}')
            else:
                self.get_logger().info(f'No pose message received yet - will continue listening. Robot object will be created when pose becomes available.')
        else:
            self.get_logger().warn('No pose topic available - robot object will use default position (0,0,0)')
        
        # Create service (use default QoS - let ROS2 handle compatibility)
        self.service = self.create_service(
            QuerySemanticMap,
            '/demo_service/semantic_map/query',
            self.query_callback
        )
        
        # Start background update thread
        self.update_thread_running = True
        self.update_thread = threading.Thread(target=self._background_update_loop, daemon=True)
        self.update_thread.start()
        self.get_logger().info('Started background semantic map update thread')
        
        self.get_logger().info('Semantic map service started')
        self.get_logger().info('  Service: /demo_service/semantic_map/query')
        self.get_logger().info('  Using Qwen3-VL for object detection (background updates)')
    
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
    
    def _query_pose_primitive(self):
        """Query robot pose primitive from OS with retry logic."""
        max_retries = 5
        retry_delay = 2.0  # seconds
        primitive_name = 'prm::base.pose.amcl'
        
        for attempt in range(max_retries):
            self.get_logger().info(f'Querying {primitive_name}... (attempt {attempt + 1}/{max_retries})')
            try:
                wait_timeout = 10.0 if attempt < 2 else 5.0
                if not self.query_primitive_client.wait_for_service(timeout_sec=wait_timeout):
                    self.get_logger().warn(f'  query_primitive service not available (attempt {attempt + 1}/{max_retries})')
                    if attempt < max_retries - 1:
                        import time
                        time.sleep(retry_delay)
                        continue
                    else:
                        break
                
                # Create request
                request = QueryPrimitive.Request()
                request.name = primitive_name
                request.filter = '{}'
                
                # Call service
                future = self.query_primitive_client.call_async(request)
                
                # Wait for response
                import time
                start_time = time.time()
                timeout_sec = 3.0
                while not future.done() and (time.time() - start_time) < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.01)
                
                if not future.done():
                    elapsed = time.time() - start_time
                    self.get_logger().warn(f'  Service call timeout after {elapsed:.1f}s (attempt {attempt + 1}/{max_retries})')
                    try:
                        future.cancel()
                    except:
                        pass
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        break
                
                # Get response
                response = future.result()
                
                if response and response.instances:
                    instance = response.instances[0]
                    if isinstance(instance.output_schema, str):
                        output_schema = json.loads(instance.output_schema)
                    else:
                        output_schema = instance.output_schema
                    
                    if 'pose' in output_schema:
                        self.pose_topic = output_schema['pose']
                        self.get_logger().info(f'  Found pose topic: {self.pose_topic} (from {primitive_name})')
                        return  # Success
                    else:
                        self.get_logger().warn(f'  {primitive_name} found but no "pose" in output_schema')
                        break
                else:
                    self.get_logger().warn(f'  No {primitive_name} primitive found (attempt {attempt + 1}/{max_retries})')
                    if attempt < max_retries - 1:
                        import time
                        time.sleep(retry_delay)
                        continue
                    else:
                        break
                        
            except Exception as e:
                self.get_logger().warn(f'Error querying {primitive_name}: {e}')
                if attempt < max_retries - 1:
                    import time
                    time.sleep(retry_delay)
                    continue
                else:
                    break
        
        # If we reach here, pose primitive was not found
        self.get_logger().warn('Failed to query pose primitive after all retries, continuing without pose')
        return
    
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
    
    def pose_cov_callback(self, msg):
        """Callback for PoseWithCovarianceStamped messages."""
        # Convert PoseWithCovarianceStamped to PoseStamped format for internal use
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        was_none = self.latest_pose is None
        self.latest_pose = pose_stamped
        
        # Add to pose history for timestamp matching
        with self.pose_history_lock:
            # Convert stamp to seconds (for comparison)
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
            self.pose_history.append((stamp_sec, pose_stamped))
            # Keep only last 100 poses to avoid memory growth
            if len(self.pose_history) > 100:
                self.pose_history.pop(0)
        
        if was_none:
            self.get_logger().info(f'Received first pose update (PoseWithCovarianceStamped): x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f} - robot object will now be available')
        else:
            self.get_logger().debug(f'Received pose update (PoseWithCovarianceStamped): x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f}')
    
    def query_callback(self, request, response):
        """Handle semantic map query request - returns latest memory state immediately."""
        self.get_logger().info(f'Received query with types filter: {request.types}')
        
        # Get current pose snapshot (thread-safe, no lock needed for read)
        snapshot_pose = self.latest_pose  # May be None if no pose available
        
        # Quickly copy memory data (minimize lock time)
        memory_copy = {}
        with self.memory_lock:
            # Fast shallow copy of memory dict
            memory_copy = dict(self.semantic_map_memory)
        
        # Process outside the lock (clustering and filtering can take time)
        memory_objects = self._get_all_objects_from_memory_unlocked(memory_copy, request.types, snapshot_pose=snapshot_pose)
        
        # Ensure no duplicates by ID (safety check)
        seen_ids = set()
        unique_objects = []
        for obj in memory_objects:
            if obj.id not in seen_ids:
                unique_objects.append(obj)
                seen_ids.add(obj.id)
        
        # Use current time as stamp (since we're returning latest state)
        response.objects = unique_objects
        response.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f'Returning {len(unique_objects)} objects from memory (immediate response)')
        return response
    
    def _background_update_loop(self):
        """Background thread that continuously updates semantic map by calling VLM."""
        self.get_logger().info('Background update loop started')
        
        while self.update_thread_running:
            try:
                # Wait for image and camera info
                if not self.latest_rgb_image or not self.latest_camera_info:
                    self.get_logger().debug('Waiting for image/camera_info...')
                    time.sleep(1.0)
                    continue
                
                # Create snapshot: capture current image, camera info, and pose at the same time
                snapshot_image = self.latest_rgb_image
                snapshot_camera_info = self.latest_camera_info
                snapshot_stamp = snapshot_image.header.stamp
                
                # Find pose that matches image timestamp (CRITICAL: must use pose from image capture time)
                # Convert image stamp to seconds for comparison
                image_stamp_sec = float(snapshot_stamp.sec) + float(snapshot_stamp.nanosec) / 1e9
                snapshot_pose = self._find_pose_by_timestamp(image_stamp_sec)
                
                if snapshot_pose:
                    # Extract pose timestamp for logging
                    pose_stamp_sec = float(snapshot_pose.header.stamp.sec) + float(snapshot_pose.header.stamp.nanosec) / 1e9
                    time_diff = abs(pose_stamp_sec - image_stamp_sec)
                    self.get_logger().info(
                        f'Using synchronized pose: image_t={image_stamp_sec:.3f}, pose_t={pose_stamp_sec:.3f}, '
                        f'diff={time_diff*1000:.1f}ms, robot_pos=[{snapshot_pose.pose.position.x:.2f}, '
                        f'{snapshot_pose.pose.position.y:.2f}, {snapshot_pose.pose.position.z:.2f}]'
                    )
                else:
                    # CRITICAL: Without matching pose, we cannot accurately compute map coordinates
                    # Skip this update cycle to ensure synchronization
                    self.get_logger().warn(
                        f'No matching pose found for image timestamp {image_stamp_sec:.3f} '
                        f'(tolerance: 0.5s). Skipping this update to ensure pose-image synchronization. '
                        f'Latest pose available: {self.latest_pose is not None}'
                    )
                    time.sleep(self.update_interval)
                    continue  # Skip this update cycle
                
                # Convert ROS image to OpenCV format
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(snapshot_image, "rgb8")
                except Exception as e:
                    self.get_logger().error(f'Failed to convert image: {e}')
                    time.sleep(self.update_interval)
                    continue
                
                # Convert image to base64 for API
                from PIL import Image as PILImage
                pil_image = PILImage.fromarray(cv_image)
                import io
                buffer = io.BytesIO()
                pil_image.save(buffer, format='JPEG')
                image_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
                
                # Prepare camera info for VLM
                camera_info_text = self._format_camera_info(snapshot_camera_info)
                
                # Call Qwen3-VL API to detect objects and get distance + direction vector
                self.get_logger().info('Calling VLM for object detection with distance and direction...')
                detected_objects = self._detect_objects_with_vlm(
                    image_base64, 
                    camera_info_text, 
                    snapshot_camera_info, 
                    []
                )
                
                # Process detected objects: convert to map frame using snapshot pose and merge with memory
                # This function already updates memory internally, but we need to do it thread-safely
                with self.memory_lock:
                    processed_objects = self._process_detected_objects(detected_objects, snapshot_pose=snapshot_pose)
                    
                    # Log update
                    memory_count = len(self.semantic_map_memory)
                    self.get_logger().info(f'Updated semantic map: {len(processed_objects)} new objects processed, {memory_count} total objects in memory')
                
                # Wait before next update
                time.sleep(self.update_interval)
                
            except Exception as e:
                self.get_logger().error(f'Error in background update loop: {e}')
                import traceback
                self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
                time.sleep(self.update_interval)  # Wait before retrying
        
        self.get_logger().info('Background update loop stopped')
    
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
    
    def _detect_objects_with_vlm(self, image_base64, camera_info_text, camera_info, type_filter):
        """Use Qwen3-VL to detect objects and estimate distance + direction vector.
        
        Args:
            image_base64: Base64-encoded image
            camera_info_text: Formatted camera info text for VLM prompt
            camera_info: CameraInfo message (for coordinate calculation)
            type_filter: List of object types to filter
        """
        # Prepare prompt for VLM - ask for distance and direction vector
        prompt = f"""Analyze this image and detect all visible objects. For each object, provide:
1. Object label/name
2. 2D bounding box coordinates (x_min, y_min, x_max, y_max) in pixels
3. Estimated distance from camera in meters (straight-line 3D distance to object center)
4. Estimated object size (width, height, depth in meters)

Camera info: {camera_info_text}

IMPORTANT: Only detect actual objects/items, NOT structural elements or surfaces. 
DO NOT include:
- Walls, walls, wall surfaces
- Floor, floor surfaces, ground
- Ceiling, ceiling surfaces
- Doors, windows (unless they are the focus as movable objects)
- Any architectural elements or room structures

Only include discrete, movable, or identifiable objects such as:
- Furniture (tables, chairs, sofas, cabinets, etc.)
- Appliances (refrigerators, ovens, microwaves, etc.)
- Electronics (TVs, computers, monitors, etc.)
- Containers (boxes, bags, bottles, etc.)
- People, animals
- Other tangible items that can be interacted with

CRITICAL for distance estimation:
- The "distance" should be the actual 3D straight-line distance from the camera to the object center
- Consider the object's position in the image: objects on the left/right sides are further away in 3D space
- Use visual cues: object size, perspective, shadows, and relative positions to estimate accurate 3D distance
- For objects at the image edges, the distance should account for the horizontal offset (objects on the left/right are further in 3D)

Return the results as a JSON array, where each object has:
{{
    "label": "object_name",
    "bbox_2d": [x_min, y_min, x_max, y_max],
    "distance": <actual_3d_distance_in_meters>,
    "size": [width, height, depth]
}}

Only include objects that are clearly visible. Estimate distance as accurately as possible based on visual cues, object size, and perspective."""
        
        try:
            # Call Qwen3-VL API
            api_response = self.qwen_client.chat.completions.create(
                model=self.qwen_model,  # "qwen3-vl-plus"
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
            
            # Convert to Object messages with camera frame coordinates
            objects = []
            # Keywords to filter out non-object concepts
            excluded_keywords = ['wall', 'floor', 'ceiling', 'ground', 'surface', 'door', 'window']
            
            for idx, obj_data in enumerate(objects_data):
                label = obj_data.get('label', '').lower()
                
                # Filter out structural elements and surfaces
                if any(keyword in label for keyword in excluded_keywords):
                    self.get_logger().debug(f'Filtered out non-object: {obj_data.get("label", "unknown")}')
                    continue
                
                # Apply type filter if specified
                if type_filter and len(type_filter) > 0:
                    # Simple type matching based on label (can be improved)
                    obj_type = self._infer_object_type(obj_data.get('label', ''))
                    if obj_type not in type_filter:
                        continue
                
                obj = Object()
                # Generate short UUID for object ID
                obj.id = self._generate_short_uuid()
                obj.label = obj_data.get('label', 'unknown')
                obj.registered_skills = []
                obj.registered_primitives = []
                obj.relations = []
                
                # Get bbox and distance from VLM
                bbox_2d = obj_data.get('bbox_2d', [0, 0, 640, 480])
                distance = float(obj_data.get('distance', 1.0))
                
                # Calculate camera frame coordinates from distance and bbox
                # Camera frame: x-right, y-down, z-forward (ROS convention)
                # bbox center position determines direction vector, VLM distance determines magnitude
                pos_cam = self._calculate_camera_coords_from_distance(
                    distance, bbox_2d, camera_info
                )
                
                # Create frame mapping with camera frame coordinates
                frame_mapping = FrameMapping()
                frame_mapping.frame_id = 'head_front_camera_rgb_optical_frame'  # Camera frame
                
                # Set center position in camera frame
                frame_mapping.center = Point3D()
                frame_mapping.center.x = pos_cam[0]
                frame_mapping.center.y = pos_cam[1]
                frame_mapping.center.z = pos_cam[2]
                
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
    
    def _generate_short_uuid(self):
        """Generate a short UUID (8 characters)."""
        return uuid.uuid4().hex[:8]
    
    def _calculate_camera_coords_from_distance(self, distance, bbox_2d, camera_info):
        """Calculate camera frame coordinates from distance and 2D bbox.
        
        Args:
            distance: Actual 3D straight-line distance from camera to object center (in meters, from VLM)
            bbox_2d: 2D bounding box [x_min, y_min, x_max, y_max] in pixels
            camera_info: Camera info message with intrinsics
        
        Returns:
            [camera_x, camera_y, camera_z] in camera frame (meters)
        
        Note:
            The bbox center position determines the direction vector.
            The VLM-estimated distance is used to scale the direction vector to the correct magnitude.
            Camera frame: x-right, y-down, z-forward (ROS convention)
        """
        # Get camera center from bbox
        x_min, y_min, x_max, y_max = bbox_2d
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        
        # Get camera intrinsics
        fx = camera_info.k[0] if len(camera_info.k) > 0 else 1.0
        fy = camera_info.k[4] if len(camera_info.k) > 4 else 1.0
        cx = camera_info.k[2] if len(camera_info.k) > 2 else camera_info.width / 2.0
        cy = camera_info.k[5] if len(camera_info.k) > 5 else camera_info.height / 2.0
        
        # Convert pixel coordinates to normalized coordinates (direction vector)
        # (u - cx) / fx = X / Z, (v - cy) / fy = Y / Z
        normalized_x = (center_x - cx) / fx
        normalized_y = (center_y - cy) / fy
        
        # The normalized coordinates represent a direction vector (X/Z, Y/Z, 1)
        # We need to scale this vector to have the given distance magnitude
        # Direction vector: [normalized_x, normalized_y, 1]
        # Magnitude of direction vector: sqrt(normalized_x^2 + normalized_y^2 + 1)
        direction_magnitude = math.sqrt(normalized_x**2 + normalized_y**2 + 1.0)
        
        # Scale the direction vector to match the actual distance
        # camera_coords = (direction_vector / direction_magnitude) * distance
        scale_factor = distance / direction_magnitude
        
        # Camera frame: x-right, y-down, z-forward (ROS convention)
        camera_x = normalized_x * scale_factor
        camera_y = normalized_y * scale_factor
        camera_z = 1.0 * scale_factor  # Forward direction component
        
        return [camera_x, camera_y, camera_z]
    
    def _transform_camera_to_map(self, camera_pos, robot_pose_stamped=None):
        """Transform coordinates from camera frame to map frame.
        
        Args:
            camera_pos: [x, y, z] in camera frame
            robot_pose_stamped: PoseStamped message for robot pose (if None, uses self.latest_pose)
        """
        # Use provided pose or fall back to latest_pose
        pose_to_use = robot_pose_stamped if robot_pose_stamped is not None else self.latest_pose
        if not pose_to_use:
            return None
        
        # Get robot pose in map frame
        robot_pose = pose_to_use.pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_z = robot_pose.position.z
        
        # Get robot orientation (quaternion)
        qx = robot_pose.orientation.x
        qy = robot_pose.orientation.y
        qz = robot_pose.orientation.z
        qw = robot_pose.orientation.w
        
        # Convert quaternion to yaw (rotation around z-axis)
        # Simplified: assuming robot mainly rotates around z-axis
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Camera offset from robot base (assuming camera is mounted at front)
        # These values should ideally come from TF, but we use defaults
        camera_offset_x = 0.3  # 30cm forward from robot center
        camera_offset_y = 0.0  # No lateral offset
        camera_offset_z = 0.2  # 20cm above robot center
        
        # Transform camera position to robot base frame
        # Camera frame: x-right, y-down, z-forward
        # Robot base frame: x-forward, y-left, z-up
        # So we need to transform: camera (x,y,z) -> base (x,y,z)
        # Camera z (forward) -> Base x (forward)
        # Camera -x (left) -> Base y (left)
        # Camera -y (up) -> Base z (up)
        base_x = camera_pos[2] + camera_offset_x  # camera z -> base x
        base_y = -camera_pos[0] + camera_offset_y  # -camera x -> base y
        base_z = -camera_pos[1] + camera_offset_z  # -camera y -> base z
        
        # Transform from robot base frame to map frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        map_x = robot_x + cos_yaw * base_x - sin_yaw * base_y
        map_y = robot_y + sin_yaw * base_x + cos_yaw * base_y
        map_z = robot_z + base_z
        
        return [map_x, map_y, map_z]
    
    def _cluster_same_label_objects(self, objects_with_map_pos):
        """Cluster objects with the same label that are close to each other.
        
        Args:
            objects_with_map_pos: List of tuples (obj, map_pos) where obj has map coordinates
        
        Returns:
            List of clustered objects with updated positions (averaged for clusters)
        """
        CLUSTER_DISTANCE_THRESHOLD = 1.5  # 1.5m - objects within this distance are clustered
        
        if not objects_with_map_pos:
            return []
        
        # Group objects by normalized label (to handle variations like "monitor" vs "computer monitor")
        objects_by_label = {}
        for obj, map_pos in objects_with_map_pos:
            normalized_label = self._normalize_label(obj.label)
            if normalized_label not in objects_by_label:
                objects_by_label[normalized_label] = []
            objects_by_label[normalized_label].append((obj, map_pos))
        
        clustered_objects = []
        
        # For each label, cluster nearby objects
        for label, obj_list in objects_by_label.items():
            if len(obj_list) == 1:
                # Single object, no clustering needed
                clustered_objects.append(obj_list[0])
                continue
            
            # Cluster objects by proximity
            clusters = []
            for obj, map_pos in obj_list:
                # Find existing cluster within threshold
                assigned = False
                for cluster in clusters:
                    # Check distance to cluster center
                    cluster_center = cluster['center']
                    distance = math.sqrt(
                        (map_pos[0] - cluster_center[0]) ** 2 +
                        (map_pos[1] - cluster_center[1]) ** 2 +
                        (map_pos[2] - cluster_center[2]) ** 2
                    )
                    if distance < CLUSTER_DISTANCE_THRESHOLD:
                        # Add to existing cluster
                        cluster['objects'].append((obj, map_pos))
                        # Update cluster center (average of all positions)
                        n = len(cluster['objects'])
                        cluster['center'] = [
                            sum(p[1][0] for p in cluster['objects']) / n,
                            sum(p[1][1] for p in cluster['objects']) / n,
                            sum(p[1][2] for p in cluster['objects']) / n
                        ]
                        assigned = True
                        break
                
                if not assigned:
                    # Create new cluster
                    clusters.append({
                        'objects': [(obj, map_pos)],
                        'center': map_pos
                    })
            
            # Create clustered objects with averaged positions
            for cluster in clusters:
                if len(cluster['objects']) == 1:
                    # Single object in cluster, use as-is
                    clustered_objects.append(cluster['objects'][0])
                else:
                    # Multiple objects, use the first one with averaged position
                    obj, _ = cluster['objects'][0]
                    avg_pos = cluster['center']
                    
                    # Update object's map frame mapping with averaged position
                    for fm in obj.frame_mapping:
                        if fm.frame_id == 'map':
                            fm.center.x = avg_pos[0]
                            fm.center.y = avg_pos[1]
                            fm.center.z = avg_pos[2]
                            break
                    
                    self.get_logger().info(f'Clustered {len(cluster["objects"])} objects with label "{obj.label}" at average position [{avg_pos[0]:.2f}, {avg_pos[1]:.2f}, {avg_pos[2]:.2f}]')
                    clustered_objects.append((obj, avg_pos))
        
        return clustered_objects
    
    def _process_detected_objects(self, detected_objects, snapshot_pose=None):
        """Process detected objects: convert to map frame, cluster same-label objects, and merge with memory.
        
        Args:
            detected_objects: List of detected objects
            snapshot_pose: PoseStamped message for snapshot pose (MUST be from image capture time, not None)
        
        Note:
            snapshot_pose is REQUIRED for accurate coordinate transformation.
            Objects without a valid synchronized pose will NOT be stored in memory.
        """
        # CRITICAL: snapshot_pose must be provided (from image capture time)
        if not snapshot_pose:
            self.get_logger().error(
                'snapshot_pose is None - cannot process objects without synchronized pose. '
                'Skipping object processing to ensure accuracy.'
            )
            return []  # Return empty list - no objects processed without pose
        
        # First pass: convert all objects to map frame
        objects_with_map_pos = []
        objects_without_map = []
        
        for obj in detected_objects:
            # Get camera frame position
            if not obj.frame_mapping or len(obj.frame_mapping) == 0:
                continue
            
            camera_frame = obj.frame_mapping[0]
            camera_pos = [
                camera_frame.center.x,
                camera_frame.center.y,
                camera_frame.center.z
            ]
            
            # Transform to map frame using snapshot pose (synchronized with image)
            # snapshot_pose is guaranteed to be non-None here (checked above)
            # Use snapshot_pose directly (it's the synchronized pose from image capture time)
            # Transform to map frame using snapshot pose (synchronized with image capture time)
            map_pos = self._transform_camera_to_map(camera_pos, robot_pose_stamped=snapshot_pose)
            if map_pos:
                # Create map frame mapping
                map_frame = FrameMapping()
                map_frame.frame_id = 'map'
                map_frame.center = Point3D()
                map_frame.center.x = map_pos[0]
                map_frame.center.y = map_pos[1]
                map_frame.center.z = map_pos[2]
                map_frame.bbox = camera_frame.bbox.copy() if camera_frame.bbox else []
                map_frame.texture = camera_frame.texture.copy() if camera_frame.texture else []
                
                # Add map frame mapping
                obj.frame_mapping.append(map_frame)
                objects_with_map_pos.append((obj, map_pos))
            else:
                # Transform failed (shouldn't happen if snapshot_pose is valid, but handle gracefully)
                self.get_logger().warn(f'Transform failed for object {obj.label} (id={obj.id}), skipping')
                objects_without_map.append(obj)
        
        # Cluster objects with same label that are close to each other
        clustered_objects = self._cluster_same_label_objects(objects_with_map_pos)
        
        # Process clustered objects: merge with memory or add to memory
        processed_objects = []
        for obj, map_pos in clustered_objects:
            # Try to merge with existing objects in memory
            merged = self._merge_with_memory(obj, map_pos)
            if not merged:
                # New object, add to memory
                self._add_to_memory(obj, map_pos)
            
            processed_objects.append(obj)
        
        # Add objects without map coordinates (for display only, not stored in memory)
        processed_objects.extend(objects_without_map)
        
        # Return only newly processed objects (not from memory)
        # Memory objects will be retrieved separately in query_callback
        return processed_objects
    
    def _normalize_label(self, label):
        """Normalize label to handle variations (e.g., 'computer monitor' -> 'monitor')."""
        label_lower = label.lower()
        # Handle common variations
        if 'monitor' in label_lower or 'computer monitor' in label_lower:
            return 'monitor'
        if 'office chair' in label_lower or 'chair' in label_lower:
            return 'chair'
        if 'desk' in label_lower or 'table' in label_lower:
            return 'desk'
        if 'cabinet' in label_lower or 'shelf' in label_lower:
            return 'cabinet'
        return label_lower
    
    def _merge_with_memory(self, obj, map_pos):
        """Try to merge object with existing objects in memory based on label and position.
        Uses clustering algorithm: objects with same/similar label within 1.5m are considered the same.
        Objects are never deleted once they have map coordinates."""
        MERGE_DISTANCE_THRESHOLD = 1.5  # 1.5m - objects within this distance are considered the same
        
        obj_label_normalized = self._normalize_label(obj.label)
        
        # Find closest matching object with same/similar label
        best_match = None
        best_distance = float('inf')
        
        for existing_id, existing_obj in self.semantic_map_memory.items():
            existing_label_normalized = self._normalize_label(existing_obj.label)
            
            # Check if labels match (after normalization)
            if existing_label_normalized != obj_label_normalized:
                continue
            
            # Check if existing object has map coordinates
            existing_map_pos = None
            for fm in existing_obj.frame_mapping:
                if fm.frame_id == 'map':
                    existing_map_pos = [fm.center.x, fm.center.y, fm.center.z]
                    break
            
            if not existing_map_pos:
                continue
            
            # Calculate distance
            distance = math.sqrt(
                (map_pos[0] - existing_map_pos[0]) ** 2 +
                (map_pos[1] - existing_map_pos[1]) ** 2 +
                (map_pos[2] - existing_map_pos[2]) ** 2
            )
            
            if distance < MERGE_DISTANCE_THRESHOLD and distance < best_distance:
                best_match = existing_obj
                best_distance = distance
        
        if best_match:
            # Merge: update existing object with new information using weighted average
            # Give more weight to existing position (more stable)
            weight_new = 0.3
            weight_existing = 0.7
            
            # Get existing position
            existing_map_pos = None
            for fm in best_match.frame_mapping:
                if fm.frame_id == 'map':
                    existing_map_pos = [fm.center.x, fm.center.y, fm.center.z]
                    break
            
            if existing_map_pos:
                merged_x = weight_existing * existing_map_pos[0] + weight_new * map_pos[0]
                merged_y = weight_existing * existing_map_pos[1] + weight_new * map_pos[1]
                merged_z = weight_existing * existing_map_pos[2] + weight_new * map_pos[2]
                
                # Update existing object's map frame
                for fm in best_match.frame_mapping:
                    if fm.frame_id == 'map':
                        fm.center.x = merged_x
                        fm.center.y = merged_y
                        fm.center.z = merged_z
                        break
                
                # Update new object's ID to match existing (so it won't be added as duplicate)
                obj.id = best_match.id
                
                self.get_logger().info(f'Merged object "{obj.label}" with existing object (id={best_match.id}) at distance {best_distance:.2f}m')
                return True
        
        return False
    
    def _add_to_memory(self, obj, map_pos):
        """Add object to memory. Objects are never deleted once they have map coordinates."""
        # Use object ID as key - ensures each object is stored only once
        # Objects are never deleted, only updated through merging
        self.semantic_map_memory[obj.id] = obj
        self.get_logger().info(f'Added object "{obj.label}" (id={obj.id}) to memory at map position [{map_pos[0]:.2f}, {map_pos[1]:.2f}, {map_pos[2]:.2f}]')
    
    def _get_all_objects_from_memory_unlocked(self, memory_dict, type_filter, snapshot_pose=None):
        """Get all objects from memory dict (unlocked version for query_callback).
        All objects with map coordinates are returned - objects are never deleted.
        Objects with same/similar label within 1.5m are clustered before returning.
        
        Args:
            memory_dict: Dictionary of objects (copy of self.semantic_map_memory)
            type_filter: List of object types to filter
            snapshot_pose: PoseStamped message for snapshot pose (if None, uses self.latest_pose for robot)
        """
        objects_with_map_pos = []
        seen_ids = set()  # Track IDs to avoid duplicates
        
        # Get objects from memory dict (not self.semantic_map_memory)
        for obj_id, obj in memory_dict.items():
            # Skip if already seen (shouldn't happen, but safety check)
            if obj_id in seen_ids:
                continue
            
            # Only return objects that have map coordinates
            map_pos = None
            for fm in obj.frame_mapping:
                if fm.frame_id == 'map':
                    map_pos = [fm.center.x, fm.center.y, fm.center.z]
                    break
            
            if not map_pos:
                continue
            
            # Apply type filter if specified
            if type_filter and len(type_filter) > 0:
                obj_type = self._infer_object_type(obj.label)
                if obj_type not in type_filter:
                    continue
            
            objects_with_map_pos.append((obj, map_pos))
            seen_ids.add(obj_id)
        
        # Cluster objects with same/similar label that are close to each other
        clustered_objects = self._cluster_same_label_objects(objects_with_map_pos)
        
        # Extract objects from clustered results
        objects = []
        for item in clustered_objects:
            if isinstance(item, tuple):
                obj, _ = item
            else:
                obj = item
            objects.append(obj)
        
        # Add robot object if pose is available (only once, check ID)
        # Use snapshot pose to ensure robot position matches the image snapshot
        robot_object = self._create_robot_object(snapshot_pose=snapshot_pose)
        if robot_object and robot_object.id not in seen_ids:
            # Apply type filter to robot if specified
            if not type_filter or len(type_filter) == 0 or 'robot' in type_filter:
                objects.append(robot_object)
                seen_ids.add(robot_object.id)
        
        return objects
    
    def _create_robot_object(self, snapshot_pose=None):
        """Create robot object for semantic map. Only creates if pose is available.
        
        Args:
            snapshot_pose: PoseStamped message for snapshot pose (if None, uses self.latest_pose)
        """
        # Use snapshot pose or fall back to latest_pose
        pose_to_use = snapshot_pose if snapshot_pose is not None else self.latest_pose
        if not pose_to_use:
            return None
        
        obj = Object()
        obj.id = "robot_self"  # Fixed ID for robot
        obj.label = "robot"
        obj.registered_skills = []
        obj.registered_primitives = []
        obj.relations = []
        
        # Create map frame mapping
        map_frame = FrameMapping()
        map_frame.frame_id = 'map'
        map_frame.center = Point3D()
        
        robot_pose = pose_to_use.pose
        map_frame.center.x = robot_pose.position.x
        map_frame.center.y = robot_pose.position.y
        map_frame.center.z = robot_pose.position.z
        
        # Extract yaw from quaternion (rotation around z-axis)
        # Standard formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        qx = robot_pose.orientation.x
        qy = robot_pose.orientation.y
        qz = robot_pose.orientation.z
        qw = robot_pose.orientation.w
        # Yaw is rotation around z-axis: from +X axis, counter-clockwise is positive
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(f'Created robot object with pose: x={map_frame.center.x:.2f}, y={map_frame.center.y:.2f}, z={map_frame.center.z:.2f}, yaw={math.degrees(yaw):.1f}°')
        
        # Create bounding box for robot (approximate size)
        bbox = BoundingBox()
        bbox.scale_x = 0.5  # Robot width
        bbox.scale_y = 0.5  # Robot depth
        bbox.scale_z = 1.0  # Robot height
        bbox.yaw = yaw  # Set yaw from pose orientation
        map_frame.bbox = [bbox]
        map_frame.texture = []  # Initialize empty texture list
        
        obj.frame_mapping = [map_frame]
        
        return obj
    
    def _find_pose_by_timestamp(self, target_stamp_sec):
        """Find pose closest to target timestamp from pose history.
        
        Args:
            target_stamp_sec: Target timestamp in seconds (float)
        
        Returns:
            PoseStamped message closest to target timestamp, or None if no pose found
        
        Note:
            This ensures pose-image synchronization for accurate coordinate transformation.
            Only returns poses within tolerance to guarantee accuracy.
        """
        with self.pose_history_lock:
            if not self.pose_history:
                return None
            
            # Find closest pose by timestamp (within 0.5 second tolerance)
            # Use tighter tolerance for better synchronization
            best_pose = None
            best_diff = float('inf')
            TOLERANCE_SEC = 0.5  # 500ms tolerance - tight enough for accurate sync
            
            for stamp_sec, pose in self.pose_history:
                diff = abs(stamp_sec - target_stamp_sec)
                if diff < best_diff and diff <= TOLERANCE_SEC:
                    best_diff = diff
                    best_pose = pose
            
            if best_pose:
                self.get_logger().debug(
                    f'Found matching pose: target_t={target_stamp_sec:.3f}, '
                    f'pose_t={best_diff + target_stamp_sec:.3f}, diff={best_diff*1000:.1f}ms'
                )
            else:
                self.get_logger().debug(
                    f'No pose within tolerance: target_t={target_stamp_sec:.3f}, '
                    f'history_size={len(self.pose_history)}, tolerance={TOLERANCE_SEC*1000:.0f}ms'
                )
            
            return best_pose
    
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
