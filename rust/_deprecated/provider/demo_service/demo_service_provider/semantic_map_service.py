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
import math
import threading
import time
import yaml
import numpy as np
from pathlib import Path
from datetime import datetime
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from robonix_sdk.srv import QuerySemanticMap
from robonix_sdk.msg import (
    Object,
    FrameMapping,
    Point3D,
    BoundingBox,
)
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from cv_bridge import CvBridge
from dotenv import load_dotenv
from openai import OpenAI
from robonix_sdk.client import RobonixClient


class SemanticMapService(Node):
    """Implements semantic_map service using front camera and qwen3-vl VLM."""

    def __init__(self, config_filename=None):
        super().__init__("demo_semantic_map_service")
        self.config_filename = config_filename

        current_file = Path(__file__).resolve()
        package_root = current_file.parent
        while package_root != package_root.parent:
            if (package_root / "setup.py").exists():
                break
            package_root = package_root.parent
        if not (package_root / "setup.py").exists():
            package_root = current_file.parent.parent

        load_dotenv(package_root / ".env")
        # Same key as task_plan: DASHSCOPE_API_KEY (or legacy QWEN3_VL_API_KEY)
        self.qwen_api_key = os.getenv("DASHSCOPE_API_KEY") or os.getenv("QWEN3_VL_API_KEY")
        if not self.qwen_api_key:
            self.get_logger().error(
                "DASHSCOPE_API_KEY not found in .env file. "
                "Please configure DASHSCOPE_API_KEY in .env file."
            )
            raise ValueError("DASHSCOPE_API_KEY not found in .env file.")

        try:
            self.qwen_client = OpenAI(
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
                api_key=self.qwen_api_key,
            )
            self.qwen_model = "qwen3-vl-plus"
            self.get_logger().info(
                f"Qwen3-VL API client initialized with model: {self.qwen_model}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Qwen3-VL API client: {e}")
            raise

        # Initialize Robonix client helper
        self.robonix = RobonixClient(self, self.get_logger())
        self.query_primitive_client = self.robonix.create_query_primitive_client()

        self.cv_bridge = CvBridge()
        self.cache_dir = package_root / "cache"
        try:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Cache directory: {self.cache_dir}")
        except Exception as e:
            self.get_logger().error(
                f"Failed to create cache directory {self.cache_dir}: {e}"
            )
            raise

        self.rgb_image_topic = None
        self.camera_info_topic = None
        self.latest_rgb_image = None
        self.latest_camera_info = None
        self.image_counter = 0

        self.pose_topic = None
        self.latest_pose = None
        self.pose_history = []
        self.pose_history_lock = threading.Lock()

        self.semantic_map_memory = {}
        self.memory_lock = threading.Lock()

        self.label_counters = {}
        self.id_generation_lock = threading.Lock()

        self.update_thread = None
        self.update_thread_running = False
        # VLM API (Aliyun DashScope) is charged per request; default 30s to reduce cost
        default_interval = 15.0
        try:
            self.update_interval = float(
                os.getenv("SEMANTIC_MAP_UPDATE_INTERVAL_SEC", str(default_interval))
            )
            if self.update_interval < 5.0:
                self.update_interval = 5.0
        except (TypeError, ValueError):
            self.update_interval = default_interval
        self.get_logger().info(
            f"Semantic map VLM update interval: {self.update_interval}s "
            "(set SEMANTIC_MAP_UPDATE_INTERVAL_SEC to change; lower = more API cost)"
        )

        # Skip VLM when pose/image unchanged to save tokens
        self.last_vlm_pose = None  # (x, y, z, yaw) when last VLM was called
        self.last_vlm_image_lowres = None  # 32x32 grayscale for similarity
        self._last_vlm_lock = threading.Lock()

        try:
            self.pose_position_threshold = float(
                os.getenv("SEMANTIC_MAP_POSE_POSITION_THRESHOLD_M", "0.15"))
        except (TypeError, ValueError):
            self.pose_position_threshold = 0.15
        try:
            yaw_deg = float(
                os.getenv("SEMANTIC_MAP_POSE_YAW_THRESHOLD_DEG", "10.0"))
            self.pose_yaw_threshold_rad = math.radians(yaw_deg)
        except (TypeError, ValueError):
            self.pose_yaw_threshold_rad = math.radians(10.0)
        try:
            self.image_mse_threshold = float(
                os.getenv("SEMANTIC_MAP_IMAGE_MSE_THRESHOLD", "80.0"))
        except (TypeError, ValueError):
            self.image_mse_threshold = 80.0

        self.get_logger().info(
            f"VLM skip: pose_threshold={self.pose_position_threshold}m / "
            f"{math.degrees(self.pose_yaw_threshold_rad):.1f}°, "
            f"image_mse_threshold={self.image_mse_threshold}"
        )

        self._query_camera_primitives()
        self._query_pose_primitive()

        if self.rgb_image_topic is None:
            raise ValueError("Failed to get rgb_image_topic from camera primitive")

        self.rgb_subscriber = self.create_subscription(
            Image, self.rgb_image_topic, self.rgb_image_callback, 10
        )
        self.get_logger().info(f"Subscribed to RGB image: {self.rgb_image_topic}")

        if self.rgb_image_topic.endswith("/image_raw"):
            self.camera_info_topic = self.rgb_image_topic.replace(
                "/image_raw", "/camera_info"
            )
        else:
            import re

            match = re.match(r"(.+)/(rgb|depth)/image_raw", self.rgb_image_topic)
            if match:
                self.camera_info_topic = (
                    f"{match.group(1)}/{match.group(2)}/camera_info"
                )
            else:
                self.get_logger().error(
                    f"Cannot infer camera_info topic from image topic: {self.rgb_image_topic}"
                )
                raise ValueError(
                    f"Cannot infer camera_info topic from image topic: {self.rgb_image_topic}"
                )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.get_logger().info(f"Subscribed to camera info: {self.camera_info_topic}")

        if self.pose_topic:
            pose_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.pose_subscriber = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_cov_callback,
                pose_qos,
            )
            self.get_logger().info(
                f"Subscribed to robot pose (PoseWithCovarianceStamped from primitive::base.pose.cov): {self.pose_topic} with RELIABLE QoS"
            )

            import time

            wait_start = time.time()
            wait_timeout = 2.0
            while not self.latest_pose and (time.time() - wait_start) < wait_timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose:
                self.get_logger().info(
                    f"Received initial pose: x={self.latest_pose.pose.position.x:.2f}, y={self.latest_pose.pose.position.y:.2f}"
                )
            else:
                self.get_logger().info(
                    "No pose message received yet - will continue listening. Robot object will be created when pose becomes available."
                )
        else:
            self.get_logger().warn(
                "No pose topic available - robot object will use default position (0,0,0)"
            )

        self.service = self.create_service(
            QuerySemanticMap, "/demo_service/semantic_map/query", self.query_callback
        )

        self.update_thread_running = True
        self.update_thread = threading.Thread(
            target=self._background_update_loop, daemon=True
        )
        self.update_thread.start()
        self.get_logger().info("Started background semantic map update thread")

        # Load manual objects from config file
        self._load_manual_objects_from_config(package_root)

        self.get_logger().info("Semantic map service started")
        self.get_logger().info("  Service: /demo_service/semantic_map/query")
        self.get_logger().info(
            "  Using Qwen3-VL for object detection (background updates)"
        )

    def _query_camera_primitives(self):
        """Query front camera primitives from OS with retry logic. Exits if failed."""
        self.rgb_image_topic = self.robonix.query_primitive_and_extract_field(
            "primitive::camera.rgb",
            field_name="image",
            filter_dict={"camera": "front"},
            max_retries=5,
            retry_delay=2.0,
            wait_timeout=10.0,
            call_timeout=3.0,
            raise_on_error=True,
            raise_on_missing_field=True,
            log_success=True,
        )

    def _query_pose_primitive(self):
        """Query robot pose primitive from OS with retry logic."""
        self.pose_topic = self.robonix.query_primitive_and_extract_field(
            "primitive::base.pose.cov",
            field_name="pose",
            filter_dict=None,
            max_retries=5,
            retry_delay=2.0,
            wait_timeout=10.0,
            call_timeout=3.0,
            raise_on_error=False,  # Don't raise, just log warning
            raise_on_missing_field=False,  # Don't raise, just log warning
            log_success=True,
        )

    def rgb_image_callback(self, msg):
        """Callback for RGB image messages."""
        self.latest_rgb_image = msg
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
            from PIL import Image as PILImage

            pil_image = PILImage.fromarray(cv_image)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_filename = (
                self.cache_dir / f"rgb_{timestamp}_{self.image_counter:04d}.jpg"
            )
            pil_image.save(image_filename, quality=95)
            self.image_counter += 1

            if self.image_counter > 20:
                image_files = sorted(self.cache_dir.glob("rgb_*.jpg"))
                if len(image_files) > 20:
                    for old_file in image_files[:-20]:
                        old_file.unlink()
                    self.image_counter = 20

            self.get_logger().debug(f"Saved RGB image to cache: {image_filename.name}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save RGB image to cache: {e}")

    def camera_info_callback(self, msg):
        """Callback for camera info messages."""
        self.latest_camera_info = msg

    def pose_cov_callback(self, msg):
        """Callback for PoseWithCovarianceStamped messages."""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        was_none = self.latest_pose is None
        self.latest_pose = pose_stamped

        with self.pose_history_lock:
            stamp_sec = (
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
            )
            self.pose_history.append((stamp_sec, pose_stamped))
            if len(self.pose_history) > 100:
                self.pose_history.pop(0)

        if was_none:
            self.get_logger().info(
                f"Received first pose update (PoseWithCovarianceStamped): x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f} - robot object will now be available"
            )
        else:
            self.get_logger().debug(
                f"Received pose update (PoseWithCovarianceStamped): x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f}"
            )

    def query_callback(self, request, response):
        """Handle semantic map query request - returns latest memory state immediately."""
        self.get_logger().info(f"Received query with types filter: {request.types}")

        snapshot_pose = self.latest_pose
        with self.memory_lock:
            memory_copy = dict(self.semantic_map_memory)

        memory_objects = self._get_all_objects_from_memory_unlocked(
            memory_copy, request.types, snapshot_pose=snapshot_pose
        )

        seen_ids = set()
        unique_objects = []
        for obj in memory_objects:
            if obj.id not in seen_ids:
                unique_objects.append(obj)
                seen_ids.add(obj.id)

        response.objects = unique_objects
        response.stamp = self.get_clock().now().to_msg()

        self.get_logger().debug(
            f"Returning {len(unique_objects)} objects from memory (immediate response)"
        )
        return response

    def _background_update_loop(self):
        """Background thread that continuously updates semantic map by calling VLM."""
        self.get_logger().info("Background update loop started")

        while self.update_thread_running:
            try:
                if not self.latest_rgb_image or not self.latest_camera_info:
                    self.get_logger().debug("Waiting for image/camera_info...")
                    time.sleep(1.0)
                    continue

                snapshot_image = self.latest_rgb_image
                snapshot_camera_info = self.latest_camera_info
                snapshot_stamp = snapshot_image.header.stamp
                image_stamp_sec = (
                    float(snapshot_stamp.sec) + float(snapshot_stamp.nanosec) / 1e9
                )
                snapshot_pose = self._find_pose_by_timestamp(image_stamp_sec)

                # If no synchronized pose found, use latest pose if available
                if not snapshot_pose:
                    if self.latest_pose:
                        snapshot_pose = self.latest_pose
                        self.get_logger().debug(
                            f"No synchronized pose found for image timestamp {image_stamp_sec:.3f}, "
                            f"using latest available pose (robot_pos=[{snapshot_pose.pose.position.x:.2f}, "
                            f"{snapshot_pose.pose.position.y:.2f}, {snapshot_pose.pose.position.z:.2f}])"
                        )
                    else:
                        self.get_logger().warn(
                            f"No pose available for image timestamp {image_stamp_sec:.3f}. "
                            f"Skipping this update."
                        )
                        time.sleep(self.update_interval)
                        continue
                else:
                    # Log synchronized pose info
                    pose_stamp_sec = (
                        float(snapshot_pose.header.stamp.sec)
                        + float(snapshot_pose.header.stamp.nanosec) / 1e9
                    )
                    time_diff = abs(pose_stamp_sec - image_stamp_sec)
                    self.get_logger().info(
                        f"Using synchronized pose: image_t={image_stamp_sec:.3f}, pose_t={pose_stamp_sec:.3f}, "
                        f"diff={time_diff * 1000:.1f}ms, robot_pos=[{snapshot_pose.pose.position.x:.2f}, "
                        f"{snapshot_pose.pose.position.y:.2f}, {snapshot_pose.pose.position.z:.2f}]"
                    )

                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(snapshot_image, "rgb8")
                except Exception as e:
                    self.get_logger().error(f"Failed to convert image: {e}")
                    time.sleep(self.update_interval)
                    continue

                # Skip VLM if pose or image unchanged to save tokens
                if self._pose_unchanged(snapshot_pose):
                    self.get_logger().info(
                        "Skipping VLM: robot pose unchanged (saving tokens)"
                    )
                    time.sleep(self.update_interval)
                    continue
                if self._image_unchanged(cv_image):
                    self.get_logger().info(
                        "Skipping VLM: image similar to last (saving tokens)"
                    )
                    time.sleep(self.update_interval)
                    continue

                from PIL import Image as PILImage
                import io

                pil_image = PILImage.fromarray(cv_image)
                buffer = io.BytesIO()
                pil_image.save(buffer, format="JPEG")
                image_base64 = base64.b64encode(buffer.getvalue()).decode("utf-8")

                camera_info_text = self._format_camera_info(snapshot_camera_info)
                self.get_logger().info(
                    "Calling VLM for object detection with distance and direction..."
                )
                detected_objects = self._detect_objects_with_vlm(
                    image_base64, camera_info_text, snapshot_camera_info, []
                )

                with self.memory_lock:
                    processed_objects = self._process_detected_objects(
                        detected_objects, snapshot_pose=snapshot_pose
                    )
                    memory_count = len(self.semantic_map_memory)
                    self.get_logger().info(
                        f"Updated semantic map: {len(processed_objects)} new objects processed, {memory_count} total objects in memory"
                    )

                # Remember pose and image so next cycle can skip VLM if unchanged
                with self._last_vlm_lock:
                    self.last_vlm_pose = self._pose_to_tuple(snapshot_pose)
                    lowres = self._compute_image_lowres(cv_image)
                    if lowres is not None:
                        self.last_vlm_image_lowres = lowres.copy()

                time.sleep(self.update_interval)

            except Exception as e:
                self.get_logger().error(f"Error in background update loop: {e}")
                import traceback

                self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
                time.sleep(self.update_interval)  # Wait before retrying

        self.get_logger().info("Background update loop stopped")

    def _pose_to_tuple(self, pose_stamped):
        """Extract (x, y, z, yaw) from PoseStamped for comparison."""
        if not pose_stamped or not pose_stamped.pose:
            return None
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        yaw = math.atan2(
            2.0 * (o.w * o.z + o.x * o.y),
            1.0 - 2.0 * (o.y * o.y + o.z * o.z),
        )
        return (p.x, p.y, p.z, yaw)

    def _pose_unchanged(self, current_pose_stamped):
        """Return True if current pose is close to last VLM pose (skip VLM to save tokens)."""
        with self._last_vlm_lock:
            last = self.last_vlm_pose
        if last is None:
            return False
        curr = self._pose_to_tuple(current_pose_stamped)
        if curr is None:
            return False
        dx = curr[0] - last[0]
        dy = curr[1] - last[1]
        dz = curr[2] - last[2]
        position_delta = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw_delta = abs(curr[3] - last[3])
        if yaw_delta > math.pi:
            yaw_delta = 2.0 * math.pi - yaw_delta
        return (
            position_delta < self.pose_position_threshold
            and yaw_delta < self.pose_yaw_threshold_rad
        )

    def _compute_image_lowres(self, cv_image, size=(32, 32)):
        """Compute small grayscale image for similarity (no extra deps)."""
        try:
            if len(cv_image.shape) == 3:
                gray = np.dot(cv_image[..., :3], [0.299, 0.587, 0.114]).astype(np.uint8)
            else:
                gray = cv_image
            from PIL import Image as PILImage
            pil = PILImage.fromarray(gray)
            # 2 = BILINEAR (PIL.Image.Resampling.BILINEAR in Pillow 9+)
            pil_small = pil.resize(size, 2)
            return np.array(pil_small, dtype=np.uint8)
        except Exception:
            return None

    def _image_unchanged(self, cv_image):
        """Return True if image is very similar to last VLM image (skip VLM to save tokens)."""
        with self._last_vlm_lock:
            last_lowres = self.last_vlm_image_lowres
        if last_lowres is None:
            return False
        curr_lowres = self._compute_image_lowres(cv_image)
        if curr_lowres is None:
            return False
        mse = float(np.mean((curr_lowres.astype(np.float64) - last_lowres.astype(np.float64)) ** 2))
        return mse < self.image_mse_threshold

    def _format_camera_info(self, camera_info):
        """Format camera info as text for VLM prompt."""
        fx = camera_info.k[0] if len(camera_info.k) > 0 else 0
        fy = camera_info.k[4] if len(camera_info.k) > 4 else 0
        cx = camera_info.k[2] if len(camera_info.k) > 2 else 0
        cy = camera_info.k[5] if len(camera_info.k) > 5 else 0
        return f"Camera parameters: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}, resolution={camera_info.width}x{camera_info.height}"

    def _detect_objects_with_vlm(
        self, image_base64, camera_info_text, camera_info, type_filter
    ):
        """Use Qwen3-VL to detect objects and estimate distance + direction vector."""
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
                                },
                            },
                            {"type": "text", "text": prompt},
                        ],
                    }
                ],
                max_tokens=2000,
            )

            if not api_response.choices or len(api_response.choices) == 0:
                self.get_logger().error("VLM API response has no choices")
                return []

            response_text = api_response.choices[0].message.content
            if not response_text:
                self.get_logger().error("VLM API response content is empty")
                return []

            self.get_logger().info(f"VLM response: {response_text[:200]}...")

            import re

            json_match = re.search(r"\[.*\]", response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group()
                if json_str:
                    objects_data = json.loads(json_str)
                else:
                    self.get_logger().error("Failed to extract JSON from VLM response")
                    return []
            else:
                objects_data = json.loads(response_text)

            objects = []
            excluded_keywords = ["wall", "floor", "ceiling", "ground", "surface"]

            for obj_data in objects_data:
                label = obj_data.get("label", "").lower()
                if any(keyword in label for keyword in excluded_keywords):
                    self.get_logger().debug(
                        f"Filtered out non-object: {obj_data.get('label', 'unknown')}"
                    )
                    continue

                if type_filter and len(type_filter) > 0:
                    obj_type = self._infer_object_type(obj_data.get("label", ""))
                    if obj_type not in type_filter:
                        continue

                obj = Object()
                obj.label = obj_data.get("label", "unknown")
                obj.id = self._generate_label_based_id(obj.label)
                obj.registered_skills = []
                obj.registered_primitives = []
                obj.relations = []

                bbox_2d = obj_data.get("bbox_2d", [0, 0, 640, 480])
                distance = float(obj_data.get("distance", 1.0))
                pos_cam = self._calculate_camera_coords_from_distance(
                    distance, bbox_2d, camera_info
                )

                frame_mapping = FrameMapping()
                frame_mapping.frame_id = "head_front_camera_rgb_optical_frame"
                frame_mapping.center = Point3D()
                frame_mapping.center.x = pos_cam[0]
                frame_mapping.center.y = pos_cam[1]
                frame_mapping.center.z = pos_cam[2]

                size = obj_data.get("size", [0.1, 0.1, 0.1])
                bbox = BoundingBox()
                bbox.scale_x = float(size[0]) if len(size) > 0 else 0.1
                bbox.scale_y = float(size[1]) if len(size) > 1 else 0.1
                bbox.scale_z = float(size[2]) if len(size) > 2 else 0.1
                bbox.yaw = 0.0
                frame_mapping.bbox = [bbox]

                obj.frame_mapping = [frame_mapping]
                objects.append(obj)

            return objects

        except Exception as e:
            self.get_logger().error(f"Error calling Qwen3-VL API: {e}")
            import traceback

            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            return []

    def _generate_label_based_id(self, label):
        """Generate a unique object ID based on label with index."""
        label_lower = label.lower() if label else ""
        normalized_label = self._normalize_label(label_lower)
        sanitized_label = normalized_label.replace(" ", "_").replace("-", "_")
        import re

        sanitized_label = re.sub(r"[^a-z0-9_]", "_", sanitized_label)
        sanitized_label = re.sub(r"_+", "_", sanitized_label)
        sanitized_label = sanitized_label.strip("_")

        with self.id_generation_lock:
            existing_indices = set()
            with self.memory_lock:
                prefix = f"{sanitized_label}_"
                for existing_id in self.semantic_map_memory.keys():
                    if existing_id.startswith(prefix):
                        suffix = existing_id[len(prefix) :]
                        if suffix.isdigit():
                            try:
                                existing_indices.add(int(suffix))
                            except ValueError:
                                pass

            next_index = self.label_counters.get(sanitized_label, 0)
            while next_index in existing_indices:
                next_index += 1
            self.label_counters[sanitized_label] = next_index + 1
            return f"{sanitized_label}_{next_index}"

    def _calculate_camera_coords_from_distance(self, distance, bbox_2d, camera_info):
        """Calculate camera frame coordinates from distance and 2D bbox."""
        x_min, y_min, x_max, y_max = bbox_2d
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0

        fx = camera_info.k[0] if len(camera_info.k) > 0 else 1.0
        fy = camera_info.k[4] if len(camera_info.k) > 4 else 1.0
        cx = camera_info.k[2] if len(camera_info.k) > 2 else camera_info.width / 2.0
        cy = camera_info.k[5] if len(camera_info.k) > 5 else camera_info.height / 2.0

        normalized_x = (center_x - cx) / fx
        normalized_y = (center_y - cy) / fy
        direction_magnitude = math.sqrt(normalized_x**2 + normalized_y**2 + 1.0)
        scale_factor = distance / direction_magnitude

        return [
            normalized_x * scale_factor,
            normalized_y * scale_factor,
            1.0 * scale_factor,
        ]

    def _transform_camera_to_map(self, camera_pos, robot_pose_stamped=None):
        """Transform coordinates from camera frame to map frame."""
        pose_to_use = (
            robot_pose_stamped if robot_pose_stamped is not None else self.latest_pose
        )
        if not pose_to_use:
            return None

        robot_pose = pose_to_use.pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_z = robot_pose.position.z

        qx = robot_pose.orientation.x
        qy = robot_pose.orientation.y
        qz = robot_pose.orientation.z
        qw = robot_pose.orientation.w
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        camera_offset_x = 0.3
        camera_offset_y = 0.0
        camera_offset_z = 0.2

        base_x = camera_pos[2] + camera_offset_x
        base_y = -camera_pos[0] + camera_offset_y
        base_z = -camera_pos[1] + camera_offset_z

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        return [
            robot_x + cos_yaw * base_x - sin_yaw * base_y,
            robot_y + sin_yaw * base_x + cos_yaw * base_y,
            robot_z + base_z,
        ]

    def _cluster_same_label_objects(self, objects_with_map_pos):
        """Cluster objects with the same label that are close to each other."""
        CLUSTER_DISTANCE_THRESHOLD = 1.5
        if not objects_with_map_pos:
            return []

        objects_by_label = {}
        for obj, map_pos in objects_with_map_pos:
            normalized_label = self._normalize_label(obj.label)
            if normalized_label not in objects_by_label:
                objects_by_label[normalized_label] = []
            objects_by_label[normalized_label].append((obj, map_pos))

        clustered_objects = []
        for label, obj_list in objects_by_label.items():
            if len(obj_list) == 1:
                clustered_objects.append(obj_list[0])
                continue

            clusters = []
            for obj, map_pos in obj_list:
                assigned = False
                for cluster in clusters:
                    cluster_center = cluster["center"]
                    distance = math.sqrt(
                        (map_pos[0] - cluster_center[0]) ** 2
                        + (map_pos[1] - cluster_center[1]) ** 2
                        + (map_pos[2] - cluster_center[2]) ** 2
                    )
                    if distance < CLUSTER_DISTANCE_THRESHOLD:
                        cluster["objects"].append((obj, map_pos))
                        n = len(cluster["objects"])
                        cluster["center"] = [
                            sum(p[1][0] for p in cluster["objects"]) / n,
                            sum(p[1][1] for p in cluster["objects"]) / n,
                            sum(p[1][2] for p in cluster["objects"]) / n,
                        ]
                        assigned = True
                        break

                if not assigned:
                    clusters.append({"objects": [(obj, map_pos)], "center": map_pos})

            for cluster in clusters:
                if len(cluster["objects"]) == 1:
                    clustered_objects.append(cluster["objects"][0])
                else:
                    obj, _ = cluster["objects"][0]
                    avg_pos = cluster["center"]
                    for fm in obj.frame_mapping:
                        if fm.frame_id == "map":
                            fm.center.x = avg_pos[0]
                            fm.center.y = avg_pos[1]
                            fm.center.z = avg_pos[2]
                            break
                    self.get_logger().info(
                        f'Clustered {len(cluster["objects"])} objects with label "{obj.label}" at average position [{avg_pos[0]:.2f}, {avg_pos[1]:.2f}, {avg_pos[2]:.2f}]'
                    )
                    clustered_objects.append((obj, avg_pos))

        return clustered_objects

    def _process_detected_objects(self, detected_objects, snapshot_pose=None):
        """Process detected objects: convert to map frame, cluster same-label objects, and merge with memory."""
        if not snapshot_pose:
            self.get_logger().error(
                "snapshot_pose is None - cannot process objects without synchronized pose. "
                "Skipping object processing to ensure accuracy."
            )
            return []

        objects_with_map_pos = []
        objects_without_map = []

        for obj in detected_objects:
            if not obj.frame_mapping or len(obj.frame_mapping) == 0:
                continue

            camera_frame = obj.frame_mapping[0]
            camera_pos = [
                camera_frame.center.x,
                camera_frame.center.y,
                camera_frame.center.z,
            ]

            map_pos = self._transform_camera_to_map(
                camera_pos, robot_pose_stamped=snapshot_pose
            )
            if map_pos:
                map_frame = FrameMapping()
                map_frame.frame_id = "map"
                map_frame.center = Point3D()
                map_frame.center.x = map_pos[0]
                map_frame.center.y = map_pos[1]
                map_frame.center.z = map_pos[2]
                map_frame.bbox = camera_frame.bbox.copy() if camera_frame.bbox else []
                map_frame.texture = (
                    camera_frame.texture.copy() if camera_frame.texture else []
                )
                obj.frame_mapping.append(map_frame)
                objects_with_map_pos.append((obj, map_pos))
            else:
                self.get_logger().warn(
                    f"Transform failed for object {obj.label} (id={obj.id}), skipping"
                )
                objects_without_map.append(obj)

        clustered_objects = self._cluster_same_label_objects(objects_with_map_pos)
        processed_objects = []
        for obj, map_pos in clustered_objects:
            merged = self._merge_with_memory(obj, map_pos)
            if not merged:
                self._add_to_memory(obj, map_pos)
            processed_objects.append(obj)

        processed_objects.extend(objects_without_map)
        return processed_objects

    def _normalize_label(self, label):
        """Normalize label to handle variations (e.g., 'computer monitor' -> 'monitor')."""
        label_lower = label.lower()
        if "monitor" in label_lower or "computer monitor" in label_lower:
            return "monitor"
        if "office chair" in label_lower or "chair" in label_lower:
            return "chair"
        if "desk" in label_lower or "table" in label_lower:
            return "desk"
        if "cabinet" in label_lower or "shelf" in label_lower:
            return "cabinet"
        return label_lower

    def _merge_with_memory(self, obj, map_pos):
        """Try to merge object with existing objects in memory based on label and position."""
        MERGE_DISTANCE_THRESHOLD = 1.5
        obj_label_normalized = self._normalize_label(obj.label)
        best_match = None
        best_distance = float("inf")

        for existing_id, existing_obj in self.semantic_map_memory.items():
            existing_label_normalized = self._normalize_label(existing_obj.label)
            if existing_label_normalized != obj_label_normalized:
                continue

            existing_map_pos = None
            for fm in existing_obj.frame_mapping:
                if fm.frame_id == "map":
                    existing_map_pos = [fm.center.x, fm.center.y, fm.center.z]
                    break

            if not existing_map_pos:
                continue

            distance = math.sqrt(
                (map_pos[0] - existing_map_pos[0]) ** 2
                + (map_pos[1] - existing_map_pos[1]) ** 2
                + (map_pos[2] - existing_map_pos[2]) ** 2
            )

            if distance < MERGE_DISTANCE_THRESHOLD and distance < best_distance:
                best_match = existing_obj
                best_distance = distance

        if best_match:
            weight_new = 0.3
            weight_existing = 0.7
            existing_map_pos = None
            for fm in best_match.frame_mapping:
                if fm.frame_id == "map":
                    existing_map_pos = [fm.center.x, fm.center.y, fm.center.z]
                    break

            if existing_map_pos:
                merged_x = (
                    weight_existing * existing_map_pos[0] + weight_new * map_pos[0]
                )
                merged_y = (
                    weight_existing * existing_map_pos[1] + weight_new * map_pos[1]
                )
                merged_z = (
                    weight_existing * existing_map_pos[2] + weight_new * map_pos[2]
                )

                for fm in best_match.frame_mapping:
                    if fm.frame_id == "map":
                        fm.center.x = merged_x
                        fm.center.y = merged_y
                        fm.center.z = merged_z
                        break

                obj.id = best_match.id
                self.get_logger().info(
                    f'Merged object "{obj.label}" with existing object (id={best_match.id}) at distance {best_distance:.2f}m'
                )
                return True

        return False

    def _add_to_memory(self, obj, map_pos):
        """Add object to memory. Objects are never deleted once they have map coordinates."""
        self.semantic_map_memory[obj.id] = obj
        self.get_logger().info(
            f'Added object "{obj.label}" (id={obj.id}) to memory at map position [{map_pos[0]:.2f}, {map_pos[1]:.2f}, {map_pos[2]:.2f}]'
        )

    def _get_all_objects_from_memory_unlocked(
        self, memory_dict, type_filter, snapshot_pose=None
    ):
        """Get all objects from memory dict (unlocked version for query_callback)."""
        objects_with_map_pos = []
        seen_ids = set()

        for obj_id, obj in memory_dict.items():
            if obj_id in seen_ids:
                continue

            map_pos = None
            for fm in obj.frame_mapping:
                if fm.frame_id == "map":
                    map_pos = [fm.center.x, fm.center.y, fm.center.z]
                    break

            if not map_pos:
                continue

            if type_filter and len(type_filter) > 0:
                obj_type = self._infer_object_type(obj.label)
                if obj_type not in type_filter:
                    continue

            objects_with_map_pos.append((obj, map_pos))
            seen_ids.add(obj_id)

        clustered_objects = self._cluster_same_label_objects(objects_with_map_pos)
        objects = []
        for item in clustered_objects:
            obj, _ = item if isinstance(item, tuple) else (item, None)
            objects.append(obj)

        robot_object = self._create_robot_object(snapshot_pose=snapshot_pose)
        if robot_object and robot_object.id not in seen_ids:
            if not type_filter or len(type_filter) == 0 or "robot" in type_filter:
                objects.append(robot_object)
                seen_ids.add(robot_object.id)

        return objects

    def _create_robot_object(self, snapshot_pose=None):
        """Create robot object for semantic map. Only creates if pose is available."""
        pose_to_use = snapshot_pose if snapshot_pose is not None else self.latest_pose
        if not pose_to_use:
            return None

        obj = Object()
        obj.id = "robot_self"
        obj.label = "robot"
        obj.registered_skills = []
        obj.registered_primitives = []
        obj.relations = []

        map_frame = FrameMapping()
        map_frame.frame_id = "map"
        map_frame.center = Point3D()

        robot_pose = pose_to_use.pose
        map_frame.center.x = robot_pose.position.x
        map_frame.center.y = robot_pose.position.y
        map_frame.center.z = robot_pose.position.z

        qx = robot_pose.orientation.x
        qy = robot_pose.orientation.y
        qz = robot_pose.orientation.z
        qw = robot_pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(
            f"Created robot object with pose: x={map_frame.center.x:.2f}, y={map_frame.center.y:.2f}, z={map_frame.center.z:.2f}, yaw={math.degrees(yaw):.1f}°"
        )

        bbox = BoundingBox()
        bbox.scale_x = 0.5
        bbox.scale_y = 0.5
        bbox.scale_z = 1.0
        bbox.yaw = yaw
        map_frame.bbox = [bbox]
        map_frame.texture = []

        obj.frame_mapping = [map_frame]
        return obj

    def _find_pose_by_timestamp(self, target_stamp_sec):
        """Find pose closest to target timestamp from pose history."""
        with self.pose_history_lock:
            if not self.pose_history:
                return None

            best_pose = None
            best_diff = float("inf")
            TOLERANCE_SEC = 0.5

            for stamp_sec, pose in self.pose_history:
                diff = abs(stamp_sec - target_stamp_sec)
                if diff < best_diff and diff <= TOLERANCE_SEC:
                    best_diff = diff
                    best_pose = pose

            if best_pose:
                self.get_logger().debug(
                    f"Found matching pose: target_t={target_stamp_sec:.3f}, "
                    f"pose_t={best_diff + target_stamp_sec:.3f}, diff={best_diff * 1000:.1f}ms"
                )
            else:
                self.get_logger().debug(
                    f"No pose within tolerance: target_t={target_stamp_sec:.3f}, "
                    f"history_size={len(self.pose_history)}, tolerance={TOLERANCE_SEC * 1000:.0f}ms"
                )

            return best_pose

    def _infer_object_type(self, label):
        """Infer object type from label (simple heuristic)."""
        label_lower = label.lower()
        if "table" in label_lower or "desk" in label_lower:
            return "table"
        elif "box" in label_lower or "container" in label_lower:
            return "box"
        elif "chair" in label_lower:
            return "chair"
        elif "robot" in label_lower:
            return "robot"
        elif "waypoint" in label_lower:
            return "waypoint"
        else:
            return "object"

    def _load_manual_objects_from_config(self, package_root):
        """Load manual objects from config YAML file (e.g. building_map_config.yaml or webots_map_config.yaml)."""
        # Resolve config filename: short name -> *_map_config.yaml, or use as-is if contains '.yaml'
        if not self.config_filename:
            config_name = "building_map_config.yaml"
        elif self.config_filename.endswith(".yaml") or self.config_filename.endswith(".yml"):
            config_name = self.config_filename
        else:
            config_name = f"{self.config_filename}_map_config.yaml"
        config_path = package_root / "rbnx" / config_name
        
        if not config_path.exists():
            self.get_logger().info(
                f"Config file not found at {config_path}, skipping manual objects loading"
            )
            return
        self.get_logger().info(f"Loading manual objects from config: {config_name}")

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                config_data = yaml.safe_load(f)
            
            if not config_data or "manual_objects" not in config_data:
                self.get_logger().info("No manual_objects found in config file")
                return

            manual_objects_config = config_data.get("manual_objects", [])
            if not manual_objects_config:
                self.get_logger().info("manual_objects list is empty")
                return

            loaded_count = 0
            for obj_config in manual_objects_config:
                try:
                    obj = self._create_object_from_config(obj_config)
                    if obj:
                        # Generate ID if not provided, using same logic as auto-generated objects
                        if not obj.id or obj.id == "":
                            obj.id = self._generate_label_based_id(obj.label)
                        
                        # Check if object already exists (by ID)
                        with self.memory_lock:
                            if obj.id in self.semantic_map_memory:
                                self.get_logger().warn(
                                    f"Manual object with id '{obj.id}' already exists, skipping"
                                )
                                continue
                            
                            # Add to memory
                            self.semantic_map_memory[obj.id] = obj
                            loaded_count += 1
                            
                            # Log object position
                            map_pos = None
                            for fm in obj.frame_mapping:
                                if fm.frame_id == "map":
                                    map_pos = [fm.center.x, fm.center.y, fm.center.z]
                                    break
                            
                            if map_pos:
                                self.get_logger().info(
                                    f'Loaded manual object "{obj.label}" (id={obj.id}) at map position '
                                    f'[{map_pos[0]:.2f}, {map_pos[1]:.2f}, {map_pos[2]:.2f}]'
                                )
                            else:
                                self.get_logger().info(
                                    f'Loaded manual object "{obj.label}" (id={obj.id})'
                                )
                
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to load manual object from config: {e}"
                    )
                    import traceback
                    self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
                    continue

            self.get_logger().info(
                f"Successfully loaded {loaded_count} manual object(s) from config"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to load manual objects from config file {config_path}: {e}"
            )
            import traceback
            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")

    def _create_object_from_config(self, obj_config):
        """Create an Object from config dictionary."""
        if "label" not in obj_config:
            self.get_logger().error("Manual object config missing 'label' field")
            return None

        obj = Object()
        obj.label = obj_config["label"]
        
        # Use provided ID or None (will be auto-generated later)
        obj.id = obj_config.get("id") or ""
        
        # Set default empty lists for optional fields
        obj.registered_skills = obj_config.get("registered_skills", [])
        obj.registered_primitives = obj_config.get("registered_primitives", [])
        obj.relations = obj_config.get("relations", [])
        
        # Parse frame_mapping
        if "frame_mapping" not in obj_config:
            self.get_logger().error(
                f"Manual object '{obj.label}' config missing 'frame_mapping' field"
            )
            return None

        obj.frame_mapping = []
        for fm_config in obj_config["frame_mapping"]:
            frame_mapping = FrameMapping()
            frame_mapping.frame_id = fm_config.get("frame_id", "map")
            
            # Parse center
            if "center" not in fm_config:
                self.get_logger().error(
                    f"Manual object '{obj.label}' frame_mapping missing 'center' field"
                )
                continue
            
            center_config = fm_config["center"]
            frame_mapping.center = Point3D()
            frame_mapping.center.x = float(center_config.get("x", 0.0))
            frame_mapping.center.y = float(center_config.get("y", 0.0))
            frame_mapping.center.z = float(center_config.get("z", 0.0))
            
            # Parse bbox (optional, defaults to small box)
            frame_mapping.bbox = []
            if "bbox" in fm_config and fm_config["bbox"]:
                for bbox_config in fm_config["bbox"]:
                    bbox = BoundingBox()
                    bbox.scale_x = float(bbox_config.get("scale_x", 0.1))
                    bbox.scale_y = float(bbox_config.get("scale_y", 0.1))
                    bbox.scale_z = float(bbox_config.get("scale_z", 0.1))
                    bbox.yaw = float(bbox_config.get("yaw", 0.0))
                    frame_mapping.bbox.append(bbox)
            else:
                # Default bbox for waypoints
                bbox = BoundingBox()
                bbox.scale_x = 0.1
                bbox.scale_y = 0.1
                bbox.scale_z = 0.1
                bbox.yaw = 0.0
                frame_mapping.bbox.append(bbox)
            
            # Parse texture (optional, defaults to empty)
            frame_mapping.texture = fm_config.get("texture", [])
            
            obj.frame_mapping.append(frame_mapping)

        if not obj.frame_mapping:
            self.get_logger().error(
                f"Manual object '{obj.label}' has no valid frame_mapping"
            )
            return None

        return obj


def main(args=None):
    import argparse
    import sys
    parser = argparse.ArgumentParser(description="Semantic map service (manual objects from config)")
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        metavar="NAME",
        help="Config name or file: 'building' (default), 'webots', or filename e.g. my_map_config.yaml",
    )
    # Parse only known args so rclpy can handle the rest
    argv = args if args is not None else sys.argv[1:]
    parsed, remaining = parser.parse_known_args(argv)
    config_filename = parsed.config

    rclpy.init(args=remaining)
    semantic_map_service = None
    try:
        semantic_map_service = SemanticMapService(config_filename=config_filename)
        rclpy.spin(semantic_map_service)
        semantic_map_service.destroy_node()
    except (RuntimeError, ValueError) as e:
        import sys

        print(f"FATAL: Failed to initialize semantic map service: {e}", file=sys.stderr)
        if semantic_map_service is not None:
            try:
                semantic_map_service.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(1)
    except Exception as e:
        import sys

        print(f"FATAL: Unexpected error: {e}", file=sys.stderr)
        if semantic_map_service is not None:
            try:
                semantic_map_service.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
