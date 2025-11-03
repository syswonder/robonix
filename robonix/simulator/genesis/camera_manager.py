# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import threading
import time
import os
import cv2
import numpy as np

from robonix.manager.log import logger

class CameraManager:
    def __init__(
        self, camera, car, scene_lock, output_dir="cam_output", capture_interval=1.0
    ):
        self.camera = camera
        self.car = car
        self.scene_lock = scene_lock

        # Set output directory to be relative to robot1.py location
        if not os.path.isabs(output_dir):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.output_dir = os.path.join(script_dir, output_dir)
        else:
            self.output_dir = output_dir

        self.capture_interval = capture_interval  # capture interval (seconds)
        self.stop_event = threading.Event()
        self.camera_thread = None

        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"camera output directory: {self.output_dir}")

        # Image counter
        self.image_counter = 0

    def start_camera_thread(self):
        """Start camera capture thread"""
        self.camera_thread = threading.Thread(target=self._camera_update_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        logger.info(
            f"camera thread started, saving images to {self.output_dir} every {self.capture_interval}s"
        )
        logger.info(
            f"camera thread ID: {self.camera_thread.ident}, alive: {self.camera_thread.is_alive()}"
        )

    def stop_camera_thread(self):
        """Stop camera capture thread"""
        self.stop_event.set()
        if self.camera_thread is not None:
            try:
                self.camera_thread.join(timeout=2.0)
                logger.info("camera thread stopped")
            except Exception as e:
                logger.warning(f"camera thread join error: {e}")

    def _camera_update_loop(self):
        """Camera update loop, periodically capture and save images"""
        while not self.stop_event.is_set():
            try:
                with self.scene_lock:
                    # Update camera position to follow vehicle
                    car_pos = self.car.get_pos()
                    car_x, car_y, car_z = (
                        float(car_pos[0]),
                        float(car_pos[1]),
                        float(car_pos[2]),
                    )
                    car_yaw = getattr(self.car, "_my_yaw", 0.0)

                    # Calculate camera position in front of vehicle (0.3 units from vehicle center)
                    camera_offset_x = 0.3 * np.sin(car_yaw)
                    camera_offset_y = 0.3 * np.cos(car_yaw)
                    camera_x = car_x + camera_offset_x
                    camera_y = car_y + camera_offset_y
                    camera_z = car_z + 0.2  # Slightly above vehicle center

                    # Calculate lookat point (forward direction)
                    lookat_x = camera_x + np.sin(car_yaw)
                    lookat_y = camera_y + np.cos(car_yaw)
                    lookat_z = camera_z

                    # Update camera position and orientation
                    self.camera.set_pose(
                        pos=(camera_x, camera_y, camera_z),
                        lookat=(lookat_x, lookat_y, lookat_z),
                    )

                    # Render images
                    rgb, depth, segmentation, normal = self.camera.render(
                        depth=True, segmentation=True, normal=True
                    )

                    # Save RGB image
                    self._save_rgb_image(rgb)

                    # Save depth image
                    self._save_depth_image(depth)

            except Exception as e:
                logger.error(f"camera update error: {e}")

            # Wait for specified interval
            time.sleep(self.capture_interval)

    def _save_rgb_image(self, rgb):
        """Save RGB image"""
        try:
            # Convert to BGR format for OpenCV saving
            rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # Use fixed filename (overwrite mode)
            filename = "rgb_current.jpg"
            filepath = os.path.join(self.output_dir, filename)

            # Save image
            success = cv2.imwrite(filepath, rgb_bgr, [
                                  cv2.IMWRITE_JPEG_QUALITY, 90])

            if success:
                self.image_counter += 1
                # logger.trace(f"rgb image saved: {filepath}")
            else:
                logger.error(f"failed to save rgb image: {filepath}")

        except Exception as e:
            logger.error(f"error saving rgb image: {e}")
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    def _save_depth_image(self, depth):
        """Save depth image"""
        try:
            # first dump the stucture of depth
            # logger.info(f"type of depth: {type(depth)}")
            # pprint(depth)
            # # depth is np.ndarray, dump shape and other info
            # logger.info(f"shape of depth: {depth.shape}")
            # logger.info(f"dtype of depth: {depth.dtype}")
            # logger.info(
            #     f"min of depth: {np.min(depth)}, at index {np.argmin(depth)}, pixel location {np.unravel_index(np.argmin(depth), depth.shape)}"
            # )
            # logger.info(
            #     f"max of depth: {np.max(depth)}, at index {np.argmax(depth)}, pixel location {np.unravel_index(np.argmax(depth), depth.shape)}"
            # )
            # # print the center pixel's "depth"
            # logger.info(
            #     f"center pixel's depth: {depth[depth.shape[0]//2, depth.shape[1]//2]}"
            # )

            # Use fixed filename (overwrite mode)
            filename = "depth_current.png"
            filepath = os.path.join(self.output_dir, filename)

            # Optimize depth visualization with better contrast and scaling
            success = self._save_optimized_depth_image(depth, filepath)

            # Also save a colored version for better visualization
            if success:
                colored_filename = "depth_current_colored.png"
                colored_filepath = os.path.join(
                    self.output_dir, colored_filename)
                self._save_colored_depth_image(depth, colored_filepath)

            if success:
                # logger.trace(f"depth image saved: {filepath}")
                pass
            else:
                logger.error(f"failed to save depth image: {filepath}")

        except Exception as e:
            logger.error(f"error saving depth image: {e}")
            import traceback

            logger.error(f"traceback: {traceback.format_exc()}")

    def _save_optimized_depth_image(self, depth, filepath):
        """Save depth image with optimized visualization (grayscale + value scaling)"""
        try:
            # Convert to float32 if not already
            depth_float = depth.astype(np.float32)

            # Get depth statistics for better scaling
            min_depth = np.min(depth_float)
            max_depth = np.max(depth_float)

            # Handle edge cases (all same values or invalid depths)
            if min_depth == max_depth or np.isnan(min_depth) or np.isnan(max_depth):
                logger.warning(
                    "depth image has uniform or invalid values, using default scaling")
                min_depth = 0.0
                max_depth = 1.0

            # Apply depth normalization with clipping to remove outliers
            # Use percentile-based clipping for more robust scaling
            depth_clipped = np.clip(depth_float,
                                    np.percentile(depth_float, 1),
                                    np.percentile(depth_float, 99))

            # Normalize to 0-1 range
            depth_normalized = (depth_clipped - np.min(depth_clipped)) / \
                (np.max(depth_clipped) - np.min(depth_clipped) + 1e-8)

            # Apply gamma correction for better visual contrast
            gamma = 0.6  # Darker mid-tones for better depth perception
            depth_gamma = np.power(depth_normalized, gamma)

            # Convert to 8-bit grayscale (0-255)
            depth_8bit = (depth_gamma * 255).astype(np.uint8)

            # Apply slight Gaussian blur to reduce noise (optional)
            depth_smoothed = cv2.GaussianBlur(depth_8bit, (3, 3), 0.5)

            # Save the optimized depth image
            success = cv2.imwrite(filepath, depth_smoothed)

            if success:
                logger.debug(f"optimized depth image saved: {filepath}")
                logger.debug(
                    f"depth range: {min_depth:.4f} to {max_depth:.4f}")
                logger.debug(
                    f"normalized range: {np.min(depth_normalized):.4f} to {np.max(depth_normalized):.4f}")

            return success

        except Exception as e:
            logger.error(f"Error in depth image optimization: {e}")
            # Fallback to original method
            return cv2.imwrite(filepath, depth)

    def _save_colored_depth_image(self, depth, filepath):
        """Save depth image with color mapping for better visualization"""
        try:
            # Convert to float32 if not already
            depth_float = depth.astype(np.float32)

            # Get depth statistics for better scaling
            min_depth = np.min(depth_float)
            max_depth = np.max(depth_float)

            # Handle edge cases
            if min_depth == max_depth or np.isnan(min_depth) or np.isnan(max_depth):
                logger.warning(
                    "depth image has uniform or invalid values, using default scaling")
                min_depth = 0.0
                max_depth = 1.0

            # Apply depth normalization with clipping to remove outliers
            depth_clipped = np.clip(depth_float,
                                    np.percentile(depth_float, 1),
                                    np.percentile(depth_float, 99))

            # Normalize to 0-1 range
            depth_normalized = (depth_clipped - np.min(depth_clipped)) / \
                (np.max(depth_clipped) - np.min(depth_clipped) + 1e-8)

            # Apply gamma correction for better visual contrast
            gamma = 0.7  # Slightly different gamma for color version
            depth_gamma = np.power(depth_normalized, gamma)

            # Convert to 8-bit (0-255)
            depth_8bit = (depth_gamma * 255).astype(np.uint8)

            # Apply color mapping using OpenCV's COLORMAP_JET (blue to red)
            # This provides intuitive depth perception: blue = near, red = far
            depth_colored = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

            # Apply slight Gaussian blur to reduce noise
            depth_smoothed = cv2.GaussianBlur(depth_colored, (3, 3), 0.5)

            # Save the colored depth image
            success = cv2.imwrite(filepath, depth_smoothed)

            if success:
                logger.debug(f"colored depth image saved: {filepath}")

            return success

        except Exception as e:
            logger.error(f"Error in colored depth image generation: {e}")
            return False

    def get_rgb_image(self, width=800, height=600):
        """Get BGR image data (for gRPC service) - Genesis returns BGR format"""
        try:
            with self.scene_lock:
                rgb, _, _, _ = self.camera.render(
                    depth=True, segmentation=True, normal=True
                )

                # Debug: Check original rendered image format
                logger.debug(
                    f"Original rendered image - type: {type(rgb)}, shape: {rgb.shape}, dtype: {rgb.dtype}")
                if rgb.size > 0:
                    # Check center pixel colors in original image
                    center_y, center_x = rgb.shape[0] // 2, rgb.shape[1] // 2
                    center_color = rgb[center_y, center_x]
                    logger.debug(
                        f"Original center pixel values: [0]={center_color[0]}, [1]={center_color[1]}, [2]={center_color[2]}")

                    # Check some sample pixels
                    sample_positions = [(100, 100), (200, 200), (300, 300)]
                    for y, x in sample_positions:
                        if y < rgb.shape[0] and x < rgb.shape[1]:
                            color = rgb[y, x]
                            logger.debug(
                                f"Original sample pixel at ({x}, {y}): [0]={color[0]}, [1]={color[1]}, [2]={color[2]}")

                # Resize image
                if width > 0 and height > 0:
                    rgb = cv2.resize(rgb, (width, height))

                # Convert to JPEG format
                # FIXED: Genesis returns BGR format (like OpenCV), so no conversion needed
                # The _save_rgb_image works because it saves the raw BGR image directly
                # But here we need to encode to JPEG for gRPC transmission
                rgb_bgr = rgb  # Genesis returns BGR, no conversion needed
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, jpeg_data = cv2.imencode(".jpg", rgb_bgr, encode_param)
                logger.debug(
                    "Using Genesis BGR format directly for JPEG encoding (no conversion)")

                return jpeg_data.tobytes(), rgb.shape[1], rgb.shape[0]

        except Exception as e:
            logger.error(f"error getting rgb image: {e}")
            return b"", 0, 0

    def get_depth_image(self, width=800, height=600):
        """Get depth image data (for gRPC service)"""
        try:
            with self.scene_lock:
                _, depth, _, _ = self.camera.render(
                    depth=True, segmentation=True, normal=True
                )

                # Resize image
                if width > 0 and height > 0:
                    logger.warning(
                        f"resizing depth image to {width}x{height} from {depth.shape}"
                    )
                    depth = cv2.resize(depth, (width, height))

                # Convert to float32 array
                depth_float32 = depth.astype(np.float32)
                min_depth = float(np.min(depth_float32))
                max_depth = float(np.max(depth_float32))

                logger.info(
                    f"depth image shape: {depth.shape}, min depth: {min_depth}, max depth: {max_depth}, center pixel's depth: {depth[depth.shape[0]//2, depth.shape[1]//2]}"
                )

                # dump depth_float32 with first 10, middle 10, last 10 rows
                logger.info(
                    f"first 10 rows of depth_float32: {depth_float32[:10]}")
                logger.info(
                    f"middle 10 rows of depth_float32: {depth_float32[depth.shape[0]//2-5:depth.shape[0]//2+5]}")
                logger.info(
                    f"last 10 rows of depth_float32: {depth_float32[-10:]}")

                return (
                    depth_float32.tobytes(),
                    depth.shape[1],
                    depth.shape[0],
                    min_depth,
                    max_depth,
                )

        except Exception as e:
            logger.error(f"error getting depth image: {e}")
            return b"", 0, 0, 0.0, 0.0
