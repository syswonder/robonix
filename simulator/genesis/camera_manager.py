# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import threading
import time
import os
import cv2
import numpy as np

# Try to import uapi.log, fallback to standard logging if failed
try:
    from uapi.log import logger
except ImportError:
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)


class CameraManager:
    def __init__(self, camera, car, scene_lock, output_dir="cam_output", capture_interval=1.0):
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
        self.camera_thread = threading.Thread(
            target=self._camera_update_loop
        )
        self.camera_thread.daemon = True
        self.camera_thread.start()
        logger.info(f"camera thread started, saving images to {self.output_dir} every {self.capture_interval}s")
        logger.info(f"camera thread ID: {self.camera_thread.ident}, alive: {self.camera_thread.is_alive()}")
        
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
            success = cv2.imwrite(filepath, rgb_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
            
            if success:
                self.image_counter += 1
                logger.info(f"rgb image saved: {filepath}")
                
                # Log every 10 images for testing
                if self.image_counter % 10 == 0:
                    logger.info(f"saved {self.image_counter} rgb images total")
            else:
                logger.error(f"failed to save rgb image: {filepath}")
                
        except Exception as e:
            logger.error(f"error saving rgb image: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
            
    def _save_depth_image(self, depth):
        """Save depth image"""
        try:
            # Normalize depth values to 0-255 range
            depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_uint8 = depth_normalized.astype(np.uint8)
            
            # Apply colormap for better visualization
            depth_colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
            
            # Use fixed filename (overwrite mode)
            filename = "depth_current.png"
            filepath = os.path.join(self.output_dir, filename)
            
            # Save image
            success = cv2.imwrite(filepath, depth_colored)
            
            if success:
                logger.info(f"depth image saved: {filepath}")
            else:
                logger.error(f"failed to save depth image: {filepath}")
            
        except Exception as e:
            logger.error(f"error saving depth image: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
            
    def get_rgb_image(self, width=800, height=600):
        """Get RGB image data (for gRPC service)"""
        try:
            with self.scene_lock:
                rgb, _, _, _ = self.camera.render(
                    depth=True, segmentation=True, normal=True
                )
                
                # Resize image
                if width > 0 and height > 0:
                    rgb = cv2.resize(rgb, (width, height))
                    
                # Convert to JPEG format
                rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, jpeg_data = cv2.imencode(".jpg", rgb_bgr, encode_param)
                
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
                    depth = cv2.resize(depth, (width, height))
                    
                # Convert to float32 array
                depth_float32 = depth.astype(np.float32)
                min_depth = float(np.min(depth_float32))
                max_depth = float(np.max(depth_float32))
                
                return depth_float32.tobytes(), depth.shape[1], depth.shape[0], min_depth, max_depth
                
        except Exception as e:
            logger.error(f"error getting depth image: {e}")
            return b"", 0, 0, 0.0, 0.0
