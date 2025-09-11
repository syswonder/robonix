import sys
import os
import numpy as np
import cv2
import time
import threading

# Add the project root to Python path
PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
sys.path.insert(0, PROJECT_ROOT)
sys.path.append("./simulator/genesis")

# Import driver functions directly
from Robonix.driver.sim_genesis_ranger.driver import (
    get_rgb_image as driver_get_rgb_image,
    get_depth_image as driver_get_depth_image,
    get_pose as driver_get_pose,
)


class SimulatorCameraGetter:
    """Base class for getting camera data from simulator via driver functions"""

    def __init__(self):
        self._lock = threading.Lock()
        print("[sim_vision] initialized camera getter, this class type is " + str(type(self)))

    def _check_connection(self):
        """Check if driver is connected to simulator"""
        try:
            # Try to get pose to check connection
            driver_get_pose()
            return True
        except Exception as e:
            print(f"[sim_vision] connection check failed: {e}")
            return False


class SimulatorRGBGetter(SimulatorCameraGetter):
    """Get RGB images from simulator camera using driver functions"""

    def get_rgb_image(self, width=None, height=None, timeout_sec=5.0):
        """
        Get RGB image from simulator camera using driver function

        Args:
            width (int, optional): Desired image width
            height (int, optional): Desired image height
            timeout_sec (float): Timeout in seconds

        Returns:
            numpy.ndarray: RGB image array (H, W, 3) or None if failed
        """
        with self._lock:
            try:
                if not self._check_connection():
                    return None

                # Call driver function directly
                image, timestamp = driver_get_rgb_image(width=width, height=height)
                return image

            except Exception as e:
                print(f"[sim_vision] error getting RGB image: {e}")
                return None


class SimulatorRGBDGetter(SimulatorCameraGetter):
    """Get synchronized RGB and depth images from simulator camera using driver functions"""

    def get_rgbd_images(self, width=None, height=None, timeout_sec=5.0):
        """
        Get synchronized RGB and depth images from simulator camera using driver functions

        Args:
            width (int, optional): Desired image width
            height (int, optional): Desired image height
            timeout_sec (float): Timeout in seconds

        Returns:
            tuple: (rgb_image, depth_image) or (None, None) if failed
        """
        with self._lock:
            try:
                if not self._check_connection():
                    return None, None

                # Call driver functions directly
                rgb_image, rgb_timestamp = driver_get_rgb_image(
                    width=width, height=height
                )
                depth_image, min_depth, max_depth, depth_timestamp = (
                    driver_get_depth_image(width=width, height=height)
                )

                return rgb_image, depth_image

            except Exception as e:
                print(f"[sim_vision] error getting RGBD images: {e}")
                return None, None


class SimulatorCameraInfoGetter(SimulatorCameraGetter):
    """Get camera information from simulator using driver functions"""

    def get_camera_info(self, timeout_sec=5.0):
        """
        Get camera information from simulator

        Args:
            timeout_sec (float): Timeout in seconds

        Returns:
            dict: Camera information or None if failed
        """
        with self._lock:
            try:
                if not self._check_connection():
                    return None

                # For simulator, we'll return default camera parameters
                # Based on actual image size from logs: 600x800 (height x width)
                # These parameters should match the actual camera setup
                camera_info = {
                    "k": [
                        800.0,  # fx - focal length x
                        0.0,    # 0
                        400.0,  # cx - principal point x (width/2 = 800/2 = 400)
                        0.0,    # 0
                        800.0,  # fy - focal length y  
                        300.0,  # cy - principal point y (height/2 = 600/2 = 300)
                        0.0,    # 0
                        0.0,    # 0
                        1.0,    # 1
                    ],
                    "p": [
                        800.0,  # fx, 0, cx, 0
                        0.0,
                        400.0,
                        0.0,
                        0.0,    # 0, fy, cy, 0
                        800.0,
                        300.0,
                        0.0,
                        0.0,    # 0, 0, 1, 0
                        0.0,
                        1.0,
                        0.0,
                    ],
                    "d": [0.0, 0.0, 0.0, 0.0, 0.0],  # No distortion
                    "r": [
                        1.0, 0.0, 0.0,  # Identity rotation matrix
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0,
                    ],
                    "width": 800,   # Image width
                    "height": 600,  # Image height
                    "roi": {"x_offset": 0, "y_offset": 0, "width": 800, "height": 600},
                }

                return camera_info

            except Exception as e:
                print(f"[sim_vision] error getting camera info: {e}")
                return None
