import sys
import os
import numpy as np
import cv2
import time
import math

# Add the project root to Python path
PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
sys.path.insert(0, PROJECT_ROOT)
sys.path.append("./simulator/genesis")

from .vision import (
    SimulatorRGBGetter,
    SimulatorRGBDGetter,
    SimulatorCameraInfoGetter,
)

# Import driver functions directly
from DeepEmbody.driver.sim_genesis_ranger.driver import (
    get_rgb_image as driver_get_rgb_image,
    get_depth_image as driver_get_depth_image,
    get_pose as driver_get_pose,
    save_rgb_image as driver_save_rgb_image,
    save_depth_image as driver_save_depth_image,
)


def _get_eaios_decorator():
    """Get the eaios decorator if available"""
    try:
        from manager.eaios_decorators import eaios

        return eaios
    except ImportError:
        # Fallback decorator if eaios is not available
        def fallback_decorator(func):
            return func

        return fallback_decorator


eaios = _get_eaios_decorator()


@eaios.api
def c_camera_rgb(camera_name="robot_camera", timeout_sec=5.0):
    """
    Get the color image (OpenCV format) from the simulator camera using driver functions.
    Args:
        camera_name: Camera name (e.g., 'robot_camera') - for simulator, this is ignored
        timeout_sec: Timeout in seconds to wait for the image (default: 5.0 seconds)
    Returns:
        Returns the image in OpenCV format if successful, or None if timeout occurs.
    """
    try:
        getter = SimulatorRGBGetter()
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            image = getter.get_rgb_image()
            if image is not None:
                return image
            time.sleep(0.1)

        print(f"[sim_vision] Timeout getting RGB image after {timeout_sec} seconds")
        return None

    except Exception as e:
        print(f"[sim_vision] Error in s_camera_rgb: {e}")
        return None


@eaios.api
def c_camera_dep_rgb(camera_name="robot_camera", timeout_sec=5.0):
    """
    Get the RGB and depth images (with the same timestamp) from the simulator camera using driver functions.
    Args:
        camera_name: Camera name (e.g., 'robot_camera') - for simulator, this is ignored
        timeout_sec: Timeout in seconds to wait for the images (default: 5.0 seconds)
    Returns:
        Returns a tuple (rgb_image, depth_image) in OpenCV format if successful, or (None, None) if timeout occurs.
    """
    try:
        getter = SimulatorRGBDGetter()
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            rgb_image, depth_image = getter.get_rgbd_images()
            if rgb_image is not None and depth_image is not None:
                return rgb_image, depth_image
            time.sleep(0.1)

        print(f"[sim_vision] Timeout getting RGBD images after {timeout_sec} seconds")
        return None, None

    except Exception as e:
        print(f"[sim_vision] Error in s_camera_dep_rgb: {e}")
        return None, None


@eaios.api
def c_camera_info(camera_name="robot_camera", timeout_sec=5.0) -> dict:
    """
    Get parameter matrix for the simulator camera.
    Args:
        camera_name: Camera name (e.g., 'robot_camera') - for simulator, this is ignored
        timeout_sec: Timeout in seconds to wait for the camera info (default: 5.0 seconds)
    Returns:
        Dict: {
            'k': float64[9],      # 3x3 相机内参矩阵
            'p': float64[12],     # 3x4 投影矩阵
            'd': float64[],       # 畸变系数数组
            'r': float64[9],      # 3x3 旋转矩阵
            'width': uint32,      # 图像宽度
            'height': uint32,     # 图像高度
            'roi': RegionOfInterest  # 感兴趣区域
        }
        Returns the camera info dictionary if successful, or None if timeout occurs.
    """
    try:
        getter = SimulatorCameraInfoGetter()
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            camera_info = getter.get_camera_info()
            if camera_info is not None:
                return camera_info
            time.sleep(0.1)

        print(f"[sim_vision] Timeout getting camera info after {timeout_sec} seconds")
        return None

    except Exception as e:
        print(f"[sim_vision] Error in s_camera_info: {e}")
        return None


@eaios.api
def c_save_rgb_image(filename, camera_name="robot_camera", width=None, height=None):
    """
    Capture and save RGB image from simulator camera to file using driver functions.
    Args:
        filename: Output filename (should include extension like .jpg or .png)
        camera_name: Camera name (e.g., 'robot_camera') - for simulator, this is ignored
        width: Desired image width (optional)
        height: Desired image height (optional)
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Call driver function directly
        driver_save_rgb_image(filename, width=width, height=height)
        return {"success": True}

    except Exception as e:
        print(f"[sim_vision] Error saving RGB image: {e}")
        return {"success": False}


@eaios.api
def c_save_depth_image(filename, camera_name="robot_camera", width=None, height=None):
    """
    Capture and save depth image from simulator camera to file using driver functions.
    Args:
        filename: Output filename (should include extension like .npy or .png)
        camera_name: Camera name (e.g., 'robot_camera') - for simulator, this is ignored
        width: Desired image width (optional)
        height: Desired image height (optional)
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Call driver function directly
        driver_save_depth_image(filename, width=width, height=height)
        return {"success": True}

    except Exception as e:
        print(f"[sim_vision] Error saving depth image: {e}")
        return {"success": False}


@eaios.api
def c_get_robot_pose(timeout_sec=5.0):
    """
    Get the current pose of the robot in the simulator using driver functions.
    Args:
        timeout_sec: Timeout in seconds to wait for the pose (default: 5.0 seconds)
    Returns:
        dict: Robot pose {'x': float, 'y': float, 'z': float, 'yaw': float} or None if failed
    """
    try:
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            try:
                x, y, z, yaw = driver_get_pose()
                return {"x": x, "y": y, "z": z, "yaw": yaw}
            except Exception as e:
                print(f"[sim_vision] Error getting pose: {e}")
                time.sleep(0.1)

        print(f"[sim_vision] Timeout getting robot pose after {timeout_sec} seconds")
        return None

    except Exception as e:
        print(f"[sim_vision] Error in s_get_robot_pose: {e}")
        return None


@eaios.api
def c_tf_transform(from_frame: str, to_frame: str, x: float, y: float, z: float) -> tuple:
    """
    Transform coordinates from one frame to another.
    Currently supports transformation from 'camera_link' to 'map' frame.
    
    Args:
        from_frame: Source frame name (e.g., 'camera_link')
        to_frame: Target frame name (e.g., 'map')
        x: X coordinate in source frame
        y: Y coordinate in source frame  
        z: Z coordinate in source frame
        
    Returns:
        tuple: (x, y, z) coordinates in target frame
    """
    try:
        if from_frame == 'camera_link' and to_frame == 'map':
            # Get current robot pose
            robot_pose = c_get_robot_pose()
            if robot_pose is None:
                print("[sim_vision] Failed to get robot pose for coordinate transformation")
                return x, y, z
            
            robot_x = robot_pose['x']
            robot_y = robot_pose['y']
            robot_z = robot_pose['z']
            robot_yaw = robot_pose['yaw']
            
            # Convert yaw from degrees to radians
            yaw_rad = math.radians(robot_yaw)
            
            # Camera is fixed at the front of the robot
            # Transform from camera coordinates to robot coordinates
            # Camera coordinate system: x forward, y left, z up
            # Robot coordinate system: x forward, y left, z up
            # The camera is mounted at the front of the robot, so we need to add the camera offset
            
            # Camera offset from robot center (assuming camera is mounted at front)
            camera_offset_x = 0.3  # 30cm forward from robot center
            camera_offset_y = 0.0  # No lateral offset
            camera_offset_z = 0.2  # 20cm above robot center
            
            # Apply camera offset to get camera position in robot frame
            camera_x_in_robot = camera_offset_x
            camera_y_in_robot = camera_offset_y
            camera_z_in_robot = camera_offset_z
            
            # Transform from camera coordinates to robot coordinates
            # The camera coordinates are already in the camera frame
            # We just need to add the camera position in robot frame
            robot_x_coord = x + camera_x_in_robot
            robot_y_coord = y + camera_y_in_robot
            robot_z_coord = z + camera_z_in_robot
            
            # Transform from robot coordinates to global map coordinates
            # Apply rotation and translation
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            
            map_x = robot_x + robot_x_coord * cos_yaw - robot_y_coord * sin_yaw
            map_y = robot_y + robot_x_coord * sin_yaw + robot_y_coord * cos_yaw
            map_z = robot_z + robot_z_coord
            
            return map_x, map_y, map_z
            
        else:
            print(f"[sim_vision] Unsupported frame transformation: {from_frame} -> {to_frame}")
            return x, y, z
            
    except Exception as e:
        print(f"[sim_vision] Error in c_tf_transform: {e}")
        return x, y, z


# Example usage functions
def test_vision_api():
    """Test function to demonstrate the vision API usage"""
    print("[sim_vision] Testing vision API...")

    # Test RGB image capture
    print("[sim_vision] Testing RGB image capture...")
    rgb_image = c_camera_rgb()
    if rgb_image is not None:
        print(f"[sim_vision] RGB image shape: {rgb_image.shape}")
        c_save_rgb_image("test_rgb.jpg")

    # Test RGBD image capture
    print("[sim_vision] Testing RGBD image capture...")
    rgb_image, depth_image = c_camera_dep_rgb()
    if rgb_image is not None and depth_image is not None:
        print(f"[sim_vision] RGB image shape: {rgb_image.shape}")
        print(f"[sim_vision] Depth image shape: {depth_image.shape}")
        c_save_depth_image("test_depth.npy")
        c_save_depth_image("test_depth_vis.png")

    # Test camera info
    print("[sim_vision] Testing camera info...")
    camera_info = c_camera_info()
    if camera_info is not None:
        print(f"[sim_vision] Camera info: {camera_info}")

    # Test robot pose
    print("[sim_vision] Testing robot pose...")
    pose = c_get_robot_pose()
    if pose is not None:
        print(f"[sim_vision] Robot pose: {pose}")

    print("[sim_vision] Vision API test completed!")


if __name__ == "__main__":
    test_vision_api()
