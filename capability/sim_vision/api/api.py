import sys
import os
import time
import math
import numpy as np

# Add the project root to Python path
PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
sys.path.insert(0, PROJECT_ROOT)
sys.path.append("./simulator/genesis")

from DeepEmbody.manager.eaios_decorators import eaios

from .vision import (
    SimulatorRGBGetter,
    SimulatorRGBDGetter,
    SimulatorCameraInfoGetter,
)

# Import driver functions directly
from DeepEmbody.driver.sim_genesis_ranger.driver import (
    get_pose as driver_get_pose,
    save_rgb_image as driver_save_rgb_image,
    save_depth_image as driver_save_depth_image,
)

@eaios.api
def cap_camera_rgb(camera_name="robot_camera", timeout_sec=5.0):
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

        print(f"[sim_vision] timeout getting RGB image after {timeout_sec} seconds")
        return None

    except Exception as e:
        print(f"[sim_vision] error in skl_camera_rgb: {e}")
        return None


@eaios.api
def cap_camera_dep_rgb(camera_name="robot_camera", timeout_sec=5.0):
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

        print(f"[sim_vision] timeout getting RGBD images after {timeout_sec} seconds")
        return None, None

    except Exception as e:
        print(f"[sim_vision] error in skl_camera_dep_rgb: {e}")
        return None, None


@eaios.api
def cap_camera_info(camera_name="robot_camera", timeout_sec=5.0) -> dict:
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

        print(f"[sim_vision] timeout getting camera info after {timeout_sec} seconds")
        return None

    except Exception as e:
        print(f"[sim_vision] error in skl_camera_info: {e}")
        return None


@eaios.api
def cap_save_rgb_image(filename, camera_name="robot_camera", width=None, height=None):
    """
    Capture and save RGB image from simulator camera to file using driver functions.
    Args:
        filename: Output filename (should include extension like .jpg or .png)
        camera_name: Camera name (e.g., 'robot_camera') - for simulator, this is ignored
        width: Desired image width (optional)
        height: Desired image height (optional)
    Returns:
        dict: {"success": True} if successful, {"success": False} otherwise
    """
    try:
        # Call driver function directly
        driver_save_rgb_image(filename, width=width, height=height)
        return {"success": True}

    except Exception as e:
        print(f"[sim_vision] error saving RGB image: {e}")
        return {"success": False}


@eaios.api
def cap_save_depth_image(filename, camera_name="robot_camera", width=None, height=None):
    """
    Capture and save depth image from simulator camera to file using driver functions.
    Args:
        filename: Output filename (should include extension like .npy or .png)
        camera_name: Camera name (e.g., 'robot_camera') - for simulator, this is ignored
        width: Desired image width (optional)
        height: Desired image height (optional)
    Returns:
        dict: {"success": True} if successful, {"success": False} otherwise
    """
    try:
        # Call driver function directly
        driver_save_depth_image(filename, width=width, height=height)
        return {"success": True}

    except Exception as e:
        print(f"[sim_vision] error saving depth image: {e}")
        return {"success": False}


@eaios.api
def cap_get_robot_pose(timeout_sec=5.0):
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
                print(f"[sim_vision] error getting pose: {e}")
                time.sleep(0.1)

        print(f"[sim_vision] timeout getting robot pose after {timeout_sec} seconds")
        return None

    except Exception as e:
        print(f"[sim_vision] error getting robot pose: {e}")
        return None


@eaios.api
def cap_get_object_global_pos(
    pixel_x: float, 
    pixel_y: float, 
    depth: float, 
    camera_info: dict,
    robot_pose: dict
) -> tuple:
    """
    Calculate the global position of an object based on:
    1. Robot's global position (x, y, z, yaw)
    2. Object's pixel coordinates and depth
    3. Camera intrinsic parameters
    
    Args:
        pixel_x: X coordinate in pixel (0 = left, width = right)
        pixel_y: Y coordinate in pixel (0 = top, height = bottom)  
        depth: Depth in meters
        camera_info: Camera intrinsic parameters (K matrix, D coefficients)
        robot_pose: Robot pose {'x': float, 'y': float, 'z': float, 'yaw': float}
        
    Returns:
        tuple: (global_x, global_y, global_z) in map frame
    """
    try:
        # Get robot pose components
        robot_x = robot_pose['x']
        robot_y = robot_pose['y']
        robot_z = robot_pose['z']
        robot_yaw = robot_pose['yaw']
        
        # Convert yaw from degrees to radians
        yaw_rad = math.radians(robot_yaw)
        
        # Camera offset from robot center (assuming camera is mounted at front)
        camera_offset_x = 0.3  # 30cm forward from robot center
        camera_offset_y = 0.0  # No lateral offset
        camera_offset_z = 0.2  # 20cm above robot center
        
        # Simplified coordinate calculation: only consider horizontal offset
        # 1. Get robot position and yaw
        # 2. Calculate object direction vector based on depth and pixel offset
        # 3. Add vector to robot position to get object global coordinates
        
        # Calculate pixel offset from image center (camera is pointing forward)
        image_center_x = camera_info["width"] / 2
        image_center_y = camera_info["height"] / 2
        
        # Pixel offset from center (positive = right, negative = left)
        pixel_offset_x = pixel_x - image_center_x
        pixel_offset_y = pixel_y - image_center_y
        
        # Convert pixel offset to world offset using depth and camera parameters
        # Use focal length to convert pixel offset to angle, then to world distance
        fx = camera_info["k"][0]  # focal length x
        fy = camera_info["k"][4]  # focal length y
        
        # Calculate horizontal angle offset (approximate)
        angle_offset_x = math.atan2(pixel_offset_x, fx)  # left-right angle
        angle_offset_y = math.atan2(pixel_offset_y, fy)  # up-down angle (ignored for z)
        
        # Calculate horizontal world offset based on depth
        world_offset_x = depth * math.tan(angle_offset_x)  # left-right offset
        world_offset_y = depth * math.tan(angle_offset_y)  # up-down offset (ignored)
        
        # Construct object direction vector relative to robot
        # When yaw=0° (robot facing +y direction):
        # - depth goes in +y direction (forward)
        # - world_offset_x goes in -x direction (left)
        # - world_offset_y goes in +z direction (up, ignored)
        
        # Transform to global coordinates based on robot yaw
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        
        # Object position = robot position + depth vector + horizontal offset vector
        global_x = robot_x + world_offset_x * cos_yaw - depth * sin_yaw
        global_y = robot_y + depth * cos_yaw + world_offset_x * sin_yaw
        global_z = robot_z + camera_offset_z  # Same height as camera
        
        print(f"[sim_vision] simplified object global position calculation:")
        print(f"  robot pose: {robot_pose}")
        print(f"  pixel offset: ({pixel_offset_x:.1f}, {pixel_offset_y:.1f})")
        print(f"  world offset: ({world_offset_x:.3f}, {world_offset_y:.3f})")
        print(f"  object global: ({global_x:.3f}, {global_y:.3f}, {global_z:.3f})")
        
        return global_x, global_y, global_z
        
    except Exception as e:
        print(f"[sim_vision] error calculating object global position: {e}")
        return None, None, None


def _pixel_to_camera_coords(pixel_x: float, pixel_y: float, depth: float, K: list, D: list) -> tuple:
    """
    Convert pixel coordinates to camera frame coordinates using depth and camera intrinsics.
    
    Args:
        pixel_x: X coordinate in pixel (0 = left, width = right)
        pixel_y: Y coordinate in pixel (0 = top, height = bottom)
        depth: Depth in meters
        K: Camera intrinsic matrix (3x3) as list
        D: Distortion coefficients as list
        
    Returns:
        tuple: (camera_x, camera_y) in camera frame (meters)
    """
    try:
        # Convert K matrix from list to numpy array if needed
        if isinstance(K, list):
            K = np.array(K).reshape(3, 3)
        
        # Extract camera parameters
        fx = K[0, 0]  # focal length x
        fy = K[1, 1]  # focal length y
        cx = K[0, 2]  # principal point x
        cy = K[1, 2]  # principal point y
        
        # Convert pixel coordinates to normalized coordinates
        # (u - cx) / fx = X / Z, (v - cy) / fy = Y / Z
        # Since we know Z (depth), we can calculate X and Y
        normalized_x = (pixel_x - cx) / fx
        normalized_y = (pixel_y - cy) / fy
        
        # Convert to camera frame coordinates using depth
        camera_x = normalized_x * depth
        camera_y = normalized_y * depth
        
        # Note: We're ignoring distortion correction for simplicity
        # In a more accurate implementation, you would apply D coefficients
        
        return camera_x, camera_y
        
    except Exception as e:
        print(f"[sim_vision] error in _pixel_to_camera_coords: {e}")
        return 0.0, 0.0



# Example usage functions
def test_vision_api():
    """Test function to demonstrate the vision API usage"""
    print("[sim_vision] testing vision API...")

    # Test RGB image capture
    print("[sim_vision] testing RGB image capture...")
    rgb_image = cap_camera_rgb()
    if rgb_image is not None:
        print(f"[sim_vision] RGB image shape: {rgb_image.shape}")
        cap_save_rgb_image("test_rgb.jpg")

    # Test RGBD image capture
    print("[sim_vision] testing RGBD image capture...")
    rgb_image, depth_image = cap_camera_dep_rgb()
    if rgb_image is not None and depth_image is not None:
        print(f"[sim_vision] RGB image shape: {rgb_image.shape}")
        print(f"[sim_vision] Depth image shape: {depth_image.shape}")
        cap_save_depth_image("test_depth.npy")
        cap_save_depth_image("test_depth_vis.png")

    # Test camera info
    print("[sim_vision] testing camera info...")
    camera_info = cap_camera_info()
    if camera_info is not None:
        print(f"[sim_vision] Camera info: {camera_info}")

    # Test robot pose
    print("[sim_vision] testing robot pose...")
    pose = cap_get_robot_pose()
    if pose is not None:
        print(f"[sim_vision] robot pose: {pose}")

    print("[sim_vision] vision API test completed!")


if __name__ == "__main__":
    test_vision_api()
