import sys
import os
import numpy as np
import cv2
import time

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
        return True

    except Exception as e:
        print(f"[sim_vision] Error saving RGB image: {e}")
        return False


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
        return True

    except Exception as e:
        print(f"[sim_vision] Error saving depth image: {e}")
        return False


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
