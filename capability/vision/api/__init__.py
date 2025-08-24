import rclpy

# Global ROS2 initialization with safety check
def _ensure_rclpy_initialized():
    """Ensure rclpy is initialized, but don't reinitialize if already done."""
    try:
        # Check if rclpy is already initialized
        rclpy.get_default_context()
    except RuntimeError:
        # rclpy is not initialized, so initialize it
        rclpy.init()

# Initialize ROS2 when package is imported
_ensure_rclpy_initialized()

# Import all API functions to make them available when the package is imported
from .api import (
    c_camera_rgb,
    c_camera_dep_rgb,
    c_camera_info,
    c_tf_transform
)

__all__ = [
    'c_camera_rgb',
    'c_camera_dep_rgb', 
    'c_camera_info',
    'c_tf_transform'
]
