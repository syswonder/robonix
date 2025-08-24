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
    cap_camera_rgb,
    cap_camera_dep_rgb,
    cap_camera_info,
    cap_tf_transform
)

__all__ = [
    'cap_camera_rgb',
    'cap_camera_dep_rgb', 
    'cap_camera_info',
    'cap_tf_transform'
]
