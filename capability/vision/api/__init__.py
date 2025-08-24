import rclpy
import atexit

# Global ROS2 initialization with safety check
def _ensure_rclpy_initialized():
    """Ensure rclpy is initialized, but don't reinitialize if already done."""
    try:
        # Check if rclpy is already initialized
        rclpy.get_default_context()
    except RuntimeError:
        # rclpy is not initialized, so initialize it
        rclpy.init()

# ROS2 shutdown function
def _ensure_rclpy_shutdown():
    """Ensure rclpy is properly shut down."""
    try:
        # Check if rclpy is still initialized
        rclpy.get_default_context()
        # If we get here, rclpy is still initialized, so shut it down
        rclpy.shutdown()
    except RuntimeError:
        # rclpy is already shut down, nothing to do
        pass

# Initialize ROS2 when package is imported
_ensure_rclpy_initialized()

# Register shutdown function to be called at program exit
atexit.register(_ensure_rclpy_shutdown)

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
