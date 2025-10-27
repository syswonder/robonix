# robonix Capability Package

import rclpy
import atexit

# Global ROS2 initialization with safety check
def _ensure_rclpy_initialized():
    """Ensure rclpy is initialized, but don't reinitialize if already done."""
    if rclpy.ok():
        return
    rclpy.init()

# ROS2 shutdown function
def _ensure_rclpy_shutdown():
    """Ensure rclpy is properly shut down."""
    if rclpy.ok():
        rclpy.shutdown()

# Initialize ROS2 when package is imported
_ensure_rclpy_initialized()

# Register shutdown function to be called at program exit
atexit.register(_ensure_rclpy_shutdown)