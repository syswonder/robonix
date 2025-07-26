import rclpy
from manager.eaios_decorators import eaios
from .vision import CameraImageGetter, CameraRGBDGetter, CameraInfoGetter
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import numpy as np

@eaios.api
def c_camera_rgb(camera_name, timeout_sec=5.0) -> np.ndarray:
    """
    Get the color image (OpenCV format) from the specified camera.
    Args:
        camera_name: Camera name (e.g., 'camera')
        timeout_sec: Timeout in seconds to wait for the image (default: 5.0 seconds)
    Returns:
        Returns the image in OpenCV format if successful, or None if timeout occurs.
    """
    rclpy.init()
    topic = f"/{camera_name}/color/image"
    node = CameraImageGetter(topic)
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while rclpy.ok() and node.image is None and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=1.0)
        if node._event.is_set():
            break
    result = node.image
    node.destroy_node()
    rclpy.shutdown()
    return result

@eaios.api
def c_camera_dep_rgb(camera_name, timeout_sec=5.0) -> tuple[np.ndarray, np.ndarray]:
    """
    Get the RGB and depth images (with the same timestamp) from the specified camera.
    Args:
        camera_name: Camera name (e.g., 'camera')
        timeout_sec: Timeout in seconds to wait for the images (default: 5.0 seconds)
    Returns:
        Returns a tuple (rgb_image, depth_image) in OpenCV format if successful, or (None, None) if timeout occurs.
    """
    rclpy.init()
    rgb_topic = f"/{camera_name}/color/image"
    depth_topic = f"/{camera_name}/aligned_depth_to_color/image"
    node = CameraRGBDGetter(rgb_topic, depth_topic)
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while rclpy.ok() and (node.rgb_image is None or node.depth_image is None) and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=1.0)
        if node._event.is_set():
            break
    result = (node.rgb_image, node.depth_image)
    node.destroy_node()
    rclpy.shutdown()
    return result

@eaios.api
def c_camera_info(camera_name, timeout_sec=5.0) -> dict:
    """
    Get parameter matrix for the specified camera.
    Args:
        camera_name: Camera name (e.g., 'camera')
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
    rclpy.init()
    topic = f"/{camera_name}/color/camera_info"
    node = CameraInfoGetter(topic)
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while rclpy.ok() and node.camera_info is None and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=1.0)
        if node._event.is_set():
            break
    result = node.camera_info
    node.destroy_node()
    rclpy.shutdown()
    return result

@eaios.api
def c_tf_transform(source_frame, target_frame, x, y, z, timeout_sec=10.0) -> tuple:
    """
    In a ROS environment, perform coordinate system transformation, converting the coordinates under source_frame in the input parameters to coordinates under target_frame.
    Args:
        source_frame: Source coordinate frame name (e.g., 'camera_link')
        target_frame: Target coordinate frame name (e.g., 'map')
        x: X coordinate in source frame (float)
        y: Y coordinate in source frame (float)
        z: Z coordinate in source frame (float)
        timeout_sec: Timeout in seconds to wait for the transformation (default: 10.0 seconds)
    Returns:
        Returns a tuple (x, y, z) representing the transformed 3D coordinate in target frame, or (None, None, None) if transformation fails or timeout occurs.
    """
    rclpy.init()
    
    # Create a simple node for TF operations
    node = rclpy.create_node('tf_transform_node')
    
    # Initialize TF buffer and listener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    # Create PointStamped message with source coordinates
    point_stamped = PointStamped()
    point_stamped.header.frame_id = source_frame
    point_stamped.header.stamp = node.get_clock().now().to_msg()
    point_stamped.point.x = x
    point_stamped.point.y = y
    point_stamped.point.z = z
    
    # Wait for transformation to be available
    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    transform = None
    
    while rclpy.ok() and transform is None and node.get_clock().now().nanoseconds < end_time:
        try:
            # Check if transformation is available
            if tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=1.0)):
                transform = tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                break
        except Exception:
            pass
        rclpy.spin_once(node, timeout_sec=0.1)
    
    result = (None, None, None)
    if transform is not None:
        try:
            # Perform the transformation
            transformed_point = do_transform_point(point_stamped, transform)
            result = (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
        except Exception as e:
            node.get_logger().error(f"Transformation failed: {str(e)}")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    return result
