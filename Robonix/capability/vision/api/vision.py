import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import threading
import message_filters

class CameraImageGetter(Node):
    def __init__(self, topic_name):
        super().__init__('camera_image_getter')
        self.image = None
        self._event = threading.Event()
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        self.image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("Got camera image.")
        self._event.set()
        self.destroy_subscription(self.subscription)

class CameraRGBDGetter(Node):
    def __init__(self, rgb_topic, depth_topic):
        super().__init__('camera_rgbd_getter')
        self.rgb_image = None
        self.depth_image = None
        self._event = threading.Event()
        self.cv_bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 3, 0.02)
        self.sync.registerCallback(self.callback)

    def callback(self, rgb_msg, depth_msg):
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        self.get_logger().info("Got synchronized RGB and depth images.")
        self._event.set()
        self.destroy_subscription(self.rgb_sub.sub)
        self.destroy_subscription(self.depth_sub.sub)

class CameraInfoGetter(Node):
    def __init__(self, topic_name):
        super().__init__('camera_info_getter')
        self.camera_info = None
        self._event = threading.Event()
        self.subscription = self.create_subscription(
            CameraInfo,
            topic_name,
            self.camera_info_callback,
            10
        )

    def camera_info_callback(self, msg: CameraInfo):
        """
        Get camera parameters through a callback function.
        """
        self.camera_info = {
            'k': msg.k,           # 3x3 相机内参矩阵
            'p': msg.p,           # 3x4 投影矩阵
            'd': msg.d,           # 畸变系数数组
            'r': msg.r,           # 3x3 旋转矩阵
            'width': msg.width,   # 图像宽度
            'height': msg.height, # 图像高度
            'roi': msg.roi        # 感兴趣区域
        }
        self.get_logger().info("Got camera info.")
        self._event.set()
        self.destroy_subscription(self.subscription)
