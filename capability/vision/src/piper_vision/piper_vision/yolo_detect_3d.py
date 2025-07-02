import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from piper_vision.cv_tool import px2xy
import os
from piper_msgs.msg import ObjectPos, AllObjectPos
from piper_msgs.srv import SetInterest
import message_filters
import queue
import time
import threading
from std_srvs.srv import SetBool
import numpy as np
import traceback 
from ultralytics import YOLOE
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

# Get the ROS distribution version and set the shared directory for Yolo configuration files.
ros_distribution = os.environ.get("ROS_DISTRO")
package_share_directory = get_package_share_directory('piper_vision')

# Create a ROS 2 Node class YoloRos2.
class YoloRos2(Node):
    def __init__(self):
        super().__init__('yolo_ros2')
        self.get_logger().info(f"Current ROS 2 distribution: {ros_distribution}")

        # Declare ROS parameters.
        self.declare_parameter("device", "cuda:0", ParameterDescriptor(
            name="device", description="Compute device selection, default: cpu, options: cuda:0"))

        self.declare_parameter("model", "yoloe-11l-seg-pf", ParameterDescriptor(
            name="model", description="Default model selection: yoloe-11l-seg-pf"))
        
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info", ParameterDescriptor(
            name="camera_info_topic", description="Camera information topic, default: /camera/color/camera_info"))
        
        self.declare_parameter("interest", "bottle", ParameterDescriptor(
            name="interest", description="The object to detect, default: bottle"))
        self.declare_parameter("depth_threshold", 2.0, ParameterDescriptor(
            name="depth_threshold", description="The depth threshold for detection, only valid when bg_removal is True. default: 2.0"
        ))
        self.declare_parameter("conf_threshold", 0.7, ParameterDescriptor(
            name="conf_threshold", description="The confidence threshold for detection, default: 0.7"
        ))
        self.declare_parameter("bg_removal", True, ParameterDescriptor(
            name="bg_removal", description="Whether to remove the background, default: True"
        ))
        self.declare_parameter("target_frame_id", "map", ParameterDescriptor(
            name="target_frame_id", description="The target frame id for tf, default: map"
        ))
        self.declare_parameter("tf_translation", [0.3, 0.0, 0.1, -1.5708, 0.0, -1.5708], ParameterDescriptor(
            name="tf_translation", description="The tf translation of the camera to base_link, format: [x, y, z, x_roll y_pitch z_yaw], default: [0.3, 0, 0.1, -1.5708, 0, -1.5708]"
        ))
        self.depth_threshold = self.get_parameter("depth_threshold").value * 1000 # convert to mm
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.interest = self.get_parameter('interest').value
        self.camera_info = {}
        self.device = self.get_parameter('device').value
        self.enable_bg_removal = self.get_parameter('bg_removal').value
        self.model = self.get_parameter('model').value
        self.target_frame_id = self.get_parameter('target_frame_id').value
        self.camera_frame_id = "camera_link"
        self.transform = None
        self.tf_translation = self.get_parameter('tf_translation').get_parameter_value().double_array_value


        if self.model == "yoloe-11l-seg":
            self.text_prompt = True
        else:
            self.text_prompt = False

        # 1. Load the model.
        model_path = package_share_directory + "/config/" + self.model + ".pt"
        self.yolo = YOLOE(model_path)

        # 2. Create publishers.
        self.camera_target_point_pub = self.create_publisher(ObjectPos, "/piper_vision/target_point", 10)
        # self.all_objects_pub = 
        self._pred_pub = self.create_publisher(Image, "/piper_vision/pred_image", 10)
        self.camera_all_objects_pub = self.create_publisher(AllObjectPos, "/piper_vision/all_object_points", 10)
        # 3. Create an image subscriber (subscribe to depth information for 3D cameras, load camera info for 2D cameras).
        self.image_queue = queue.Queue(maxsize=2)

        rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        # Synchronize timestamps
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback) # Execute callback function

        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 1)

        # 4. Image format conversion (using cv_bridge).
        self.cv_bridge = CvBridge()

        # TODO: send_request to shutdown ldp
        # self.ldp_client = self.create_client(SetBool, '/camera/set_ldp_enable')
        # while not self.ldp_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('/camera/set_ldp_enable service not available, waiting again...')
        # self.send_request(self.ldp_client, False)

        # tf
        self._broadcaster = StaticTransformBroadcaster(self)
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = "base_link"
        static_transform_stamped.child_frame_id = self.camera_frame_id

        static_transform_stamped.transform.translation.x = self.tf_translation[0]
        static_transform_stamped.transform.translation.y = self.tf_translation[1]
        static_transform_stamped.transform.translation.z = self.tf_translation[2]

        quat = tf_transformations.quaternion_from_euler(self.tf_translation[3], self.tf_translation[4], self.tf_translation[5])
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        self._broadcaster.sendTransform(static_transform_stamped)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("tf: camera_link to base_link is ready")
        self.interest_srv = self.create_service(SetInterest, '/set_interest', self.interest_callback)
        threading.Thread(target=self.yolo_main, daemon=True).start()

        # map.txt
        self.f = None
        # self.f = open("map.txt", "w")  # 可换为 "a" 追加模式
        self.written_names = set()

        self.get_logger().info("YoloRos2 node init.")

    def write_unique_point(self, name, point):
        """写入 name: (x, y, z)，如果 name 是第一次出现"""
        if name not in self.written_names:
            self.written_names.add(name)
            self.f.write(f"{name}: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f})\n")

    def interest_callback(self, request, response):
        self.interest = request.interest
        response.success = True
        self.get_logger().info(f"object detect interest changed to {self.interest}")
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
            
    def multi_callback(self, ros_rgb_image, ros_depth_image):
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((ros_rgb_image, ros_depth_image))

    def camera_info_callback(self, msg: CameraInfo):
        """
        Get camera parameters through a callback function.
        """
        self.camera_info['k'] = msg.k
        self.camera_info['p'] = msg.p
        self.camera_info['d'] = msg.d
        self.camera_info['r'] = msg.r
        self.camera_info['width'] = msg.width
        self.camera_info['height'] = msg.height
        self.camera_info['roi'] = msg.roi

        self.destroy_subscription(self.camera_info_sub)

    def yolo_main(self):
        while rclpy.ok():
            while self.tf_buffer.can_transform(self.target_frame_id,
                                    self.camera_frame_id,
                                    rclpy.time.Time(),
                                    timeout=Duration(seconds=1.0)) == False:
                self.get_logger().info(f"Waiting for tf: {self.camera_frame_id} to {self.target_frame_id}")
            
            self.transform = self.tf_buffer.lookup_transform(self.target_frame_id,
                                                self.camera_frame_id,
                                                rclpy.time.Time())
            try:
                ros_rgb_image, ros_depth_image= self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                self.get_logger().error("image_queue is empty")
                time.sleep(0.1)
                continue
            try:
                self.image_proc(ros_rgb_image, ros_depth_image)
            except Exception as e:
                error_msg = traceback.format_exc()  # This will include file name, line number, and call stack
                self.get_logger().error('yolo_main:' + error_msg)
            time.sleep(0.5)
    
    def bg_removal(self, color_img_msg: Image, depth_img_msg: Image, enable_bg_removal: bool):
        """
        parameters 
        color_img_msg: Message class 'Image'--RGB image
        depth_img_msg: Message class 'Image'--Z16 image
        
        returns 
        Backgroung removed image

        Works by removing background in original image
        """
        if color_img_msg is None or depth_img_msg is None:
            self.get_logger().error("bg_removal: Background removal error, color or depth msg was None")
            return None, None, None
        
        # Convert color image msg to opencv gbr8 image
        cv_color_image = self.cv_bridge.imgmsg_to_cv2(color_img_msg, desired_encoding='bgr8')
        np_color_image = np.array(cv_color_image, dtype=np.uint8)

        # Convert depth image msg
        cv_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding='passthrough')
        np_depth_image = np.array(cv_depth_image, dtype=np.uint16)

        # background removal
        grey_color = 153
        # Convert depth image to 3 channel, so the shape of depth_image_3d is (height, width, 3)
        depth_image_3d = np.dstack((np_depth_image, np_depth_image, np_depth_image)) # depth image is 1 channel, color is 3 channels
        if enable_bg_removal == True:
            bg_removed = np.where((depth_image_3d > self.depth_threshold) | (depth_image_3d != depth_image_3d) | (depth_image_3d == 0), grey_color, np_color_image)
        else :
            bg_removed = np_color_image
        return bg_removed, np_color_image, np_depth_image
    
    def remove_mask_outliers(self, data, lower_percentile=10, upper_percentile=90):
        arr = np.array(data)
        lower = np.percentile(arr, lower_percentile)
        upper = np.percentile(arr, upper_percentile)
        filtered = arr[(arr >= lower) & (arr <= upper)]
        return filtered.tolist()

    def cluster_select(self, num_list, threshold=1.5):
        """
        Remove outliers using IQR (Interquartile Range) method.

        Parameters:
            num_list: List of numbers.
            threshold: IQR multiplier (default 1.5, higher means stricter filtering).

        Returns:
            Filtered list without outliers.
        """
        if not num_list:
            return []

        q1 = np.percentile(num_list, 25)  # First quartile (25%)
        q3 = np.percentile(num_list, 75)  # Third quartile (75%)
        iqr = q3 - q1                     # Interquartile range (IQR)
        lower_bound = q1 - threshold * iqr
        upper_bound = q3 + threshold * iqr
        return [x for x in num_list if lower_bound <= x <= upper_bound]
    
    def get_mask_center_opencv(self, mask_points):
        """
        Input: mask_points (np.array) - Polygon point set, shape (n, 1, 2)
        Output: (cx, cy) - Center point coordinates
        """
        # Calculate moments
        M = cv2.moments(mask_points)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

    def get_mask_center(self, mask_points):
        """
        Input: mask_points (np.array) - Polygon point set, shape (n, 2)
        Output: (cx, cy) - Center point coordinates
        """
        # Calculate the mean of all points (geometric center)
        cx, cy = np.mean(mask_points, axis=0)
        return cx, cy
    
    def image_proc(self, ros_rgb_image: Image, ros_depth_image: Image):
        # 5. Detect and publish results.
        bg_removed, np_color_image, np_depth_image = self.bg_removal(ros_rgb_image, ros_depth_image, self.enable_bg_removal)
        if self.text_prompt == True:
            prompt_names = [self.interest]
            self.yolo.set_classes(prompt_names, self.yolo.get_text_pe(prompt_names))

        # Alias for the predict method, enabling the model instance to be callable for predictions.
        results = self.yolo(source=bg_removed, device=self.device)
        detection = results[0]

        if detection is None:
            self.get_logger().info("No object detected.")
            return
        
        # publish detection image
        pred_img = detection.plot()
        pred_image_msg = self.cv_bridge.cv2_to_imgmsg(pred_img, encoding='bgr8')
        self._pred_pub.publish(pred_image_msg)

        # Get number of objects in the scene
        object_boxes = detection.boxes.xyxy.cpu().numpy()
        n_objects = object_boxes.shape[0]
        masks = detection.masks.cpu()
        detection_class = detection.boxes.cls.cpu().numpy()
        detection_conf = detection.boxes.conf.cpu().numpy()
        
        all_object_pos = AllObjectPos()
        all_object_pos.header.frame_id = self.target_frame_id
        all_object_pos.header.stamp = self.get_clock().now().to_msg()
        all_object_pos.names = []
        all_object_pos.points = []
        all_object_pos.widths = []
        all_object_pos.heights = []

        for i in range(n_objects):
            name = detection.names[detection_class[i]]
            conf = detection_conf[i]

            # self.get_logger().info(f"image_proc: find {name}, conf is {conf}")
            # if name != self.interest and self.interest != "all": 
            #     continue
            if conf < self.conf_threshold:
                continue
            
            mask_points = masks.xy[i].reshape(-1, 1, 2).astype(np.int32)
            # get the centroid of the mask object
            center_x, center_y = self.get_mask_center_opencv(mask_points)

            single_selection_mask = np.array(masks.xy[i])
            # Output the mask to an image
            # cv2.imwrite("mask.png", np.array(masks.data[i]) * 255) 
            single_object_box = object_boxes[i]
            x1, y1, x2, y2 = single_object_box

            depths = []
            max_depth = 0
            min_depth = 10000
            # np_depth_image 高720, 宽1280
            for point in single_selection_mask:
                p_x = int(point[0])
                p_y = int(point[1])
                if 0 <= p_x < np_depth_image.shape[1] and 0 <= p_y < np_depth_image.shape[0]:
                    depth = np_depth_image.item(p_y, p_x)
                    if not (depth == 0 or depth != depth):
                        max_depth = max(max_depth, depth)
                        min_depth = min(min_depth, depth)
                        depths.append(depth)
                else :
                    self.get_logger().debug(f"Point ({p_y}, {p_x}) is out of bounds for depth image with shape {np_depth_image.shape}")

            # print(f"max depth is {max_depth}, min depth is {min_depth}")
            # remove outliers
            selected_depth = self.remove_mask_outliers(depths, lower_percentile=10, upper_percentile=70)  
            # selected_depth = self.cluster_select(selected_depth)
            # Calculate the maximum and minimum values of selected_depth
            # max_depth = max(selected_depth)
            # min_depth = min(selected_depth)
            # print(f"max depth is {max_depth}, min depth is {min_depth}")

            # Calculate the average value of selected_depth
            if len(selected_depth) > 0:
                avg_depth = sum(selected_depth) / len(selected_depth)
            else:
                avg_depth = 0
            # self.get_logger().info(f"depth len is {len(selected_depth)}")

            avg_depth = avg_depth / 1000 #convert to meter
            center_depth = np_depth_image[int(center_y), int(center_x)] / 1000
            if center_depth == 0 or center_depth != center_depth:
                center_depth = avg_depth

            world_x, world_y = px2xy(
                [center_x, center_y], self.camera_info["k"], self.camera_info["d"], center_depth)
            # Calculate the width and height of the object
            world_x1, world_y1 = px2xy(
                [x1, y1], self.camera_info["k"], self.camera_info["d"], avg_depth)
            world_x2, world_y2 = px2xy(
                [x2, y2], self.camera_info["k"], self.camera_info["d"], avg_depth)

            # 相机坐标系下的x、y、z轴和像素坐标系下的是不同的.
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'camera_link'
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.z = avg_depth
            point_stamped.point.x = world_x
            point_stamped.point.y = world_y
            point_in_base = do_transform_point(point_stamped, self.transform)

            all_object_pos.names.append(name)
            point = Point()
            point.x = point_in_base.point.x
            point.y = point_in_base.point.y
            point.z = point_in_base.point.z
            all_object_pos.points.append(point)
            all_object_pos.widths.append(abs(world_x2 - world_x1))
            all_object_pos.heights.append(abs(world_y2 - world_y1))

            # 输出检测到的物体到文件
            # for name, point in zip(all_object_pos.names, all_object_pos.points):
            #     self.write_unique_point(name, point)

            if name == self.interest: 
                object_pos = ObjectPos()
                object_pos.header.frame_id = "camera_link"
                object_pos.header.stamp = self.get_clock().now().to_msg()
                # Since the camera coordinate system is different from the robotic arm coordinate system, a transformation is required:
                object_pos.point.x = avg_depth
                object_pos.point.y = -world_x
                object_pos.point.z = -world_y

                object_pos.width = abs(world_x2 - world_x1)
                object_pos.height = abs(world_y2 - world_y1)
                self.camera_target_point_pub.publish(object_pos)
        
        self.camera_all_objects_pub.publish(all_object_pos)

def main():
    rclpy.init()
    node = YoloRos2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
