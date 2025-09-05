import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import struct

# ANSI color codes for colored output
class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'  # Reset to default color

def log_success(logger, message):
    logger.info(f"{Colors.GREEN}{Colors.BOLD}[SUCCESS]{Colors.END} {message}")

def log_error(logger, message):
    logger.error(f"{Colors.RED}{Colors.BOLD}[ERROR]{Colors.END} {message}")

def log_warning(logger, message):
    logger.warn(f"{Colors.YELLOW}{Colors.BOLD}[WARNING]{Colors.END} {message}")

def log_info(logger, message):
    logger.info(f"{Colors.CYAN}{Colors.BOLD}[INFO]{Colors.END} {message}")

def log_debug(logger, message):
    logger.debug(f"{Colors.BLUE}[DEBUG]{Colors.END} {message}")

# PR2 robot parameters
WHEEL_RADIUS = 0.08  # PR2 wheel radius 80mm
WHEEL_SEPARATION_X = 0.33  # PR2 front-back wheel distance (meters)
WHEEL_SEPARATION_Y = 0.26  # PR2 left-right wheel distance (meters)


class MyRobotDriver:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self.__node = rclpy.create_node("sim_webots_driver")
        self.__logger = self.__node.get_logger()
        
        log_info(self.__logger, "WebotsNode: Initializing...")
        self.__robot = webots_node.robot
        log_success(self.__logger, "WebotsNode: Robot device obtained")

        # Get PR2's 8 drive wheel motors
        self.__fl_l_motor = self.__robot.getDevice("fl_caster_l_wheel_joint")
        self.__fl_r_motor = self.__robot.getDevice("fl_caster_r_wheel_joint")
        self.__fr_l_motor = self.__robot.getDevice("fr_caster_l_wheel_joint")
        self.__fr_r_motor = self.__robot.getDevice("fr_caster_r_wheel_joint")
        self.__bl_l_motor = self.__robot.getDevice("bl_caster_l_wheel_joint")
        self.__bl_r_motor = self.__robot.getDevice("bl_caster_r_wheel_joint")
        self.__br_l_motor = self.__robot.getDevice("br_caster_l_wheel_joint")
        self.__br_r_motor = self.__robot.getDevice("br_caster_r_wheel_joint")

        # Set all motors to velocity control mode
        motors = [
            self.__fl_l_motor,
            self.__fl_r_motor,
            self.__fr_l_motor,
            self.__fr_r_motor,
            self.__bl_l_motor,
            self.__bl_r_motor,
            self.__br_l_motor,
            self.__br_r_motor,
        ]

        for motor in motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0)

        self.__target_twist = Twist()
        
        # Initialize odometry
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0
        self.__last_time = self.__robot.getTime()
        
        # Initialize laser scan counters
        self.__base_laser_count = 0
        self.__laser_tilt_count = 0

        # Get laser sensors
        self.__base_laser = self.__robot.getDevice("base_laser")
        self.__laser_tilt = self.__robot.getDevice("laser_tilt")
        
        # Check if laser devices were found
        if self.__base_laser is None:
            log_error(self.__logger, "base_laser device not found! Check robot name and device configuration.")
        else:
            log_success(self.__logger, "base_laser device found")
            
        if self.__laser_tilt is None:
            log_error(self.__logger, "laser_tilt device not found! Check robot name and device configuration.")
        else:
            log_success(self.__logger, "laser_tilt device found")
        
        # Enable laser sensors
        if self.__base_laser is not None:
            self.__base_laser.enable(32)  # 32ms update rate
            log_info(self.__logger, "base_laser enabled with 32ms update rate")
        if self.__laser_tilt is not None:
            self.__laser_tilt.enable(32)
            log_info(self.__logger, "laser_tilt enabled with 32ms update rate")

        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        
        # Create laser scan publishers
        self.__base_laser_pub = self.__node.create_publisher(LaserScan, "base_scan", 10)
        self.__laser_tilt_pub = self.__node.create_publisher(LaserScan, "laser_tilt_scan", 10)
        
        # Create point cloud publishers
        self.__base_pointcloud_pub = self.__node.create_publisher(PointCloud2, "base_pointcloud", 10)
        self.__laser_tilt_pointcloud_pub = self.__node.create_publisher(PointCloud2, "laser_tilt_pointcloud", 10)
        
        # Create odometry publisher
        self.__odom_pub = self.__node.create_publisher(Odometry, "odom", 10)
        
        # Create TF broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        
        # Publish initial static transform from odom to base_footprint
        self.__publish_static_transform()
        log_success(self.__logger, "WebotsNode: Initialization complete")

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __publish_laser_scan(self, laser_device, publisher, frame_id, device_name):
        """Publish laser scan data"""
        if laser_device is None:
            return  # Skip publishing if device is not available
        ranges = laser_device.getRangeImage()
        if ranges:
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.__node.get_clock().now().to_msg()
            scan_msg.header.frame_id = frame_id
            scan_msg.angle_min = -laser_device.getFieldOfView() / 2
            scan_msg.angle_max = laser_device.getFieldOfView() / 2
            scan_msg.angle_increment = laser_device.getFieldOfView() / len(ranges)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.032  # 32ms
            scan_msg.range_min = 0.1
            scan_msg.range_max = 30.0
            scan_msg.ranges = ranges
            publisher.publish(scan_msg)
            
            # Update counter and log periodically
            if device_name == "base_laser":
                self.__base_laser_count += 1
                if self.__base_laser_count % 50 == 0:  # Log every 50 messages
                    log_debug(self.__logger, f"Published {self.__base_laser_count} base_laser scans")
            elif device_name == "laser_tilt":
                self.__laser_tilt_count += 1
                if self.__laser_tilt_count % 50 == 0:  # Log every 50 messages
                    log_debug(self.__logger, f"Published {self.__laser_tilt_count} laser_tilt scans")

    def __publish_point_cloud(self, laser_device, publisher, frame_id, device_name):
        """Convert laser scan data to point cloud and publish"""
        if laser_device is None:
            return  # Skip publishing if device is not available
        ranges = laser_device.getRangeImage()
        if ranges:
            # Create point cloud message
            cloud_msg = PointCloud2()
            cloud_msg.header.stamp = self.__node.get_clock().now().to_msg()
            cloud_msg.header.frame_id = frame_id
            
            # Set up point cloud fields
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 16  # 4 bytes per field
            cloud_msg.row_step = cloud_msg.point_step * len(ranges)
            
            # Convert laser scan to 3D points
            points = []
            fov = laser_device.getFieldOfView()
            angle_min = -fov / 2
            angle_increment = fov / len(ranges)
            
            for i, range_val in enumerate(ranges):
                if range_val > 0.1 and range_val < 30.0:  # Valid range
                    angle = angle_min + i * angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    z = 0.0  # 2D laser scan, z=0
                    intensity = 1.0  # Default intensity
                    
                    # Pack point data
                    point_data = struct.pack('ffff', x, y, z, intensity)
                    points.append(point_data)
            
            cloud_msg.data = b''.join(points)
            cloud_msg.width = len(points)
            cloud_msg.height = 1
            cloud_msg.is_dense = True
            
            publisher.publish(cloud_msg)

    def __update_odometry(self, vx, vy, omega, dt):
        """Update odometry based on velocity commands"""
        # Simple integration (in real robot, this would use wheel encoders)
        self.__x += vx * math.cos(self.__theta) * dt
        self.__y += vx * math.sin(self.__theta) * dt
        self.__theta += omega * dt
        
        # Normalize angle
        while self.__theta > math.pi:
            self.__theta -= 2 * math.pi
        while self.__theta < -math.pi:
            self.__theta += 2 * math.pi

    def __publish_odometry(self):
        """Publish odometry and TF transform"""
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.__node.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # Set position
        odom_msg.pose.pose.position.x = self.__x
        odom_msg.pose.pose.position.y = self.__y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.__theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.__theta / 2.0)
        
        # Set velocity
        odom_msg.twist.twist.linear.x = self.__target_twist.linear.x
        odom_msg.twist.twist.linear.y = self.__target_twist.linear.y
        odom_msg.twist.twist.angular.z = self.__target_twist.angular.z
        
        # Publish odometry
        self.__odom_pub.publish(odom_msg)
        
        # Create and publish TF transform
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.__x
        t.transform.translation.y = self.__y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.__theta / 2.0)
        t.transform.rotation.w = math.cos(self.__theta / 2.0)
        
        self.__tf_broadcaster.sendTransform(t)

    def __publish_static_transform(self):
        """Publish initial static transform from odom to base_footprint"""
        t = TransformStamped()
        t.header.stamp = self.__node.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.__tf_broadcaster.sendTransform(t)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Calculate time step
        current_time = self.__robot.getTime()
        dt = current_time - self.__last_time
        self.__last_time = current_time

        # Get target velocities
        vx = self.__target_twist.linear.x  # Forward/backward velocity
        vy = self.__target_twist.linear.y  # Left/right translation velocity
        omega = self.__target_twist.angular.z  # Angular velocity
        
        # Update odometry
        self.__update_odometry(vx, vy, omega, dt)

        # Calculate wheel velocities
        # For PR2 robot, we use a simplified differential drive model
        # Group the 8 wheels into left and right groups for differential control

        # Calculate average velocities for left and right wheels
        left_speed = vx - omega * WHEEL_SEPARATION_Y / 2
        right_speed = vx + omega * WHEEL_SEPARATION_Y / 2

        # Convert linear velocity to angular velocity
        left_angular_velocity = left_speed / WHEEL_RADIUS
        right_angular_velocity = right_speed / WHEEL_RADIUS

        # Set all left wheel velocities
        self.__fl_l_motor.setVelocity(left_angular_velocity)
        self.__bl_l_motor.setVelocity(left_angular_velocity)

        # Set all right wheel velocities
        self.__fl_r_motor.setVelocity(right_angular_velocity)
        self.__fr_l_motor.setVelocity(right_angular_velocity)
        self.__fr_r_motor.setVelocity(right_angular_velocity)
        self.__bl_r_motor.setVelocity(right_angular_velocity)
        self.__br_l_motor.setVelocity(right_angular_velocity)
        self.__br_r_motor.setVelocity(right_angular_velocity)
        
        # Publish odometry and TF
        self.__publish_odometry()
        
        # Publish laser scan data
        self.__publish_laser_scan(self.__base_laser, self.__base_laser_pub, "base_laser_link", "base_laser")
        self.__publish_laser_scan(self.__laser_tilt, self.__laser_tilt_pub, "laser_tilt_link", "laser_tilt")
        
        # Publish point cloud data
        self.__publish_point_cloud(self.__base_laser, self.__base_pointcloud_pub, "base_laser_link", "base_laser")
        self.__publish_point_cloud(self.__laser_tilt, self.__laser_tilt_pointcloud_pub, "laser_tilt_link", "laser_tilt")


# Alias for compatibility
SimWebotsDriver = WebotsNode
