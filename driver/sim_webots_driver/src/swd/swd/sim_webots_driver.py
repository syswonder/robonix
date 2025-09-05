import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

# PR2 robot parameters
WHEEL_RADIUS = 0.08  # PR2 wheel radius 80mm
WHEEL_SEPARATION_X = 0.33  # PR2 front-back wheel distance (meters)
WHEEL_SEPARATION_Y = 0.26  # PR2 left-right wheel distance (meters)


class WebotsNode:
    def __init__(self, webots_node, properties):
        print("WebotsNode: Initializing...")
        self.__robot = webots_node.robot
        print("WebotsNode: Robot device obtained")

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

        # Get laser sensors
        self.__base_laser = self.__robot.getDevice("base_laser")
        self.__laser_tilt = self.__robot.getDevice("laser_tilt")
        
        # Enable laser sensors
        self.__base_laser.enable(32)  # 32ms update rate
        self.__laser_tilt.enable(32)

        rclpy.init(args=None)
        self.__node = rclpy.create_node("sim_webots_driver")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        
        # Create laser scan publishers
        self.__base_laser_pub = self.__node.create_publisher(LaserScan, "base_scan", 10)
        self.__laser_tilt_pub = self.__node.create_publisher(LaserScan, "laser_tilt_scan", 10)
        
        # Create odometry publisher
        self.__odom_pub = self.__node.create_publisher(Odometry, "odom", 10)
        
        # Create TF broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        
        # Publish initial static transform from odom to base_footprint
        self.__publish_static_transform()
        print("WebotsNode: Initialization complete")

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __publish_laser_scan(self, laser_device, publisher, frame_id):
        """Publish laser scan data"""
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
        self.__publish_laser_scan(self.__base_laser, self.__base_laser_pub, "base_laser_link")
        self.__publish_laser_scan(self.__laser_tilt, self.__laser_tilt_pub, "laser_tilt_link")


# Alias for compatibility
SimWebotsDriver = WebotsNode
