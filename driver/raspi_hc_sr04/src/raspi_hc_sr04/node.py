#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 wheatfox
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Range
from driver import MultiHCSR04Driver
import yaml
import os


class HCSR04Node(Node):
    def __init__(self):
        super().__init__("hc_sr04_node")
        self.get_logger().info("initializing hc_sr04_node...")

        # Load sensor configurations from YAML file in the same directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "sensors.yaml")

        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
                self.sensor_configs = config["sensors"]  # Keep as list
                self.get_logger().info(
                    f"successfully loaded sensor configurations from {config_path}"
                )
        except Exception as e:
            self.get_logger().error(f"failed to load sensor configurations: {str(e)}")
            raise

        self.sensor_publishers = {}
        for sensor in self.sensor_configs:
            sensor_name = sensor["name"]
            topic_name = f"ultrasonic/{sensor_name}"
            self.sensor_publishers[sensor_name] = self.create_publisher(
                Range, topic_name, 10
            )
            self.get_logger().info(
                f"created publisher for {sensor_name} on topic {topic_name}"
            )

        # Create heartbeat publisher
        # self.heartbeat_publisher = self.create_publisher(Float32, "ultrasonic/heartbeat", 10)
        #   self.get_logger().info("created heartbeat publisher on topic ultrasonic/heartbeat")

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # self.heartbeat_timer = self.create_timer(5, self.heartbeat_callback)

        self.driver = MultiHCSR04Driver(self.sensor_configs)
        self.get_logger().info(
            f"HC-SR04 node started with {len(self.sensor_configs)} sensors"
        )

    def heartbeat_callback(self):
        self.get_logger().debug("Heartbeat callback triggered")
        current_time = self.get_clock().now().to_msg().sec
        self.get_logger().info(f"heartbeat at {current_time}")

        # Publish heartbeat message
        msg = Float32()
        msg.data = float(current_time)
        self.heartbeat_publisher.publish(msg)
        self.get_logger().debug(f"Published heartbeat message: {msg.data}")

    def timer_callback(self):
        distances = self.driver.get_all_distances()

        for sensor_name, distance in distances.items():
            if distance is not None:
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = sensor_name
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.1
                msg.min_range = 0.02
                msg.max_range = 4.0
                msg.range = float(distance) / 100.0  # convert cm to m

                self.sensor_publishers[sensor_name].publish(msg)
                self.get_logger().info(
                    f"published distance for {sensor_name}: {distance:.6f} cm, range: {msg.range:.6f} m"
                )
            else:
                self.get_logger().info(
                    f"device not connected or result too small/too large for {sensor_name}, ignoring!"
                )

    def cleanup(self):
        self.get_logger().info("Cleaning up node...")
        self.driver.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = HCSR04Node()

    try:
        node.get_logger().info("starting to spin node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt received")
    except Exception as e:
        node.get_logger().error(f"exception occurred: {str(e)}")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
