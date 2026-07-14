# Runtime configuration accepted by the Webots Tiago chassis primitive.
#
# This documents the mapping passed as the package instance's `config:` value.
# It is not loaded as a schema. Values below are runtime defaults.

config:
  # string absolute ROS 2 topic, default: /odom.
  # nav_msgs/Odometry source exposed through the chassis odometry capability.
  odom_topic: /odom

  # string absolute ROS 2 topic, default: /cmd_vel.
  # geometry_msgs/Twist command target used by chassis movement capabilities.
  twist_in_topic: /cmd_vel
