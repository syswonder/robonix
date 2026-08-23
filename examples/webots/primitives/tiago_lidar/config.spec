# Runtime configuration accepted by the Webots Tiago planar lidar primitive.
#
# This documents the mapping passed as the package instance's `config:` value.
# It is not loaded as a schema. Values below are runtime defaults.

config:
  # string absolute ROS 2 topic, default: /scanner_normalized.
  # sensor_msgs/LaserScan source exposed through the planar lidar capability.
  scan_topic: /scanner_normalized

  # float seconds, default: 15.0; must be greater than zero.
  # Maximum startup wait for the first LaserScan message before init fails.
  sentinel_timeout_s: 15.0
