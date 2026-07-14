# Runtime configuration accepted by the Webots Tiago RGB-D camera primitive.
#
# This documents the mapping passed as the package instance's `config:` value.
# It is not loaded as a schema. Values below are runtime defaults.

config:
  # string absolute ROS 2 topic, default shown below.
  # sensor_msgs/Image source exposed through the RGB capability.
  rgb_topic: /head_front_camera/rgb/image_raw

  # string absolute ROS 2 topic, default shown below.
  # Registered sensor_msgs/Image source exposed through the depth capability.
  depth_topic: /head_front_camera/depth_registered/image_raw

  # string absolute ROS 2 topic, default shown below.
  # Preferred sensor_msgs/CameraInfo source for camera intrinsics.
  camera_info_topic: /head_front_camera/rgb/camera_info

  # string absolute ROS 2 topic, default: /tiago/camera/intrinsics.
  # Latched CameraInfo output that backs the Robonix intrinsics capability.
  intrinsics_topic: /tiago/camera/intrinsics

  # string absolute ROS 2 topic, default: /tiago/camera/extrinsics.
  # Latched TransformStamped output from base_frame to cam_frame.
  extrinsics_topic: /tiago/camera/extrinsics

  # string TF frame, default: base_link.
  # Robot frame used as the parent of the published camera extrinsics.
  base_frame: base_link

  # string TF frame, default: head_front_camera_rgb_optical_frame.
  # Camera optical frame used as the child of the published extrinsics.
  cam_frame: head_front_camera_rgb_optical_frame

  # integer pixels, default: 0; width and height must both be positive when
  # deployment-provided intrinsics are used instead of CameraInfo.
  width: 0
  height: 0

  # float pixels, default: 0.0.
  # Focal lengths. Positive fx and fy select explicit calibration; otherwise
  # fx may be derived from width and horizontal_fov_rad.
  fx: 0.0
  fy: 0.0

  # float pixels, default: 0.0.
  # Principal point. Zero values are resolved from the configured image size.
  cx: 0.0
  cy: 0.0

  # float radians, default: 0.0; valid useful range: 0 < value < pi.
  # Horizontal field of view used to derive fx when explicit focal lengths
  # are unavailable.
  horizontal_fov_rad: 0.0

  # float seconds, default: 60.0; must be greater than zero.
  # Maximum startup wait for the first RGB image before init fails.
  sentinel_timeout_s: 60.0

  # float seconds, default: 1.0; must be non-negative.
  # Time allowed for CameraInfo before configured intrinsics are used.
  camera_info_source_timeout_s: 1.0

  # float seconds, default: 0.5; minimum effective value: 0.1.
  # Republish interval for configured intrinsics until CameraInfo is observed.
  intrinsics_publish_interval_s: 0.5
