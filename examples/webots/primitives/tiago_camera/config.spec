# Webots Tiago RGB-D camera topic and calibration config.
# The simulator normally provides the topics and TF. Intrinsics may come from
# CameraInfo or from width/height/fx/fy/cx/cy below.

config:
  rgb_topic: /head_front_camera/rgb/image_raw
  depth_topic: /head_front_camera/depth_registered/image_raw
  camera_info_topic: /head_front_camera/rgb/camera_info
  intrinsics_topic: /tiago/camera/intrinsics
  extrinsics_topic: /tiago/camera/extrinsics
  base_frame: base_link
  cam_frame: head_front_camera_rgb_optical_frame
  width: 0
  height: 0
  fx: 0.0
  fy: 0.0
  cx: 0.0
  cy: 0.0
  horizontal_fov_rad: 0.0
  sentinel_timeout_s: 60.0
  camera_info_source_timeout_s: 1.0
  intrinsics_publish_interval_s: 0.5
