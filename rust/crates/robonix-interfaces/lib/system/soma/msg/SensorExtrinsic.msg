# One static-mounted sensor's extrinsics: where it sits relative to a
# parent frame (typically `base_link`). Read from the URDF at soma
# startup; consumers (scene's camera-to-world projection, multi-cam
# fusion) call `system/soma/sensor_extrinsics` instead of looking up
# tf2 themselves so the dependency graph is auditable via atlas.

string sensor_name        # e.g. "rgb_camera", "depth_camera", "lidar3d"
string parent_frame       # e.g. "base_link"
string child_frame        # e.g. "camera_color_optical_frame"
geometry_msgs/Transform transform   # parent → child
