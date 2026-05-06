# Composable filter for scene/query. ROS .msg sentinel rules:
#   cls                 empty array == match any class
#   relations           empty array == no relation filter
#   pose_frame          empty string == ignore pose filter
#   pose_radius_m       0.0          == ignore (use any non-zero value)
#   snapshot_time       empty string == "latest" (only value supported in v1)
string[] cls
RelationConstraint[] relations
string pose_frame
float64 pose_center_x
float64 pose_center_y
float64 pose_center_z
float64 pose_radius_m
float64 min_confidence
string snapshot_time
