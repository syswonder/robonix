# Snapshot of full robot state, returned by robonix/primitive/robot/state (RPC).
# Designed for single-shot queries (MCP tools, VLA state input, teleop feedback).
# For continuous streaming use nav_msgs/Odometry + sensor_msgs/JointState separately.

std_msgs/Header header

# Base pose in world frame (odometry-derived, x/y/z + quaternion).
geometry_msgs/PoseStamped base_pose

# All joints: name[], position[] (rad/m), velocity[] (rad·s⁻¹/m·s⁻¹), effort[] (Nm/N).
# Empty arrays if the joint state is unavailable.
sensor_msgs/JointState joint_state

# End-effector (TCP) pose in world frame. Empty frame_id means unavailable.
geometry_msgs/PoseStamped tcp_pose

# Gripper opening in metres (0 = fully closed, provider-defined max = fully open).
float64 gripper_width

# True when the gripper detects contact / is holding an object.
bool is_grasped
