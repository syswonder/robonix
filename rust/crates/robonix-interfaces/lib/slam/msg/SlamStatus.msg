# Current SLAM system status.
#
# mode: "mapping" | "localization" | "idle"
# odom_hz: odometry output frequency (Hz)
# map_file: path to currently loaded map (empty if mapping mode)

std_msgs/Header header

string mode
bool odom_alive
float32 odom_hz
bool cloud_alive
string map_file
uint64 total_odom_frames
