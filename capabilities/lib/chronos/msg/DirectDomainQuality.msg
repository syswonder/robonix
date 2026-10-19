# Timestamp scale values mirror Chronos TimestampScaleKind.
uint8 TIME_SCALE_LINUX_SYSTEM=0
uint8 TIME_SCALE_ROS_TIME=1
uint8 TIME_SCALE_DEVICE_UPTIME=2

string domain_id
uint8 time_scale
uint64 reference_generation
bool reference_available
bool reference_verified
uint64 uncertainty_ns
string reason
