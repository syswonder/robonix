# 标准 Header（时间戳 + frame_id）
std_msgs/Header header

string[] names
# 多个物体的位置（可动态扩展）
geometry_msgs/Point[] points

# 可选：每个物体的尺寸
float32[] widths
float32[] heights