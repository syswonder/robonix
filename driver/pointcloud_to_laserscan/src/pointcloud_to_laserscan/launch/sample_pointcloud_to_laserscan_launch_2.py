from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math

# 将度数转换为弧度的辅助函数
def deg_to_rad(degrees):
    return degrees * math.pi / 180.0

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud']),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
        parameters=[{
            'target_frame': 'unilidar_lidar',
            'transform_tolerance': 0.01,
            'min_height': 0.3,
            'max_height': 1.0,
            'angle_min': deg_to_rad(-179.0),
            'angle_max': deg_to_rad(179.0),
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
