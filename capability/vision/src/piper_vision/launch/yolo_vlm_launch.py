from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piper_vision',
            executable='yolo_detect_3d',
            name='yolo_detect_3d',
            parameters=[{
                'device': 'cuda:0',
                'interest': 'person',
                'depth_threshold': 15.0,
                'model': 'yoloe-11l-seg-pf',
                'bg_removal': False,
                'target_frame_id': 'map',
            }],
            output='screen',
        ),
        Node(
            package='piper_vision',
            executable='vlm_mapper_node',
            name='vlm_mapper_node',
            output='screen',
        )
    ])
