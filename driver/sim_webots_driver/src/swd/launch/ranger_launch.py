import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory("swd")
    robot_description_path = os.path.join(package_dir, "resource", "ranger.urdf")
    nav2_params_path = os.path.join(package_dir, "config", "nav2_params.yaml")
    rviz_config_path = os.path.join(package_dir, "config", "ranger_nav2.rviz")

    DEEP_EMBODY_SRC_ROOT = os.path.join(package_dir, "../../../../../../../../")

    selected_world = "complete_apartment.wbt"
    world_path = os.path.join(
        DEEP_EMBODY_SRC_ROOT, "simulator/webots/worlds", selected_world
    )
    print(f"World path: {world_path}")
    print(f"Robot description path: {robot_description_path}")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    webots = WebotsLauncher(world=world_path)

    my_robot_driver = WebotsController(
        robot_name="ranger",
        parameters=[
            {"robot_description": robot_description_path},
        ],
        env={
            'PYTHONPATH': os.path.join(package_dir, '..', '..') + ':' + os.environ.get('PYTHONPATH', ''),
            'WEBOTS_HOME': '/usr/local/webots'
        }
    )

    # Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_path,
            'slam': 'True',
            'map': '',  # Empty map for SLAM mode
        }.items()
    )

    # RViz2 launch
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            webots,
            my_robot_driver,
            nav2_launch,
            rviz2_node,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
