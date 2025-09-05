import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory("swd")
    robot_description_path = os.path.join(package_dir, "resource", "ranger.urdf")
    rviz_config_path = os.path.join(package_dir, "config", "ranger_lidar.rviz")

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
        robot_name="Pr2",
        parameters=[
            {"robot_description": robot_description_path},
        ],
        env={
            'PYTHONPATH': os.path.join(package_dir, '..', '..') + ':' + os.environ.get('PYTHONPATH', ''),
            'WEBOTS_HOME': '/usr/local/webots'
        }
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
            rviz2_node,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
