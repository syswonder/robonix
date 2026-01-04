"""Launch Webots and the controller."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('ranger_mini_v3')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)
    use_slam_cartographer = LaunchConfiguration('slam_cartographer', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['four_wheel_steering_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'ranger_mini_v3_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
    if use_twist_stamped:
        mappings = [('/four_wheel_steering_controller/cmd_vel', '/cmd_vel'), ('/four_wheel_steering_controller/odom', '/odom')]
    else:
        mappings = [('/four_wheel_steering_controller/cmd_vel_unstamped', '/cmd_vel'), ('/four_wheel_steering_controller/odom', '/odom')]
    ranger_driver = WebotsController(
        robot_name='RangerMiniV3',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('ranger_mini_v3'), 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    # Navigation
    navigation_nodes = []
    if 'nav2_bringup' in get_packages_with_prefixes():
        nav2_params_file = 'nav2_params.yaml'
        nav2_params = os.path.join(package_dir, 'resource', nav2_params_file)
        nav2_map = os.path.join(package_dir, 'resource', 'map.yaml')
        # Only include navigation if params file exists and use_nav is True
        # Map file is optional - can be provided via launch argument or created later
        if os.path.exists(nav2_params):
            nav2_launch_args = [
                ('params_file', nav2_params),
                ('use_sim_time', use_sim_time),
            ]
            # Only add map argument if map file exists
            if os.path.exists(nav2_map):
                nav2_launch_args.append(('map', nav2_map))
            
            navigation_nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
                launch_arguments=nav2_launch_args,
                condition=launch.conditions.IfCondition(use_nav)))

    # SLAM
    cartographer_config_dir = os.path.join(package_dir, 'resource')
    cartographer_config_basename = 'cartographer.lua'
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', cartographer_config_basename],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer)

    grid_executable = 'cartographer_occupancy_grid_node'
    cartographer_grid = Node(
        package='cartographer_ros',
        executable=grid_executable,
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer_grid)

    toolbox_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params.yaml')
    slam_toolbox = Node(
        parameters=[toolbox_params,
                    {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam_toolbox)
    )
    navigation_nodes.append(slam_toolbox)

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=ranger_driver,
        nodes_to_start=[rviz] + navigation_nodes + ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='test1.wbt',
            description='Choose one of the world files from `world/` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        ranger_driver,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
