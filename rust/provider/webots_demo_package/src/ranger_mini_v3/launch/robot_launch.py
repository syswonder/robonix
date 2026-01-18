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
    use_nav = LaunchConfiguration('nav', default=False)  # Disable nav2 for now
    use_rtabmap = LaunchConfiguration('rtabmap', default=True)  # Enable rtabmap by default
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
    
    # Camera info publisher for RGBD cameras
    camera_info_publisher = Node(
        package='ranger_mini_v3',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rtabmap)
    )
    
    # Temporary static transform from map to odom to bootstrap SLAM
    # This will be overridden by rtabmap once it initializes and starts publishing
    # The static transform ensures map frame exists immediately for other nodes
    map_to_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=launch.conditions.IfCondition(use_rtabmap)
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

    # Navigation - Disabled for SLAM-only mode
    navigation_nodes = []
    # Nav2 is disabled by default, only enable if explicitly requested
    # if 'nav2_bringup' in get_packages_with_prefixes():
    #     nav2_params_file = 'nav2_params.yaml'
    #     nav2_params = os.path.join(package_dir, 'resource', nav2_params_file)
    #     nav2_map = os.path.join(package_dir, 'resource', 'map.yaml')
    #     navigation_nodes.append(IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
    #         launch_arguments={
    #             'map': nav2_map,
    #             'params_file': nav2_params,
    #             'use_sim_time': use_sim_time,
    #             'autostart': 'true',
    #             'slam': 'True',
    #         }.items(),
    #         condition=launch.conditions.IfCondition(use_nav)))

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
    
    # RTAB-Map for RGBD SLAM
    # RGBD Odometry node
    rtabmap_rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'head_front_camera_rgb_optical_frame',
            'odom_frame_id': 'odom',
            'ground_truth_frame_id': '',
            'wait_imu_to_init': False,
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/head_front_camera/rgb/image_raw'),
            ('depth/image', '/head_front_camera/depth_registered/image_raw'),
            ('rgb/camera_info', '/head_front_camera/rgb/camera_info'),
        ],
        condition=launch.conditions.IfCondition(use_rtabmap)
    )
    navigation_nodes.append(rtabmap_rgbd_odometry)
    
    # RTAB-Map main node
    # Note: All rtabmap-specific parameters (RGBD/*, Grid/*, Vis/*) are string types in ROS2
    # They must be passed as strings, not as their native types
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        respawn=True,  # Auto-respawn if node crashes
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'queue_size': 10,
            'approx_sync': True,
            'wait_for_transform_duration': 0.2,
            'publish_tf': True,
            'publish_tf_map': True,
            'odom_sensor_sync': False,
            # Subscribe settings - use depth, not scan
            'subscribe_scan': False,  # Disable scan subscription to use depth images
            'subscribe_scan_cloud': False,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            # rtabmap-specific parameters (as strings)
            'RGBD/CreateOccupancyGrid': 'true',  # Enable occupancy grid (string)
            'RGBD/ImagesAlreadyRectified': 'false',  # Images need rectification (string)
            'Grid/FromDepth': 'true',  # Use depth images for grid (string)
            'RGBD/LinearUpdate': '0.01',  # Update when robot moves 1cm (string)
            'RGBD/AngularUpdate': '0.01',  # Update when robot rotates 0.01 rad (string)
        }],
        remappings=[
            ('rgb/image', '/head_front_camera/rgb/image_raw'),
            ('depth/image', '/head_front_camera/depth_registered/image_raw'),
            ('rgb/camera_info', '/head_front_camera/rgb/camera_info'),
            ('odom', '/rgbd_odometry/odom'),  # Use RGBD odometry instead of wheel odometry
        ],
        condition=launch.conditions.IfCondition(use_rtabmap)
    )
    navigation_nodes.append(rtabmap_node)
    
    # RTAB-Map visualization node (optional, for rtabmapviz)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'queue_size': 10,
            'approx_sync': True,
            'wait_for_transform_duration': 0.2,
        }],
        remappings=[
            ('rgb/image', '/head_front_camera/rgb/image_raw'),
            ('depth/image', '/head_front_camera/depth_registered/image_raw'),
            ('rgb/camera_info', '/head_front_camera/rgb/camera_info'),
            ('odom', '/rgbd_odometry/odom'),  # Use RGBD odometry
        ],
        condition=launch.conditions.IfCondition(use_rtabmap)
    )
    navigation_nodes.append(rtabmap_viz)

    # Start rtabmap nodes directly (they don't need to wait for controller)
    # Only wait for controller for rviz and ros_control_spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=ranger_driver,
        nodes_to_start=[rviz] + ros_control_spawners
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
        camera_info_publisher,
        map_to_odom_publisher,

        ranger_driver,
        # Start navigation nodes (rtabmap) directly, they don't need to wait
        *navigation_nodes,
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
