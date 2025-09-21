import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('eaios_webots')
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value=TextSubstitution(text='tiago_webots.urdf'),  # Default to the turtlebot_urdf file
        description='Path to the robot URDF file (relative to the package share directory)'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='office.wbt'),  # Default to the office.wbt file
        description='Path to the Webots world file (relative to the package share directory)'
    )
    
    robot_urdf_file = LaunchConfiguration('robot')
    world_wbt_file = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    robot_description_path = PathJoinSubstitution([
        package_dir,
        'resource', # Assuming URDF files are in the 'resource' folder
        robot_urdf_file
    ])
    world_description_path = PathJoinSubstitution([
        package_dir,
        'worlds', # Assuming world files are in the 'worlds' folder
        world_wbt_file
    ])

    print(f"using robot_path:{robot_description_path}")
    print(f"using world_path:{world_description_path}")

    webots = WebotsLauncher(
        world=world_description_path,
        mode="realtime",
        ros2_supervisor=True
        # Other possible Webots parameters, e.g., gui, mode, etc.
    )
    
    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]
    

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


    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    my_robot_driver = WebotsController(
        robot_name='my_robot', # Ensure this name matches the robot node name in the Webots world.
        parameters=[
            {'robot_description': robot_description_path, # This robot_description is typically for the Webots driver.
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True}, # Set to True if WebotsController should launch its own RSP.
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=ros_control_spawners
    )

    return LaunchDescription([
        robot_arg,
        world_arg,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=TextSubstitution(text='True'),
            description='Use simulation (Webots) clock if true'
        ),

        webots,
        webots._supervisor,
        robot_state_publisher, # Ensure robot_state_publisher starts before my_robot_driver if my_robot_driver depends on it.
        footprint_publisher,
        my_robot_driver,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])

