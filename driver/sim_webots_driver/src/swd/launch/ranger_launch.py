import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory("swd")
    robot_description_path = os.path.join(package_dir, "resource", "ranger.urdf")

    DEEP_EMBODY_SRC_ROOT = os.path.join(package_dir, "../../../../../../../../")

    selected_world = "complete_apartment.wbt"
    world_path = os.path.join(
        DEEP_EMBODY_SRC_ROOT, "simulator/webots/worlds", selected_world
    )
    print(f"World path: {world_path}")
    print(f"Robot description path: {robot_description_path}")

    webots = WebotsLauncher(world=world_path)

    my_robot_driver = WebotsController(
        robot_name="ranger",
        parameters=[
            {"robot_description": robot_description_path},
        ],
    )

    return LaunchDescription(
        [
            webots,
            my_robot_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
