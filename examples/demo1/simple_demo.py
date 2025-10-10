#!/usr/bin/env python3

import sys
import os
import argparse
from pathlib import Path
import traceback

from robonix.uapi.runtime.runtime import Runtime

project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent  # robonix root
sys.path.insert(0, str(project_root_parent))

from robonix.manager.log import logger
from robonix.uapi import get_runtime, set_runtime

def init_skill_providers(runtime):
    """Initialize skill providers"""
    from robonix.uapi.runtime.provider import SkillProvider

    # dump __all__ in robonix.skill to skills list
    try:
        from robonix.skill import __all__
        skills = __all__
    except ImportError:
        logger.warning("robonix.skill module not available")
        skills = []

    local_provider = SkillProvider(
        name="local_provider",
        IP="127.0.0.1",
        skills=skills,
    )

    runtime.registry.add_provider(local_provider)
    logger.info(f"Added skill providers: {runtime.registry}")


def create_manual_entity_builder():
    """Create a manual entity graph builder"""
    def builder(runtime: Runtime, **kwargs):
        from robonix.uapi.graph.entity import create_root_room, create_controllable_entity

        root_room = create_root_room()

        entity_a = create_controllable_entity("A")
        root_room.add_child(entity_a)

        entity_b = create_controllable_entity("B")
        root_room.add_child(entity_b)

        # Bind mock skills
        def mock_getpos(**kwargs):
            logger.info("mock cap_get_pose called")
            return (1.0, 2.0, 0.0)

        try:
            from robonix.skill import debug_test_skill
            entity_a.bind_skill("cap_get_pose", mock_getpos)
            entity_a.bind_skill("skl_debug_test_skill", debug_test_skill)
        except ImportError:
            logger.error(f"error: import error, {traceback.format_exc()}")
            sys.exit(1)

        runtime.set_graph(root_room)

        logger.info("Manual entity graph initialized:")
        logger.info(f"  root room: {root_room.get_absolute_path()}")
        logger.info(f"  entity A: {entity_a.get_absolute_path()}")
        logger.info(f"  entity B: {entity_b.get_absolute_path()}")

    return builder


def create_yolo_entity_builder():
    """Create a YOLO-based entity graph builder"""
    def builder(runtime: Runtime, **kwargs):
        logger.info("Building entity graph from YOLO detection...")

        try:
            from robonix.skill import (
                sim_skl_detect_objs,
                sim_save_rgb_image,
                sim_save_depth_image,
                sim_camera_dep_rgb,
                sim_camera_info,
                sim_get_robot_pose,
            )
        except ImportError:
            logger.error("Required skills not available")
            return

        from robonix.uapi.graph.entity import create_root_room, create_controllable_entity

        root_room = create_root_room()
        runtime.set_graph(root_room)

        robot = create_controllable_entity("robot")
        root_room.add_child(robot)

        def robot_move_impl(x, y, z):
            try:
                from robonix.driver.sim_genesis_ranger.driver import move_to_point
                move_to_point(x, y)  # Uncommet when you want to really move the robot
                return {"success": True}
            except ImportError:
                logger.error(f"error: import error, {traceback.format_exc()}")
                sys.exit(1)

        def robot_getpos_impl():
            try:
                from robonix.driver.sim_genesis_ranger.driver import get_pose
                x, y, z, yaw = get_pose()
                return {"x": x, "y": y, "z": z}
            except ImportError:
                logger.warning(f"error: import error, {traceback.format_exc()}")
                return {"x": 0.0, "y": 0.0, "z": 0.0}

        # Bind skills to robot entity
        robot.bind_skill("cap_space_move", robot_move_impl)
        robot.bind_skill("cap_space_getpos", robot_getpos_impl)
        robot.bind_skill("cap_save_rgb_image", sim_save_rgb_image)
        robot.bind_skill("cap_save_depth_image", sim_save_depth_image)

        move_base = create_controllable_entity("move_base")
        robot.add_child(move_base)

        camera = create_controllable_entity("camera")
        robot.add_child(camera)

        # Bind camera capabilities
        camera.bind_skill("cap_camera_dep_rgb", sim_camera_dep_rgb)
        camera.bind_skill("cap_camera_info", sim_camera_info)
        camera.bind_skill("cap_get_robot_pose", sim_get_robot_pose)
        camera.bind_skill("skl_detect_objs", sim_skl_detect_objs)

        # Detect objects
        detect_objs = camera.skl_detect_objs(camera_name="camera0")
        logger.info(f"Detected objects: {detect_objs}")

        detected_entities_getpos_handler = {}

        for obj_name, obj_info in detect_objs.items():
            obj_entity = create_controllable_entity(obj_name)
            root_room.add_child(obj_entity)

            if obj_info["position"] is None:
                logger.warning(f"Object {obj_name} has no position")
                x, y = 0.0, 0.0
            else:
                x, y = obj_info["position"][0], obj_info["position"][1]

            def create_getpos_handler(obj_x, obj_y):
                return lambda: {"x": obj_x, "y": obj_y, "z": 0.0}

            detected_entities_getpos_handler[obj_name] = create_getpos_handler(
                x, y)
            obj_entity.bind_skill(
                "cap_space_getpos", detected_entities_getpos_handler[obj_name]
            )

            logger.info(
                f"Created entity for {obj_name}: {obj_entity.get_absolute_path()}")

        logger.info("YOLO-based entity graph initialized:")
        logger.info(f"  root room: {root_room.get_absolute_path()}")
        logger.info(f"  robot: {robot.get_absolute_path()}")
        logger.info(f"  detected entities: {list(detect_objs.keys())}")

    return builder


def main():
    parser = argparse.ArgumentParser(description="Simple Demo 1")
    parser.add_argument(
        "--mode",
        type=str,
        default="manual",
        choices=["manual", "auto"],
        help="Mode to init entity graph",
    )
    parser.add_argument(
        "--export-scene",
        type=str,
        help="Export scene information to JSON file",
    )
    args = parser.parse_args()

    logger.info("Starting simple demo 1")

    # Get runtime instance
    runtime = get_runtime()

    # Register entity builders
    runtime.register_entity_builder("manual", create_manual_entity_builder())
    runtime.register_entity_builder("auto", create_yolo_entity_builder())

    # Initialize skill providers
    init_skill_providers(runtime)

    # Build entity graph based on mode
    runtime.build_entity_graph(args.mode)

    # Set runtime for action system
    set_runtime(runtime)

    # Print entity tree structure
    runtime.print_entity_tree()

    # Export scene information if requested
    if args.export_scene:
        scene_info = runtime.export_scene_info(args.export_scene)
        logger.info(f"Scene information exported to: {args.export_scene}")

    # Load action program
    action_program_path = os.path.join(
        os.path.dirname(__file__), "simple.action")
    logger.info(f"Loading action program from: {action_program_path}")

    try:
        action_names = runtime.load_action_program(action_program_path)
        logger.info(f"Loaded action functions: {action_names}")

        if args.mode == "manual":
            runtime.configure_action("debug_test_action", a="/A")
        elif args.mode == "auto":
            # however, the "/chair" argument is hard-coded now for debugging purposes
            runtime.configure_action(
                "move_and_capture_action", a="/robot", b="/chair"
            )

        # Start action based on mode
        if args.mode == "manual":
            thread = runtime.start_action("debug_test_action")
            result = runtime.wait_for_action("debug_test_action", timeout=30.0)
        elif args.mode == "auto":
            thread = runtime.start_action("move_and_capture_action")
            result = runtime.wait_for_action("move_and_capture_action", timeout=30.0)

        logger.info("Demo completed successfully")
        return 0

    except Exception as e:
        logger.error(f"Demo failed: {str(e)}", exc_info=True)
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
