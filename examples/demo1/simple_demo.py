#!/usr/bin/env python3

import sys
import os
import time
import argparse
from pathlib import Path

project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent.parent  # DeepEmbody root
sys.path.insert(0, str(project_root_parent))

from DeepEmbody.uapi.runtime.runtime import Runtime
from DeepEmbody.uapi.runtime.provider import SkillProvider
from DeepEmbody.uapi.graph.entity import create_root_room, create_controllable_entity
from DeepEmbody.uapi.runtime.action import set_runtime
from DeepEmbody.uapi.log import logger


def init_skill_providers(runtime: Runtime):
    # dump __all__ in DeepEmbody.skill to skills list
    from DeepEmbody.skill import __all__
    skills = __all__
    local_provider = SkillProvider(
        name="local_provider",
        IP="127.0.0.1",
        skills=skills,
    )

    runtime.registry.add_provider(local_provider)

    logger.info(f"added skill providers: {runtime.registry}")


def init_entity_graph_manually(runtime: Runtime):
    root_room = create_root_room()

    entity_a = create_controllable_entity("A")
    root_room.add_child(entity_a)

    entity_b = create_controllable_entity("B")
    root_room.add_child(entity_b)

    def mock_getpos(**kwargs):
        print("mock cap_get_pose called")
        return (1.0, 2.0, 0.0)  # Return tuple (x, y, yaw) as per skill spec

    
    from DeepEmbody.skill import debug_test_skill

    entity_a.bind_skill("cap_get_pose", mock_getpos)
    entity_a.bind_skill("skl_debug_test_skill", debug_test_skill)

    runtime.set_graph(root_room)

    logger.info("initd entity graph:")
    logger.info(f"  root room: {root_room.get_absolute_path()}")
    logger.info(f"  entity A: {entity_a.get_absolute_path()}")
    logger.info(f"  entity B: {entity_b.get_absolute_path()}")


def init_entity_graph_from_yolo(runtime: Runtime):
    logger.info("importing skills...")
    from DeepEmbody.skill import (
        skl_detect_objs,
        cap_save_rgb_image,
        cap_save_depth_image,
    )

    root_room = create_root_room()
    runtime.set_graph(root_room)

    robot = create_controllable_entity("robot")
    root_room.add_child(robot)

    def robot_move_impl(x, y, z):
        from DeepEmbody.driver.sim_genesis_ranger.driver import move_to_point

        # move_to_point(x, y)  # THIS IS A FUNCTION FROM DRIVER !
        return {"success": True}

    def robot_getpos_impl():
        from DeepEmbody.driver.sim_genesis_ranger.driver import get_pose

        x, y, z, yaw = get_pose()
        return {"x": x, "y": y, "z": z}

    # Bind skills to robot entity using standard names
    robot.bind_skill("cap_space_move", robot_move_impl)
    robot.bind_skill("cap_space_getpos", robot_getpos_impl)
    robot.bind_skill("cap_save_rgb_image", cap_save_rgb_image)
    robot.bind_skill("cap_save_depth_image", cap_save_depth_image)

    move_base = create_controllable_entity("move_base")
    robot.add_child(move_base)

    camera = create_controllable_entity("camera")
    robot.add_child(camera)

    # Bind the detection skill to camera entity
    camera.bind_skill("skl_detect_objs", skl_detect_objs)

    # Call the detection skill through the camera entity
    # This will automatically inject self_entity parameter
    detect_objs = camera.skl_detect_objs(camera_name="camera0")
    logger.info(f"detected objects: {detect_objs}")
    # detect_objs is a dict of {obj_name: obj_info}

    global detected_entities
    detected_entities = {}

    global detected_entities_getpos_handler
    detected_entities_getpos_handler = {}

    for obj_name, obj_info in detect_objs.items():
        obj_entity = create_controllable_entity(obj_name)
        root_room.add_child(obj_entity)
        detected_entities[obj_name] = obj_entity

        x, y = obj_info["position"][0], obj_info["position"][1]

        # Fix closure issue by creating a proper function with default arguments
        def create_getpos_handler(obj_x, obj_y):
            return lambda: {"x": obj_x, "y": obj_y, "z": 0.0}

        detected_entities_getpos_handler[obj_name] = create_getpos_handler(x, y)
        obj_entity.bind_skill(
            "cap_space_getpos", detected_entities_getpos_handler[obj_name]
        )

        logger.info(f"created entity for {obj_name}: {obj_entity.get_absolute_path()}")

    logger.info("initd entity graph from YOLO detection:")
    logger.info(f"  root room: {root_room.get_absolute_path()}")
    logger.info(f"  robot: {robot.get_absolute_path()}")
    logger.info(f"  detected entities: {list(detected_entities.keys())}")

    return detected_entities


def main():
    parser = argparse.ArgumentParser(description="Simple Demo 1")
    parser.add_argument(
        "--mode",
        type=str,
        default="manual",
        choices=["manual", "auto"],
        help="Mode to init entity graph",
    )
    args = parser.parse_args()

    logger.info("starting simple demo 1")

    runtime = Runtime()

    init_skill_providers(runtime)
    detected_entities = {}  # Initialize with empty dict
    if args.mode == "manual":
        init_entity_graph_manually(runtime)
    elif args.mode == "auto":
        detected_entities = init_entity_graph_from_yolo(runtime)
    else:
        raise ValueError(f"invalid mode: {args.mode}")

    set_runtime(runtime)

    action_program_path = os.path.join(os.path.dirname(__file__), "simple.action")
    logger.info(f"loading action program from: {action_program_path}")

    try:
        action_names = runtime.load_program(action_program_path)
        logger.info(f"loaded action functions: {action_names}")

        if args.mode == "manual":
            runtime.set_action_args("debug_test_action", a="/A")
            
        elif args.mode == "auto":
            # in auto mode, the entity graph is constructed using registered
            # skl_detect_objs skill, and we choose the first object detected as
            # action argument "b", and "a" fixed to "/robot". - wheatfox 2025.8.13
            if detected_entities:
                first_obj_name = list(detected_entities.keys())[0]
                first_obj_path = f"/{first_obj_name}"
                # runtime.set_action_args("move_a_to_b", a="/robot", b=first_obj_path)
                runtime.set_action_args(
                    "move_and_capture_action", a="/robot", b=first_obj_path
                )
            else:
                logger.warning(
                    "auto mode: no objects detected, using default robot paths"
                )
                # Use move_and_capture_action instead of move_a_to_b
                runtime.set_action_args(
                    "move_and_capture_action", a="/robot", b="/robot"
                )

        if args.mode == "manual":
            runtime.start_action("debug_test_action")
        elif args.mode == "auto":
            runtime.start_action("move_and_capture_action")

        logger.info("waiting for all actions to complete...")
        results = runtime.wait_for_all_actions(timeout=30.0)

        logger.info("action execution results:")
        for action_name, result in results.items():
            logger.info(f"  {action_name}: {result}")

    except Exception as e:
        logger.error(f"demo failed: {str(e)}", exc_info=True)
        return 1

    logger.info("demo completed successfully")
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
