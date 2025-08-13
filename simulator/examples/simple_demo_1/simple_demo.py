#!/usr/bin/env python3

import sys
import os
import time
import argparse
from pathlib import Path
import cv2

project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent.parent  # DeepEmbody root
sys.path.insert(0, str(project_root_parent))

from uapi.runtime.runtime import Runtime
from uapi.runtime.provider import SkillProvider
from uapi.graph.entity import create_root_room, create_controllable_entity
from uapi.runtime.flow import set_runtime
from uapi.log import logger


def init_skill_providers(runtime: Runtime):
    local_provider = SkillProvider(
        name="local_provider",
        IP="127.0.0.1",
        skills=[
            "c_space_getpos",
            "c_space_move",
            "c_camera_rgb",
            "c_camera_dep_rgb",
            "c_camera_info",
            "c_save_rgb_image",
            "c_save_depth_image",
            "c_get_robot_pose",
        ],
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
        print("mock c_space_getpos called")
        return {"x": 1.0, "y": 2.0, "z": 0.0}

    def mock_move(**kwargs):
        x, y, z = kwargs.get("x", 0), kwargs.get("y", 0), kwargs.get("z", 0)
        print(f"mock c_space_move called with x={x}, y={y}, z={z}")
        time.sleep(2)
        return {"success": True}

    # Import real camera APIs directly from skill module
    from skill import (
        c_camera_rgb,
        c_camera_dep_rgb,
        c_camera_info,
        c_save_rgb_image,
        c_save_depth_image,
        c_get_robot_pose,
    )

    # Bind skills to entities
    entity_a.bind_skill("c_space_getpos", mock_getpos)
    entity_a.bind_skill("c_space_move", mock_move)
    entity_a.bind_skill("c_camera_rgb", c_camera_rgb)
    entity_a.bind_skill("c_camera_dep_rgb", c_camera_dep_rgb)
    entity_a.bind_skill("c_camera_info", c_camera_info)
    entity_a.bind_skill("c_save_rgb_image", c_save_rgb_image)
    entity_a.bind_skill("c_save_depth_image", c_save_depth_image)
    entity_a.bind_skill("c_get_robot_pose", c_get_robot_pose)

    entity_b.bind_skill("c_space_getpos", mock_getpos)
    entity_b.bind_skill("c_space_move", mock_move)
    entity_b.bind_skill("c_camera_rgb", c_camera_rgb)
    entity_b.bind_skill("c_camera_dep_rgb", c_camera_dep_rgb)
    entity_b.bind_skill("c_camera_info", c_camera_info)
    entity_b.bind_skill("c_save_rgb_image", c_save_rgb_image)
    entity_b.bind_skill("c_save_depth_image", c_save_depth_image)
    entity_b.bind_skill("c_get_robot_pose", c_get_robot_pose)

    runtime.set_graph(root_room)

    logger.info("initd entity graph:")
    logger.info(f"  root room: {root_room.get_absolute_path()}")
    logger.info(f"  entity A: {entity_a.get_absolute_path()}")
    logger.info(f"  entity B: {entity_b.get_absolute_path()}")


def init_entity_graph_from_yolo(runtime: Runtime):
    from skill import s_detect_objs, c_save_rgb_image, c_save_depth_image

    root_room = create_root_room()
    runtime.set_graph(root_room)

    robot = create_controllable_entity("robot")
    root_room.add_child(robot)

    def robot_move_impl(x, y, z):
        from driver.sim_genesis_ranger.driver import move_to_point

        move_to_point(x, y)  # THIS IS A FUNCTION FROM DRIVER !
        return {"success": True}

    def robot_getpos_impl():
        from driver.sim_genesis_ranger.driver import get_pose

        x, y, z, yaw = get_pose()
        return {"x": x, "y": y, "z": z}

    robot.bind_skill("c_space_move", robot_move_impl)
    robot.bind_skill("c_space_getpos", robot_getpos_impl)
    robot.bind_skill("c_save_rgb_image", c_save_rgb_image)
    robot.bind_skill("c_save_depth_image", c_save_depth_image)

    move_base = create_controllable_entity("move_base")
    robot.add_child(move_base)

    camera = create_controllable_entity("camera")
    robot.add_child(camera)

    camera.bind_skill("s_detect_objs", s_detect_objs)

    detect_objs = camera.s_detect_objs(camera_name="camera0")
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
        detected_entities_getpos_handler[obj_name] = lambda: {
            "x": x,
            "y": y,
            "z": 0.0,
        }
        obj_entity.bind_skill(
            "c_space_getpos", detected_entities_getpos_handler[obj_name]
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
    if args.mode == "manual":
        init_entity_graph_manually(runtime)
    elif args.mode == "auto":
        detected_entities = init_entity_graph_from_yolo(runtime)
    else:
        raise ValueError(f"invalid mode: {args.mode}")

    set_runtime(runtime)

    flow_program_path = os.path.join(os.path.dirname(__file__), "simple.flow")
    logger.info(f"loading flow program from: {flow_program_path}")

    try:
        flow_names = runtime.load_program(flow_program_path)
        logger.info(f"loaded flow functions: {flow_names}")

        if args.mode == "manual":
            # runtime.set_flow_args("move_a_to_b", a="/A", b="/B")
            runtime.set_flow_args("move_and_capture_flow", a="/A", b="/B")
        elif args.mode == "auto":
            # in auto mode, the entity graph is constructed using registered
            # s_detect_objs skill, and we choose the first object detected as
            # flow argument "b", and "a" fixed to "/robot". - wheatfox 2025.8.13
            if detected_entities:
                first_obj_name = list(detected_entities.keys())[0]
                first_obj_path = f"/{first_obj_name}"
                # runtime.set_flow_args("move_a_to_b", a="/robot", b=first_obj_path)
                runtime.set_flow_args(
                    "move_and_capture_flow", a="/robot", b=first_obj_path
                )
                logger.info(f"auto mode: set flow args for {first_obj_name}")
                logger.info(f"  move_a_to_b: a=/robot, b={first_obj_path}")
                logger.info(f"  move_and_capture_flow: a=/robot, b={first_obj_path}")
            else:
                logger.warning(
                    "auto mode: no objects detected, using default robot paths"
                )
                runtime.set_flow_args("move_a_to_b", a="/robot", b="/robot")
                runtime.set_flow_args("move_and_capture_flow", a="/robot", b="/robot")

        logger.info("starting all flows...")
        threads = runtime.start_all_flows()

        logger.info("waiting for all flows to complete...")
        results = runtime.wait_for_all_flows(timeout=30.0)

        logger.info("flow execution results:")
        for flow_name, result in results.items():
            logger.info(f"  {flow_name}: {result}")

    except Exception as e:
        logger.error(f"demo failed: {str(e)}", exc_info=True)
        return 1

    logger.info("demo completed successfully")
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
