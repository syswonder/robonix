#!/usr/bin/env python3

import sys
import os
import time
import argparse
from pathlib import Path

project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

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
            "c_image_capture",
        ],
    )

    runtime.registry.add_provider(local_provider)
    logger.info(
        f"added skill provider: {local_provider.name} with skills: {local_provider.skills}"
    )


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

    entity_a.bind_skill("c_space_getpos", mock_getpos)
    entity_a.bind_skill("c_space_move", mock_move)
    entity_b.bind_skill("c_space_getpos", mock_getpos)
    entity_b.bind_skill("c_space_move", mock_move)

    runtime.set_graph(root_room)

    logger.info("initd entity graph:")
    logger.info(f"  root room: {root_room.get_absolute_path()}")
    logger.info(f"  entity A: {entity_a.get_absolute_path()}")
    logger.info(f"  entity B: {entity_b.get_absolute_path()}")


def init_entity_graph_from_yolo(runtime: Runtime):
    pass


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
        init_entity_graph_from_yolo(runtime)
    else:
        raise ValueError(f"invalid mode: {args.mode}")

    set_runtime(runtime)

    flow_program_path = os.path.join(os.path.dirname(__file__), "flow.py")
    logger.info(f"loading flow program from: {flow_program_path}")

    try:
        flow_names = runtime.load_program(flow_program_path)
        logger.info(f"loaded flow functions: {flow_names}")

        runtime.set_flow_args("move_a_to_b", a="/A", b="/B")
        runtime.set_flow_args("simple_test_flow")

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
