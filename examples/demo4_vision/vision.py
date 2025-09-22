#!/usr/bin/env python3

import sys
import os
import argparse
from pathlib import Path

project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent.parent  # Robonix root
sys.path.insert(0, str(project_root_parent))


from robonix.uapi import create_runtime_manager, set_runtime
from robonix.uapi.log import logger, set_log_level
from robonix.uapi.runtime.action import EOS_TYPE_ActionResult

from robonix.skill import *

set_log_level("debug")

def init_skill_providers(manager):
    """Initialize skill providers for ranger demo"""
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

    manager.get_runtime().registry.add_provider(local_provider)
    logger.info(f"Added skill providers: {manager.get_runtime().registry}")

def create_ranger_entity_builder():
    """Create a ranger-specific entity graph builder"""
    def builder(runtime, **kwargs):
        from robonix.uapi.graph.entity import create_root_room, create_controllable_entity

        root_room = create_root_room()
        runtime.set_graph(root_room)

        ranger = create_controllable_entity("ranger")
        root_room.add_child(ranger)

        ranger.bind_skill("cap_camera_rgb", cap_camera_rgb, "local_provider")
        ranger.bind_skill("cap_camera_dep_rgb", cap_camera_dep_rgb, "local_provider")
        ranger.bind_skill("cap_camera_info", cap_camera_info, "local_provider")
        ranger.bind_skill("cap_tf_transform", cap_tf_transform, "local_provider")
        
        ranger.bind_skill("skl_detect_objs", skl_detect_objs, "local_provider")

        logger.info("Ranger entity graph initialized:")
        logger.info(f"  root room: {root_room.get_absolute_path()}")
        logger.info(f"  ranger: {ranger.get_absolute_path()}")

    return builder


def main():
    parser = argparse.ArgumentParser(description="Vision Demo")
    parser.add_argument(
        "--export-scene",
        type=str,
        help="Export scene information to JSON file",
    )
    args = parser.parse_args()

    logger.info("Starting vision demo")

    manager = create_runtime_manager()
    manager.register_entity_builder("ranger", create_ranger_entity_builder())
    init_skill_providers(manager)
    manager.build_entity_graph("ranger")
    set_runtime(manager.get_runtime())
    manager.print_entity_tree()
    if args.export_scene:
        scene_info = manager.export_scene_info(args.export_scene)
        logger.info(f"Scene information exported to: {args.export_scene}")

    action_program_path = os.path.join(
        os.path.dirname(__file__), "vision.action")
    logger.info(f"Loading action program from: {action_program_path}")

    try:
        action_names = manager.load_action_program(action_program_path)
        logger.info(f"Loaded action functions: {action_names}")

        manager.configure_action("test_vision", ranger_path="/ranger")

        logger.info("Executing ranger test action...")
        result = manager.execute_action("test_vision")

        logger.info(f"Ranger test completed with result: {result}")
        
        if result != EOS_TYPE_ActionResult.SUCCESS:
            logger.error("Demo failed")
            return 1
        
        logger.info("Demo completed successfully")
        return 0

    except Exception as e:
        logger.error(f"Demo failed: {str(e)}", exc_info=True)
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
