#!/usr/bin/env python3

import sys
import os
import argparse
from pathlib import Path

project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent  # robonix root
sys.path.insert(0, str(project_root_parent))

from robonix.uapi import get_runtime, set_runtime
from robonix.manager.log import logger, set_log_level

from robonix.skill import *

set_log_level("debug")

def init_skill_providers(runtime):
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

    runtime.registry.add_provider(local_provider)
    logger.info(f"Added skill providers: {runtime.registry}")
    rtx5090server_provider = SkillProvider(
        name="rtx5090server_provider",
        IP="162.105.88.184",
        port=50051,
        skills=["skl_spatiallm_detect"],
    )

    runtime.registry.add_provider(rtx5090server_provider)
    logger.info("Skill providers registered successfully")
    runtime.dump_registry()

def create_ranger_entity_builder():
    """Create a ranger-specific entity graph builder"""
    def builder(runtime, **kwargs):
        from robonix.uapi.graph.entity import create_root_room, create_controllable_entity

        root_room = create_root_room()
        runtime.set_graph(root_room)

        ranger = create_controllable_entity("ranger")
        root_room.add_child(ranger)

        ranger.bind_skill("cap_get_pose", get_pose)
        ranger.bind_skill("cap_set_goal", simple_set_goal)
        ranger.bind_skill("skl_move_to_rel_pos", move_to_rel_pos)
        ranger.bind_skill("skl_move_to_ab_pos", move_to_ab_pos)
        ranger.bind_skill("cap_pointcloud_to_file", cap_pointcloud_to_file)

        logger.info("Ranger entity graph initialized:")
        logger.info(f"  root room: {root_room.get_absolute_path()}")
        logger.info(f"  ranger: {ranger.get_absolute_path()}")
        ranger.bind_skill("cap_pointcloud_to_file",
                          cap_pointcloud_to_file, provider_name="local_provider")

        server_pc = create_controllable_entity("server_pc")
        root_room.add_child(server_pc)

        server_pc.bind_skill("skl_spatiallm_detect",
                             __rpc_skl_spatiallm_detect,
                             provider_name="rtx5090server_provider")
        ranger.bind_skill("skl_spatiallm_to_world_pose", skl_spatiallm_to_world_pose, provider_name="local_provider")

    return builder


def main():
    parser = argparse.ArgumentParser(description="Find Chair Demo")
    parser.add_argument(
        "--export-scene",
        type=str,
        help="Export scene information to JSON file",
    )
    args = parser.parse_args()

    logger.info("Starting find_chair demo")

    runtime = get_runtime()
    runtime.register_entity_builder("ranger", create_ranger_entity_builder())
    init_skill_providers(runtime)
    runtime.build_entity_graph("ranger")
    set_runtime(runtime)
    runtime.print_entity_tree()
    if args.export_scene:
        scene_info = runtime.export_scene_info(args.export_scene)
        logger.info(f"Scene information exported to: {args.export_scene}")

    action_program_path = os.path.join(
        os.path.dirname(__file__), "findchair.action")
    logger.info(f"Loading action program from: {action_program_path}")

    try:
        action_names = runtime.load_action_program(action_program_path)
        logger.info(f"Loaded action functions: {action_names}")

        runtime.configure_action("find_chair", ranger_path="/ranger", server_pc_path="/server_pc")

        logger.info("Executing find_chair action...")
        thread = runtime.start_action("find_chair")
        result = runtime.wait_for_action("find_chair", timeout=90.0)

        logger.info(f"find_chair completed with result: {result}")
        logger.info("Demo completed successfully")
        return 0

    except Exception as e:
        logger.error(f"Demo failed: {str(e)}", exc_info=True)
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
