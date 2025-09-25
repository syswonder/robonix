#!/usr/bin/env python3

import argparse
import sys
import os
from pathlib import Path

project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent  # Robonix root
sys.path.insert(0, str(project_root_parent))


from robonix.uapi import create_runtime_manager, set_runtime, RuntimeManager
from robonix.manager.log import logger, set_log_level
from robonix.skill import *

set_log_level("debug")


def init_skill_providers(manager: RuntimeManager):
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

    rtx5090server_provider = SkillProvider(
        name="rtx5090server_provider",
        IP="162.105.88.184",
        port=50051,
        skills=["skl_spatiallm_detect"],
    )

    manager.get_runtime().registry.add_provider(local_provider)
    manager.get_runtime().registry.add_provider(rtx5090server_provider)
    logger.info("Skill providers registered successfully")
    manager.get_runtime().dump_registry()


def create_ranger_entity_builder():
    """Create a pointcloud-specific entity graph builder"""
    def builder(runtime, **kwargs):
        from robonix.uapi.graph.entity import create_root_room, create_controllable_entity

        root_room = create_root_room()
        runtime.set_graph(root_room)

        ranger = create_controllable_entity("ranger")
        root_room.add_child(ranger)
        ranger.bind_skill("cap_pointcloud_to_file",
                          cap_pointcloud_to_file, provider_name="local_provider")

        server_pc = create_controllable_entity("server_pc")
        root_room.add_child(server_pc)

        server_pc.bind_skill("skl_spatiallm_detect",
                             __rpc_skl_spatiallm_detect,
                             provider_name="rtx5090server_provider")

    return builder


def main():
    parser = argparse.ArgumentParser(description="Pointcloud Demo")
    parser.add_argument(
        "--export-scene",
        type=str,
        help="Export scene information to JSON file",
    )
    args = parser.parse_args()

    logger.info("Starting pointcloud demo")

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
        os.path.dirname(__file__), "pointcloud.action")
    logger.info(f"Loading action program from: {action_program_path}")

    try:
        action_names = manager.load_action_program(action_program_path)
        logger.info(f"Loaded action functions: {action_names}")

        manager.configure_action(
            "test_ranger_pointcloud", ranger_path="/ranger", server_pc_path="/server_pc")

        logger.info("Executing pointcloud test action...")
        result = manager.execute_action("test_ranger_pointcloud")

        logger.info(f"Pointcloud test completed with result: {result}")
        logger.info("Demo completed successfully")
        return 0

    except Exception as e:
        logger.error(f"Demo failed: {str(e)}", exc_info=True)
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
