#!/usr/bin/env python3

from uapi.log import logger
from uapi import create_runtime_manager, set_runtime
import sys
import os
import argparse
from pathlib import Path

project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

project_root_parent = Path(
    __file__
).parent.parent.parent.parent.parent  # DeepEmbody root
sys.path.insert(0, str(project_root_parent))


def init_skill_providers(manager):
    """Initialize skill providers for ranger demo"""
    from uapi.runtime.provider import SkillProvider

    # dump __all__ in DeepEmbody.skill to skills list
    try:
        from DeepEmbody.skill import __all__
        skills = __all__
    except ImportError:
        logger.warning("DeepEmbody.skill module not available")
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
        from uapi.graph.entity import create_root_room, create_controllable_entity

        root_room = create_root_room()
        runtime.set_graph(root_room)

        # Create ranger entity
        ranger = create_controllable_entity("ranger")
        root_room.add_child(ranger)

        # Bind ranger-specific skills
        def ranger_get_pose_impl():
            try:
                from DeepEmbody.driver.sim_genesis_ranger.driver import get_pose
                x, y, z, yaw = get_pose()
                return (x, y, yaw)  # Return tuple as per skill spec
            except ImportError:
                logger.warning("Ranger driver not available, using mock pose")
                return (0.0, 0.0, 0.0)

        def ranger_move_impl(x, y, z):
            try:
                from DeepEmbody.driver.sim_genesis_ranger.driver import move_to_point
                # move_to_point(x, y)  # Uncomment when driver is available
                return {"success": True}
            except ImportError:
                logger.warning("Ranger driver not available, using mock move")
                return {"success": True}

        def ranger_getpos_impl():
            try:
                from DeepEmbody.driver.sim_genesis_ranger.driver import get_pose
                x, y, z, yaw = get_pose()
                return {"x": x, "y": y, "z": z}
            except ImportError:
                logger.warning(
                    "Ranger driver not available, using mock position")
                return {"x": 0.0, "y": 0.0, "z": 0.0}

        # Bind skills to ranger entity
        ranger.bind_skill("cap_get_pose", ranger_get_pose_impl)
        ranger.bind_skill("cap_space_move", ranger_move_impl)
        ranger.bind_skill("cap_space_getpos", ranger_getpos_impl)

        logger.info("Ranger entity graph initialized:")
        logger.info(f"  root room: {root_room.get_absolute_path()}")
        logger.info(f"  ranger: {ranger.get_absolute_path()}")

    return builder


def main():
    parser = argparse.ArgumentParser(description="Ranger Demo")
    parser.add_argument(
        "--export-scene",
        type=str,
        help="Export scene information to JSON file",
    )
    args = parser.parse_args()

    logger.info("Starting ranger demo")

    # Create runtime manager
    manager = create_runtime_manager()

    # Register ranger-specific entity builder
    manager.register_entity_builder("ranger", create_ranger_entity_builder())

    # Initialize skill providers
    init_skill_providers(manager)

    # Build entity graph
    manager.build_entity_graph("ranger")

    # Set runtime for action system
    set_runtime(manager.get_runtime())

    # Print entity tree structure
    manager.print_entity_tree()

    # Export scene information if requested
    if args.export_scene:
        scene_info = manager.export_scene_info(args.export_scene)
        logger.info(f"Scene information exported to: {args.export_scene}")

    # Load action program
    action_program_path = os.path.join(
        os.path.dirname(__file__), "ranger.action")
    logger.info(f"Loading action program from: {action_program_path}")

    try:
        action_names = manager.load_action_program(action_program_path)
        logger.info(f"Loaded action functions: {action_names}")

        # Configure action arguments
        manager.configure_action("test_ranger", ranger_path="/ranger")

        # Execute action
        logger.info("Executing ranger test action...")
        result = manager.execute_action("test_ranger")

        logger.info(f"Ranger test completed with result: {result}")
        logger.info("Demo completed successfully")
        return 0

    except Exception as e:
        logger.error(f"Demo failed: {str(e)}", exc_info=True)
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
