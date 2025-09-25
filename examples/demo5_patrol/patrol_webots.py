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

from robonix.uapi import create_runtime_manager, set_runtime
from robonix.manager.log import logger, set_log_level
from robonix.uapi.runtime.action import EOS_TYPE_ActionResult
from robonix.skill import *

set_log_level("debug", "demo5_patrol.log")

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

        ranger.bind_skill("cap_get_pose", get_pose)
        ranger.bind_skill("cap_set_goal", simple_set_goal)
        ranger.bind_skill("skl_move_to_rel_pos", move_to_rel_pos)

    return builder


def main():
    parser = argparse.ArgumentParser(description="Patrol Demo")
    parser.add_argument(
        "--export-scene",
        type=str,
        help="Export scene information to JSON file",
    )
    args = parser.parse_args()

    logger.info("Starting patrol demo")

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
        os.path.dirname(__file__), "patrol.action")
    logger.info(f"Loading action program from: {action_program_path}")

    try:
        action_names = manager.load_action_program(action_program_path)
        logger.info(f"Loaded action functions: {action_names}")

        manager.configure_action("test_patrol", ranger_path="/ranger")

        logger.info("Executing ranger test action finished...")
        result = manager.execute_action("test_patrol")

        logger.info(f"Ranger test finished with result: {result}")
        
        if result != EOS_TYPE_ActionResult.SUCCESS:
            logger.error("Demo failed")
            return 1
        
        logger.info("Demo finished")
        return 0

    except KeyboardInterrupt:
        logger.info("Demo interrupted by user")
        return 0
    except Exception as e:
        logger.error(f"Demo failed: {str(e)}", exc_info=True)
        return 1
    finally:
        # 确保 ROS2 节点正确关闭
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        logger.info("Program interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Program failed: {str(e)}", exc_info=True)
        sys.exit(1)
