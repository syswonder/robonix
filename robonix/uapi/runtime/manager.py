"""
Runtime Manager - Provides convenient interfaces for managing Robonix runtime
"""

from typing import Dict, List, Callable, Optional, Any
from .runtime import Runtime
from ...manager.log import logger
import os
import json
from datetime import datetime


class RuntimeManager:
    """High-level manager for Robonix runtime operations"""

    def __init__(self):
        self.runtime = Runtime()
        self._entity_builders: Dict[str, Callable] = {}

    def get_runtime(self) -> Runtime:
        """Get the underlying runtime instance"""
        return self.runtime

    def register_entity_builder(
        self, name: str, builder_func: Callable[[Runtime], None]
    ):
        """Register an entity graph builder function"""
        self._entity_builders[name] = builder_func
        logger.info(f"Registered entity builder: {name}")

    def build_entity_graph(self, builder_name: str, **kwargs):
        """Build entity graph using a registered builder"""
        if builder_name not in self._entity_builders:
            raise ValueError(f"Entity builder '{builder_name}' not registered")

        logger.info(f"Building entity graph using builder: {builder_name}")
        self._entity_builders[builder_name](self.runtime, **kwargs)
        logger.info(f"Entity graph built successfully")

    def load_action_program(self, program_path: str) -> List[str]:
        """Load an action program and return action function names"""
        if not os.path.exists(program_path):
            raise FileNotFoundError(f"Action program not found: {program_path}")

        action_names = self.runtime.load_program(program_path)
        logger.info(f"Loaded action program: {program_path}")
        # Avoid f-string formatting conflicts by listing actions separately
        logger.info("Available actions:")
        for action in action_names:
            logger.info(f"  - {action}")
        return action_names

    def configure_action(self, action_name: str, **kwargs):
        """Configure arguments for an action"""
        self.runtime.set_action_args(action_name, **kwargs)
        # Avoid f-string formatting conflicts by using separate log statements
        logger.info(f"Configured action '{action_name}' with args:")
        for key, value in kwargs.items():
            logger.info(f"  {key}: {value}")

    def execute_action(
        self, action_name: str, wait: bool = True, timeout: float = 30.0
    ):
        """Execute an action and optionally wait for completion"""
        logger.info(f"Starting action execution: {action_name}")
        self.runtime.start_action(action_name)

        if wait:
            logger.info(f"Waiting for action '{action_name}' to complete...")
            result = self.runtime.wait_for_action(action_name, timeout=timeout)
            logger.info(f"Action '{action_name}' completed with result: {result}")
            return result
        else:
            logger.info(f"Action '{action_name}' started in background")
            return None

    def execute_all_actions(self, timeout: float = 30.0):
        """Execute all configured actions and wait for completion"""
        logger.info("Starting execution of all configured actions...")
        self.runtime.start_all_actions()

        logger.info("Waiting for all actions to complete...")
        results = self.runtime.wait_for_all_actions(timeout=timeout)

        logger.info("All actions completed:")
        for action_name, result in results.items():
            logger.info(f"  {action_name}: {result}")

        return results

    def export_scene_info(self, file_path: Optional[str] = None) -> Dict[str, Any]:
        """Export comprehensive scene information including entity graph and skill specs"""
        scene_info = {
            "entity_graph": self.runtime.export_entity_graph_info(),
            "skill_specs": self.runtime.export_skill_specs(),
            "exported_at": datetime.now().isoformat(),
        }

        if file_path:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(scene_info, f, indent=2, ensure_ascii=False)
            logger.info(f"Scene information saved to: {file_path}")

        return scene_info

    def print_entity_tree(self):
        """Print the current entity tree structure"""
        if self.runtime.graph is None:
            logger.warning("No entity graph initialized")
            return

        print("\n" + "=" * 50)
        print("Entity Graph Structure:")
        print("=" * 50)

        def print_entity_tree_recursive(entity, prefix="", is_last=True):
            """Recursively print entity tree structure with skills"""
            entity_path = entity.get_absolute_path()
            entity_name = entity.entity_name

            # ANSI color codes
            RESET = "\033[0m"
            BOLD = "\033[1m"
            BLUE = "\033[94m"  # Entity name color
            GREEN = "\033[92m"  # Skills color
            YELLOW = "\033[93m"  # Path color
            GRAY = "\033[90m"  # No skills color

            # Print current entity with skills on the same line
            connector = "└── " if is_last else "├── "
            if entity.skills:
                skills_str = f" {GREEN}[skills: {', '.join(entity.skills)}]{RESET}"
            else:
                skills_str = f" {GRAY}[no skills]{RESET}"

            print(
                f"{prefix}{connector}{BOLD}{BLUE}{entity_name}{RESET} {YELLOW}({entity_path}){RESET}{skills_str}"
            )

            # Recursively print children
            children = entity.get_children()
            for i, child in enumerate(children):
                is_last_child = i == len(children) - 1
                print_entity_tree_recursive(
                    child, prefix + ("    " if is_last else "│   "), is_last_child
                )

        print_entity_tree_recursive(self.runtime.graph)
        print("=" * 50)


# Convenience function to create a configured manager
def create_runtime_manager() -> RuntimeManager:
    """Create a RuntimeManager"""
    return RuntimeManager()
