"""
Runtime Module
==============

This module provides the core runtime system for robonix OS.
The Runtime class manages entity graphs, action programs, skill registries,
and execution control for the entire system.

This is a singleton class that provides convenient interfaces for managing
robonix runtime operations.
"""

from ..graph.entity import Entity
from .registry import Registry
from .action import get_action_functions
from ...manager.log import logger
import threading
import importlib.util
import os
import inspect
from typing import List, Dict, Callable, Optional, Any
import sys
import json
from datetime import datetime


class Runtime:
    """
    Core runtime system for robonix OS (Singleton).

    The Runtime class is responsible for:
    - Managing entity graphs and their lifecycle
    - Loading and executing action programs
    - Coordinating skill providers through the registry
    - Controlling concurrent action execution
    - Providing hooks for system extensions
    - Entity builder registration and management
    - Convenient action execution interfaces
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(Runtime, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        # Only initialize once
        if hasattr(self, '_initialized'):
            return
        
        self.graph: Entity = None
        self.registry: Registry = Registry()
        self.action_threads: Dict[str, threading.Thread] = {}
        self.action_results: Dict[str, any] = {}
        self._action_args: Dict[str, dict] = {}

        # Entity graph construction hooks
        self._graph_hooks: List[Callable] = []
        self._graph_initialized: bool = False

        # Action program management
        self._loaded_programs: Dict[str, Any] = {}
        self._current_program: Optional[str] = None

        # Entity builder management (from RuntimeManager)
        self._entity_builders: Dict[str, Callable] = {}

        self._initialized = True

    def set_graph(self, graph: Entity):
        self.graph = graph
        self._graph_initialized = True

        # Execute all registered hooks
        for hook in self._graph_hooks:
            try:
                hook(self)
            except Exception as e:
                logger.error(f"Graph hook execution failed: {str(e)}", exc_info=True)

    def get_graph(self) -> Entity:
        return self.graph

    def add_graph_hook(self, hook: Callable[[Any], None]):
        """Add a hook function that will be called after graph is set"""
        self._graph_hooks.append(hook)

        # If graph is already initialized, execute the hook immediately
        if self._graph_initialized and self.graph is not None:
            try:
                hook(self)
            except Exception as e:
                logger.error(f"Graph hook execution failed: {str(e)}", exc_info=True)

    def remove_graph_hook(self, hook: Callable[[Any], None]):
        """Remove a graph hook function"""
        if hook in self._graph_hooks:
            self._graph_hooks.remove(hook)

    def load_program(self, program_path: str) -> List[str]:
        if not os.path.exists(program_path):
            raise FileNotFoundError(f"program file not found: {program_path}")

        # Read the action file content
        with open(program_path, "r", encoding="utf-8") as f:
            code = f.read()

        # Create a new module
        module = type(sys.modules[__name__])(
            f"program_{os.path.basename(program_path)}"
        )

        # Set __file__ attribute so action_print can determine the correct log directory
        module.__file__ = os.path.abspath(program_path)

        # Execute the code in the module's namespace
        exec(code, module.__dict__)

        action_functions = get_action_functions(module)
        action_names = [func.__name__ for func in action_functions]

        logger.info(
            f"loaded program {program_path} with action functions: {action_names}"
        )

        # Store the loaded program
        program_name = os.path.basename(program_path)
        self._loaded_programs[program_name] = {
            "module": module,
            "path": program_path,
            "action_names": action_names,
            "loaded_at": datetime.now().isoformat(),
        }

        self._program_module = module
        self._current_program = program_name

        return action_names

    def get_loaded_programs(self) -> Dict[str, Dict]:
        """Get information about all loaded programs"""
        return self._loaded_programs.copy()

    def get_current_program(self) -> Optional[str]:
        """Get the name of the currently active program"""
        return self._current_program

    def switch_program(self, program_name: str):
        """Switch to a previously loaded program"""
        if program_name not in self._loaded_programs:
            raise ValueError(f"Program '{program_name}' not found in loaded programs")

        self._program_module = self._loaded_programs[program_name]["module"]
        self._current_program = program_name
        logger.info(f"Switched to program: {program_name}")

    def set_action_args(self, action_name: str, **kwargs):
        self._action_args[action_name] = kwargs

    def get_action_args(self, action_name: str) -> Dict:
        """Get the arguments for a specific action"""
        return self._action_args.get(action_name, {})

    def clear_action_args(self, action_name: Optional[str] = None):
        """Clear action arguments for a specific action or all actions"""
        if action_name:
            if action_name in self._action_args:
                del self._action_args[action_name]
        else:
            self._action_args.clear()

    def start_action(self, action_name: str):
        args = self._action_args.get(action_name, {})
        return self._execute_action_worker(action_name, **args)

    def start_all_actions(self):
        threads = []
        for action_name in self._action_args:
            threads.append(self.start_action(action_name))
        return threads

    def _execute_action_worker(self, action_name: str, *args, **kwargs):
        if not hasattr(self, "_program_module"):
            raise RuntimeError("no program loaded. call load_program() first.")

        if not hasattr(self._program_module, action_name):
            raise ValueError(
                f"action function '{action_name}' not found in loaded program"
            )

        action_func = getattr(self._program_module, action_name)
        if not hasattr(action_func, "_is_action"):
            raise ValueError(f"function '{action_name}' is not a action function")

        def action_worker():
            try:
                # Set current entity context for skills and capabilities
                # This allows them to access self_entity parameter
                self._current_entity = self.graph

                result = action_func(*args, **kwargs)
                self.action_results[action_name] = result
            except Exception as e:
                self.action_results[action_name] = None
                logger.error(
                    f"action {action_name} failed: {str(e)} at {inspect.currentframe().f_back.f_code.co_name}",
                    exc_info=True,
                )
            finally:
                # Clean up current entity context
                if hasattr(self, "_current_entity"):
                    delattr(self, "_current_entity")

        thread = threading.Thread(target=action_worker, name=f"action_{action_name}")
        thread.daemon = True
        thread.start()

        self.action_threads[action_name] = thread

        return thread

    def wait_for_action(self, action_name: str, timeout: float = None):
        if action_name not in self.action_threads:
            raise ValueError(f"action '{action_name}' not found")

        thread = self.action_threads[action_name]
        thread.join(timeout=timeout)
        return self.action_results.get(action_name)

    def wait_for_all_actions(self, timeout: float = None):
        for thread in self.action_threads.values():
            thread.join(timeout=timeout)

        return self.action_results.copy()

    def get_action_status(self) -> Dict[str, bool]:
        status = {}
        for action_name, thread in self.action_threads.items():
            status[action_name] = not thread.is_alive()
        return status

    def export_entity_graph_info(self) -> Dict[str, Any]:
        """Export entity graph structure and bound skills as JSON-serializable dict"""
        if self.graph is None:
            return {"error": "No graph initialized"}

        graph_info = {
            "entities": {},
            "skills": {},
            "graph_structure": {},
            "exported_at": datetime.now().isoformat(),
        }

        def collect_entity_info(entity, parent_path=""):
            entity_path = entity.get_absolute_path()
            entity_name = entity.entity_name

            # Collect basic entity information
            graph_info["entities"][entity_path] = {
                "name": entity_name,
                "parent": parent_path,
                "children": [],
            }

            # Collect bound skill information
            bound_skills = entity.skills
            graph_info["skills"][entity_path] = bound_skills

            # Recursively process child entities
            for child in entity.get_children():
                child_path = child.get_absolute_path()
                graph_info["entities"][entity_path]["children"].append(child_path)
                collect_entity_info(child, entity_path)

        collect_entity_info(self.graph)

        # Build hierarchical graph structure
        def build_graph_structure(entity_path):
            entity_info = graph_info["entities"][entity_path]
            structure = {
                "name": entity_info["name"],
                "path": entity_path,
                "skills": graph_info["skills"][entity_path],
                "children": {},
            }

            for child_path in entity_info["children"]:
                child_name = graph_info["entities"][child_path]["name"]
                structure["children"][child_name] = build_graph_structure(child_path)

            return structure

        graph_info["graph_structure"] = build_graph_structure(
            self.graph.get_absolute_path()
        )

        return graph_info

    def export_skill_specs(self) -> Dict[str, Any]:
        """Export skill specifications as JSON-serializable dict"""
        from ..specs.skill_specs import EOS_SKILL_SPECS

        serializable_specs = {}
        for skill_name, skill_info in EOS_SKILL_SPECS.items():
            skill_type = skill_info["type"].value
            serializable_specs[skill_name] = {
                "description": str(skill_info["description"]),
                "type": str(skill_type),
                "input": (
                    str(skill_info["input"])
                    if skill_info["input"] is not None
                    else None
                ),
                "output": (
                    str(skill_info["output"])
                    if skill_info["output"] is not None
                    else None
                ),
                "dependencies": (
                    skill_info.get("dependencies", []) if skill_type == "SKILL" else []
                ),
            }

        return {
            "skill_specs": serializable_specs,
            "exported_at": datetime.now().isoformat(),
        }

    def export_runtime_info(self) -> Dict[str, Any]:
        """Export comprehensive runtime information including graph, specs, and programs"""
        return {
            "entity_graph": self.export_entity_graph_info(),
            "skill_specs": self.export_skill_specs(),
            "loaded_programs": self.get_loaded_programs(),
            "current_program": self.get_current_program(),
            "action_args": self._action_args,
            "action_status": self.get_action_status(),
            "exported_at": datetime.now().isoformat(),
        }

    def save_runtime_info(self, file_path: str):
        """Save runtime information to a JSON file"""
        runtime_info = self.export_runtime_info()

        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(runtime_info, f, indent=2, ensure_ascii=False)

        logger.info(f"Runtime information saved to: {file_path}")

    def dump_registry(self):
        # print a beatified structured of all skill providers and their skills in color
        print("\n" + "=" * 50)
        print("Skill Providers Registry:")
        print("=" * 50)

        for provider in self.registry.providers:
            print(f"Skill Provider: {provider.name}")
            print(f"  IP: {provider.IP}")
            if provider.port is not None:
                print(f"  Port: {provider.port}")
            print("  Skills: ", end="")
            provider.dump_skills()  # This method prints the skills list
            print("=" * 50)

    # Entity builder management methods (from RuntimeManager)
    def register_entity_builder(
        self, name: str, builder_func: Callable[["Runtime"], None]
    ):
        """Register an entity graph builder function"""
        self._entity_builders[name] = builder_func
        logger.info(f"Registered entity builder: {name}")

    def build_entity_graph(self, builder_name: str, **kwargs):
        """Build entity graph using a registered builder"""
        if builder_name not in self._entity_builders:
            raise ValueError(f"Entity builder '{builder_name}' not registered")

        logger.info(f"Building entity graph using builder: {builder_name}")
        self._entity_builders[builder_name](self, **kwargs)
        logger.info(f"Entity graph built successfully")

    # Convenient action execution methods (from RuntimeManager)
    def load_action_program(self, program_path: str) -> List[str]:
        """Load an action program and return action function names"""
        if not os.path.exists(program_path):
            raise FileNotFoundError(f"Action program not found: {program_path}")

        action_names = self.load_program(program_path)
        logger.info(f"Loaded action program: {program_path}")
        # Avoid f-string formatting conflicts by listing actions separately
        logger.info("Available actions:")
        for action in action_names:
            logger.info(f"  - {action}")
        return action_names

    def configure_action(self, action_name: str, **kwargs):
        """Configure arguments for an action"""
        self.set_action_args(action_name, **kwargs)
        # Avoid f-string formatting conflicts by using separate log statements
        logger.info(f"Configured action '{action_name}' with args:")
        for key, value in kwargs.items():
            logger.info(f"  {key}: {value}")

    def execute_action(
        self, action_name: str, wait: bool = True, timeout: float = 30.0
    ):
        """Execute an action and optionally wait for completion"""
        logger.info(f"Starting action execution: {action_name}")
        self.start_action(action_name)

        if wait:
            logger.info(f"Waiting for action '{action_name}' to complete...")
            result = self.wait_for_action(action_name, timeout=timeout)
            logger.info(f"Action '{action_name}' completed with result: {result}")
            return result
        else:
            logger.info(f"Action '{action_name}' started in background")
            return None

    def execute_all_actions(self, timeout: float = 30.0):
        """Execute all configured actions and wait for completion"""
        logger.info("Starting execution of all configured actions...")
        self.start_all_actions()

        logger.info("Waiting for all actions to complete...")
        results = self.wait_for_all_actions(timeout=timeout)

        logger.info("All actions completed:")
        for action_name, result in results.items():
            logger.info(f"  {action_name}: {result}")

        return results

    def export_scene_info(self, file_path: Optional[str] = None) -> Dict[str, Any]:
        """Export comprehensive scene information including entity graph and skill specs"""
        scene_info = {
            "entity_graph": self.export_entity_graph_info(),
            "skill_specs": self.export_skill_specs(),
            "exported_at": datetime.now().isoformat(),
        }

        if file_path:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(scene_info, f, indent=2, ensure_ascii=False)
            logger.info(f"Scene information saved to: {file_path}")

        return scene_info

    def print_entity_tree(self):
        """Print the current entity tree structure"""
        if self.graph is None:
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

        print_entity_tree_recursive(self.graph)
        print("=" * 50)


# Convenience function to get the singleton runtime instance
def get_runtime() -> Runtime:
    """Get the singleton Runtime instance"""
    return Runtime()
