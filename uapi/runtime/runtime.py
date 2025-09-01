from ..graph.entity import Entity
from .registry import Registry
from .action import get_action_functions
from ..log import logger
import threading
import importlib.util
import os
import inspect
from typing import List, Dict
import sys


class Runtime:
    def __init__(self):
        self.graph: Entity = None
        self.registry: Registry = Registry()
        self.action_threads: Dict[str, threading.Thread] = {}
        self.action_results: Dict[str, any] = {}
        self._action_args: Dict[str, dict] = {}

    def set_graph(self, graph: Entity):
        self.graph = graph

    def get_graph(self) -> Entity:
        return self.graph

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

        self._program_module = module

        return action_names

    def set_action_args(self, action_name: str, **kwargs):
        self._action_args[action_name] = kwargs

    def start_action(self, action_name: str):
        args = self._action_args.get(action_name, {})
        return self.execute_action(action_name, **args)

    def start_all_actions(self):
        threads = []
        for action_name in self._action_args:
            threads.append(self.start_action(action_name))
        return threads

    def execute_action(self, action_name: str, *args, **kwargs):
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
                logger.error(f"action {action_name} failed: {str(e)} at {inspect.currentframe().f_back.f_code.co_name}", exc_info=True)
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
