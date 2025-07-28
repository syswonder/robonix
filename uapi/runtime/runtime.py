from uapi.graph.entity import Entity
from uapi.runtime.registry import Registry
from uapi.runtime.flow import get_flow_functions
from uapi.log import logger
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
        self.flow_threads: Dict[str, threading.Thread] = {}
        self.flow_results: Dict[str, any] = {}
        self._flow_args: Dict[str, dict] = {}

    def set_graph(self, graph: Entity):
        self.graph = graph

    def get_graph(self) -> Entity:
        return self.graph

    def load_program(self, program_path: str) -> List[str]:
        if not os.path.exists(program_path):
            raise FileNotFoundError(f"program file not found: {program_path}")

        # Read the flow file content
        with open(program_path, 'r', encoding='utf-8') as f:
            code = f.read()

        # Create a new module
        module = type(sys.modules[__name__])(f"program_{os.path.basename(program_path)}")
        
        # Set __file__ attribute so flow_print can determine the correct log directory
        module.__file__ = os.path.abspath(program_path)
        
        # Execute the code in the module's namespace
        exec(code, module.__dict__)

        flow_functions = get_flow_functions(module)
        flow_names = [func.__name__ for func in flow_functions]

        logger.info(f"loaded program {program_path} with flow functions: {flow_names}")

        self._program_module = module

        return flow_names

    def set_flow_args(self, flow_name: str, **kwargs):
        self._flow_args[flow_name] = kwargs

    def start_flow(self, flow_name: str):
        args = self._flow_args.get(flow_name, {})
        return self.execute_flow(flow_name, **args)

    def start_all_flows(self):
        threads = []
        for flow_name in self._flow_args:
            threads.append(self.start_flow(flow_name))
        return threads

    def execute_flow(self, flow_name: str, *args, **kwargs):
        if not hasattr(self, "_program_module"):
            raise RuntimeError("no program loaded. call load_program() first.")

        if not hasattr(self._program_module, flow_name):
            raise ValueError(f"flow function '{flow_name}' not found in loaded program")

        flow_func = getattr(self._program_module, flow_name)
        if not hasattr(flow_func, "_is_flow"):
            raise ValueError(f"function '{flow_name}' is not a flow function")

        def flow_worker():
            try:
                result = flow_func(*args, **kwargs)
                self.flow_results[flow_name] = result
            except Exception as e:
                self.flow_results[flow_name] = None
                logger.error(f"flow {flow_name} failed: {str(e)}", exc_info=True)

        thread = threading.Thread(target=flow_worker, name=f"flow_{flow_name}")
        thread.daemon = True
        thread.start()

        self.flow_threads[flow_name] = thread

        return thread

    def wait_for_flow(self, flow_name: str, timeout: float = None):
        if flow_name not in self.flow_threads:
            raise ValueError(f"flow '{flow_name}' not found")

        thread = self.flow_threads[flow_name]
        thread.join(timeout=timeout)
        return self.flow_results.get(flow_name)

    def wait_for_all_flows(self, timeout: float = None):
        for thread in self.flow_threads.values():
            thread.join(timeout=timeout)

        return self.flow_results.copy()

    def get_flow_status(self) -> Dict[str, bool]:
        status = {}
        for flow_name, thread in self.flow_threads.items():
            status[flow_name] = not thread.is_alive()
        return status
