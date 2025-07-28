from enum import Enum
import threading
import inspect
from typing import Dict, List, Callable, Any
from uapi.log import logger


class EOS_TYPE_FlowResult(Enum):
    SUCCESS = 0
    FAILURE = 1
    ABORT = 2


__RUNTIME_INSTANCE__ = None


def get_runtime():
    global __RUNTIME_INSTANCE__
    if __RUNTIME_INSTANCE__ is None:
        from uapi.runtime.runtime import Runtime

        __RUNTIME_INSTANCE__ = Runtime()
    return __RUNTIME_INSTANCE__


def flow(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        try:
            result = func(*args, **kwargs)
            return result
        except Exception as e:
            logger.error(f"flow function {func.__name__} failed: {str(e)}", exc_info=True)
            return EOS_TYPE_FlowResult.FAILURE

    wrapper._is_flow = True
    wrapper._original_func = func
    wrapper.__name__ = func.__name__

    return wrapper


def get_flow_functions(module) -> List[Callable]:
    flow_functions = []
    for name, obj in inspect.getmembers(module):
        if inspect.isfunction(obj) and hasattr(obj, "_is_flow"):
            flow_functions.append(obj)
    return flow_functions


def set_runtime(runtime):
    global __RUNTIME_INSTANCE__
    __RUNTIME_INSTANCE__ = runtime
