from enum import Enum
import threading
import inspect
from typing import Dict, List, Callable, Any
from uapi.log import logger
import os
from datetime import datetime


class EOS_TYPE_ActionResult(Enum):
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


def action(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        try:
            result = func(*args, **kwargs)
            return result
        except Exception as e:
            logger.error(
                f"action function {func.__name__} failed: {str(e)}", exc_info=True
            )
            return EOS_TYPE_ActionResult.FAILURE

    wrapper._is_action = True
    wrapper._original_func = func
    wrapper.__name__ = func.__name__

    return wrapper


def get_action_functions(module) -> List[Callable]:
    action_functions = []
    for name, obj in inspect.getmembers(module):
        if inspect.isfunction(obj) and hasattr(obj, "_is_action"):
            action_functions.append(obj)
    return action_functions


def set_runtime(runtime):
    global __RUNTIME_INSTANCE__
    __RUNTIME_INSTANCE__ = runtime


def action_print(message: str, level: str = "INFO"):

    frame = inspect.currentframe()
    caller_frame = frame.f_back

    action_name = "unknown"
    if caller_frame:
        for frame_info in inspect.stack():
            func_name = frame_info.function
            if func_name in frame_info.frame.f_globals:
                func_obj = frame_info.frame.f_globals[func_name]
                if hasattr(func_obj, "_is_action"):
                    action_name = func_name
                    break

    thread_id = threading.get_ident()
    thread_name = threading.current_thread().name

    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    colors = {
        "DEBUG": "\033[36m",  # Cyan
        "INFO": "\033[37m",  # White
        "WARN": "\033[33m",  # Yellow
        "ERROR": "\033[31m",  # Red
        "CRITICAL": "\033[35m",  # Magenta
        "RESET": "\033[0m",  # Reset
        "WHITE": "\033[37m",  # White for message
        "CYAN": "\033[36m",  # Cyan for timestamp and action name
        "YELLOW": "\033[33m",  # Yellow for level
    }

    level_color = colors.get(level.upper(), colors["INFO"])
    console_message = f"ACTION_LOG:{colors['CYAN']}[{timestamp}]{colors['RESET']}{level_color}[{level.upper()}]{colors['RESET']}{colors['CYAN']}[{action_name}]{colors['RESET']} {colors['WHITE']}{message}{colors['RESET']}"

    full_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    file_message = f"[{full_timestamp}][{level.upper()}][{action_name}][{thread_name}({thread_id})] {message}"

    # Print to console
    print(console_message)

    try:
        frame = inspect.currentframe()
        caller_frame = frame.f_back
        module_path = caller_frame.f_globals.get("__file__", "")

        if module_path:
            log_dir = os.path.dirname(os.path.abspath(module_path))
        else:
            log_dir = os.getcwd()

        log_file = os.path.join(log_dir, f"{action_name}.log")
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(file_message + "\n")
    except Exception as e:
        print(f"Warning: Could not write to log file: {e}")


def action_debug(message: str):
    action_print(message, "DEBUG")


def action_info(message: str):
    action_print(message, "INFO")


def action_warning(message: str):
    action_print(message, "WARN")


def action_error(message: str):
    action_print(message, "ERROR")


def action_critical(message: str):
    action_print(message, "CRITICAL")
