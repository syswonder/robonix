"""
Action Module
=============

This module provides the action system for robonix OS.
It includes action decorators, execution control, runtime management,
and result types for the action framework.
"""

from enum import Enum
import threading
import inspect
from typing import Dict, List, Callable, Any
from ...manager.log import logger
import os
from datetime import datetime


class EOS_TYPE_ActionResult(Enum):
    """Enumeration for action execution results."""

    SUCCESS = 0
    FAILURE = 1
    ABORT = 2


__RUNTIME_INSTANCE__ = None


def get_runtime():
    """Get the singleton Runtime instance"""
    from .runtime import get_runtime as _get_runtime
    return _get_runtime()


def action(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        try:
            result = func(*args, **kwargs)
            return result
        except Exception as e:
            logger.error(
                f"action function {func.__name__} failed: {str(e)} at {inspect.currentframe().f_back.f_code.co_name}, line {inspect.currentframe().f_back.f_lineno}",
                exc_info=True,
            )
            import traceback

            colors = {
                "RED": "\033[31m",
                "YELLOW": "\033[33m",
                "CYAN": "\033[36m",
                "WHITE": "\033[37m",
                "BOLD": "\033[1m",
                "RESET": "\033[0m",
                "BG_RED": "\033[41m",
                "BG_YELLOW": "\033[43m",
            }

            print(
                f"\n{colors['BOLD']}{colors['BG_RED']}{colors['WHITE']}============= STACKTRACE for {func.__name__} ============={colors['RESET']}"
            )
            print(
                f"{colors['YELLOW']}Error:{colors['RESET']} {colors['RED']}{str(e)}{colors['RESET']}"
            )
            print(
                f"{colors['CYAN']}Time:{colors['RESET']} {colors['WHITE']}{datetime.now().strftime('%H:%M:%S')}{colors['RESET']}"
            )
            print(
                f"{colors['CYAN']}Thread:{colors['RESET']} {colors['WHITE']}{threading.current_thread().name}{colors['RESET']}"
            )
            print()
            print(f"{colors['YELLOW']}{traceback.format_exc()}{colors['RESET']}")
            print(
                f"{colors['BOLD']}{colors['BG_YELLOW']}{colors['WHITE']}{'=' * 60}{colors['RESET']}"
            )
            return EOS_TYPE_ActionResult.FAILURE

    # Get the source file information when decorating
    try:
        frame = inspect.currentframe()
        if frame and frame.f_back:
            source_file = frame.f_back.f_globals.get('__file__', 'unknown')
            if source_file.endswith('.action'):
                wrapper._action_filename = os.path.splitext(os.path.basename(source_file))[0]
            else:
                wrapper._action_filename = os.path.splitext(os.path.basename(source_file))[0]
        else:
            wrapper._action_filename = 'unknown'
    except:
        wrapper._action_filename = 'unknown'

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
    """Set the runtime instance (deprecated - Runtime is now singleton)"""
    global __RUNTIME_INSTANCE__
    __RUNTIME_INSTANCE__ = runtime
    logger.warning("set_runtime() is deprecated - Runtime is now singleton")


def action_print(message: str, level: str = "INFO"):

    frame = inspect.currentframe()
    caller_frame = frame.f_back

    action_name = "unknown"
    action_filename = "unknown"
    if caller_frame:
        for frame_info in inspect.stack():
            func_name = frame_info.function
            if func_name in frame_info.frame.f_globals:
                func_obj = frame_info.frame.f_globals[func_name]
                if hasattr(func_obj, "_is_action"):
                    action_name = func_name
                    # Use the saved filename from the action decorator
                    action_filename = getattr(func_obj, "_action_filename", "unknown")
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
    console_message = f"ACTION_LOG:{colors['CYAN']}[{timestamp}]{colors['RESET']}{level_color}[{level.upper()}]{colors['RESET']}{colors['CYAN']}[{action_filename}:{action_name}]{colors['RESET']} {colors['WHITE']}{message}{colors['RESET']}"

    print(console_message)

    try:
        full_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        file_message = f"[{full_timestamp}][{action_filename}:{action_name}({thread_id})] {message}"
        
        log_filename = f"{action_filename}__{action_name}.log"
        log_file = os.path.join(os.getcwd(), log_filename)
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
