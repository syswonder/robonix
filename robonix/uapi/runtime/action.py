"""
Action Module
=============

This module provides the action system for Robonix OS.
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
    global __RUNTIME_INSTANCE__
    if __RUNTIME_INSTANCE__ is None:
        from .runtime import Runtime

        __RUNTIME_INSTANCE__ = Runtime()
    return __RUNTIME_INSTANCE__


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
        
        # Try to find the actual .action file path by looking through the stack
        action_file_path = None
        
        # First, try to find .action files in the stack
        for frame_info in inspect.stack():
            frame_file = frame_info.filename
            if frame_file.endswith('.action'):
                action_file_path = frame_file
                break
        
        # If no .action file found, look for the closest action-related file
        if not action_file_path:
            for frame_info in inspect.stack():
                frame_file = frame_info.filename
                # Look for files that might contain action definitions
                if ('action' in frame_file.lower() and 
                    not frame_file.endswith('.py') and
                    'runtime' not in frame_file):
                    action_file_path = frame_file
                    break
        
        # If still no action file found, try to find from action_name context
        if not action_file_path and action_name != "unknown":
            # Look for directories that might contain the action
            import glob
            possible_dirs = [
                os.getcwd(),
                os.path.join(os.getcwd(), 'examples'),
                os.path.join(os.getcwd(), 'examples', 'demo5_patrol'),
            ]
            
            for dir_path in possible_dirs:
                if os.path.exists(dir_path):
                    # Look for .action files in this directory
                    action_files = glob.glob(os.path.join(dir_path, '*.action'))
                    if action_files:
                        action_file_path = action_files[0]
                        break
        
        # If still no action file found, use the current working directory
        if action_file_path:
            log_dir = os.path.dirname(os.path.abspath(action_file_path))
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
