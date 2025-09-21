# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, syswonder
# this is robonix's logging module, and can be used in both manager and uapi
from loguru import logger
import sys
import shutil
from enum import Enum
from rich.traceback import install


class EAIOS_LOG_LEVEL(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


LOG_LEVEL = EAIOS_LOG_LEVEL.INFO

# stacktrace
install(show_locals=False, width=120, word_wrap=True, extra_lines=3)

def get_terminal_width():
    try:
        return shutil.get_terminal_size().columns
    except:
        return 80

def create_format_function(use_alignment=True):
    if use_alignment:
        def format_record(record):
            file_line = f"{record['file']}:{record['line']}"
            return f"[<level>{record['level'].name: <8}</level> {record['elapsed']!s: <12} <green>{file_line: <35}</green>] <level>{record['message']}</level>\n"
    else:
        def format_record(record):
            file_line = f"{record['file']}:{record['line']}"
            return f"[<level>{record['level']}</level> {record['elapsed']} <green>{file_line}</green>] <level>{record['message']}</level>\n"
    return format_record


def set_log_level(level):
    """
    Dynamically set the log level. 'level' can be an EAIOS_LOG_LEVEL enum value or a string (e.g., "DEBUG").
    """
    global LOG_LEVEL
    logger.remove()
    if isinstance(level, EAIOS_LOG_LEVEL):
        LOG_LEVEL = level
        log_level_value = level.value
    elif isinstance(level, str):
        try:
            LOG_LEVEL = EAIOS_LOG_LEVEL[level.upper()]
            log_level_value = LOG_LEVEL.value
        except KeyError:
            raise ValueError(f"Invalid log level: {level}")
    else:
        raise TypeError("level must be EAIOS_LOG_LEVEL or str")
    
    terminal_width = get_terminal_width()
    use_alignment = terminal_width >= 90
    format_function = create_format_function(use_alignment)
    
    logger.add(
        sys.stderr,
        format=format_function,
        level=log_level_value,
        colorize=True,
        backtrace=True,
        diagnose=True,
    )


terminal_width = get_terminal_width()
use_alignment = terminal_width >= 90
format_function = create_format_function(use_alignment)

logger.remove()
logger.add(
    sys.stderr,
    format=format_function,
    level=LOG_LEVEL.value,
    colorize=True,
    backtrace=True,
    diagnose=True,
)
