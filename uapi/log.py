# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, syswonder
# this is deepembody's logging module, and can be used in both manager and uapi
from loguru import logger
import sys
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
    logger.add(
        sys.stderr,
        format="[{elapsed} <green>{name}</green> <level>{level}</level>] <level>{message}</level>",
        level=log_level_value,
        colorize=True,
        backtrace=True,
        diagnose=True,
    )


# Add handler at initialization
logger.remove()
logger.add(
    sys.stderr,
    format="[{elapsed} <green>{name}</green> <level>{level}</level>] <level>{message}</level>",
    level=LOG_LEVEL.value,
    colorize=True,
    backtrace=True,
    diagnose=True,
)
