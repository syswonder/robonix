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


LOG_LEVEL = EAIOS_LOG_LEVEL.DEBUG

# stacktrace
install(show_locals=False, width=120, word_wrap=True, extra_lines=3)

logger.remove()
logger.add(
    sys.stderr,
    format="[{elapsed} <green>{name}</green> <level>{level}</level>] <level>{message}</level>",
    level=LOG_LEVEL.value,
    colorize=True,
    backtrace=True,
    diagnose=True,
)
