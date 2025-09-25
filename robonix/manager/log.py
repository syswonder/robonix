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


# Global configuration
LOG_LEVEL = EAIOS_LOG_LEVEL.INFO
LOG_FILENAME = "robonix.log"

# Install rich traceback
install(show_locals=False, width=120, word_wrap=True, extra_lines=3)


def get_terminal_width():
    """Get terminal width, fallback to 80 if unable to determine."""
    try:
        return shutil.get_terminal_size().columns
    except:
        return 80


def _setup_logger_handlers(level_value, filename):
    """Setup logger handlers with given level and filename."""
    # Console handler - 使用 Loguru 官方推荐的简单格式
    logger.add(
        sys.stderr,
        format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
        level=level_value,
        colorize=True,
        backtrace=True,
        diagnose=True,
    )

    # File handler - 使用简单的文件格式
    logger.add(
        filename,
        format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {level: <8} | {name}:{function}:{line} - {message}",
        level=level_value,
        colorize=False,
        backtrace=True,
        diagnose=True,
        rotation="10 MB",
        retention="7 days",
    )


def set_log_level(level, filename=None):
    """
    Dynamically set the log level. 
    
    Args:
        level: EAIOS_LOG_LEVEL enum value or string (e.g., "DEBUG")
        filename: Log filename, defaults to LOG_FILENAME if not provided
    """
    global LOG_LEVEL, LOG_FILENAME, _logger_initialized
    
    # Determine log level value
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
    
    # Use provided filename or default
    if filename is not None:
        LOG_FILENAME = filename
    
    # Remove existing handlers and setup new ones
    logger.remove()
    _setup_logger_handlers(log_level_value, LOG_FILENAME)
    _logger_initialized = True


# Global flag to track if logger has been initialized
_logger_initialized = False

def _ensure_logger_initialized():
    """Ensure logger is initialized only once."""
    global _logger_initialized
    if not _logger_initialized:
        logger.remove()
        _setup_logger_handlers(LOG_LEVEL.value, LOG_FILENAME)
        _logger_initialized = True

def init_logger(level=None, filename=None):
    """
    Initialize or reinitialize the logger with custom settings.
    This should be called before any logging operations.
    
    Args:
        level: Log level (EAIOS_LOG_LEVEL enum or string), defaults to LOG_LEVEL
        filename: Log filename, defaults to LOG_FILENAME
    """
    global _logger_initialized
    _logger_initialized = False
    
    if level is not None or filename is not None:
        set_log_level(level or LOG_LEVEL, filename)
    else:
        _ensure_logger_initialized()

# Initialize logger with default settings only if not already initialized
# This prevents automatic initialization when module is imported
# Users should call init_logger() or set_log_level() explicitly
