import os
import importlib.metadata
import importlib.util
from functools import lru_cache
from typing import TYPE_CHECKING

import torch
from packaging import version
from transformers.utils import (
    is_torch_cuda_available,
    is_torch_mps_available,
    is_torch_npu_available,
    is_torch_xpu_available,
)
from transformers.utils.versions import require_version

from ..framework import logging

if TYPE_CHECKING:
    from packaging.version import Version

logger = logging.get_logger(__name__)


def has_length(dataset):
    """
    Checks if the dataset implements __len__() and it doesn't raise an error
    """
    try:
        return len(dataset) is not None
    except TypeError:
        # TypeError: len() of unsized object
        return False


def is_env_enabled(env_var: str, default: str = "0") -> bool:
    r"""Check if the environment variable is enabled."""
    return os.getenv(env_var, default).lower() in ["true", "y", "1"]


def check_version(requirement: str, mandatory: bool = False) -> None:
    r"""Optionally check the package version."""
    if is_env_enabled("DISABLE_VERSION_CHECK") and not mandatory:
        logger.warning_rank0_once(
            "Version checking has been disabled, may lead to unexpected behaviors."
        )
        return

    if "gptmodel" in requirement or "autoawq" in requirement:
        pip_command = f"pip install {requirement} --no-build-isolation"
    else:
        pip_command = f"pip install {requirement}"

    if mandatory:
        hint = f"To fix: run `{pip_command}`."
    else:
        hint = f"To fix: run `{pip_command}` or set `DISABLE_VERSION_CHECK=1` to skip this check."

    require_version(requirement, hint)


def count_parameters(model: "torch.nn.Module") -> tuple[int, int]:
    r"""Return the number of trainable parameters and number of all parameters in the model."""
    trainable_params, all_param = 0, 0
    for param in model.parameters():
        num_params = param.numel()

        all_param += num_params
        if param.requires_grad:
            trainable_params += num_params

    return trainable_params, all_param


def _get_package_version(name: str) -> "Version":
    try:
        return version.parse(importlib.metadata.version(name))
    except Exception:
        return version.parse("0.0.0")


@lru_cache
def is_transformers_version_greater_than(content: str):
    return _get_package_version("transformers") >= version.parse(content)


def get_current_device() -> "torch.device":
    r"""
    Gets the current available device.
    """
    if is_torch_xpu_available():
        device = "xpu:{}".format(os.environ.get("LOCAL_RANK", "0"))
    elif is_torch_npu_available():
        device = "npu:{}".format(os.environ.get("LOCAL_RANK", "0"))
    elif is_torch_mps_available():
        device = "mps:{}".format(os.environ.get("LOCAL_RANK", "0"))
    elif is_torch_cuda_available():
        device = "cuda:{}".format(os.environ.get("LOCAL_RANK", "0"))
    else:
        device = "cpu"

    return torch.device(device)


def get_device_count() -> int:
    r"""Get the number of available devices."""
    if is_torch_xpu_available():
        return torch.xpu.device_count()
    elif is_torch_npu_available():
        return torch.npu.device_count()
    elif is_torch_mps_available():
        return torch.mps.device_count()
    elif is_torch_cuda_available():
        return torch.cuda.device_count()
    else:
        return 0


def check_dependencies() -> None:
    r"""
    Checks the version of the required packages.
    """
    check_version("transformers>=4.41.2,<=4.46.1")
    check_version("datasets>=2.16.0,<=3.6.0")
    check_version("accelerate>=0.34.0,<=1.7.0")
