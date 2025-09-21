# Copyright 2025 HuggingFace Inc., Daniel Han-Chen & the Unsloth team and the LlamaFactory team.
#
# This code is inspired by the HuggingFace's Transformers and PEFT library,
# https://github.com/huggingface/transformers/blob/v4.40.0/src/transformers/modeling_utils.py
# https://github.com/huggingface/peft/blob/v0.10.0/src/peft/utils/other.py
# and the Unsloth library.
# https://github.com/unslothai/unsloth/blob/July-2024/unsloth/models/_utils.py
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import inspect
from functools import WRAPPER_ASSIGNMENTS, partial, wraps
from types import MethodType
from typing import TYPE_CHECKING, Any, Callable, Optional, Union

import torch

from ..framework import logging

if TYPE_CHECKING:
    from transformers import PreTrainedModel

    from ..hparams import ModelArguments

logger = logging.get_logger(__name__)


def get_custom_gradient_checkpointing_func(
    gradient_checkpointing_func: Callable,
) -> Callable:
    r"""Only applies gradient checkpointing to trainable layers."""

    @wraps(gradient_checkpointing_func, assigned=WRAPPER_ASSIGNMENTS + ("__self__",))
    def custom_gradient_checkpointing_func(
        func: Callable, *args: Union["torch.Tensor", Any], **kwargs
    ):
        if isinstance(func, partial):
            module: torch.nn.Module = func.func.__self__
        else:
            module: torch.nn.Module = func.__self__

        has_grad = False
        if any(param.requires_grad for param in module.parameters()):
            has_grad = True
            for arg in args:
                if torch.is_tensor(arg) and torch.is_floating_point(arg):
                    arg.requires_grad_(True)
                    break  # assume the first tensor is always the hidden states

        if has_grad:
            return gradient_checkpointing_func(func, *args, **kwargs)
        else:
            return func(*args, **kwargs)

    return custom_gradient_checkpointing_func


def _gradient_checkpointing_enable(
    self: "PreTrainedModel",
    gradient_checkpointing_kwargs: Optional[dict[str, Any]] = None,
) -> None:
    r"""Activates gradient checkpointing for the current model.

    Modification of the original method to enable gradient checkpointing for block-wise optimizer.
    """
    from torch.utils.checkpoint import checkpoint

    if not self.supports_gradient_checkpointing:
        raise ValueError(
            f"{self.__class__.__name__} does not support gradient checkpointing."
        )

    if gradient_checkpointing_kwargs is None:
        gradient_checkpointing_kwargs = {"use_reentrant": True}

    gradient_checkpointing_func = partial(checkpoint, **gradient_checkpointing_kwargs)

    gradient_checkpointing_func = get_custom_gradient_checkpointing_func(
        gradient_checkpointing_func
    )
    if (
        "value" in inspect.signature(self._set_gradient_checkpointing).parameters
    ):  # old GC format
        self.apply(partial(self._set_gradient_checkpointing, value=True))
        self.enable_input_require_grads()
        logger.warning_rank0_once(
            "You are using the old GC format, some features (e.g. BAdam) will be invalid."
        )
    else:  # have already enabled input require gradients
        self._set_gradient_checkpointing(
            enable=True, gradient_checkpointing_func=gradient_checkpointing_func
        )


def prepare_model_for_training(
    model: "PreTrainedModel", model_args: "ModelArguments"
) -> None:
    r"""Prepare the model before training."""
    if not model_args.disable_gradient_checkpointing:
        if not getattr(model, "supports_gradient_checkpointing", False):
            logger.warning_rank0(
                "Current model does not support gradient checkpointing."
            )
        else:
            # use_reentrant=False might increase VRAM usage (have not been empirically verified yet)
            # According to: https://github.com/huggingface/transformers/issues/28339
            gradient_checkpointing_enable = partial(_gradient_checkpointing_enable)
            model.gradient_checkpointing_enable = MethodType(
                gradient_checkpointing_enable, model
            )
            model.gradient_checkpointing_enable(
                gradient_checkpointing_kwargs={
                    "use_reentrant": model_args.use_reentrant_gc
                }
            )
            setattr(
                model.config, "use_cache", False
            )  # turn off when gradient checkpointing is enabled
            logger.info_rank0("Gradient checkpointing enabled.")
