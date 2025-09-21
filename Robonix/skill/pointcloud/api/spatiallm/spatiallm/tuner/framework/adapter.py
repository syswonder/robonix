# Copyright 2025 the LlamaFactory team.
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

from typing import TYPE_CHECKING, Set

import torch

from ..framework import logging


if TYPE_CHECKING:
    from transformers import PreTrainedModel

    from ..hparams import FinetuningArguments


logger = logging.get_logger(__name__)


def get_forbidden_modules(finetuning_args: "FinetuningArguments") -> Set[str]:
    r"""
    Freezes network modules for tuning.
    """
    forbidden_modules = set()
    if finetuning_args.train_proj_only:
        forbidden_modules.update({"point_backbone", "model", "lm_head"})
    elif finetuning_args.freeze_point_tower:
        forbidden_modules.add("point_backbone")
    elif finetuning_args.freeze_language_tower:
        forbidden_modules.update({"model", "lm_head"})

    return forbidden_modules


def _setup_full_tuning(
    model: "PreTrainedModel",
    finetuning_args: "FinetuningArguments",
    is_trainable: bool,
    cast_trainable_params_to_fp32: bool,
) -> None:
    if not is_trainable:
        return

    logger.info_rank0("Fine-tuning method: Full")
    forbidden_modules = get_forbidden_modules(finetuning_args)
    for name, param in model.named_parameters():
        if not any(forbidden_module in name for forbidden_module in forbidden_modules):
            if cast_trainable_params_to_fp32:
                param.data = param.data.to(torch.float32)
        else:
            param.data = param.data.to(torch.float32)
            param.requires_grad_(False)

    # force point_backbone to have float32
    model.set_point_backbone_dtype(torch.float32)


def init_adapter(
    model: "PreTrainedModel",
    finetuning_args: "FinetuningArguments",
    is_trainable: bool,
) -> "PreTrainedModel":
    r"""Initialize the adapters.

    Support only full-parameter training for now.

    Note that the trainable parameters must be cast to float32.
    """

    # cast trainable parameters to float32 if:
    # 1. is_trainable and not pure_bf16
    cast_trainable_params_to_fp32 = False
    if not is_trainable:
        pass
    elif finetuning_args.pure_bf16:
        logger.info_rank0(
            "Pure bf16 detected, remaining trainable params in half precision."
        )
    else:
        logger.info_rank0("Upcasting trainable params to float32.")
        cast_trainable_params_to_fp32 = True

    _setup_full_tuning(
        model, finetuning_args, is_trainable, cast_trainable_params_to_fp32
    )

    return model
