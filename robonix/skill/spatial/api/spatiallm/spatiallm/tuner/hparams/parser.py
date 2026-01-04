# Copyright 2025 HuggingFace Inc. and the LlamaFactory team.
#
# This code is inspired by the HuggingFace's transformers library.
# https://github.com/huggingface/transformers/blob/v4.40.0/examples/pytorch/language-modeling/run_clm.py
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

import json
import os
import sys
from pathlib import Path
from typing import Any, Optional, Union

import torch
import transformers
import yaml
from omegaconf import OmegaConf
from transformers import HfArgumentParser
from transformers.integrations import is_deepspeed_zero3_enabled
from transformers.trainer_utils import get_last_checkpoint
from transformers.training_args import ParallelMode
from transformers.utils import is_torch_bf16_gpu_available, is_torch_npu_available
from transformers.utils import (
    SAFE_WEIGHTS_INDEX_NAME,
    SAFE_WEIGHTS_NAME,
    WEIGHTS_INDEX_NAME,
    WEIGHTS_NAME,
)

from ..framework import logging
from ..framework.utils import (
    check_dependencies,
    get_current_device,
    is_env_enabled,
)
from .data_args import DataArguments
from .finetuning_args import FinetuningArguments
from .generating_args import GeneratingArguments
from .model_args import ModelArguments
from .training_args import TrainingArguments

CHECKPOINT_NAMES = {
    SAFE_WEIGHTS_INDEX_NAME,
    SAFE_WEIGHTS_NAME,
    WEIGHTS_INDEX_NAME,
    WEIGHTS_NAME,
}
logger = logging.get_logger(__name__)

check_dependencies()


_TRAIN_ARGS = [
    ModelArguments,
    DataArguments,
    TrainingArguments,
    FinetuningArguments,
    GeneratingArguments,
]
_TRAIN_CLS = tuple[
    ModelArguments,
    DataArguments,
    TrainingArguments,
    FinetuningArguments,
    GeneratingArguments,
]


def read_args(
    args: Optional[Union[dict[str, Any], list[str]]] = None,
) -> Union[dict[str, Any], list[str]]:
    r"""Get arguments from the command line or a config file."""
    if args is not None:
        return args

    if sys.argv[1].endswith(".yaml") or sys.argv[1].endswith(".yml"):
        override_config = OmegaConf.from_cli(sys.argv[2:])
        dict_config = yaml.safe_load(Path(sys.argv[1]).absolute().read_text())
        return OmegaConf.to_container(OmegaConf.merge(dict_config, override_config))
    elif sys.argv[1].endswith(".json"):
        override_config = OmegaConf.from_cli(sys.argv[2:])
        dict_config = json.loads(Path(sys.argv[1]).absolute().read_text())
        return OmegaConf.to_container(OmegaConf.merge(dict_config, override_config))
    else:
        return sys.argv[1:]


def _parse_args(
    parser: "HfArgumentParser",
    args: Optional[Union[dict[str, Any], list[str]]] = None,
    allow_extra_keys: bool = False,
) -> tuple[Any]:
    args = read_args(args)
    if isinstance(args, dict):
        return parser.parse_dict(args, allow_extra_keys=allow_extra_keys)

    (*parsed_args, unknown_args) = parser.parse_args_into_dataclasses(
        args=args, return_remaining_strings=True
    )

    if unknown_args and not allow_extra_keys:
        print(parser.format_help())
        print(f"Got unknown args, potentially deprecated arguments: {unknown_args}")
        raise ValueError(
            f"Some specified arguments are not used by the HfArgumentParser: {unknown_args}"
        )

    return tuple(parsed_args)


def _set_transformers_logging() -> None:
    if os.getenv("LLAMAFACTORY_VERBOSITY", "INFO") in ["DEBUG", "INFO"]:
        transformers.utils.logging.set_verbosity_info()
        transformers.utils.logging.enable_default_handler()
        transformers.utils.logging.enable_explicit_format()


def _set_env_vars() -> None:
    if is_torch_npu_available():
        # avoid JIT compile on NPU devices, see https://zhuanlan.zhihu.com/p/660875458
        torch.npu.set_compile_mode(jit_compile=is_env_enabled("NPU_JIT_COMPILE"))


def _parse_train_args(
    args: Optional[Union[dict[str, Any], list[str]]] = None,
) -> _TRAIN_CLS:
    parser = HfArgumentParser(_TRAIN_ARGS)
    allow_extra_keys = is_env_enabled("ALLOW_EXTRA_ARGS")
    return _parse_args(parser, args, allow_extra_keys=allow_extra_keys)


def get_train_args(
    args: Optional[Union[dict[str, Any], list[str]]] = None,
) -> _TRAIN_CLS:
    model_args, data_args, training_args, finetuning_args, generating_args = (
        _parse_train_args(args)
    )

    # Setup logging
    if training_args.should_log:
        _set_transformers_logging()

    if training_args.parallel_mode == ParallelMode.NOT_DISTRIBUTED:
        raise ValueError(
            "Please launch distributed training with `llamafactory-cli` or `torchrun`."
        )

    if (
        training_args.deepspeed
        and training_args.parallel_mode != ParallelMode.DISTRIBUTED
    ):
        raise ValueError("Please use `FORCE_TORCHRUN=1` to launch DeepSpeed training.")

    if training_args.do_train and data_args.dataset is None:
        raise ValueError("Please specify dataset for training.")

    if training_args.do_eval and (
        data_args.eval_dataset is None and data_args.val_size < 1e-6
    ):
        raise ValueError("Please specify dataset for evaluation.")

    if finetuning_args.pure_bf16:
        if not (
            is_torch_bf16_gpu_available()
            or (is_torch_npu_available() and torch.npu.is_bf16_supported())
        ):
            raise ValueError("This device does not support `pure_bf16`.")

        if is_deepspeed_zero3_enabled():
            raise ValueError("`pure_bf16` is incompatible with DeepSpeed ZeRO-3.")

    _set_env_vars()

    if training_args.do_train and (not training_args.fp16) and (not training_args.bf16):
        logger.warning_rank0("We recommend enable mixed precision training.")

    # Post-process training arguments
    training_args.generation_max_length = (
        training_args.generation_max_length or data_args.cutoff_len
    )
    training_args.generation_num_beams = (
        data_args.eval_num_beams or training_args.generation_num_beams
    )
    training_args.remove_unused_columns = False  # important for multimodal dataset
    can_resume_from_checkpoint = True

    if (
        training_args.resume_from_checkpoint is None
        and training_args.do_train
        and os.path.isdir(training_args.output_dir)
        and not training_args.overwrite_output_dir
        and can_resume_from_checkpoint
    ):
        last_checkpoint = get_last_checkpoint(training_args.output_dir)
        if last_checkpoint is None and any(
            os.path.isfile(os.path.join(training_args.output_dir, name))
            for name in CHECKPOINT_NAMES
        ):
            raise ValueError(
                "Output directory already exists and is not empty. Please set `overwrite_output_dir`."
            )

        if last_checkpoint is not None:
            training_args.resume_from_checkpoint = last_checkpoint
            logger.info_rank0(
                f"Resuming training from {training_args.resume_from_checkpoint}."
            )
            logger.info_rank0(
                "Change `output_dir` or use `overwrite_output_dir` to avoid."
            )

    # Post-process model arguments
    if training_args.bf16 or finetuning_args.pure_bf16:
        model_args.compute_dtype = torch.bfloat16
    elif training_args.fp16:
        model_args.compute_dtype = torch.float16

    model_args.device_map = {"": get_current_device()}
    model_args.model_max_length = data_args.cutoff_len

    # Log on each process the small summary
    logger.info(
        f"Process rank: {training_args.process_index}, "
        f"world size: {training_args.world_size}, device: {training_args.device}, "
        f"distributed training: {training_args.parallel_mode == ParallelMode.DISTRIBUTED}, "
        f"compute dtype: {str(model_args.compute_dtype)}"
    )
    transformers.set_seed(training_args.seed)

    return model_args, data_args, training_args, finetuning_args, generating_args
