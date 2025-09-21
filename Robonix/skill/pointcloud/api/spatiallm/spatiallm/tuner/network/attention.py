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

from enum import Enum
from typing import TYPE_CHECKING

from transformers.utils import is_flash_attn_2_available, is_torch_sdpa_available

from ..framework import logging

if TYPE_CHECKING:
    from transformers import PretrainedConfig

    from ..hparams import ModelArguments

logger = logging.get_logger(__name__)


class AttentionFunction(str, Enum):
    AUTO = "auto"
    DISABLED = "disabled"
    SDPA = "sdpa"
    FA2 = "fa2"


def configure_attn_implementation(
    config: "PretrainedConfig", model_args: "ModelArguments"
) -> None:
    if model_args.flash_attn == AttentionFunction.AUTO:
        return

    elif model_args.flash_attn == AttentionFunction.DISABLED:
        requested_attn_implementation = "eager"

    elif model_args.flash_attn == AttentionFunction.SDPA:
        if not is_torch_sdpa_available():
            logger.warning_rank0("torch>=2.1.1 is required for SDPA attention.")
            return

        requested_attn_implementation = "sdpa"
    elif model_args.flash_attn == AttentionFunction.FA2:
        if not is_flash_attn_2_available():
            logger.warning_rank0("FlashAttention-2 is not installed.")
            return

        requested_attn_implementation = "flash_attention_2"
    else:
        raise NotImplementedError(f"Unknown attention type: {model_args.flash_attn}")

    setattr(config, "_attn_implementation", requested_attn_implementation)
