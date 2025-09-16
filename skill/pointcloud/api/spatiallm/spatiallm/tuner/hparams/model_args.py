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

from dataclasses import asdict, dataclass, field, fields
from typing import Any, Literal, Optional, Union

import torch
from typing_extensions import Self

from ..network import AttentionFunction, RopeScaling


@dataclass
class BaseModelArguments:
    r"""Arguments pertaining to the model."""

    model_name_or_path: Optional[str] = field(
        default=None,
        metadata={
            "help": "Path to the model weight or identifier from huggingface.co/models or modelscope.cn/models."
        },
    )
    cache_dir: Optional[str] = field(
        default=None,
        metadata={
            "help": "Where to store the pre-trained models downloaded from huggingface.co or modelscope.cn."
        },
    )
    use_fast_tokenizer: bool = field(
        default=True,
        metadata={
            "help": "Whether or not to use one of the fast tokenizer (backed by the tokenizers library)."
        },
    )
    resize_vocab: bool = field(
        default=False,
        metadata={
            "help": "Whether or not to resize the tokenizer vocab and the embedding layers."
        },
    )
    split_special_tokens: bool = field(
        default=False,
        metadata={
            "help": "Whether or not the special tokens should be split during the tokenization process."
        },
    )
    add_tokens: Optional[str] = field(
        default=None,
        metadata={
            "help": "Non-special tokens to be added into the tokenizer. Use commas to separate multiple tokens."
        },
    )
    add_special_tokens: Optional[str] = field(
        default=None,
        metadata={
            "help": "Special tokens to be added into the tokenizer. Use commas to separate multiple tokens."
        },
    )
    model_revision: str = field(
        default="main",
        metadata={
            "help": "The specific model version to use (can be a branch name, tag name or commit id)."
        },
    )
    low_cpu_mem_usage: bool = field(
        default=True,
        metadata={"help": "Whether or not to use memory-efficient model loading."},
    )
    rope_scaling: Optional[RopeScaling] = field(
        default=None,
        metadata={
            "help": "Which scaling strategy should be adopted for the RoPE embeddings."
        },
    )
    flash_attn: AttentionFunction = field(
        default=AttentionFunction.AUTO,
        metadata={"help": "Enable FlashAttention for faster training and inference."},
    )
    disable_gradient_checkpointing: bool = field(
        default=False,
        metadata={"help": "Whether or not to disable gradient checkpointing."},
    )
    use_reentrant_gc: bool = field(
        default=True,
        metadata={"help": "Whether or not to use reentrant gradient checkpointing."},
    )
    train_from_scratch: bool = field(
        default=False,
        metadata={"help": "Whether or not to randomly initialize the model weights."},
    )
    offload_folder: str = field(
        default="offload",
        metadata={"help": "Path to offload model weights."},
    )
    use_cache: bool = field(
        default=True,
        metadata={"help": "Whether or not to use KV cache in generation."},
    )
    infer_dtype: Literal["auto", "float16", "bfloat16", "float32"] = field(
        default="auto",
        metadata={"help": "Data type for model weights and activations at inference."},
    )
    hf_hub_token: Optional[str] = field(
        default=None,
        metadata={"help": "Auth token to log in with Hugging Face Hub."},
    )
    ms_hub_token: Optional[str] = field(
        default=None,
        metadata={"help": "Auth token to log in with ModelScope Hub."},
    )
    print_param_status: bool = field(
        default=False,
        metadata={
            "help": "For debugging purposes, print the status of the parameters in the model."
        },
    )
    trust_remote_code: bool = field(
        default=False,
        metadata={
            "help": "Whether to trust the execution of code from datasets/models defined on the Hub or not."
        },
    )

    def __post_init__(self):
        if self.model_name_or_path is None:
            raise ValueError("Please provide `model_name_or_path`.")

        if self.split_special_tokens and self.use_fast_tokenizer:
            raise ValueError(
                "`split_special_tokens` is only supported for slow tokenizers."
            )

        if self.add_tokens is not None:  # support multiple tokens
            self.add_tokens = [token.strip() for token in self.add_tokens.split(",")]

        if self.add_special_tokens is not None:  # support multiple special tokens
            self.add_special_tokens = [
                token.strip() for token in self.add_special_tokens.split(",")
            ]


@dataclass
class ExportArguments:
    r"""Arguments pertaining to the model export."""

    export_dir: Optional[str] = field(
        default=None,
        metadata={"help": "Path to the directory to save the exported model."},
    )
    export_size: int = field(
        default=5,
        metadata={"help": "The file shard size (in GB) of the exported model."},
    )
    export_device: Literal["cpu", "auto"] = field(
        default="cpu",
        metadata={
            "help": "The device used in model export, use `auto` to accelerate exporting."
        },
    )


@dataclass
class ModelArguments(
    ExportArguments,
    BaseModelArguments,
):
    r"""Arguments pertaining to which model/config/tokenizer we are going to fine-tune or infer.

    The class on the most right will be displayed first.
    """

    compute_dtype: Optional[torch.dtype] = field(
        default=None,
        init=False,
        metadata={
            "help": "Torch data type for computing model outputs, derived from `fp/bf16`. Do not specify it."
        },
    )
    device_map: Optional[Union[str, dict[str, Any]]] = field(
        default=None,
        init=False,
        metadata={
            "help": "Device map for model placement, derived from training stage. Do not specify it."
        },
    )
    model_max_length: Optional[int] = field(
        default=None,
        init=False,
        metadata={
            "help": "The maximum input length for model, derived from `cutoff_len`. Do not specify it."
        },
    )
    block_diag_attn: bool = field(
        default=False,
        init=False,
        metadata={
            "help": "Whether use block diag attention or not, derived from `neat_packing`. Do not specify it."
        },
    )

    def __post_init__(self):
        BaseModelArguments.__post_init__(self)

    @classmethod
    def copyfrom(cls, source: "Self", **kwargs) -> "Self":
        init_args, lazy_args = {}, {}
        for attr in fields(source):
            if attr.init:
                init_args[attr.name] = getattr(source, attr.name)
            else:
                lazy_args[attr.name] = getattr(source, attr.name)

        init_args.update(kwargs)
        result = cls(**init_args)
        for name, value in lazy_args.items():
            setattr(result, name, value)

        return result

    def to_dict(self) -> dict[str, Any]:
        args = asdict(self)
        args = {
            k: f"<{k.upper()}>" if k.endswith("token") else v for k, v in args.items()
        }
        return args
