# Copyright 2025 OpenAccess AI Collective and the LlamaFactory team.
#
# This code is inspired by the OpenAccess AI Collective's axolotl library.
# https://github.com/OpenAccess-AI-Collective/axolotl/blob/main/src/axolotl/monkeypatch/utils.py
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

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Dict, List, Literal, Optional, Sequence, Tuple

import torch
import torch.nn.functional as F
from transformers import DataCollatorForSeq2Seq

from .template import IGNORE_INDEX

if TYPE_CHECKING:
    from transformers import PreTrainedTokenizer

    from .template import Template
    from .mm_plugin import PointCloudInput


def prepare_4d_attention_mask(
    attention_mask_with_indices: "torch.Tensor", dtype: "torch.dtype"
) -> "torch.Tensor":
    r"""Expand 2d attention mask to 4d attention mask.

    Expand the attention mask with indices from (batch_size, seq_len) to (batch_size, 1, seq_len, seq_len),
    handle packed sequences and transforms the mask to lower triangular form to prevent future peeking.

    e.g.
    ```python
    # input
    [[1, 1, 2, 2, 2, 0]]
    # output
    [
        [
            [
                [o, x, x, x, x, x],
                [o, o, x, x, x, x],
                [x, x, o, x, x, x],
                [x, x, o, o, x, x],
                [x, x, o, o, o, x],
                [x, x, x, x, x, x],
            ]
        ]
    ]
    ```
    where `o` equals to `0.0`, `x` equals to `min_dtype`.
    """
    _, seq_len = attention_mask_with_indices.size()
    min_dtype = torch.finfo(dtype).min
    zero_tensor = torch.tensor(0, dtype=dtype)

    # Create a non-padding mask.
    non_padding_mask = (attention_mask_with_indices != 0).unsqueeze(1).unsqueeze(2)
    # Create indices for comparison.
    indices = attention_mask_with_indices.unsqueeze(1).unsqueeze(
        2
    )  # [bsz, 1, 1, seq_len]
    indices_t = attention_mask_with_indices.unsqueeze(1).unsqueeze(
        3
    )  # [bsz, 1, seq_len, 1]
    # Create a lower triangular mask.
    tril_mask = torch.tril(torch.ones((seq_len, seq_len), dtype=torch.bool))
    attention_mask_4d = (indices == indices_t) & non_padding_mask & tril_mask
    # Invert the attention mask.
    attention_mask_4d = torch.where(attention_mask_4d, zero_tensor, min_dtype)
    return attention_mask_4d


def infer_seqlen(source_len: int, target_len: int, cutoff_len: int) -> Tuple[int, int]:
    r"""
    Computes the real sequence length after truncation by the cutoff_len.
    """
    if target_len * 2 < cutoff_len:  # truncate source
        max_target_len = cutoff_len
    elif source_len * 2 < cutoff_len:  # truncate target
        max_target_len = cutoff_len - source_len
    else:  # truncate both
        max_target_len = int(cutoff_len * (target_len / (source_len + target_len)))

    new_target_len = min(max_target_len, target_len)
    max_source_len = max(cutoff_len - new_target_len, 0)
    new_source_len = min(max_source_len, source_len)
    return new_source_len, new_target_len


def _encode_messages_example(
    messages: Sequence[Dict[str, str]],
    system: Optional[str],
    point_clouds: Sequence["PointCloudInput"],
    template: "Template",
    tokenizer: "PreTrainedTokenizer",
    cutoff_len: int,
) -> Tuple[List[int], List[int]]:
    input_ids, labels = template.mm_plugin.process_token_ids(
        [], [], point_clouds, tokenizer
    )
    encoded_pairs = template.encode_multiturn(tokenizer, messages, system)
    total_length = len(input_ids)

    for turn_idx, (source_ids, target_ids) in enumerate(encoded_pairs):
        if total_length >= cutoff_len:
            break

        source_len, target_len = infer_seqlen(
            len(source_ids), len(target_ids), cutoff_len - total_length
        )
        source_ids = source_ids[:source_len]
        target_ids = target_ids[:target_len]
        total_length += source_len + target_len
        source_label = [IGNORE_INDEX] * source_len

        target_label = target_ids
        input_ids += source_ids + target_ids
        labels += source_label + target_label

    return input_ids, labels


@dataclass
class MultiModalDataCollatorForSeq2Seq(DataCollatorForSeq2Seq):
    r"""
    Data collator that supports VLMs.

    Features should contain input_ids, attention_mask, labels, and optionally contain images and videos.
    """

    template: Optional["Template"] = None

    def __post_init__(self):
        if self.template is None:
            raise ValueError("Template is required for MultiModalDataCollator.")

    def __call__(self, features: Sequence[Dict[str, Any]]) -> Dict[str, "torch.Tensor"]:
        batch_point_clouds, batch_input_prompts = [], []
        for feature in features:
            prompts = feature.pop("_prompt")
            responses = feature.pop("_response")
            point_clouds = feature.pop("_point_clouds", None) or []
            batch_point_clouds.extend(point_clouds)
            batch_input_prompts.append(prompts + responses)

        mm_inputs = self.template.mm_plugin.get_mm_inputs(
            batch_point_clouds, batch_input_prompts
        )
        batched_messages = mm_inputs.pop("messages")

        for mi, messages in enumerate(batched_messages):
            feature = features[mi]
            input_ids, labels = _encode_messages_example(
                messages=messages,
                system=feature.pop("_system", ""),
                point_clouds=batch_point_clouds[mi] if batch_point_clouds else [],
                template=self.template,
                tokenizer=self.tokenizer,
                cutoff_len=self.template.cutoff_len,
            )
            feature["input_ids"] = input_ids
            feature["attention_mask"] = [1] * len(input_ids)
            feature["labels"] = labels

        features: Dict[str, "torch.Tensor"] = super().__call__(features)
        features.update(mm_inputs)

        return features


@dataclass
class SFTDataCollatorWith4DAttentionMask(MultiModalDataCollatorForSeq2Seq):
    r"""Data collator for 4d attention mask."""

    block_diag_attn: bool = False
    attn_implementation: Literal["eager", "sdpa", "flash_attention_2"] = "eager"
    compute_dtype: "torch.dtype" = torch.float32

    def __call__(self, features: list[dict[str, Any]]) -> dict[str, "torch.Tensor"]:
        features = super().__call__(features)
        if self.block_diag_attn and self.attn_implementation != "flash_attention_2":
            features["attention_mask"] = prepare_4d_attention_mask(
                features["attention_mask"], self.compute_dtype
            )

        for key, value in features.items():  # cast data dtype
            if torch.is_tensor(value) and torch.is_floating_point(value):
                features[key] = value.to(self.compute_dtype)

        return features
