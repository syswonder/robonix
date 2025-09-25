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

from dataclasses import asdict, dataclass, field
from typing import Any, Literal, Optional


@dataclass
class SwanLabArguments:
    use_swanlab: bool = field(
        default=False,
        metadata={
            "help": "Whether or not to use the SwanLab (an experiment tracking and visualization tool)."
        },
    )
    swanlab_project: Optional[str] = field(
        default="llamafactory",
        metadata={"help": "The project name in SwanLab."},
    )
    swanlab_workspace: Optional[str] = field(
        default=None,
        metadata={"help": "The workspace name in SwanLab."},
    )
    swanlab_run_name: Optional[str] = field(
        default=None,
        metadata={"help": "The experiment name in SwanLab."},
    )
    swanlab_mode: Literal["cloud", "local"] = field(
        default="cloud",
        metadata={"help": "The mode of SwanLab."},
    )
    swanlab_api_key: Optional[str] = field(
        default=None,
        metadata={"help": "The API key for SwanLab."},
    )
    swanlab_logdir: Optional[str] = field(
        default=None,
        metadata={"help": "The log directory for SwanLab."},
    )
    swanlab_lark_webhook_url: Optional[str] = field(
        default=None,
        metadata={"help": "The Lark(飞书) webhook URL for SwanLab."},
    )
    swanlab_lark_secret: Optional[str] = field(
        default=None,
        metadata={"help": "The Lark(飞书) secret for SwanLab."},
    )


@dataclass
class FinetuningArguments(SwanLabArguments):
    r"""Arguments pertaining to which techniques we are going to fine-tuning with."""

    pure_bf16: bool = field(
        default=False,
        metadata={
            "help": "Whether or not to train model in purely bf16 precision (without AMP)."
        },
    )
    freeze_language_tower: bool = field(
        default=False,
        metadata={"help": "Whether ot not to freeze language tower in training."},
    )
    freeze_point_tower: bool = field(
        default=False,
        metadata={"help": "Whether ot not to freeze point tower in training."},
    )
    train_proj_only: bool = field(
        default=False,
        metadata={"help": "Whether or not to train the projector only."},
    )

    def to_dict(self) -> dict[str, Any]:
        args = asdict(self)
        args = {
            k: f"<{k.upper()}>" if k.endswith("api_key") else v for k, v in args.items()
        }
        return args
