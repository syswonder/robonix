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


from .loader import get_dataset
from .template import (
    IGNORE_INDEX,
    TEMPLATES,
    Role,
    Template,
    get_template_and_fix_tokenizer,
    register_spatiallm_templates,
)
from .collator import SFTDataCollatorWith4DAttentionMask
from .mm_plugin import (
    LAYOUT_S_PLACEHOLDER,
    LAYOUT_E_PLACEHOLDER,
    POINT_S_TOKEN,
    POINT_E_TOKEN,
    POINT_CLOUD_PLACEHOLDER,
)

__all__ = [
    "IGNORE_INDEX",
    "TEMPLATES",
    "LAYOUT_S_PLACEHOLDER",
    "LAYOUT_E_PLACEHOLDER",
    "POINT_S_TOKEN",
    "POINT_E_TOKEN",
    "POINT_CLOUD_PLACEHOLDER",
    "Role",
    "Template",
    "get_dataset",
    "get_template_and_fix_tokenizer",
    "register_spatiallm_templates",
    "SFTDataCollatorWith4DAttentionMask",
]
