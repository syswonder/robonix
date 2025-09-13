from enum import Enum, unique
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from ..framework import logging
from .formatter import EmptyFormatter, StringFormatter
from .mm_plugin import get_mm_plugin

if TYPE_CHECKING:
    from transformers import PreTrainedTokenizer

    from ..hparams import DataArguments
    from .formatter import SLOTS, Formatter
    from .mm_plugin import SpatialLMPlugin


logger = logging.get_logger(__name__)


IGNORE_INDEX = -100


@unique
class Role(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"


@dataclass
class Template:
    format_user: "Formatter"
    format_assistant: "Formatter"
    format_system: "Formatter"
    format_prefix: "Formatter"
    default_system: str
    stop_words: list[str]
    replace_eos: bool
    mm_plugin: "SpatialLMPlugin"
    cutoff_len: int

    def encode_oneturn(
        self,
        tokenizer: "PreTrainedTokenizer",
        messages: list[dict[str, str]],
        system: Optional[str] = None,
    ) -> tuple[list[int], list[int]]:
        r"""Return a single pair of token ids representing prompt and response respectively."""
        encoded_messages = self._encode(tokenizer, messages, system)
        prompt_ids = []
        for encoded_ids in encoded_messages[:-1]:
            prompt_ids += encoded_ids

        response_ids = encoded_messages[-1]
        return prompt_ids, response_ids

    def encode_multiturn(
        self,
        tokenizer: "PreTrainedTokenizer",
        messages: list[dict[str, str]],
        system: Optional[str] = None,
    ) -> list[tuple[list[int], list[int]]]:
        r"""Return multiple pairs of token ids representing prompts and responses respectively."""
        encoded_messages = self._encode(tokenizer, messages, system)
        return [
            (encoded_messages[i], encoded_messages[i + 1])
            for i in range(0, len(encoded_messages), 2)
        ]

    def _convert_elements_to_ids(
        self, tokenizer: "PreTrainedTokenizer", elements: "SLOTS"
    ) -> list[int]:
        r"""Convert elements to token ids."""
        token_ids = []
        for elem in elements:
            if isinstance(elem, str):
                if len(elem) != 0:
                    token_ids += tokenizer.encode(elem, add_special_tokens=False)
            elif isinstance(elem, dict):
                token_ids += [tokenizer.convert_tokens_to_ids(elem.get("token"))]
            elif isinstance(elem, set):
                if "bos_token" in elem and tokenizer.bos_token_id is not None:
                    token_ids += [tokenizer.bos_token_id]
                elif "eos_token" in elem and tokenizer.eos_token_id is not None:
                    token_ids += [tokenizer.eos_token_id]
            else:
                raise ValueError(
                    f"Input must be string, set[str] or dict[str, str], got {type(elem)}"
                )

        return token_ids

    def _encode(
        self,
        tokenizer: "PreTrainedTokenizer",
        messages: list[dict[str, str]],
        system: Optional[str],
    ) -> list[list[int]]:
        r"""Encode formatted inputs to pairs of token ids.

        Turn 0: prefix + system + query        resp
        Turn t: query                          resp.
        """
        system = system or self.default_system
        encoded_messages = []
        for i, message in enumerate(messages):
            elements = []

            if i == 0:
                elements += self.format_prefix.apply()
                if system:
                    elements += self.format_system.apply(content=system)

            if message["role"] == Role.USER:
                elements += self.format_user.apply(
                    content=message["content"], idx=str(i // 2)
                )
            elif message["role"] == Role.ASSISTANT:
                elements += self.format_assistant.apply(content=message["content"])
            else:
                raise NotImplementedError("Unexpected role: {}".format(message["role"]))

            encoded_messages.append(self._convert_elements_to_ids(tokenizer, elements))

        return encoded_messages

    @staticmethod
    def _add_or_replace_eos_token(
        tokenizer: "PreTrainedTokenizer", eos_token: str
    ) -> None:
        r"""Add or replace eos token to the tokenizer."""
        if tokenizer.eos_token == eos_token:
            return

        is_added = tokenizer.eos_token_id is None
        num_added_tokens = tokenizer.add_special_tokens({"eos_token": eos_token})

        if is_added:
            logger.info_rank0(f"Add eos token: {tokenizer.eos_token}.")
        else:
            logger.info_rank0(f"Replace eos token: {tokenizer.eos_token}.")

        if num_added_tokens > 0:
            logger.warning_rank0(
                "New tokens have been added, make sure `resize_vocab` is True."
            )

    def fix_special_tokens(self, tokenizer: "PreTrainedTokenizer") -> None:
        r"""Add eos token and pad token to the tokenizer."""
        stop_words = self.stop_words
        if self.replace_eos:
            if not stop_words:
                raise ValueError("Stop words are required to replace the EOS token.")

            self._add_or_replace_eos_token(tokenizer, eos_token=stop_words[0])
            stop_words = stop_words[1:]

        if tokenizer.eos_token_id is None:
            self._add_or_replace_eos_token(tokenizer, eos_token="<|endoftext|>")

        if tokenizer.pad_token_id is None:
            tokenizer.pad_token = tokenizer.eos_token
            logger.info_rank0(f"Add pad token: {tokenizer.pad_token}")

        if stop_words:
            num_added_tokens = tokenizer.add_special_tokens(
                dict(additional_special_tokens=stop_words),
                replace_additional_special_tokens=False,
            )
            logger.info_rank0("Add {} to stop words.".format(",".join(stop_words)))
            if num_added_tokens > 0:
                logger.warning_rank0(
                    "New tokens have been added, make sure `resize_vocab` is True."
                )

    @staticmethod
    def _jinja_escape(content: str) -> str:
        r"""Escape single quotes in content."""
        return content.replace("'", r"\'")

    @staticmethod
    def _convert_slots_to_jinja(
        slots: "SLOTS", tokenizer: "PreTrainedTokenizer", placeholder: str = "content"
    ) -> str:
        r"""Convert slots to jinja template."""
        slot_items = []
        for slot in slots:
            if isinstance(slot, str):
                slot_pieces = slot.split("{{content}}")
                if slot_pieces[0]:
                    slot_items.append(
                        "'" + Template._jinja_escape(slot_pieces[0]) + "'"
                    )
                if len(slot_pieces) > 1:
                    slot_items.append(placeholder)
                    if slot_pieces[1]:
                        slot_items.append(
                            "'" + Template._jinja_escape(slot_pieces[1]) + "'"
                        )
            elif isinstance(
                slot, set
            ):  # do not use {{ eos_token }} since it may be replaced
                if "bos_token" in slot and tokenizer.bos_token_id is not None:
                    slot_items.append("'" + tokenizer.bos_token + "'")
                elif "eos_token" in slot and tokenizer.eos_token_id is not None:
                    slot_items.append("'" + tokenizer.eos_token + "'")
            elif isinstance(slot, dict):
                raise ValueError("Dict is not supported.")

        return " + ".join(slot_items)

    def _get_jinja_template(self, tokenizer: "PreTrainedTokenizer") -> str:
        r"""Return the jinja template."""
        prefix = self._convert_slots_to_jinja(self.format_prefix.apply(), tokenizer)
        system = self._convert_slots_to_jinja(
            self.format_system.apply(), tokenizer, placeholder="system_message"
        )
        user = self._convert_slots_to_jinja(self.format_user.apply(), tokenizer)
        assistant = self._convert_slots_to_jinja(
            self.format_assistant.apply(), tokenizer
        )
        jinja_template = ""
        if prefix:
            jinja_template += "{{ " + prefix + " }}"

        if self.default_system:
            jinja_template += (
                "{% set system_message = '"
                + self._jinja_escape(self.default_system)
                + "' %}"
            )

        jinja_template += (
            "{% if messages[0]['role'] == 'system' %}{% set loop_messages = messages[1:] %}"
            "{% set system_message = messages[0]['content'] %}{% else %}{% set loop_messages = messages %}{% endif %}"
            "{% if system_message is defined %}{{ " + system + " }}{% endif %}"
            "{% for message in loop_messages %}"
            "{% set content = message['content'] %}"
            "{% if message['role'] == 'user' %}"
            "{{ " + user + " }}"
            "{% elif message['role'] == 'assistant' %}"
            "{{ " + assistant + " }}"
            "{% endif %}"
            "{% endfor %}"
        )
        return jinja_template

    def fix_jinja_template(self, tokenizer: "PreTrainedTokenizer") -> None:
        r"""Replace the jinja template in the tokenizer."""
        if tokenizer.chat_template is None:
            try:
                tokenizer.chat_template = self._get_jinja_template(tokenizer)
            except ValueError as e:
                logger.info_rank0(f"Cannot add this chat template to tokenizer: {e}.")


TEMPLATES: dict[str, "Template"] = {}


def get_template_and_fix_tokenizer(
    tokenizer: "PreTrainedTokenizer", data_args: "DataArguments"
) -> "Template":
    r"""Get chat template and fixes the tokenizer."""
    if data_args.template is None:
        raise ValueError("`template` is required.")

    if data_args.template not in TEMPLATES:
        raise ValueError(f"Template {data_args.template} does not exist.")

    template = TEMPLATES[data_args.template]

    if data_args.default_system is not None:
        logger.info_rank0(f"Using default system message: {data_args.default_system}.")
        template.default_system = data_args.default_system

    template.fix_special_tokens(tokenizer)
    template.fix_jinja_template(tokenizer)
    return template


def register_template(
    name: str,
    format_user: Optional["Formatter"] = None,
    format_assistant: Optional["Formatter"] = None,
    format_system: Optional["Formatter"] = None,
    format_prefix: Optional["Formatter"] = None,
    default_system: str = "",
    stop_words: Optional[list[str]] = None,
    replace_eos: bool = False,
    mm_plugin: "SpatialLMPlugin" = get_mm_plugin(),
    cutoff_len: int = 8192,
) -> None:
    r"""Register a chat template."""
    if name in TEMPLATES:
        raise ValueError(f"Template {name} already exists.")

    default_slots = ["{{content}}", {"eos_token"}]
    default_user_formatter = StringFormatter(slots=["{{content}}"])
    default_assistant_formatter = StringFormatter(slots=default_slots)

    default_prefix_formatter = EmptyFormatter()
    TEMPLATES[name] = Template(
        format_user=format_user or default_user_formatter,
        format_assistant=format_assistant or default_assistant_formatter,
        format_system=format_system or default_user_formatter,
        format_prefix=format_prefix or default_prefix_formatter,
        default_system=default_system,
        stop_words=stop_words or [],
        replace_eos=replace_eos,
        mm_plugin=mm_plugin,
        cutoff_len=cutoff_len,
    )


def register_spatiallm_templates(
    cutoff_len: int = 8192,
    num_bins: int = 1280,
    do_augmentation: bool = False,
    random_rotation: bool = False,
):
    register_template(
        name="spatiallm_llama",
        format_user=StringFormatter(
            slots=[
                (
                    "<|start_header_id|>user<|end_header_id|>\n\n{{content}}<|eot_id|>"
                    "<|start_header_id|>assistant<|end_header_id|>\n\n"
                )
            ]
        ),
        format_system=StringFormatter(
            slots=[
                "<|start_header_id|>system<|end_header_id|>\n\n{{content}}<|eot_id|>"
            ]
        ),
        format_prefix=EmptyFormatter(slots=[{"bos_token"}]),
        stop_words=["<|eot_id|>", "<|eom_id|>"],
        mm_plugin=get_mm_plugin(
            point_token="<|point_pad|>",
            num_bins=num_bins,
            do_augmentation=do_augmentation,
            random_rotation=random_rotation,
        ),
        cutoff_len=cutoff_len,
    )

    register_template(
        name="spatiallm_qwen",
        format_user=StringFormatter(
            slots=["<|im_start|>user\n{{content}}<|im_end|>\n<|im_start|>assistant\n"]
        ),
        format_assistant=StringFormatter(slots=["{{content}}<|im_end|>\n"]),
        format_system=StringFormatter(
            slots=["<|im_start|>system\n{{content}}<|im_end|>\n"]
        ),
        default_system="You are a helpful assistant.",
        stop_words=["<|im_end|>"],
        mm_plugin=get_mm_plugin(
            point_token="<|point_pad|>",
            num_bins=num_bins,
            do_augmentation=do_augmentation,
            random_rotation=random_rotation,
        ),
        cutoff_len=cutoff_len,
    )
