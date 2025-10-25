import re
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Union

from typing_extensions import override

SLOTS = list[Union[str, set[str], dict[str, str]]]


@dataclass
class Formatter(ABC):
    slots: SLOTS = field(default_factory=list)

    @abstractmethod
    def apply(self, **kwargs) -> SLOTS:
        r"""Forms a list of slots according to the inputs to encode."""
        ...


@dataclass
class EmptyFormatter(Formatter):
    def __post_init__(self):
        has_placeholder = False
        for slot in filter(lambda s: isinstance(s, str), self.slots):
            if re.search(r"\{\{[a-zA-Z_][a-zA-Z0-9_]*\}\}", slot):
                has_placeholder = True

        if has_placeholder:
            raise ValueError("Empty formatter should not contain any placeholder.")

    @override
    def apply(self, **kwargs) -> SLOTS:
        return self.slots


@dataclass
class StringFormatter(Formatter):
    def __post_init__(self):
        has_placeholder = False
        for slot in filter(lambda s: isinstance(s, str), self.slots):
            if re.search(r"\{\{[a-zA-Z_][a-zA-Z0-9_]*\}\}", slot):
                has_placeholder = True

        if not has_placeholder:
            raise ValueError("A placeholder is required in the string formatter.")

    @override
    def apply(self, **kwargs) -> SLOTS:
        elements = []
        for slot in self.slots:
            if isinstance(slot, str):
                for name, value in kwargs.items():
                    if not isinstance(value, str):
                        raise RuntimeError(f"Expected a string, got {value}")

                    slot = slot.replace("{{" + name + "}}", value, 1)
                elements.append(slot)
            elif isinstance(slot, (dict, set)):
                elements.append(slot)
            else:
                raise RuntimeError(
                    f"Input must be string, set[str] or dict[str, str], got {type(slot)}."
                )

        return elements
