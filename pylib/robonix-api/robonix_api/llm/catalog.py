# SPDX-License-Identifier: MulanPSL-2.0
"""Declarative model catalog for capability-based routing.

The catalog is the single source of truth for what a model can do and how to
connect to it.  It carries no secrets: `api_key_env` names the environment
variable a consumer reads before connecting directly.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .resolve import Requirements


@dataclass(frozen=True)
class ModelSpec:
    """One concrete model endpoint a consumer can connect to."""

    id: str
    provider: str
    base_url: str
    model: str
    api_key_env: str
    capabilities: dict[str, bool] = field(default_factory=dict)
    context: int | None = None
    latency_p95_ms: float | None = None
    price_input_per_mtok: float = 0.0
    price_output_per_mtok: float = 0.0
    protocol: str = "openai-chat"
    # Set by resolve() when the request asked for json_schema but the catalog
    # only had a json_object model.  Not part of the catalog file.
    downgraded: bool = False

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ModelSpec":
        if not isinstance(data, dict):
            raise ValueError("model must be an object")
        model_id = _required_str(data, "id")
        spec = cls(
            id=model_id,
            provider=_required_str(data, "provider"),
            base_url=_required_str(data, "base_url").rstrip("/"),
            model=_required_str(data, "model"),
            api_key_env=_required_str(data, "api_key_env"),
            capabilities=_capabilities(data.get("capabilities")),
            context=_optional_positive_int(data.get("context"), "context"),
            latency_p95_ms=_optional_non_negative(data.get("latency_p95_ms"),
                                                 "latency_p95_ms"),
            price_input_per_mtok=_optional_non_negative(
                data.get("price_input_per_mtok"), "price_input_per_mtok"
            ) or 0.0,
            price_output_per_mtok=_optional_non_negative(
                data.get("price_output_per_mtok"), "price_output_per_mtok"
            ) or 0.0,
            protocol=str(data.get("protocol") or "openai-chat"),
        )
        if not spec.protocol.strip():
            raise ValueError(f"model {model_id!r}.protocol must not be empty")
        return spec

    def supports(self, capability: str) -> bool:
        return bool(self.capabilities.get(capability, False))


@dataclass(frozen=True)
class Catalog:
    models: tuple[ModelSpec, ...]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Catalog":
        if not isinstance(data, dict):
            raise ValueError("catalog must be an object")
        raw_models = data.get("models")
        if not isinstance(raw_models, list) or not raw_models:
            raise ValueError("catalog.models must be a non-empty list")
        models = tuple(ModelSpec.from_dict(item) for item in raw_models)
        ids = [model.id for model in models]
        duplicate = next((model_id for model_id in ids if ids.count(model_id) > 1), None)
        if duplicate is not None:
            raise ValueError(f"duplicate model id {duplicate!r}")
        return cls(models=models)

    @classmethod
    def load(cls, path: str | Path) -> "Catalog":
        """Load a catalog from JSON or YAML, chosen by file suffix."""
        catalog_path = Path(path)
        text = catalog_path.read_text(encoding="utf-8")
        if catalog_path.suffix.lower() == ".json":
            data = json.loads(text)
        else:
            # Imported lazily so importing robonix_api.llm does not require
            # PyYAML until a YAML catalog is actually loaded.
            import yaml

            data = yaml.safe_load(text)
        return cls.from_dict(data or {})

    def model(self, model_id: str) -> ModelSpec:
        for model in self.models:
            if model.id == model_id:
                return model
        raise ValueError(f"unknown model {model_id!r}")

    def resolve(self, requirements: "Requirements") -> list[ModelSpec]:
        """Return matching models, best first.

        Delegates to the pure resolver so catalog data and ranking rules stay in
        separate modules; the local import avoids a module cycle.
        """
        from .resolve import _resolve

        return _resolve(self, requirements)


def _required_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"model.{key} is required")
    return value.strip()


def _capabilities(value: Any) -> dict[str, bool]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("model.capabilities must be an object")
    return {str(name): bool(enabled) for name, enabled in value.items()}


def _optional_positive_int(value: Any, key: str) -> int | None:
    if value is None:
        return None
    number = int(value)
    if number <= 0:
        raise ValueError(f"model.{key} must be positive")
    return number


def _optional_non_negative(value: Any, key: str) -> float | None:
    if value is None:
        return None
    number = float(value)
    if number < 0:
        raise ValueError(f"model.{key} must not be negative")
    return number
