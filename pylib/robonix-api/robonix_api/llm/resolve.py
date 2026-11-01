# SPDX-License-Identifier: MulanPSL-2.0
"""Feature matching and deployment environment lookup."""
from __future__ import annotations

import os
from dataclasses import dataclass
from functools import lru_cache

from .catalog import ModelCatalog, ModelSpec


@dataclass(frozen=True)
class Requirements:
    """Features and request size the caller requires from a model."""

    features: tuple[str, ...] = ()
    input_tokens: int = 0
    max_output_tokens: int = 0

    @classmethod
    def default(cls) -> "Requirements":
        """Return the same defaults as Rust's `Requirements::default()`."""
        return cls()


def _resolve(catalog: ModelCatalog, requirements: Requirements) -> list[ModelSpec]:
    wanted_tokens = requirements.input_tokens + requirements.max_output_tokens
    matches = [
        model
        for model in catalog.models
        if all(model.supports(feature) for feature in requirements.features)
        and (model.context is None or wanted_tokens <= model.context)
    ]
    return sorted(matches, key=_preference)


def _preference(model: ModelSpec) -> tuple[float, float, str]:
    latency = model.latency_p95_ms
    return (
        float("inf") if latency is None else latency,
        model.price_input_per_mtok + model.price_output_per_mtok,
        model.id,
    )


def catalog_from_env() -> ModelCatalog | None:
    """Load the catalog named by ``ROBONIX_LLM_CATALOG``, or ``None``."""
    path = os.environ.get("ROBONIX_LLM_CATALOG", "").strip()
    if not path:
        return None
    return _catalog_for(path)


def resolve_from_env(
    requirements: Requirements,
    *,
    env: str = "ROBONIX_LLM_CATALOG",
) -> ModelSpec | None:
    """Resolve one target from the deployment catalog, or None when disabled.

    Consumers call this while building their configuration.  When ``env`` is
    unset they keep the legacy ``VLM_*``/manifest settings.  A target with no
    ``api_key_env`` needs no credential and stays eligible.  A target whose
    ``api_key_env`` is set but missing from the environment is skipped; if no
    target is usable, the error names the variables to set.

    The catalog is read once per process; restart the consumer to pick up an
    edited catalog.
    """
    path = os.environ.get(env, "").strip()
    if not path:
        return None
    catalog = _catalog_for(path)
    targets = catalog.resolve(requirements)
    if not targets:
        raise RuntimeError(
            f"no model in {path} satisfies {sorted(requirements.features)}"
        )
    for target in targets:
        if target.api_key_env is None or api_key_for(target):
            return target
    missing = sorted({target.api_key_env for target in targets if target.api_key_env})
    raise RuntimeError(
        f"no usable model in {path} for {sorted(requirements.features)}; "
        f"set one of {', '.join(missing)}"
    )


def api_key_for(target: ModelSpec) -> str:
    """The key the consumer must send, or an empty string when unset/keyless."""
    if target.api_key_env is None:
        return ""
    return os.environ.get(target.api_key_env, "")


@lru_cache(maxsize=8)
def _catalog_for(path: str) -> ModelCatalog:
    return ModelCatalog.load(path)
