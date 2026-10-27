# SPDX-License-Identifier: MulanPSL-2.0
"""Pure capability-to-model resolution.

`Catalog.resolve` filters the catalog by required capabilities and request budgets and
returns every matching model in preference order.  The caller connects to the
first one directly; trying the next on failure is ordinary fallback and needs no
extra state.  There is deliberately no health tracking, proxy or telemetry here.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, replace
from functools import lru_cache

from .catalog import Catalog, ModelSpec


@dataclass(frozen=True)
class Requirements:
    """What the caller needs for this request.

    Capabilities are hard requirements.  Budgets are optional; an unknown
    latency is rejected when a latency budget is set because it cannot be
    proven to fit.
    """

    capabilities: tuple[str, ...] = ()
    input_tokens: int = 0
    max_output_tokens: int = 0
    max_latency_ms: float | None = None
    max_cost_usd_per_call: float | None = None
    allow_downgrade: bool = True

    @classmethod
    def default(cls) -> "Requirements":
        """Return the same defaults as Rust's `Requirements::default()`."""
        return cls()


def _resolve(catalog: Catalog, requirements: Requirements) -> list[ModelSpec]:
    """Return matching models, best first.

    Preference order is latency, then combined token price, then model id.
    When ``json_schema`` is required but no schema-capable model fits the
    request budgets, ``allow_downgrade`` lets a ``json_object`` model stand in;
    every returned model then carries ``downgraded=True``.
    """
    required = set(requirements.capabilities)
    downgraded = False

    if "json_schema" in required and requirements.allow_downgrade:
        if not _matching(catalog, required, requirements):
            relaxed = (required - {"json_schema"}) | {"json_object"}
            if _matching(catalog, relaxed, requirements):
                required = relaxed
                downgraded = True

    matches = _matching(catalog, required, requirements)
    matches.sort(key=_preference)
    return [replace(model, downgraded=downgraded) for model in matches]


def _matching(
    catalog: Catalog, required: set[str], requirements: Requirements
) -> list[ModelSpec]:
    return [
        model
        for model in catalog.models
        if _fits(model, required, requirements)
    ]


def _fits(model: ModelSpec, required: set[str], requirements: Requirements) -> bool:
    if any(not model.supports(capability) for capability in required):
        return False
    wanted_tokens = requirements.input_tokens + requirements.max_output_tokens
    if model.context is not None and wanted_tokens > model.context:
        return False
    if requirements.max_latency_ms is not None:
        if model.latency_p95_ms is None:
            return False
        if model.latency_p95_ms > requirements.max_latency_ms:
            return False
    if requirements.max_cost_usd_per_call is not None:
        if _estimated_cost(model, requirements) > requirements.max_cost_usd_per_call:
            return False
    return True


def _estimated_cost(model: ModelSpec, requirements: Requirements) -> float:
    return (
        requirements.input_tokens / 1_000_000 * model.price_input_per_mtok
        + requirements.max_output_tokens / 1_000_000 * model.price_output_per_mtok
    )


def _preference(model: ModelSpec) -> tuple[float, float, str]:
    latency = model.latency_p95_ms
    return (
        float("inf") if latency is None else latency,
        model.price_input_per_mtok + model.price_output_per_mtok,
        model.id,
    )


def resolve_from_env(
    requirements: Requirements,
    *,
    env: str = "ROBONIX_LLM_CATALOG",
) -> ModelSpec | None:
    """Resolve one target from the deployment catalog, or None when disabled.

    Consumers call this while building their configuration.  When ``env`` is
    unset they keep the legacy ``VLM_*``/manifest settings.  A target whose
    ``api_key_env`` is present wins over an earlier target with a missing key;
    if no key is set the first target is returned so the consumer's existing
    "no key -> inactive" path still applies.

    The catalog is read once per process; restart the consumer to pick up an
    edited catalog.
    """
    path = os.environ.get(env, "").strip()
    if not path:
        return None
    targets = _catalog_for(path).resolve(requirements)
    if not targets:
        raise RuntimeError(
            f"no model in {path} satisfies {sorted(requirements.capabilities)}"
        )
    for target in targets:
        if os.environ.get(target.api_key_env):
            return target
    return targets[0]


def api_key_for(target: ModelSpec) -> str:
    """The key the consumer must send, or an empty string when unset."""
    return os.environ.get(target.api_key_env, "")


@lru_cache(maxsize=8)
def _catalog_for(path: str) -> Catalog:
    return Catalog.load(path)
