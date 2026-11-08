# SPDX-License-Identifier: MulanPSL-2.0
"""Feature catalog, model resolver and usage records for Robonix consumers.

The catalog answers one question: given feature requirements, which model
endpoint should this consumer connect to?  It does not proxy traffic, track
health or manage keys; the caller resolves once and connects directly.
`metrics` records one line per model request; see README.md.
"""
from .catalog import ModelCatalog, ModelSpec
from .resolve import (
    Requirements,
    api_key_for,
    catalog_from_env,
    explicit_env_set,
    resolve_from_env,
)

__all__ = [
    "ModelCatalog",
    "ModelSpec",
    "Requirements",
    "api_key_for",
    "catalog_from_env",
    "explicit_env_set",
    "resolve_from_env",
]
