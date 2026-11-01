# SPDX-License-Identifier: MulanPSL-2.0
"""Feature catalog and model resolver for Robonix consumers.

This package answers one question: given feature requirements, which model
endpoint should this consumer connect to?  It does not proxy traffic, track
health or manage keys; the caller resolves once and connects directly.
"""
from .catalog import ModelCatalog, ModelSpec
from .resolve import Requirements, api_key_for, catalog_from_env, resolve_from_env

__all__ = [
    "ModelCatalog",
    "ModelSpec",
    "Requirements",
    "api_key_for",
    "catalog_from_env",
    "resolve_from_env",
]
