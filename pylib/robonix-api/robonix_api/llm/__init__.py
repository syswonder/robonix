# SPDX-License-Identifier: MulanPSL-2.0
"""Capability catalog and model resolver for Robonix consumers.

This package answers one question: given capability requirements, which model
endpoint should this consumer connect to?  It does not proxy traffic, track
health or manage keys; the caller resolves once and connects directly.
"""
from .catalog import Catalog, ModelSpec
from .resolve import Requirements, api_key_for, resolve_from_env

__all__ = [
    "Catalog",
    "ModelSpec",
    "Requirements",
    "api_key_for",
    "resolve_from_env",
]
