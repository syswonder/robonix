# SPDX-License-Identifier: MulanPSL-2.0
"""Shim ``np.float`` for legacy deps (e.g. transforms3d via Sapien viewer).

NumPy 1.24+ removed the ``np.float`` alias (and 1.26 still exposes no attribute);
``transforms3d`` still references ``np.float``. Import this module before any
import chain that loads Sapien/ManiSkill envs.
"""
import numpy as np

if not hasattr(np, "float"):
    np.float = np.float64  # noqa: NPY002
