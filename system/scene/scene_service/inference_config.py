# SPDX-License-Identifier: MulanPSL-2.0
"""Inference selection shared by deployment and the ConceptGraphs runtime."""
from __future__ import annotations

import os
import json
from pathlib import Path


def inference_backend() -> str:
    name = os.environ.get("SCENE_CG_INFERENCE", "").strip().lower() or "torch"
    if name not in ("torch", "tensorrt"):
        raise ValueError("SCENE_CG_INFERENCE must be torch or tensorrt")
    return name


def validate_tensorrt_config(*, profile=None, backend=None, clip_model=None) -> None:
    if (profile or os.environ.get("SCENE_PROFILE", "")).strip().lower() not in ("", "lite"):
        raise ValueError("TensorRT supports SCENE_PROFILE=lite only")
    if (backend or os.environ.get("SCENE_PERCEPTION_BACKEND", "")).strip().lower() not in ("", "concept_graphs"):
        raise ValueError("TensorRT requires SCENE_PERCEPTION_BACKEND=concept_graphs")
    if os.environ.get("SCENE_CG_FORCE_CPU", "").strip().lower() in ("1", "true", "yes"):
        raise ValueError("TensorRT requires CUDA; unset SCENE_CG_FORCE_CPU")
    if (clip_model or os.environ.get("SCENE_CLIP_MODEL", "")).strip() not in ("", "ViT-B-32"):
        raise ValueError("TensorRT supports SCENE_CLIP_MODEL=ViT-B-32 only")


def prepared_models_dir(models_dir: str) -> str:
    marker = Path(models_dir) / "prepared.json"
    if not marker.exists():
        return models_dir  # Manually staged exports remain supported.
    key = json.loads(marker.read_text())["export"]
    if len(key) != 64 or any(c not in "0123456789abcdef" for c in key):
        raise ValueError(f"Invalid export fingerprint in {marker}")
    return str(Path(models_dir) / key)


def engine_cache_dir(models_dir: str) -> Path:
    root = Path(os.environ.get("SCENE_ENGINE_CACHE_DIR") or
                Path.home() / ".cache/robonix/scene-engines")
    key = Path(models_dir).name
    if len(key) == 64 and all(c in "0123456789abcdef" for c in key):
        return root / key
    return root
