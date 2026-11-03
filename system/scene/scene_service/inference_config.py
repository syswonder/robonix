# SPDX-License-Identifier: MulanPSL-2.0
"""Inference selection shared by deployment and the ConceptGraphs runtime."""
from __future__ import annotations

import os
import json
from pathlib import Path


LEGACY_MODEL_METADATA = {
    "profile": "lite", "clip_model": "ViT-B-32", "sam_image_size": 1024,
    "clip_image_size": 224, "clip_feature_dim": 512, "clip_context_length": 77,
    "clip_batch_size": 8, "clip_mean": [0.48145466, 0.4578275, 0.40821073],
    "clip_std": [0.26862954, 0.26130258, 0.27577711],
}


def inference_backend() -> str:
    name = os.environ.get("SCENE_CG_INFERENCE", "").strip().lower() or "torch"
    if name not in ("torch", "tensorrt"):
        raise ValueError("SCENE_CG_INFERENCE must be torch or tensorrt")
    return name


def model_defaults(profile=None, weights_dir=None) -> dict[str, str]:
    """Canonical model locations, without deployment path overrides."""
    profile = (profile or os.environ.get("SCENE_PROFILE") or "lite").strip().lower()
    if profile == "annotate":
        profile = "lite"  # Build dependencies remain available with recognition disabled.
    if profile not in ("lite", "full"):
        raise ValueError(f"Model inference requires lite or full, got {profile!r}")
    root = Path(weights_dir) if weights_dir else Path("/opt/models")
    if weights_dir is None and not root.is_dir():
        root = Path(__file__).resolve().parents[1] / "docker/_weights"
    full = root / "full"
    return {
        "yolo": str(root / "yolov8l-world.pt"),
        "sam": str(full / "sam_l.pt" if profile == "full" else root / "mobile_sam.pt"),
        "clip_model": "ViT-H-14" if profile == "full" else "ViT-B-32",
        "clip_pretrained": str(full / "open_clip_vit_h14_laion2b_s32b_b79k.bin"
                               if profile == "full" else root / "open_clip_pytorch_model.bin"),
    }


def model_config(profile=None, weights_dir=None) -> dict[str, str]:
    """Resolve the same model selection for export, Torch and TensorRT."""
    models = model_defaults(profile, weights_dir)
    selected = (profile or os.environ.get("SCENE_PROFILE") or "lite").strip().lower()
    if selected == "full" and os.environ.get("SCENE_MODELS_DIR"):
        for key in ("sam", "clip_pretrained"):
            models[key] = str(Path(os.environ["SCENE_MODELS_DIR"]) / Path(models[key]).name)
    for key, variable in (("yolo", "SCENE_YOLO_WORLD_WEIGHTS"),
                          ("sam", "SCENE_MOBILE_SAM_WEIGHTS"),
                          ("clip_model", "SCENE_CLIP_MODEL"),
                          ("clip_pretrained", "SCENE_CLIP_PRETRAINED")):
        models[key] = os.environ.get(variable) or models[key]
    return models


def validate_tensorrt_config(*, profile=None, backend=None) -> None:
    if (profile or os.environ.get("SCENE_PROFILE", "")).strip().lower() not in ("", "lite", "full"):
        raise ValueError("TensorRT requires SCENE_PROFILE=lite or full")
    if (backend or os.environ.get("SCENE_PERCEPTION_BACKEND", "")).strip().lower() not in ("", "concept_graphs"):
        raise ValueError("TensorRT requires SCENE_PERCEPTION_BACKEND=concept_graphs")
    if os.environ.get("SCENE_CG_FORCE_CPU", "").strip().lower() in ("1", "true", "yes"):
        raise ValueError("TensorRT requires CUDA; unset SCENE_CG_FORCE_CPU")


def model_metadata(models_dir, *, profile=None, clip_model=None) -> dict:
    """Read export settings; unidentified legacy exports are Lite only."""
    selected_profile = (profile or os.environ.get("SCENE_PROFILE") or "lite").strip().lower()
    selected_clip = clip_model or model_config(selected_profile)["clip_model"]
    path = Path(models_dir) / "model-config.json"
    if path.exists():
        config = json.loads(path.read_text())
        if config["profile"] != selected_profile or config["clip_model"] != selected_clip:
            raise ValueError("Prepared model selection differs; run the Scene build for this profile/model")
        for key in ("sam_image_size", "clip_image_size", "clip_feature_dim",
                    "clip_context_length", "clip_batch_size", "clip_mean", "clip_std"):
            if key not in config:
                raise ValueError(f"Incomplete model metadata: {key}; rebuild the exports")
        return config
    if selected_profile != "lite" or selected_clip != "ViT-B-32":
        raise ValueError("Legacy exports are Lite/ViT-B-32 only; rebuild for the selected models")
    return dict(LEGACY_MODEL_METADATA)


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
