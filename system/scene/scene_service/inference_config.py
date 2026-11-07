# SPDX-License-Identifier: MulanPSL-2.0
"""Model selection shared by deployment and the perception runtimes."""
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


def model_profile(profile=None):
    selected = (profile or os.environ.get("SCENE_PROFILE") or "lite").strip().lower()
    if selected not in ("lite", "full", "annotate"):
        raise ValueError(f"Model preparation requires lite, full or annotate, got {selected!r}")
    return "lite" if selected == "annotate" else selected


def model_defaults(profile=None, weights_dir=None, *, backend=None) -> dict[str, str]:
    """Canonical model locations, without deployment path overrides."""
    profile = model_profile(profile)
    root = Path(weights_dir) if weights_dir else Path("/opt/models")
    if weights_dir is None and not root.is_dir():
        root = Path(__file__).resolve().parents[1] / "docker/_weights"
    full = root / "full"
    mapper = (backend or os.environ.get("SCENE_PERCEPTION_BACKEND") or "concept_graphs").strip().lower()
    if mapper == "dualmap":
        return {
            "yolo": str(root / "yolov8l-world.pt"),
            "sam": str(root / "mobile_sam.pt"),
            "clip_model": "MobileCLIP-S2",
            "clip_pretrained": str(root / "dualmap/mobileclip_s2_datacompdr.bin"),
            "fastsam": str(root / "dualmap/FastSAM-s.pt"),
            "replica_clip_pretrained": str(root / "open_clip_pytorch_model.bin"),
        }
    if mapper != "concept_graphs":
        raise ValueError(f"Unknown perception backend: {mapper}")
    return {
        "yolo": str(root / "yolov8l-world.pt"),
        "sam": str(full / "sam_l.pt" if profile == "full" else root / "mobile_sam.pt"),
        "clip_model": "ViT-H-14" if profile == "full" else "ViT-B-32",
        "clip_pretrained": str(full / "open_clip_vit_h14_laion2b_s32b_b79k.bin"
                               if profile == "full" else root / "open_clip_pytorch_model.bin"),
    }


def model_config(profile=None, weights_dir=None, *, backend=None) -> dict[str, str]:
    """Resolve the same model selection for export, Torch and TensorRT."""
    models = model_defaults(profile, weights_dir, backend=backend)
    selected = model_profile(profile)
    mapper = (backend or os.environ.get("SCENE_PERCEPTION_BACKEND") or "concept_graphs").strip().lower()
    if mapper == "concept_graphs" and selected == "full" and os.environ.get("SCENE_MODELS_DIR"):
        for key in ("sam", "clip_pretrained"):
            models[key] = str(Path(os.environ["SCENE_MODELS_DIR"]) / Path(models[key]).name)
    for key, variable in (("yolo", "SCENE_YOLO_WORLD_WEIGHTS"),
                          ("sam", "SCENE_MOBILE_SAM_WEIGHTS"),
                          ("clip_model", "SCENE_CLIP_MODEL"),
                          ("clip_pretrained", "SCENE_CLIP_PRETRAINED")):
        models[key] = os.environ.get(variable) or models[key]
    if mapper == "dualmap":
        models["replica_clip_pretrained"] = os.environ.get("SCENE_DUALMAP_EXPORT_CLIP") or models["replica_clip_pretrained"]
        for key, name in (("yolo", "YOLO"), ("sam", "SAM"), ("clip_pretrained", "MOBILECLIP"),
                          ("fastsam", "FASTSAM")):
            models[key] = os.environ.get(f"SCENE_DUALMAP_{name}_WEIGHTS", "").strip() or models[key]
    return models


def export_model_names(models):
    optional = (("fastsam", "fastsam"), ("replica_clip_pretrained", "replica"))
    return ("yolo", "sam", "clip") + tuple(name for key, name in optional if key in models)


def validate_tensorrt_config(*, profile=None, backend=None) -> None:
    model_profile(profile)
    mapper = (backend or os.environ.get("SCENE_PERCEPTION_BACKEND", "")).strip().lower()
    if mapper not in ("", "concept_graphs", "dualmap"):
        raise ValueError("TensorRT requires concept_graphs or dualmap")
    if os.environ.get("SCENE_CG_FORCE_CPU", "").strip().lower() in ("1", "true", "yes"):
        raise ValueError("TensorRT requires CUDA; unset SCENE_CG_FORCE_CPU")


def model_metadata(models_dir, *, profile=None, clip_model=None) -> dict:
    """Read export settings; unidentified legacy exports are Lite only."""
    selected_profile = model_profile(profile)
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
