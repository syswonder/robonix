# SPDX-License-Identifier: MulanPSL-2.0
"""Build-time and Torch model loading with the same inference transforms."""
from __future__ import annotations


def load_perception(classes, *, profile="lite", backend="concept_graphs", yolo_path=None,
                    sam_path=None, clip_model=None, clip_pretrained=None, device=None, use_fastsam=False):
    """Load the selected inference implementation with mapper-specific model defaults."""
    import os
    from pathlib import Path
    from ..inference_config import (
        engine_cache_dir, inference_backend, model_config, prepared_models_dir,
        validate_tensorrt_config,
    )

    models = model_config(profile, backend=backend)
    yolo = yolo_path or models["yolo"]
    clip_model = clip_model or models["clip_model"]
    if inference_backend() == "torch":
        from .perception_torch import TorchPerception

        return TorchPerception(classes, yolo, sam_path or models["sam"], clip_model,
                               clip_pretrained or models["clip_pretrained"], device=device,
                               fastsam_path=models["fastsam"] if use_fastsam else None)
    validate_tensorrt_config(profile=profile, backend=backend)
    from .perception_engines import load_engines

    directory = prepared_models_dir(os.environ.get("SCENE_ONNX_DIR") or str(Path(yolo).parent / "onnx"))
    return load_engines(classes, models_dir=directory,
                        cache_dir=str(engine_cache_dir(directory)), profile=profile,
                        clip_model=clip_model, use_fastsam=use_fastsam)


def create_clip_model(model_name, pretrained):
    from pathlib import Path
    import open_clip

    options = {}
    if pretrained and Path(pretrained).is_file():
        # Local weights bypass tag metadata. Only reuse unambiguous published transforms.
        transforms = []
        for tag in open_clip.list_pretrained_tags_by_model(model_name):
            config = open_clip.get_pretrained_cfg(model_name, tag)
            transforms.append({f"image_{key}": config[key]
                               for key in ("mean", "std", "interpolation", "resize_mode")
                               if config.get(key) is not None})
        if transforms and all(value == transforms[0] for value in transforms):
            options = transforms[0]
    model, train_transform, transform = open_clip.create_model_and_transforms(
        model_name, pretrained=pretrained, precision="fp32", **options,
    )
    model.eval()
    if any(callable(getattr(module, "reparameterize", None)) for module in model.modules()):
        from timm.utils import reparameterize_model

        model = reparameterize_model(model)
    return model.eval(), train_transform, transform
