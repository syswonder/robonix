# SPDX-License-Identifier: MulanPSL-2.0
"""Safe, architecture-explicit loaders for Scene perception models.

Deployment images store tensor-only FP16 safetensors under ``/opt/models``.
Architectures come from the installed, pinned runtime packages; no pickled
Python module is deserialized during robot boot.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable


_YOLO_PROMPT_KEY = "__scene_prompt_embeddings__"


def _sidecar(path: str | Path) -> Path:
    return Path(path).with_suffix(".classes.json")


def baked_yolo_metadata(path: str | Path) -> dict[str, Any]:
    metadata_path = _sidecar(path)
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"missing or invalid baked YOLO-World metadata {metadata_path}; "
            "run `rbnx build` so prompts are frozen into safetensors"
        ) from error
    if int(metadata.get("schema_version", 0) or 0) != 2:
        raise RuntimeError(
            f"unsupported baked YOLO-World metadata schema in {metadata_path}"
        )
    classes = metadata.get("classes")
    if not isinstance(classes, list) or not classes:
        raise RuntimeError(f"baked YOLO-World classes missing in {metadata_path}")
    normalized = [str(value).strip().lower() for value in classes]
    if any(not value for value in normalized) or len(normalized) != len(
        set(normalized)
    ):
        raise RuntimeError(f"invalid baked YOLO-World classes in {metadata_path}")
    architecture = str(metadata.get("ultralytics_yaml", "")).strip()
    if architecture not in {
        "yolov8l-world.yaml",
        "yolov8s-worldv2.yaml",
    }:
        raise RuntimeError(
            f"unsupported YOLO-World architecture {architecture!r} in "
            f"{metadata_path}"
        )
    metadata["classes"] = normalized
    metadata["ultralytics_yaml"] = architecture
    return metadata


def _reshape_compatible_state(
    state: dict[str, Any],
    target_state: dict[str, Any],
) -> dict[str, Any]:
    """Adapt scalar-vs-singleton tensors across Ultralytics patch releases."""

    adapted = dict(state)
    for key, value in tuple(adapted.items()):
        target = target_state.get(key)
        if (
            target is not None
            and value.shape != target.shape
            and value.numel() == target.numel()
        ):
            adapted[key] = value.reshape(target.shape)
    return adapted


def load_baked_yolo_world(
    path: str | Path,
    *,
    expected_classes: Iterable[str] | None = None,
):
    """Rebuild YOLO-World from package YAML and tensor-only weights."""

    from safetensors.torch import load_file
    from ultralytics import YOLO

    model_path = Path(path)
    metadata = baked_yolo_metadata(model_path)
    classes = list(metadata["classes"])
    if expected_classes is not None:
        expected = [str(value).strip().lower() for value in expected_classes]
        if expected != classes:
            raise RuntimeError(
                "configured YOLO-World vocabulary differs from the baked "
                "checkpoint; update docker/yolo_world_classes.json and run "
                "`rbnx build`"
            )

    state = load_file(str(model_path), device="cpu")
    prompt = state.pop(_YOLO_PROMPT_KEY, None)
    if prompt is None or tuple(prompt.shape[:2]) != (1, len(classes)):
        raise RuntimeError(
            "YOLO-World safetensors does not contain the baked prompt embeddings"
        )

    model = YOLO(metadata["ultralytics_yaml"])
    state = _reshape_compatible_state(state, model.model.state_dict())
    missing, unexpected = model.model.load_state_dict(state, strict=False)
    if missing or unexpected:
        raise RuntimeError(
            "YOLO-World safetensors does not match the declared architecture: "
            f"missing={missing}, unexpected={unexpected}"
        )
    model.model.txt_feats = prompt.float().contiguous()
    model.model.model[-1].nc = len(classes)
    model.model.names = dict(enumerate(classes))
    model.model.yaml["nc"] = len(classes)
    model.model.clip_model = None
    model.ckpt_path = str(model_path)
    model.model.pt_path = str(model_path)
    return model


def load_mobile_sam(path: str | Path):
    """Build MobileSAM without a pickle checkpoint and load safetensors."""

    import torch
    from safetensors import safe_open
    from safetensors.torch import load_file
    from ultralytics import SAM
    from ultralytics.models.sam.build import build_mobile_sam
    from ultralytics.utils import callbacks

    model_path = Path(path)
    with safe_open(str(model_path), framework="pt", device="cpu") as handle:
        metadata = handle.metadata() or {}
    if metadata.get("scene_model") != "mobile_sam":
        raise RuntimeError(
            f"{model_path} is not a Scene MobileSAM safetensors checkpoint"
        )

    module = build_mobile_sam(None)
    missing, unexpected = module.load_state_dict(
        load_file(str(model_path), device="cpu"),
        strict=False,
    )
    if missing or unexpected:
        raise RuntimeError(
            "MobileSAM safetensors does not match the built-in architecture: "
            f"missing={missing}, unexpected={unexpected}"
        )

    # SAM currently accepts only pickle checkpoint paths in its constructor.
    # Populate the same public Model fields around the safely rebuilt module.
    sam = SAM.__new__(SAM)
    torch.nn.Module.__init__(sam)
    sam.callbacks = callbacks.get_default_callbacks()
    sam.predictor = None
    sam.model = module
    sam.trainer = None
    sam.ckpt = {}
    sam.cfg = None
    sam.ckpt_path = str(model_path)
    sam.overrides = {"task": "segment"}
    sam.metrics = None
    sam.task = "segment"
    sam.model_name = str(model_path)
    sam.is_sam2 = False
    sam.is_sam3 = False
    sam.__dict__.pop("training", None)
    return sam


def initialize_mobile_sam_fp32_predictor(
    sam: Any,
    *,
    device: str,
    input_size: int,
):
    """Create SAM's persistent predictor with a stable FP32 decoder path.

    Ultralytics' deprecated ``half=True`` option casts the complete MobileSAM
    module, including its prompt encoder and mask decoder, to FP16.  That path
    can return correctly shaped but entirely empty masks.  Scene therefore
    initializes the pinned predictor once in FP32; the image encoder is wrapped
    separately by :func:`prepare_mobile_sam_encoder` so it may still use FP16
    or TensorRT without changing the decoder precision.
    """

    overrides = {
        **sam.overrides,
        "conf": 0.25,
        "batch": 1,
        "save": False,
        "mode": "predict",
        "rect": True,
        "embed": None,
        "task": "segment",
        "imgsz": int(input_size),
        "device": str(device),
        "quantize": None,
    }
    predictor = sam._smart_load("predictor")(
        overrides=overrides,
        _callbacks=sam.callbacks,
    )
    predictor.setup_model(model=sam.model, verbose=False)
    if bool(getattr(predictor.model, "fp16", False)):
        raise RuntimeError("MobileSAM predictor must keep its decoder in FP32")
    sam.predictor = predictor
    return sam
