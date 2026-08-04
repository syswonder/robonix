#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Freeze YOLO-World text prompts into tensor-only FP16 safetensors."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def _classes(path: Path) -> list[str]:
    values = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(values, list):
        raise ValueError(f"{path} must contain a JSON list")
    classes = [str(value).strip().lower() for value in values]
    if not classes or any(not value for value in classes):
        raise ValueError(f"{path} contains an empty class")
    if len(classes) != len(set(classes)):
        raise ValueError(f"{path} contains duplicate classes")
    return classes


def _class_digest(classes: list[str]) -> str:
    payload = json.dumps(
        classes,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--classes", type=Path, required=True)
    parser.add_argument(
        "--architecture",
        choices=("yolov8l-world.yaml", "yolov8s-worldv2.yaml"),
        required=True,
    )
    args = parser.parse_args()

    from safetensors.torch import load_file, save_file
    from ultralytics import YOLO

    classes = _classes(args.classes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.input)
    model.set_classes(list(classes))
    # Ultralytics caches the full CLIP text tower as a registered submodule
    # when set_classes() runs. The fixed txt_feats tensor is all inference
    # needs; clearing the cache before serialization removes both the ~600 MB
    # model copy and every runtime dependency on clip-anytorch.
    model.model.clip_model = None
    text_features = getattr(model.model, "txt_feats", None)
    if text_features is None or int(text_features.shape[1]) != len(classes):
        raise RuntimeError("YOLO-World checkpoint lost prompt embeddings")
    if getattr(model.model, "clip_model", None) is not None:
        raise RuntimeError("saved YOLO-World checkpoint retained its text tower")

    state = {}
    for key, value in model.model.state_dict().items():
        tensor = value.detach().cpu()
        if tensor.is_floating_point():
            tensor = tensor.half()
        state[key] = tensor.contiguous()
    state["__scene_prompt_embeddings__"] = (
        text_features.detach().cpu().half().contiguous()
    )
    save_file(
        state,
        args.output,
        metadata={"scene_model": "yolo_world", "storage_dtype": "fp16"},
    )

    loaded_state = load_file(str(args.output), device="cpu")
    loaded_prompt = loaded_state.pop("__scene_prompt_embeddings__", None)
    if loaded_prompt is None or int(loaded_prompt.shape[1]) != len(classes):
        raise RuntimeError("saved YOLO-World safetensors lost prompt embeddings")
    if any(key.startswith("clip_model.") for key in loaded_state):
        raise RuntimeError("saved YOLO-World safetensors retained text tower tensors")

    # Validate the declared package architecture, including compatibility with
    # scalar biases produced by older Ultralytics YOLO-World checkpoints.
    rebuilt = YOLO(args.architecture)
    target_state = rebuilt.model.state_dict()
    for key, value in tuple(loaded_state.items()):
        target = target_state.get(key)
        if (
            target is not None
            and value.shape != target.shape
            and value.numel() == target.numel()
        ):
            loaded_state[key] = value.reshape(target.shape)
    missing, unexpected = rebuilt.model.load_state_dict(
        loaded_state,
        strict=False,
    )
    if missing or unexpected:
        raise RuntimeError(
            "safetensors does not match declared YOLO architecture: "
            f"missing={missing}, unexpected={unexpected}"
        )

    metadata = {
        "schema_version": 2,
        "classes": classes,
        "classes_sha256": _class_digest(classes),
        "ultralytics_yaml": args.architecture,
        "prompt_embeddings_shape": [int(value) for value in text_features.shape],
        "prompt_embeddings_dtype": str(text_features.dtype),
    }
    metadata_path = args.output.with_suffix(".classes.json")
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(
        f"[scene/build] baked {len(classes)} YOLO-World classes into "
        f"{args.output} (stored as fp16 safetensors)"
    )


if __name__ == "__main__":
    main()
