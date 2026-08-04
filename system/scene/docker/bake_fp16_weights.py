#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Convert OpenCLIP and MobileSAM checkpoints to tensor-only FP16 files."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any


def _fp16_state(state: dict[str, Any]) -> dict[str, Any]:
    tensors = {}
    for key, value in state.items():
        if not hasattr(value, "detach"):
            continue
        tensor = value.detach().cpu()
        if tensor.is_floating_point():
            tensor = tensor.half()
        tensors[str(key)] = tensor.contiguous()
    if not tensors:
        raise RuntimeError("checkpoint contains no tensors")
    return tensors


def _load_tensor_mapping(path: Path) -> dict[str, Any]:
    import torch

    try:
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        checkpoint = torch.load(path, map_location="cpu")
    if isinstance(checkpoint, dict) and isinstance(
        checkpoint.get("state_dict"), dict
    ):
        checkpoint = checkpoint["state_dict"]
    if not isinstance(checkpoint, dict):
        raise RuntimeError(f"{path} does not contain a tensor mapping")
    return checkpoint


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--open-clip-input", type=Path, required=True)
    parser.add_argument("--open-clip-output", type=Path, required=True)
    parser.add_argument("--mobile-sam-input", type=Path, required=True)
    parser.add_argument("--mobile-sam-output", type=Path, required=True)
    args = parser.parse_args()

    from safetensors import safe_open
    from safetensors.torch import load_file, save_file
    from ultralytics import SAM

    args.open_clip_output.parent.mkdir(parents=True, exist_ok=True)
    args.mobile_sam_output.parent.mkdir(parents=True, exist_ok=True)

    clip_state = _fp16_state(_load_tensor_mapping(args.open_clip_input))
    save_file(
        clip_state,
        args.open_clip_output,
        metadata={"scene_model": "open_clip_vit_b32", "storage_dtype": "fp16"},
    )

    sam = SAM(str(args.mobile_sam_input))
    sam_state = _fp16_state(sam.model.state_dict())
    save_file(
        sam_state,
        args.mobile_sam_output,
        metadata={"scene_model": "mobile_sam", "storage_dtype": "fp16"},
    )

    # Reopen both artifacts so a truncated or invalid write fails the build.
    if len(load_file(str(args.open_clip_output), device="cpu")) != len(clip_state):
        raise RuntimeError("OpenCLIP safetensors validation failed")
    if len(load_file(str(args.mobile_sam_output), device="cpu")) != len(sam_state):
        raise RuntimeError("MobileSAM safetensors validation failed")
    for path, expected in (
        (args.open_clip_output, "open_clip_vit_b32"),
        (args.mobile_sam_output, "mobile_sam"),
    ):
        with safe_open(str(path), framework="pt", device="cpu") as handle:
            if (handle.metadata() or {}).get("scene_model") != expected:
                raise RuntimeError(f"{path} lost its model metadata")
    print(
        f"[scene/build] OpenCLIP {args.open_clip_output.stat().st_size} bytes; "
        f"MobileSAM {args.mobile_sam_output.stat().st_size} bytes"
    )


if __name__ == "__main__":
    main()
