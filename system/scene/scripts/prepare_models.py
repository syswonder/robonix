#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Prepare TensorRT models locally before starting the managed service."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from scene_service.inference_config import (  # noqa: E402
    inference_backend, prepared_models_dir, engine_cache_dir, validate_tensorrt_config,
)


def source_fingerprint(weights: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in [PKG / "scripts/export_models.py", *weights]:
        with path.open("rb") as source:
            for chunk in iter(lambda: source.read(1 << 20), b""):
                digest.update(chunk)
    for package in ("torch", "ultralytics", "open_clip_torch", "onnx", "onnxscript"):
        digest.update(f"{package}={importlib.metadata.version(package)}".encode())
    return digest.hexdigest()


def build_engines(models_dir: Path, check: bool = False, cache_dir=None) -> None:
    cmd = [sys.executable, str(PKG / "scripts/build_engines.py"),
           "--models-dir", str(models_dir),
           "--cache-dir", str(cache_dir or engine_cache_dir(str(models_dir)))]
    if check:
        cmd.append("--check")
    subprocess.run(cmd, check=True)


def prepare(models_dir: Path) -> None:
    weights_dir = PKG / "docker/_weights"
    weights = [Path(os.environ.get(name) or weights_dir / filename) for name, filename in (
        ("SCENE_YOLO_WORLD_WEIGHTS", "yolov8l-world.pt"),
        ("SCENE_MOBILE_SAM_WEIGHTS", "mobile_sam.pt"),
        ("SCENE_CLIP_PRETRAINED", "open_clip_pytorch_model.bin"),
    )]
    key = source_fingerprint(weights)
    destination = models_dir / key
    if destination.is_dir():
        print(f"[scene/prepare] reusing exports {key[:12]}", flush=True)
        build_engines(destination)
    else:
        pending = Path(tempfile.mkdtemp(prefix=".export-", dir=models_dir))
        try:
            # Separate processes release Torch and exporter memory between models.
            for model in ("yolo", "sam", "clip"):
                subprocess.run([
                    sys.executable, str(PKG / "scripts/export_models.py"),
                    "--only", model, "--yolo-weights", str(weights[0]),
                    "--sam-weights", str(weights[1]), "--clip-bin", str(weights[2]),
                    "--out", str(pending),
                ], check=True)
            build_engines(pending, cache_dir=engine_cache_dir(str(destination)))
            pending.rename(destination)
        finally:
            if pending.exists():
                shutil.rmtree(pending)
                shutil.rmtree(engine_cache_dir(str(destination)), ignore_errors=True)
    # Publish only after every model and engine passed its numerical gate.
    marker = models_dir / "prepared.json"
    previous = json.loads(marker.read_text()) if marker.exists() else {}
    previous_key = previous.get("export") if previous.get("export") != key else previous.get("previous")
    temporary = models_dir / ".prepared.json.tmp"
    temporary.write_text(json.dumps({"export": key, "previous": previous_key}) + "\n")
    os.replace(temporary, marker)
    for path in models_dir.iterdir():
        if (path.is_dir() and len(path.name) == 64
                and all(c in "0123456789abcdef" for c in path.name)
                and path.name not in (key, previous_key)):
            shutil.rmtree(engine_cache_dir(str(path)), ignore_errors=True)
            shutil.rmtree(path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="check prepared caches without Torch or builds")
    args = parser.parse_args()
    if inference_backend() == "torch":
        return
    validate_tensorrt_config()
    models_dir = Path(os.environ.get("SCENE_ONNX_DIR") or PKG / "docker/_weights/onnx").resolve()
    if args.check:
        build_engines(Path(prepared_models_dir(str(models_dir))), check=True)
        return
    models_dir.mkdir(parents=True, exist_ok=True)
    with (models_dir / ".prepare.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        prepare(models_dir)


if __name__ == "__main__":
    main()
