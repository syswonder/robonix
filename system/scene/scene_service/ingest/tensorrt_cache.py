# SPDX-License-Identifier: MulanPSL-2.0
"""Device/version-keyed TensorRT caches for Scene perception models."""

from __future__ import annotations

import contextlib
import fcntl
import hashlib
import importlib.util
import json
import os
import shutil
from pathlib import Path
from typing import Any


_CACHE_SCHEMA_VERSION = 2


def _mobile_sam_encoder_boundary(
    encoder: Any,
    *,
    precision: str,
    torch_module: Any,
) -> Any:
    """Run only MobileSAM's image encoder at the selected precision.

    The surrounding SAM predictor remains FP32.  Inputs are cast at this
    boundary and image embeddings are always returned as FP32 so the prompt
    encoder and mask decoder cannot inherit the encoder's reduced precision.
    """

    input_dtype = (
        torch_module.float16
        if precision == "fp16"
        else torch_module.float32
    )

    class _EncoderPrecisionBoundary(torch_module.nn.Module):
        def __init__(self, image_encoder: Any) -> None:
            super().__init__()
            self.image_encoder = image_encoder

        def forward(self, image: Any) -> Any:
            reduced = image.to(dtype=input_dtype)
            with torch_module.autocast(
                device_type=str(reduced.device.type),
                dtype=input_dtype,
                enabled=(
                    precision == "fp16"
                    and str(reduced.device.type) == "cuda"
                ),
            ):
                features = self.image_encoder(reduced)
            return features.float()

    return _EncoderPrecisionBoundary(encoder).eval()


def _install_mobile_sam_encoder(
    sam: Any,
    encoder: Any,
    *,
    precision: str,
    torch_module: Any,
) -> Any:
    boundary = _mobile_sam_encoder_boundary(
        encoder,
        precision=precision,
        torch_module=torch_module,
    )
    sam.model.image_encoder = boundary
    if getattr(sam, "predictor", None) is not None:
        sam.predictor.model.image_encoder = boundary
    return sam


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _module_version(name: str) -> str:
    try:
        module = __import__(name)
    except Exception:  # noqa: BLE001
        return "unavailable"
    return str(getattr(module, "__version__", "unknown"))


def runtime_fingerprint(torch_module: Any) -> dict[str, Any]:
    """Return fields that invalidate a device-specific TensorRT engine."""

    capability = torch_module.cuda.get_device_capability(0)
    return {
        "gpu": str(torch_module.cuda.get_device_name(0)),
        "compute_capability": [int(capability[0]), int(capability[1])],
        "cuda": str(torch_module.version.cuda or "unknown"),
        "torch": str(torch_module.__version__),
        "tensorrt": _module_version("tensorrt"),
        "torch_tensorrt": _module_version("torch_tensorrt"),
    }


def _cache_key(
    checkpoint: Path,
    *,
    kind: str,
    input_size: int,
    precision: str,
    fingerprint: dict[str, Any],
) -> str:
    payload = {
        "schema_version": _CACHE_SCHEMA_VERSION,
        "kind": kind,
        "checkpoint_sha256": _file_sha256(checkpoint),
        "input_size": int(input_size),
        "precision": precision,
        "runtime": fingerprint,
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()[:24]


@contextlib.contextmanager
def _exclusive_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def prepare_yolo_engine(
    yolo: Any,
    checkpoint: str,
    *,
    cache_dir: str,
    input_size: int,
    precision: str,
    mode: str,
    torch_module: Any,
) -> tuple[Any, dict[str, Any]]:
    """Load or build a fixed-shape YOLO TensorRT engine."""

    diagnostic: dict[str, Any] = {
        "mode": mode,
        "backend": "pytorch",
        "cache_hit": False,
        "available": False,
    }
    if mode == "off" or not torch_module.cuda.is_available():
        return yolo, diagnostic
    if importlib.util.find_spec("tensorrt") is None:
        diagnostic["reason"] = "tensorrt python module unavailable"
        if mode == "required":
            raise RuntimeError(diagnostic["reason"])
        return yolo, diagnostic

    from ultralytics import YOLO

    checkpoint_path = Path(checkpoint)
    fingerprint = runtime_fingerprint(torch_module)
    key = _cache_key(
        checkpoint_path,
        kind="yolo-world",
        input_size=input_size,
        precision=precision,
        fingerprint=fingerprint,
    )
    target_dir = Path(cache_dir) / key
    engine_path = target_dir / "yolo-world.engine"
    metadata_path = target_dir / "metadata.json"
    target_dir.mkdir(parents=True, exist_ok=True)
    with _exclusive_lock(target_dir / ".build.lock"):
        if not engine_path.is_file():
            # Ultralytics writes exports beside the loaded checkpoint.  Model
            # assets are commonly mounted read-only in deployed packages, so
            # compile from a cache-local copy instead of attempting to mutate
            # /opt/models (or the package source tree on native installs).
            source_copy = target_dir / "build-source.pt"
            # Runtime assets are tensor-only safetensors, while Ultralytics'
            # engine exporter currently accepts a .pt wrapper.  Materialize
            # that wrapper only inside the device/version-keyed cache from the
            # already validated in-memory model; never write beside or
            # deserialize a pickle from /opt/models.
            if checkpoint_path.suffix == ".safetensors":
                yolo.save(str(source_copy))
            else:
                shutil.copyfile(checkpoint_path, source_copy)
            try:
                export_model = YOLO(str(source_copy))
                exported = Path(
                    export_model.export(
                        format="engine",
                        imgsz=int(input_size),
                        half=precision == "fp16",
                        device=0,
                        dynamic=False,
                        workspace=float(
                            os.environ.get(
                                "SCENE_TENSORRT_WORKSPACE_GIB",
                                "4",
                            )
                        ),
                        verbose=False,
                    )
                )
                temporary = engine_path.with_suffix(".engine.tmp")
                shutil.copyfile(exported, temporary)
                os.replace(temporary, engine_path)
            finally:
                source_copy.unlink(missing_ok=True)
                source_copy.with_suffix(".onnx").unlink(missing_ok=True)
                generated = source_copy.with_suffix(".engine")
                if generated != engine_path:
                    generated.unlink(missing_ok=True)
            metadata_path.write_text(
                json.dumps(
                    {
                        "schema_version": _CACHE_SCHEMA_VERSION,
                        "kind": "yolo-world",
                        "key": key,
                        "checkpoint": str(checkpoint_path),
                        "input_size": int(input_size),
                        "precision": precision,
                        "runtime": fingerprint,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            cache_hit = False
        else:
            cache_hit = True
    loaded = YOLO(str(engine_path), task="detect")
    diagnostic.update(
        {
            "backend": "tensorrt",
            "available": True,
            "cache_hit": cache_hit,
            "cache_key": key,
            "engine_path": str(engine_path),
            "runtime": fingerprint,
        }
    )
    return loaded, diagnostic


def prepare_mobile_sam_encoder(
    sam: Any,
    checkpoint: str,
    *,
    cache_dir: str,
    input_size: int = 1024,
    precision: str,
    mode: str,
    torch_module: Any,
) -> tuple[Any, dict[str, Any]]:
    """Load or compile MobileSAM's image encoder with Torch-TensorRT."""

    diagnostic: dict[str, Any] = {
        "mode": mode,
        "backend": "pytorch",
        "cache_hit": False,
        "available": False,
        "encoder_precision": precision,
        "decoder_precision": "fp32",
        "mixed_precision": precision == "fp16",
    }
    if not torch_module.cuda.is_available():
        return sam, diagnostic
    if getattr(sam, "predictor", None) is None:
        raise RuntimeError(
            "MobileSAM FP32 predictor must be initialized before its encoder"
        )
    source_encoder = sam.model.image_encoder.eval().to("cuda")
    source_encoder = (
        source_encoder.half()
        if precision == "fp16"
        else source_encoder.float()
    )
    # Install the safe PyTorch path before attempting TensorRT.  If an auto
    # compile fails midway, the caller's graceful-degradation handler retains
    # this valid FP16-encoder/FP32-decoder model rather than a bare half-cast
    # encoder connected directly to the FP32 decoder.
    sam = _install_mobile_sam_encoder(
        sam,
        source_encoder,
        precision=precision,
        torch_module=torch_module,
    )
    if mode == "off":
        return sam, diagnostic
    if importlib.util.find_spec("torch_tensorrt") is None:
        diagnostic["reason"] = "torch_tensorrt python module unavailable"
        if mode == "required":
            raise RuntimeError(diagnostic["reason"])
        return sam, diagnostic

    import torch_tensorrt

    checkpoint_path = Path(checkpoint)
    fingerprint = runtime_fingerprint(torch_module)
    key = _cache_key(
        checkpoint_path,
        kind="mobile-sam-encoder",
        input_size=int(input_size),
        precision=precision,
        fingerprint=fingerprint,
    )
    target_dir = Path(cache_dir) / key
    module_path = target_dir / "mobile-sam-encoder.ts"
    metadata_path = target_dir / "metadata.json"
    target_dir.mkdir(parents=True, exist_ok=True)
    dtype = (
        torch_module.float16 if precision == "fp16" else torch_module.float32
    )
    with _exclusive_lock(target_dir / ".build.lock"):
        if module_path.is_file():
            encoder = torch_module.jit.load(str(module_path), map_location="cuda")
            cache_hit = True
        else:
            class _EncoderWithOutputDtype(torch_module.nn.Module):
                def __init__(self, image_encoder: Any) -> None:
                    super().__init__()
                    self.image_encoder = image_encoder

                def forward(self, image: Any) -> Any:
                    # Autocast keeps numerically sensitive TinyViT operations
                    # in FP32.  The compiled graph retains the encoder input
                    # precision; the outer boundary converts its output to
                    # FP32 before prompt and mask decoding.
                    return self.image_encoder(image).to(dtype=image.dtype)

            source = _EncoderWithOutputDtype(source_encoder).eval()
            example = torch_module.zeros(
                (1, 3, int(input_size), int(input_size)),
                device="cuda",
                dtype=dtype,
            )
            # TinyViT contains a checkpoint branch with ``**CKPT_KWARGS``.
            # TorchScript attempts to parse both branches and rejects keyword
            # expansion even though inference has ``use_checkpoint=False``.
            # The R3 engine is deliberately fixed-shape, so trace the active
            # inference graph first and compile that tensor-only module.
            with torch_module.inference_mode(), torch_module.autocast(
                device_type="cuda",
                dtype=dtype,
                enabled=precision == "fp16",
            ):
                traced = torch_module.jit.trace(
                    source,
                    example,
                    strict=False,
                    check_trace=False,
                )
            encoder = torch_tensorrt.compile(
                traced,
                ir="torchscript",
                inputs=[
                    torch_tensorrt.Input(
                        (1, 3, int(input_size), int(input_size)),
                        dtype=dtype,
                    )
                ],
                enabled_precisions={dtype},
                workspace_size=int(
                    float(os.environ.get("SCENE_TENSORRT_WORKSPACE_GIB", "4"))
                    * 1024**3
                ),
                # TinyViT's traced fixed-shape graph retains shape constants
                # as Int64/Float64. TensorRT represents those constants with
                # narrower types; allow the compiler's explicit conversion
                # instead of failing after the otherwise-valid trace.
                truncate_long_and_double=True,
            )
            temporary = module_path.with_suffix(".ts.tmp")
            torch_module.jit.save(encoder, str(temporary))
            os.replace(temporary, module_path)
            metadata_path.write_text(
                json.dumps(
                    {
                        "schema_version": _CACHE_SCHEMA_VERSION,
                        "kind": "mobile-sam-encoder",
                        "key": key,
                        "checkpoint": str(checkpoint_path),
                        "input_size": int(input_size),
                        "precision": precision,
                        "runtime": fingerprint,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            cache_hit = False
    sam = _install_mobile_sam_encoder(
        sam,
        encoder,
        precision=precision,
        torch_module=torch_module,
    )
    diagnostic.update(
        {
            "backend": "tensorrt",
            "available": True,
            "cache_hit": cache_hit,
            "cache_key": key,
            "engine_path": str(module_path),
            "runtime": fingerprint,
        }
    )
    return sam, diagnostic
