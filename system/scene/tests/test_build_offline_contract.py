# SPDX-License-Identifier: MulanPSL-2.0
"""Offline build contracts for Scene.

A Scene image build must not reach the network for model weights or for a
BuildKit frontend: robots are provisioned on links that are slow, rate-limited,
or absent, and a build that silently re-downloads a 600 MB checkpoint is the
difference between a five-minute and a one-hour deployment. Every weight is
staged on the host by `build.sh` and bind-mounted in.

These are static assertions over the build inputs. Actually running a build
needs Docker and the staged weights, so it belongs to `scripts/build.sh`, not
to the unit suite.
"""

from __future__ import annotations

from importlib.metadata import version
from pathlib import Path

import pytest


SCENE_ROOT = Path(__file__).resolve().parents[1]
_METRIC_DOCKERFILES = ("Dockerfile", "Dockerfile.jetson")
_ALL_DOCKERFILES = _METRIC_DOCKERFILES + ("Dockerfile.lite",)


def _dockerfile(name: str) -> str:
    return (SCENE_ROOT / "docker" / name).read_text(encoding="utf-8")


def test_weight_downloads_resume_into_a_stable_partial_file() -> None:
    """A retried build must resume, not restart, a half-fetched checkpoint."""
    build = (SCENE_ROOT / "scripts" / "build.sh").read_text(encoding="utf-8")
    assert 'local tmp="$cached.part"' in build
    assert "-C -" in build
    # A PID-suffixed temp file makes every retry start from zero bytes.
    assert 'local tmp="$cached.tmp.$$"' not in build


@pytest.mark.parametrize("name", _ALL_DOCKERFILES)
def test_dockerfile_does_not_force_a_registry_backed_frontend(name: str) -> None:
    """`# syntax=` pins a frontend image that must be pulled before the build."""
    first = _dockerfile(name).splitlines()[0].lstrip()
    assert not first.startswith("# syntax=")


@pytest.mark.parametrize("name", _METRIC_DOCKERFILES)
def test_metric_images_bake_host_staged_weights(name: str) -> None:
    """Model weights enter the image as bind mounts of host-staged files."""
    text = _dockerfile(name)
    for weight in (
        "_weights/yolov8l-world.pt",
        "_weights/mobile_sam.pt",
        "_weights/open_clip_pytorch_model.bin",
    ):
        assert f"source={weight}" in text, f"{name} must bind-mount {weight}"
    # Baked outputs, not a runtime download of the upstream laion checkpoint.
    assert "SCENE_CLIP_PRETRAINED=/opt/models/open_clip_vit_b32.fp16.safetensors" in text
    assert "pretrained='laion2b_s34b_b79k'" not in text


def test_lite_image_carries_no_perception_weights() -> None:
    """Lite's whole point is an image with no model payload and no torch."""
    text = _dockerfile("Dockerfile.lite")
    assert "_weights/" not in text
    assert "safetensors" not in text


def _verifier():
    """Load scripts/verify_python_codegen.py without importing the package."""
    import importlib.util

    path = SCENE_ROOT / "scripts" / "verify_python_codegen.py"
    spec = importlib.util.spec_from_file_location("_scene_verifier", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_codegen_version_checks_accept_floors_and_report_what_was_found() -> None:
    """Exact pins guard the generator; floors guard the runtime.

    The `>=` form exists because scene's runtime deps are floors — asserting
    equality there would fail a build on a host carrying a newer, valid grpcio.
    The return value feeds the success line, which is the only place the check
    is observed when it passes.
    """
    verifier = _verifier()
    assert verifier._release("1.81.1") > verifier._release("1.80.0")
    assert verifier._release("1.10") > verifier._release("1.9")

    installed = version("pytest")
    assert verifier._check_version(f"pytest>=0.1") == f"pytest={installed}"
    assert verifier._check_version(f"pytest={installed}") == f"pytest={installed}"
    with pytest.raises(RuntimeError, match="requires pytest>=999"):
        verifier._check_version("pytest>=999.0")
    with pytest.raises(RuntimeError, match="environment mismatch"):
        verifier._check_version("pytest=0.0.1")
    with pytest.raises(ValueError, match="invalid distribution version check"):
        verifier._check_version("pytest")
