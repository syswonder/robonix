"""Shared utilities for the Robonix zero-copy demo.

Provides GPU kernels, synthetic camera simulation, and gRPC helpers
used by the multi-process nodes and docker/ros2_bench.py.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import torch

def _find_proto_gen() -> Path:
    """Locate proto_gen/ — works both on host and inside Docker container."""
    here = Path(__file__).resolve().parent
    # Docker: /bench/common.py -> /bench/proto_gen/
    candidate = here / "proto_gen"
    if candidate.is_dir():
        return candidate
    # Host: .../zero_copy_demo/zero_copy_demo/common.py -> .../examples/proto_gen/
    parents = here.parents
    for i in range(len(parents)):
        candidate = parents[i] / "proto_gen"
        if candidate.is_dir():
            return candidate
    raise RuntimeError("proto_gen/ not found")

sys.path.insert(0, str(_find_proto_gen()))

import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc


# ── GPU helpers ──────────────────────────────────────────────────────

_SOBEL_X: torch.Tensor | None = None


def pad32(t: torch.Tensor) -> torch.Tensor:
    """Pad spatial dims to multiples of 32 (YOLO requirement)."""
    _, _, h, w = t.shape
    ph = (32 - h % 32) % 32
    pw = (32 - w % 32) % 32
    if ph or pw:
        t = torch.nn.functional.pad(t, (0, pw, 0, ph))
    return t


def sobel_gpu(tensor_hwc: torch.Tensor) -> torch.Tensor:
    """Sobel-X edge detection on an HWC uint8/float GPU tensor."""
    global _SOBEL_X
    gray = tensor_hwc.float().mean(dim=2)
    inp = gray.unsqueeze(0).unsqueeze(0)
    if _SOBEL_X is None or _SOBEL_X.device != tensor_hwc.device:
        _SOBEL_X = torch.tensor(
            [[-1., 0., 1.], [-2., 0., 2.], [-1., 0., 1.]],
            device=tensor_hwc.device,
        ).reshape(1, 1, 3, 3)
    return torch.nn.functional.conv2d(inp, _SOBEL_X, padding=1)


def warmup_models(device: torch.device, yolo_model, h: int, w: int):
    """Warm up YOLO + Sobel on the target device."""
    dummy = pad32(torch.randn(1, 3, h, w, device=device))
    for _ in range(3):
        with torch.no_grad():
            yolo_model(dummy)
    if device.type == "cuda":
        torch.cuda.synchronize()
    del dummy
    sobel_gpu(torch.randn(h, w, 3, device=device).byte())
    if device.type == "cuda":
        torch.cuda.synchronize()


# ── Synthetic camera driver ──────────────────────────────────────────

def generate_frame(h: int, w: int, idx: int) -> np.ndarray:
    """Deterministic synthetic RGB frame (Bayer-demosaiced style)."""
    f = np.empty((h, w, 3), dtype=np.uint8)
    f[:, :, 0] = np.arange(w, dtype=np.uint8)[np.newaxis, :]
    f[:, :, 1] = np.arange(h, dtype=np.uint8)[:, np.newaxis]
    f[:, :, 2] = (idx * 7) & 0xFF
    bar_y = (idx * 3) % h
    f[bar_y:min(bar_y + 20, h), :, :] = 255
    return f


class FakeCameraDriver:
    """Simulates a V4L2 / CSI camera driver.

    When *target_view* is provided (Robonix SHM numpy view), DMA writes
    directly into that buffer — downstream consumers get zero-copy access.
    When *target_view* is None (traditional path), the driver owns a
    private buffer that must be copied out before publishing.
    """

    def __init__(self, height: int, width: int, n_frames: int,
                 target_view: np.ndarray | None = None):
        self.H, self.W = height, width
        self._frames = [generate_frame(height, width, i) for i in range(n_frames)]
        if target_view is not None:
            self._target = target_view
        else:
            self._target = np.empty((height, width, 3), dtype=np.uint8)

    def capture(self, seq: int) -> tuple[np.ndarray, int]:
        """Simulate VIDIOC_DQBUF. Returns (buffer_ref, elapsed_ns)."""
        src = self._frames[seq % len(self._frames)]
        t0 = time.perf_counter_ns()
        np.copyto(self._target, src)
        return self._target, time.perf_counter_ns() - t0


# ── gRPC control-plane helpers ───────────────────────────────────────

def connect_server(
    addr: str = "127.0.0.1:50051", timeout: float = 10.0
) -> pb_grpc.RobonixRuntimeStub:
    """Connect to robonix-atlas via gRPC."""
    ch = grpc.insecure_channel(addr)
    grpc.channel_ready_future(ch).result(timeout=timeout)
    return pb_grpc.RobonixRuntimeStub(ch)


