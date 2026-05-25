# SPDX-License-Identifier: MulanPSL-2.0
"""Voiceprint service package.

Importing this package as a side effect ensures the gRPC stubs generated
from ``proto/voiceprint.proto`` are available on ``sys.path`` as the
top-level modules ``voiceprint_pb2`` and ``voiceprint_pb2_grpc``.

Stub resolution order:
  1. If already importable from ``sys.path`` (e.g. produced by an external
     build step into ``rbnx-build/codegen/proto_gen``), use those.
  2. Otherwise generate them on-demand into ``<pkg>/_generated/`` via
     ``grpc_tools.protoc`` and add that directory to ``sys.path``.

The on-demand path keeps tests and ``python -m voiceprint_service.service``
runnable straight from a fresh checkout without invoking the package build
script first. The ``_generated/`` directory is git-ignored.
"""
from __future__ import annotations

import importlib
import subprocess
import sys
from pathlib import Path

_PKG_DIR = Path(__file__).resolve().parent
_PROTO_DIR = _PKG_DIR.parent / "proto"
_GEN_DIR = _PKG_DIR / "_generated"


def _try_import_stubs() -> bool:
    try:
        importlib.import_module("voiceprint_pb2")
        importlib.import_module("voiceprint_pb2_grpc")
        return True
    except ModuleNotFoundError:
        return False


def _generate_stubs() -> None:
    proto_file = _PROTO_DIR / "voiceprint.proto"
    if not proto_file.is_file():
        raise FileNotFoundError(
            f"voiceprint.proto not found at {proto_file}; cannot bootstrap "
            "gRPC stubs"
        )
    _GEN_DIR.mkdir(parents=True, exist_ok=True)
    (_GEN_DIR / "__init__.py").touch(exist_ok=True)
    cmd = [
        sys.executable, "-m", "grpc_tools.protoc",
        f"-I{_PROTO_DIR}",
        f"--python_out={_GEN_DIR}",
        f"--grpc_python_out={_GEN_DIR}",
        str(proto_file),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            "grpc_tools.protoc failed while bootstrapping voiceprint stubs:\n"
            f"  cmd: {' '.join(cmd)}\n"
            f"  stdout: {proc.stdout}\n"
            f"  stderr: {proc.stderr}"
        )


def _ensure_stubs() -> None:
    if _try_import_stubs():
        return
    if not _GEN_DIR.is_dir() or not (_GEN_DIR / "voiceprint_pb2.py").is_file():
        _generate_stubs()
    sys.path.insert(0, str(_GEN_DIR))
    if not _try_import_stubs():
        raise ImportError(
            "Failed to import voiceprint gRPC stubs even after codegen into "
            f"{_GEN_DIR}"
        )


_ensure_stubs()
