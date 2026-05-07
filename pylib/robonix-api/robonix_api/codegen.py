# SPDX-License-Identifier: MulanPSL-2.0
"""Locate the codegen output for the calling package.

Every robonix package has its own `rbnx-build/codegen/proto_gen/` (atlas_pb2,
lifecycle_pb2, robonix_contracts_pb2_grpc, ...). We walk up from the caller's
__file__ to find it and add to sys.path. Idempotent; safe to call multiple times.
"""
from __future__ import annotations

import sys
from pathlib import Path


def ensure_proto_gen(start: Path | None = None) -> Path | None:
    """Walk up from `start` looking for `rbnx-build/codegen/proto_gen/atlas_pb2.py`.
    Returns the proto_gen dir if found (and adds to sys.path), else None."""
    d = (start or Path.cwd()).resolve()
    for _ in range(20):  # cap walks
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if (pg / "atlas_pb2.py").is_file():
            s = str(pg)
            if s not in sys.path:
                sys.path.insert(0, s)
            return pg
        if d.parent == d:
            break
        d = d.parent
    return None


def find_pkg_root(start: Path) -> Path | None:
    """Walk up looking for `package_manifest.yaml`. Returns the dir containing it."""
    d = start.resolve()
    for _ in range(20):
        if (d / "package_manifest.yaml").is_file():
            return d
        if d.parent == d:
            return None
        d = d.parent
    return None
