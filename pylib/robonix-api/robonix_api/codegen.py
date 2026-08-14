# SPDX-License-Identifier: MulanPSL-2.0
"""Locate codegen output for the calling package.

Every robonix package gets a single canonical codegen directory:

    <pkg>/rbnx-build/codegen/
        proto_gen/             # gRPC stubs (atlas_pb2, robonix_contracts_pb2_grpc, ...)
        robonix_mcp_types/     # MCP typed-input dataclasses (when --mcp was passed)

`rbnx codegen -p <pkg>` writes there and `Capability(...)` reads from there.
Developers don't have to plumb either path anywhere — these helpers walk up
from the caller and add the right dirs to `sys.path`, idempotently.
"""
from __future__ import annotations

import sys
from pathlib import Path


def _add_to_path(p: Path) -> None:
    s = str(p)
    if s not in sys.path:
        sys.path.insert(0, s)


def ensure_proto_gen(start: Path | None = None) -> Path | None:
    """Walk up from `start` looking for `rbnx-build/codegen/`. Adds proto_gen and
    (if present) robonix_mcp_types to sys.path. Returns the codegen dir, or None.
    Kept under the historical name so existing callers keep working — does the
    fuller "ensure_codegen" job now."""
    d = (start or Path.cwd()).resolve()
    for _ in range(20):
        codegen = d / "rbnx-build" / "codegen"
        proto_gen = codegen / "proto_gen"
        if (proto_gen / "atlas_pb2.py").is_file():
            _add_to_path(proto_gen)
            mcp_types = codegen / "robonix_mcp_types"
            if mcp_types.is_dir():
                _add_to_path(mcp_types)
            return codegen
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
