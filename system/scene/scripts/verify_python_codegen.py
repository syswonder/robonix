#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Verify Scene's generated imports come from this package's build output.

The caller supplies the two canonical codegen roots plus optional
``distribution=version`` checks. Importing by the generic module names alone is
not sufficient: an inherited PYTHONPATH can otherwise make a missing Scene
artifact pass by resolving another package's stale ``map_pb2`` or MCP module.
"""

from __future__ import annotations

import importlib
from importlib.metadata import version
from pathlib import Path
import sys


def _is_within(path: Path, root: Path) -> bool:
    """Return whether the resolved module path is inside its expected root."""
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def main(argv: list[str]) -> int:
    """Check requested package versions, import modules, and verify origins."""
    if len(argv) < 3:
        print(
            "usage: verify_python_codegen.py <proto-root> <mcp-root> "
            "[distribution=version ...]",
            file=sys.stderr,
        )
        return 2

    proto_root = Path(argv[1]).resolve()
    mcp_root = Path(argv[2]).resolve()
    expected_versions: dict[str, str] = {}
    for item in argv[3:]:
        name, separator, expected = item.partition("=")
        if not separator or not name or not expected:
            raise ValueError(f"invalid distribution version check: {item!r}")
        expected_versions[name] = expected

    actual_versions = {
        distribution: version(distribution)
        for distribution in expected_versions
    }
    if actual_versions != expected_versions:
        raise RuntimeError(
            f"Scene protobuf environment mismatch: "
            f"{actual_versions} != {expected_versions}"
        )

    expected_modules = {
        "map_pb2": proto_root,
        "robonix_contracts_pb2": proto_root,
        "robonix_contracts_pb2_grpc": proto_root,
        "semantic_map_mcp": mcp_root,
    }
    for module_name, expected_root in expected_modules.items():
        module = importlib.import_module(module_name)
        module_file = getattr(module, "__file__", None)
        if not module_file:
            raise RuntimeError(f"generated module {module_name} has no __file__")
        resolved = Path(module_file).resolve()
        if not _is_within(resolved, expected_root):
            raise RuntimeError(
                f"generated module {module_name} resolved outside expected root: "
                f"{resolved} (expected under {expected_root})"
            )

    versions = " ".join(
        f"{name}={value}" for name, value in actual_versions.items()
    )
    suffix = f" ({versions})" if versions else ""
    print(f"[scene/codegen] generated protobuf, gRPC, map, and MCP imports OK{suffix}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
