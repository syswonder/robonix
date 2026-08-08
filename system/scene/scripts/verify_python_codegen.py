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
from itertools import takewhile
from pathlib import Path
import sys


_REQUIRED_SEMANTIC_MAP_TYPES = (
    "DeleteObject_Request",
    "DeleteObject_Response",
    "FlushObjects_Request",
    "FlushObjects_Response",
    "UpdateObjectGeometry_Request",
    "UpdateObjectGeometry_Response",
    "UpdateObjectLabel_Request",
    "UpdateObjectLabel_Response",
)

_REQUIRED_SCENE_DRIVER_GRPC_TYPES = (
    "RobonixSystemSceneDriverServicer",
    "add_RobonixSystemSceneDriverServicer_to_server",
)


def _release(raw: str) -> tuple[int, ...]:
    """Parse a release version into comparable integer components."""
    parts: list[int] = []
    for component in raw.split("."):
        digits = "".join(takewhile(str.isdigit, component))
        if not digits:
            break
        parts.append(int(digits))
    return tuple(parts)


def _check_version(item: str) -> None:
    """Assert one `name=version` (exact) or `name>=version` (floor) requirement.

    The generator venv is pinned exactly, because a different protoc emits
    different stubs. Runtime interpreters only need a floor: `_pb2_grpc.py` and
    `_pb2.py` each enforce a *minimum* runtime, and pinning them exactly is what
    made the uv workspace unsatisfiable against services/memory.
    """
    name, separator, expected = item.partition(">=")
    if not separator:
        name, separator, expected = item.partition("=")
    if not separator or not name or not expected:
        raise ValueError(f"invalid distribution version check: {item!r}")
    actual = version(name)
    if separator == ">=":
        if _release(actual) < _release(expected):
            raise RuntimeError(
                f"Scene runtime requires {name}>={expected}; found {actual}"
            )
    elif actual != expected:
        raise RuntimeError(
            f"Scene codegen environment mismatch: {name} {actual} != {expected}"
        )


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
            "[distribution=version | distribution>=version ...]",
            file=sys.stderr,
        )
        return 2

    proto_root = Path(argv[1]).resolve()
    mcp_root = Path(argv[2]).resolve()
    for item in argv[3:]:
        _check_version(item)

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

    semantic_map = sys.modules["semantic_map_mcp"]
    missing_types = [
        name for name in _REQUIRED_SEMANTIC_MAP_TYPES
        if not hasattr(semantic_map, name)
    ]
    if missing_types:
        raise RuntimeError(
            "generated semantic_map_mcp is stale or came from a Robonix source "
            f"without Scene object-management contracts; missing: {missing_types}"
        )

    contracts_grpc = sys.modules["robonix_contracts_pb2_grpc"]
    missing_driver_types = [
        name
        for name in _REQUIRED_SCENE_DRIVER_GRPC_TYPES
        if not hasattr(contracts_grpc, name)
    ]
    if missing_driver_types:
        raise RuntimeError(
            "generated robonix_contracts_pb2_grpc is stale or came from a "
            "Robonix source without Scene's shared Driver contract; missing: "
            f"{missing_driver_types}"
        )

    versions = " ".join(
        f"{name}={value}" for name, value in actual_versions.items()
    )
    suffix = f" ({versions})" if versions else ""
    print(f"[scene/codegen] generated protobuf, gRPC, map, and MCP imports OK{suffix}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
