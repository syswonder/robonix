# SPDX-License-Identifier: MulanPSL-2.0
"""robonix-api — Python helpers for writing robonix capabilities.

Two layers:

  Layer 1 — atlas client + lifecycle gRPC + subprocess + sentinel helpers.
            What every package needs regardless of which middleware it uses.

  Layer 2 — opt-in convenience wrappers over rclpy / FastMCP / grpcio.
            Skip them if you'd rather use those libs directly; just call
            `cap.declare_ros2 / declare_grpc / declare_mcp` to register with
            atlas after spinning your own server.

Typical usage:

    from robonix_api import Capability, Ok, Err, Deferred

    cap = Capability(id="mid360_lidar",
                     namespace="robonix/primitive/lidar")

    @cap.on_init
    def init(cfg: dict):
        topic = cfg.get("lidar_topic", "/scanner/cloud")
        if not cap.wait_for_topic(topic, "PointCloud2", 30.0):
            return Deferred(f"no PointCloud2 on {topic} yet")
        cap.declare_ros2("robonix/primitive/lidar/lidar3d", topic)
        return Ok()

    if __name__ == "__main__":
        cap.run()
"""
from __future__ import annotations

from .atlas_types import (
    CapabilityRecord,
    CapabilityState,
    Channel,
    ContractDescriptor,
    InterfaceMetadata,
    Transport,
)
from .capability import Capability
from .result import Deferred, Err, Ok, Result
from .tool import mcp_contract


# ── Auto-bootstrap codegen sys.path ─────────────────────────────────
# At `from robonix_api import ...` time, walk the call stack to find
# the user's caller file, locate its package_manifest.yaml, and add
# `<pkg>/rbnx-build/codegen/{proto_gen,robonix_mcp_types}` to sys.path.
#
# Lets users import codegen modules immediately at the top of their
# file, without having to wait for `cap = Capability(...)` to construct:
#
#     from robonix_api import Capability, Ok, Err, Deferred
#     from navigation_mcp import Navigate_Request, Navigate_Response  # works
#
# Capability.__init__ still calls ensure_proto_gen() as a safety net,
# so this is purely a developer-experience improvement.
def _bootstrap_codegen_paths_from_caller() -> None:
    import inspect
    from pathlib import Path
    from .codegen import ensure_proto_gen, find_pkg_root
    here = str(Path(__file__).resolve().parent)
    for fi in inspect.stack():
        f = fi.filename
        # skip our own frames + Python's import machinery
        if f.startswith(here) or "<frozen" in f or "/importlib/" in f:
            continue
        pkg_root = find_pkg_root(Path(f))
        if pkg_root is not None:
            ensure_proto_gen(pkg_root)
        return  # only the first user frame matters


try:
    _bootstrap_codegen_paths_from_caller()
except Exception:  # noqa: BLE001
    # Non-package contexts (REPL, tests, ad-hoc scripts) — silently
    # fall back to Capability.__init__'s ensure_proto_gen call.
    pass
del _bootstrap_codegen_paths_from_caller

__all__ = [
    "Capability",
    # Lifecycle return type — every @cap.on_* handler returns one of these.
    "Ok", "Err", "Deferred", "Result",
    # Atlas dataclass mirrors — return values of cap.find / find_one / connect.
    "Transport", "CapabilityState",
    "InterfaceMetadata", "CapabilityRecord", "ContractDescriptor", "Channel",
    # MCP decorator (standalone form — `@cap.mcp` is sugar over this).
    "mcp_contract",
]
