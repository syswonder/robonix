# SPDX-License-Identifier: MulanPSL-2.0
"""Python helpers for implementing Robonix capability providers.

A package instantiates exactly one :class:`Primitive`, :class:`Service`, or
:class:`Skill`. The framework communicates with Atlas, serves the Driver
lifecycle gRPC API, and wraps common rclpy, FastMCP, and grpcio patterns.

The API has two layers:

* The always-available layer declares and connects capabilities, binds
  lifecycle handlers, spawns subprocesses, and waits for sentinels.
* The optional convenience layer registers gRPC or MCP handlers and creates
  ROS 2 publishers or subscriptions. A provider that manages middleware
  directly can call ``declare_capability`` instead.

Example::

    from robonix_api import Primitive, Ok, Deferred

    primitive_mid360 = Primitive(
        id="mid360_lidar",
        namespace="robonix/primitive/lidar",
    )

    @primitive_mid360.on_init
    def init(cfg: dict):
        topic = cfg.get("lidar_topic", "/scanner/cloud")
        if not primitive_mid360.wait_for_topic(topic, "PointCloud2", 30.0):
            return Deferred(f"no PointCloud2 on {topic} yet")
        primitive_mid360.create_publisher(
            contract_id="robonix/primitive/lidar/lidar3d",
            topic=topic,
            msg_type="PointCloud2",
        )
        return Ok()

    if __name__ == "__main__":
        primitive_mid360.run()
"""
from __future__ import annotations

from .atlas import ATLAS
from .atlas_types import (
    Capability,
    CapabilityProvider,
    Channel,
    ContractDescriptor,
    GrpcParams,
    Kind,
    LifecycleState,
    McpParams,
    Ros2Params,
    Transport,
)
from . import scribe_logger
from .capability import Primitive, Service, Skill
from .result import Deferred, Err, Ok, Result
from .tool import mcp_contract


# -- Auto-bootstrap codegen sys.path ----------------------------------------
# Walk the import-time call stack to find the user's first frame, locate
# its package_manifest.yaml, and add the package's
# `rbnx-build/codegen/{proto_gen,robonix_mcp_types}` to sys.path so the
# generated atlas_pb2 / contracts modules import cleanly at top-of-file.

def _bootstrap_codegen_paths_from_caller() -> None:
    import inspect
    from pathlib import Path
    from .codegen import ensure_proto_gen, find_pkg_root
    here = str(Path(__file__).resolve().parent)
    for fi in inspect.stack():
        f = fi.filename
        if f.startswith(here) or "<frozen" in f or "/importlib/" in f:
            continue
        pkg_root = find_pkg_root(Path(f))
        if pkg_root is not None:
            ensure_proto_gen(pkg_root)
        return  # only the first user frame matters


try:
    _bootstrap_codegen_paths_from_caller()
except Exception:  # noqa: BLE001
    # REPL / tests / ad-hoc scripts -- fall back to
    # _ProviderBase.__init__'s ensure_proto_gen call.
    pass
del _bootstrap_codegen_paths_from_caller


__all__ = [
    # Unified logging facade.
    "scribe_logger",
    # CapabilityProvider classes (three kinds).
    "Primitive", "Service", "Skill",
    # Global atlas singleton (uppercase per Python registry convention,
    # cf. prometheus_client.REGISTRY).
    "ATLAS",
    # Lifecycle handler return type.
    "Ok", "Err", "Deferred", "Result",
    # Atlas dataclass mirrors.
    "Capability", "CapabilityProvider", "Channel", "ContractDescriptor",
    "GrpcParams", "Ros2Params", "McpParams",
    "Kind", "LifecycleState", "Transport",
    # MCP decorator (standalone; `@provider.provides_mcp` is sugar over this).
    "mcp_contract",
]
