# SPDX-License-Identifier: MulanPSL-2.0
"""robonix-api — Python helpers for writing Robonix CapabilityProviders.

A package instantiates exactly one of `Primitive`, `Service`, or
`Skill`. The framework talks to atlas, serves the Driver lifecycle
gRPC, and wraps the common middleware patterns (rclpy / FastMCP /
grpcio).

Layered API:

  Layer 1 -- always available: declare_capability, connect_capability,
             on_init/on_activate/on_deactivate/on_shutdown handlers,
             spawn subprocess, sentinel waits.

  Layer 2 -- opt-in convenience: `@provider.provides_grpc(contract)`,
             `@provider.provides_mcp(contract)`, ROS create_publisher /
             create_subscription. Skip if you want to drive rclpy /
             FastMCP / grpcio directly -- just call
             `provider.declare_capability(...)` to register with atlas.

Typical usage:

    from robonix_api import ATLAS, Primitive, Ok, Err, Deferred

    camera = Primitive(
        id="camera",
        namespace="robonix/primitive/camera",
    )

    @camera.on_init
    def init(cfg: dict):
        topic = cfg.get("rgb_topic")
        if not topic:
            return Err("rgb_topic is required")
        if not camera.wait_for_topic(topic, "Image", 30.0):
            return Deferred(f"no Image on configured topic {topic} yet")
        camera.create_publisher(
            contract_id="robonix/primitive/camera/rgb",
            topic=topic, msg_type="Image",
        )
        return Ok()

    if __name__ == "__main__":
        camera.run()
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
