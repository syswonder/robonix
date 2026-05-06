# SPDX-License-Identifier: MulanPSL-2.0
"""Driver lifecycle gRPC server + per-contract Servicer resolution.

Every robonix capability declares a `*/driver` interface that `rbnx boot`
calls Driver(CMD_INIT, config_json) on. The wire shape is fixed by
lib/lifecycle/srv/Driver.srv (uint8 command + string config_json →
bool ok + string state + string error).

Per-namespace generated Servicer classes live in robonix_contracts_pb2_grpc:
- primitive/<area>/driver  → Primitive<Area>DriverServicer
- service/<area>/driver    → Service<Area>DriverServicer
- skill/<area>/driver      → Skill<Area>DriverServicer

Users can also declare arbitrary rpc-mode contracts (e.g. `primitive/chassis/move`)
which generate `PrimitiveChassisMoveServicer` with method `Move`. We resolve
both via the same `contract_id_to_pascal()` mapping.
"""
from __future__ import annotations

import inspect
import logging
from typing import Any, Callable

log = logging.getLogger("robonix_py.lifecycle")

# Driver.srv command codes.
CMD_INIT     = 0
CMD_SHUTDOWN = 1
CMD_UP       = 2  # skills
CMD_DOWN     = 3  # skills


def contract_id_to_pascal(contract_id: str) -> str:
    """Mirror of `robonix_codegen::contract_id_to_service_name`.
    `robonix/primitive/chassis/twist_in` → `PrimitiveChassisTwistIn`."""
    cid = contract_id.strip()
    if cid.startswith("robonix/"):
        cid = cid[len("robonix/"):]
    out = []
    for seg in cid.strip("/").split("/"):
        if not seg:
            continue
        # snake_case within a segment becomes CamelCase too.
        out.append("".join(part.capitalize() for part in seg.split("_") if part))
    return "".join(out)


def driver_pascal_for_namespace(namespace: str) -> str:
    """`primitive/lidar` → `PrimitiveLidarDriver` (the driver Pascal name)."""
    return contract_id_to_pascal(f"{namespace.strip('/')}/driver")


def resolve_servicer(contract_id: str, contracts_grpc_module):
    """Find the generated Servicer class + add-to-server fn + canonical method name
    for a contract. Returns (servicer_class, method_name, add_fn, pascal_base) or None."""
    base = contract_id_to_pascal(contract_id)
    servicer_cls = getattr(contracts_grpc_module, f"{base}Servicer", None)
    add_fn = getattr(contracts_grpc_module, f"add_{base}Servicer_to_server", None)
    if servicer_cls is None or add_fn is None:
        return None
    # Each generated Servicer has exactly one rpc method (no inheritance overrides
    # to filter out). Pick the first non-dunder callable defined on the class.
    method_name: str | None = None
    for n in vars(servicer_cls):
        if n.startswith("_"):
            continue
        if callable(vars(servicer_cls)[n]):
            method_name = n
            break
    if method_name is None:
        return None
    return servicer_cls, method_name, add_fn, base


def bind_user_handler(servicer_cls: type, method_name: str, fn: Callable) -> type:
    """Build a dynamic subclass overriding `method_name` to call `fn`. Adapts to
    handler arity: 1-arg signatures get `(request)`, 2-arg get `(request, context)`."""
    sig = inspect.signature(fn)
    nparams = sum(
        1 for p in sig.parameters.values()
        if p.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    )
    if nparams >= 2:
        def impl(self, request, context, _fn=fn):
            return _fn(request, context)
    else:
        def impl(self, request, context, _fn=fn):
            return _fn(request)
    return type(f"Robonix{servicer_cls.__name__}Impl", (servicer_cls,), {method_name: impl})


def build_lifecycle_servicer(
    namespace: str,
    contracts_grpc_module,
    response_cls,
    *,
    on_init=None, on_up=None, on_down=None, on_shutdown=None,
    log_tag: str = "robonix_py",
):
    """Build (without starting a server) the lifecycle Servicer instance.

    Returns `(instance, add_to_server_fn, pascal_base, method_name='Driver')`
    when codegen emitted a `<namespace>/driver` Servicer for this package, or
    `None` when the package doesn't have a driver contract (typical for
    system/* services like memory/scene/speech that have no hardware-init
    phase — they expose only MCP tools / gRPC RPCs and don't participate in
    the rbnx-boot Driver(CMD_INIT) handshake)."""
    base = driver_pascal_for_namespace(namespace)
    servicer_cls = getattr(contracts_grpc_module, f"{base}Servicer", None)
    add_fn = getattr(contracts_grpc_module, f"add_{base}Servicer_to_server", None)
    if servicer_cls is None or add_fn is None:
        log.info(
            "[%s] no %s/driver contract — skipping lifecycle servicer "
            "(this cap doesn't need a Driver(CMD_INIT) handler).",
            log_tag, namespace,
        )
        return None

    def Driver(self, request, context):  # noqa: N802 — matches generated stub
        cmd = int(request.command)
        log.info("[%s] Driver(cmd=%d) received", log_tag, cmd)
        try:
            if cmd == CMD_INIT:
                if on_init is None:
                    return response_cls(ok=False, state="error", error="no on_init handler")
                return coerce_response(response_cls, on_init(parse_cfg(request)))
            if cmd == CMD_UP:
                if on_up is None:
                    return response_cls(ok=False, state="error",
                                        error="no on_up handler (this capability isn't a skill?)")
                return coerce_response(response_cls, on_up(parse_cfg(request)))
            if cmd == CMD_DOWN:
                if on_down is None:
                    return response_cls(ok=False, state="error", error="no on_down handler")
                return coerce_response(response_cls, on_down())
            if cmd == CMD_SHUTDOWN:
                if on_shutdown is not None:
                    on_shutdown()
                return response_cls(ok=True, state="shutdown", error="")
            return response_cls(ok=False, state="error", error=f"unknown command {cmd}")
        except Exception as e:  # noqa: BLE001
            log.exception("[%s] Driver(cmd=%d) raised", log_tag, cmd)
            return response_cls(ok=False, state="error", error=f"{type(e).__name__}: {e}")

    DynServicer = type("RobonixLifecycleServicer", (servicer_cls,), {"Driver": Driver})
    return DynServicer(), add_fn, base, "Driver"


def parse_cfg(request) -> dict:
    import json
    s = (request.config_json or "").strip()
    if not s:
        return {}
    try:
        v = json.loads(s)
    except json.JSONDecodeError as e:
        raise ValueError(f"bad config_json: {e}") from e
    return v if isinstance(v, dict) else {}


def coerce_response(response_cls, ret) -> Any:
    """Allow handlers to return the response_cls directly OR a dict like
    {ok, state, error}. Capability's ready/error/deferred helpers return dicts."""
    if ret is None:
        return response_cls(ok=True, state="ready", error="")
    if isinstance(ret, dict):
        return response_cls(
            ok=bool(ret.get("ok", True)),
            state=str(ret.get("state", "ready")),
            error=str(ret.get("error", "")),
        )
    return ret  # already a response message
