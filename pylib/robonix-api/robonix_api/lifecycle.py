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

from .result import Deferred, Err, Ok, Result

log = logging.getLogger("robonix_api.lifecycle")

# Driver.srv command codes (mirrors lib/lifecycle/srv/Driver.srv).
CMD_INIT       = 0
CMD_ACTIVATE   = 1
CMD_DEACTIVATE = 2
CMD_SHUTDOWN   = 3


def contract_id_to_pascal(contract_id: str) -> str:
    """Mirror of `robonix_codegen::contract_id_to_service_name`.
    Uniform PascalCase, no prefix stripping.
    `robonix/primitive/chassis/twist_in` → `RobonixPrimitiveChassisTwistIn`.
    `mycomp/a/b/c`                       → `MycompABC`.
    """
    out = []
    for seg in contract_id.strip("/").split("/"):
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


def _is_skill_namespace(ns: str) -> bool:
    parts = [p for p in ns.strip("/").split("/") if p]
    if not parts:
        return False
    # Either "robonix/skill/<name>" or "skill/<name>".
    if parts[0] == "robonix" and len(parts) > 1:
        return parts[1] == "skill"
    return parts[0] == "skill"


def build_lifecycle_servicer(
    namespace: str,
    contracts_grpc_module,
    response_cls,
    *,
    on_init=None, on_activate=None, on_deactivate=None, on_shutdown=None,
    on_state_change=None,
    log_tag: str = "robonix_api",
):
    """Build (without starting a server) the lifecycle Servicer instance.

    Returns `(instance, add_to_server_fn, pascal_base, method_name='Driver')`
    when codegen emitted a `<namespace>/driver` Servicer for this package, or
    `None` when the package doesn't have a driver contract (typical for
    system/* services like memory/scene/speech that have no hardware-init
    phase — they expose only MCP tools / gRPC RPCs and don't participate in
    the rbnx-boot Driver(CMD_INIT) handshake).

    `on_state_change(state, detail)` is invoked AFTER each handler returns
    ok=true and is the framework's hook for pushing state transitions to
    atlas. State strings: "inactive" / "active" / "error". Capability
    layer wires this; lower-level callers can leave it None.
    """
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

    is_skill = _is_skill_namespace(namespace)

    def _emit_state(target: str | None, detail: str = "") -> None:
        """Push state transition to the Capability layer. `target=None`
        means "don't transition; just update detail" (used for Deferred
        so the cap stays in its current atlas-side state but
        rbnx caps/state_detail explains why)."""
        if on_state_change is not None:
            try:
                on_state_change(target, detail)
            except Exception:  # noqa: BLE001
                log.exception("[%s] on_state_change(%s) raised", log_tag, target)

    def _post_handler_state(cmd: int, result: Result) -> None:
        """Drive the atlas state machine based on the handler's Result.
        Ok → advance to next state. Err → ERROR. Deferred → no transition
        (cap stays in current state); the framework / operator decides
        whether to retry."""
        if isinstance(result, Err):
            _emit_state("error", result.message)
            return
        if isinstance(result, Deferred):
            # Don't transition; keep current state. Push the reason as
            # state_detail so `rbnx caps` shows why we're stuck.
            _emit_state(None, result.reason)  # type: ignore[arg-type]
            return
        # Ok: advance per command kind.
        if cmd == CMD_INIT:
            _emit_state("inactive")
        elif cmd == CMD_ACTIVATE:
            _emit_state("active")
        elif cmd == CMD_DEACTIVATE:
            _emit_state("inactive")
        elif cmd == CMD_SHUTDOWN:
            _emit_state("terminated")

    def _run_handler(handler, what: str, *args) -> Result:
        """Call user handler, normalise return into Result. Only on_init
        receives cfg; on_activate/on_deactivate/on_shutdown take no args."""
        try:
            ret = handler(*args)
        except Exception as e:  # noqa: BLE001
            log.warning(
                "[%s] %s raised — handlers should return Err(...) instead "
                "of raising. Caught: %s: %s",
                log_tag, what, type(e).__name__, e,
            )
            return Err(f"{type(e).__name__}: {e}")
        if isinstance(ret, (Ok, Err, Deferred)):
            return ret
        raise TypeError(
            f"[{log_tag}] {what} must return Ok / Err / Deferred, "
            f"got {type(ret).__name__}"
        )

    def _to_response(cmd: int, result: Result):
        """Pack Result into the proto Driver_Response shape."""
        if isinstance(result, Ok):
            target = {
                CMD_INIT: "inactive",
                CMD_ACTIVATE: "active",
                CMD_DEACTIVATE: "inactive",
                CMD_SHUTDOWN: "terminated",
            }.get(cmd, "ok")
            return response_cls(ok=True, state=target, error="")
        if isinstance(result, Err):
            return response_cls(ok=False, state="error", error=result.message)
        if isinstance(result, Deferred):
            return response_cls(ok=False, state="deferred", error=result.reason)
        # Unreachable — _run_handler has already enforced the type.
        return response_cls(ok=False, state="error",
                            error=f"unknown Result variant: {type(result).__name__}")

    def Driver(self, request, context):  # noqa: N802 — matches generated stub
        cmd = int(request.command)
        log.info("[%s] Driver(cmd=%d) received", log_tag, cmd)
        cfg = parse_cfg(request)

        # CMD_INIT must have a handler — that's where the cap parses
        # its config + validates dependencies. No reasonable default.
        if cmd == CMD_INIT:
            if on_init is None:
                err = Err("no on_init handler defined")
                _post_handler_state(cmd, err)
                return _to_response(cmd, err)
            result = _run_handler(on_init, "on_init", cfg)
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        # CMD_ACTIVATE / CMD_DEACTIVATE: optional for primitives /
        # services (framework returns Ok no-op so rbnx boot's auto-
        # ACTIVATE succeeds), required for skills (that's where they
        # allocate / release hot resources — executor eviction depends
        # on it). Handlers take no args — only on_init receives config.
        if cmd == CMD_ACTIVATE:
            if on_activate is None:
                if is_skill:
                    err = Err("skill is missing @cap.on_activate handler")
                    _post_handler_state(cmd, err)
                    return _to_response(cmd, err)
                _post_handler_state(cmd, Ok())
                return _to_response(cmd, Ok())
            result = _run_handler(on_activate, "on_activate")
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        if cmd == CMD_DEACTIVATE:
            if on_deactivate is None:
                if is_skill:
                    err = Err("skill is missing @cap.on_deactivate handler")
                    _post_handler_state(cmd, err)
                    return _to_response(cmd, err)
                _post_handler_state(cmd, Ok())
                return _to_response(cmd, Ok())
            result = _run_handler(on_deactivate, "on_deactivate")
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        # CMD_SHUTDOWN: optional handler. Cap is going away regardless;
        # Result is logged but doesn't change termination.
        if cmd == CMD_SHUTDOWN:
            if on_shutdown is not None:
                result = _run_handler(on_shutdown, "on_shutdown")
            else:
                result = Ok()
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        # Unknown command — proto evolved newer than the cap.
        err = Err(f"unknown command code {cmd}")
        _post_handler_state(cmd, err)
        return _to_response(cmd, err)

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
