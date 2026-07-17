# SPDX-License-Identifier: MulanPSL-2.0
"""Driver lifecycle gRPC server + per-contract Servicer resolution.

Every managed Robonix provider uses the shared
`robonix/lifecycle/driver` capability that `rbnx boot` calls
Driver(CMD_INIT, config_json) on. The wire shape is fixed by
lib/lifecycle/srv/Driver.srv (uint8 command + string config_json →
bool ok + string state + string error).

Older generated code may expose only `<provider namespace>/driver`.
That form remains a compatibility fallback and emits a migration warning;
new code registers only the shared lifecycle contract.

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
CMD_INIT = 0
CMD_ACTIVATE = 1
CMD_DEACTIVATE = 2
CMD_SHUTDOWN = 3
SHARED_DRIVER_CONTRACT_ID = "robonix/lifecycle/driver"
OLD_ARTIFACT_FALLBACK_ENV = "ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK"
OLD_ARTIFACT_NO_DRIVER_STATE_DETAIL = "robonix.lifecycle.old_artifact_no_driver.v1"


def _generated_service_status(contracts_grpc_module, base: str) -> tuple[bool, bool]:
    """Return ``(complete, absent)`` for one generated gRPC service pair.

    A partially generated pair is neither complete nor absent and must fail
    closed; it is not evidence of an old artifact.
    """
    servicer = getattr(contracts_grpc_module, f"{base}Servicer", None)
    add_fn = getattr(contracts_grpc_module, f"add_{base}Servicer_to_server", None)
    return (
        servicer is not None and add_fn is not None,
        servicer is None and add_fn is None,
    )


def lifecycle_stubs_prove_old_artifact_without_driver(
    namespace: str,
    contracts_grpc_module,
    requested_contract_id: str | None,
    *,
    allow_old_artifact_fallback: bool,
) -> bool:
    """Positive proof used for the managed zero-Driver fallback.

    Permission alone is insufficient: both generated service pairs must be
    wholly absent. A declaration RPC failure or partial/corrupt generated pair
    therefore cannot masquerade as an old artifact.
    """
    if not allow_old_artifact_fallback:
        return False
    requested = (requested_contract_id or "").strip()
    if requested != SHARED_DRIVER_CONTRACT_ID:
        return False
    shared_base = contract_id_to_pascal(SHARED_DRIVER_CONTRACT_ID)
    legacy_base = contract_id_to_pascal(f"{namespace.strip('/')}/driver")
    _, shared_absent = _generated_service_status(contracts_grpc_module, shared_base)
    _, legacy_absent = _generated_service_status(contracts_grpc_module, legacy_base)
    return shared_absent and legacy_absent


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


def lifecycle_contract_for_module(
    namespace: str,
    contracts_grpc_module,
    requested_contract_id: str | None = None,
    *,
    allow_old_artifact_fallback: bool = False,
) -> tuple[str, str] | None:
    """Select the manifest's lifecycle contract, with legacy compatibility.

    Returns ``(contract_id, generated_service_name)``. A package built with
    current Robonix contracts selects ``robonix/lifecycle/driver``. During
    incremental migration, ``rbnx`` supplies the exact Driver ID from the
    selected package manifest so old and new stubs can coexist safely.
    """
    shared_base = contract_id_to_pascal(SHARED_DRIVER_CONTRACT_ID)
    legacy_contract_id = f"{namespace.strip('/')}/driver"
    legacy_base = contract_id_to_pascal(legacy_contract_id)
    shared_available, shared_absent = _generated_service_status(
        contracts_grpc_module, shared_base
    )
    legacy_available, legacy_absent = _generated_service_status(
        contracts_grpc_module, legacy_base
    )
    requested = (
        requested_contract_id.strip() if requested_contract_id is not None else None
    )

    if requested == SHARED_DRIVER_CONTRACT_ID:
        if shared_available:
            return SHARED_DRIVER_CONTRACT_ID, shared_base
        if not shared_absent:
            log.error(
                "generated lifecycle service %s is incomplete; refusing compatibility fallback",
                shared_base,
            )
            return None
        if allow_old_artifact_fallback and legacy_available:
            log.warning(
                "generated artifact predates %s; falling back to legacy "
                "lifecycle contract %s. Rebuild the package to refresh stubs",
                SHARED_DRIVER_CONTRACT_ID,
                legacy_contract_id,
            )
            return legacy_contract_id, legacy_base
        if allow_old_artifact_fallback:
            if not legacy_absent:
                log.error(
                    "generated lifecycle service %s is incomplete; refusing no-Driver fallback",
                    legacy_base,
                )
                return None
            log.warning(
                "generated artifact contains neither %s nor %s; continuing "
                "without a lifecycle Driver as a last-resort compatibility "
                "fallback (package config cannot be delivered). Rebuild the package",
                SHARED_DRIVER_CONTRACT_ID,
                legacy_contract_id,
            )
            return None
        log.error(
            "requested lifecycle contract %s is unavailable in generated stubs",
            requested,
        )
        return None
    if requested == legacy_contract_id and legacy_available:
        log.warning(
            "lifecycle contract %s is deprecated; rebuild against %s",
            legacy_contract_id,
            SHARED_DRIVER_CONTRACT_ID,
        )
        return legacy_contract_id, legacy_base
    if requested:
        log.error(
            "requested lifecycle contract %s is unavailable in generated stubs",
            requested,
        )
        return None

    # Compatibility with older rbnx launchers that encoded omission as an
    # empty selection. Current rbnx always requests shared and sends a separate
    # fallback marker; an empty value is therefore never a normal author path.
    if requested_contract_id is not None:
        log.warning(
            "old launcher requested no lifecycle Driver; continuing as a "
            "last-resort compatibility fallback (package config cannot be delivered)"
        )
        return None

    # Direct launches have no manifest selection. Canonical current behavior is
    # shared-first; namespace legacy is used only when the shared pair is fully
    # absent, matching managed omission semantics.
    if shared_available:
        return SHARED_DRIVER_CONTRACT_ID, shared_base
    if not shared_absent:
        log.error("generated lifecycle service %s is incomplete", shared_base)
        return None
    if legacy_available:
        log.warning(
            "shared lifecycle stubs are absent; direct launch is falling back "
            "to deprecated contract %s. Rebuild against %s",
            legacy_contract_id,
            SHARED_DRIVER_CONTRACT_ID,
        )
        return legacy_contract_id, legacy_base
    if not legacy_absent:
        log.error("generated lifecycle service %s is incomplete", legacy_base)
    return None


def resolve_servicer(contract_id: str, contracts_grpc_module):
    """Find the generated Servicer class + add-to-server fn + canonical method name
    for a contract. Returns (servicer_class, method_name, add_fn, pascal_base) or None.
    """
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
        1
        for p in sig.parameters.values()
        if p.kind
        in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    )
    if nparams >= 2:

        def impl(self, request, context, _fn=fn):
            return _fn(request, context)

    else:

        def impl(self, request, context, _fn=fn):
            return _fn(request)

    return type(
        f"Robonix{servicer_cls.__name__}Impl", (servicer_cls,), {method_name: impl}
    )


def build_lifecycle_servicer(
    namespace: str,
    contracts_grpc_module,
    response_cls,
    *,
    on_init=None,
    on_activate=None,
    on_deactivate=None,
    on_shutdown=None,
    on_shutdown_complete=None,
    on_state_change=None,
    log_tag: str = "robonix_api",
    requested_contract_id: str | None = None,
    allow_old_artifact_fallback: bool = False,
):
    """Build (without starting a server) the lifecycle Servicer instance.

    Returns ``(instance, add_to_server_fn, pascal_base,
    method_name='Driver', contract_id)`` when codegen emitted the shared
    lifecycle Servicer (preferred) or a legacy ``<namespace>/driver``
    Servicer. Returns ``None`` when the selected contract is unavailable in
    the generated stubs.

    `on_state_change(state, detail)` is invoked AFTER each handler returns
    ok=true and is the framework's hook for pushing state transitions to
    atlas. State strings: "inactive" / "active" / "error". Capability
    layer wires this; lower-level callers can leave it None.
    """
    selected = lifecycle_contract_for_module(
        namespace,
        contracts_grpc_module,
        requested_contract_id,
        allow_old_artifact_fallback=allow_old_artifact_fallback,
    )
    if selected is None:
        log.info(
            "[%s] no %s or %s/driver contract — skipping lifecycle servicer",
            log_tag,
            SHARED_DRIVER_CONTRACT_ID,
            namespace,
        )
        return None
    contract_id, base = selected
    servicer_cls = getattr(contracts_grpc_module, f"{base}Servicer", None)
    add_fn = getattr(contracts_grpc_module, f"add_{base}Servicer_to_server", None)
    if servicer_cls is None or add_fn is None:
        log.warning("[%s] incomplete generated lifecycle service %s", log_tag, base)
        return None

    def _emit_state(target: str | None, detail: str = "") -> None:
        """Push state transition to the Capability layer. `target=None`
        means "don't transition; just update detail" (used for Deferred
        so the provider stays in its current atlas-side state but
        rbnx caps/state_detail explains why)."""
        if on_state_change is not None:
            try:
                on_state_change(target, detail)
            except Exception:  # noqa: BLE001
                log.exception("[%s] on_state_change(%s) raised", log_tag, target)

    def _post_handler_state(cmd: int, result: Result) -> None:
        """Drive the atlas state machine based on the handler's Result.
        Ok → advance to next state. Err → ERROR. Deferred → no transition
        (provider stays in current state); the framework / operator decides
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
                log_tag,
                what,
                type(e).__name__,
                e,
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
        return response_cls(
            ok=False,
            state="error",
            error=f"unknown Result variant: {type(result).__name__}",
        )

    def Driver(self, request, context):  # noqa: N802 — matches generated stub
        cmd = int(request.command)
        log.info("[%s] Driver(cmd=%d) received", log_tag, cmd)
        cfg = parse_cfg(request)

        def missing_handler_ok(handler_name: str):
            log.warning(
                "[%s] no %s handler; applying lifecycle transition as a no-op",
                log_tag,
                handler_name,
            )
            result = Ok()
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        # Lifecycle handlers are optional. The shared Driver still provides a
        # complete state machine for providers that do not need work at a
        # given transition; a warning keeps the omission visible.
        if cmd == CMD_INIT:
            if on_init is None:
                return missing_handler_ok("on_init")
            result = _run_handler(on_init, "on_init", cfg)
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        # Handlers take no args — only on_init receives config.
        if cmd == CMD_ACTIVATE:
            if on_activate is None:
                return missing_handler_ok("on_activate")
            result = _run_handler(on_activate, "on_activate")
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        if cmd == CMD_DEACTIVATE:
            if on_deactivate is None:
                return missing_handler_ok("on_deactivate")
            result = _run_handler(on_deactivate, "on_deactivate")
            _post_handler_state(cmd, result)
            return _to_response(cmd, result)

        # CMD_SHUTDOWN: optional handler. Provider is going away regardless;
        # Result is logged but doesn't change termination.
        if cmd == CMD_SHUTDOWN:
            if on_shutdown is not None:
                result = _run_handler(on_shutdown, "on_shutdown")
            else:
                log.warning(
                    "[%s] no on_shutdown handler; applying lifecycle transition as a no-op",
                    log_tag,
                )
                result = Ok()
            _post_handler_state(cmd, result)
            response = _to_response(cmd, result)
            if on_shutdown_complete is not None:
                # grpc.ServicerContext.add_callback runs after this RPC has
                # terminated, so the response is not canceled by stopping the
                # server that is currently carrying it.
                add_callback = getattr(context, "add_callback", None)
                callback_registered = bool(
                    callable(add_callback) and add_callback(on_shutdown_complete)
                )
                if not callback_registered:
                    # Unit/direct callers may not provide a real gRPC context.
                    # Production gRPC contexts always take the callback.
                    on_shutdown_complete()
            return response

        # Unknown command — proto evolved newer than the provider.
        err = Err(f"unknown command code {cmd}")
        _post_handler_state(cmd, err)
        return _to_response(cmd, err)

    DynServicer = type("RobonixLifecycleServicer", (servicer_cls,), {"Driver": Driver})
    return DynServicer(), add_fn, base, "Driver", contract_id


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
