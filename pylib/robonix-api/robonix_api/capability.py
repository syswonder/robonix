# SPDX-License-Identifier: MulanPSL-2.0
"""User-facing CapabilityProvider classes: `Primitive`, `Service`, `Skill`.

A Robonix package instantiates exactly one of these. The framework
talks to atlas (RegisterPrimitive/Service/Skill + DeclareCapability +
Heartbeat), serves the lifecycle gRPC API
(Driver(CMD_INIT/ACTIVATE/DEACTIVATE/SHUTDOWN)), and provides thin
helpers over rclpy / grpcio / FastMCP for the most common patterns.

Internal `_ProviderBase` shares all the lifecycle / decorator / server
plumbing; the three concrete classes differ only in which atlas
Register RPC they call.

Layered API:
  - Layer 1 (always available): declare_capability, connect_capability,
    spawn subprocess, sentinel waits.
  - Layer 2 (opt-in convenience): create_publisher / create_subscription
    for ROS 2; `@provider.provides_grpc(...)` / `@provider.provides_mcp(...)`
    decorators that register the handler AND atlas-declare the
    Capability in one step (description is pulled from the function's
    docstring or passed explicitly).
"""
from __future__ import annotations

import inspect
import ipaddress
import json
import logging
import os
import signal
import threading
from pathlib import Path
from typing import Any, Callable

from . import scribe_logger
from ._lifecycle_internal import _set_lifecycle_state
from .atlas import ATLAS
from .atlas_types import (
    Capability,
    CapabilityProvider,
    Channel,
    GrpcParams,
    Kind,
    LifecycleState,
    McpParams,
    Ros2Params,
    Transport,
)
from .codegen import ensure_proto_gen, find_pkg_root
from .lifecycle import (
    bind_user_handler,
    build_lifecycle_servicer,
    resolve_servicer,
)
from .result import Err, Ok
from .ros import RosBackend, resolve_msg_type
from .spawn import SpawnRegistry
from .tool import mcp_contract

log = logging.getLogger("robonix_api.capability")

_INSTANCE_NAME_ENV = "RBNX_INSTANCE_NAME"


# Transport-ENUM <-> contract.mode compatibility matrix (best-effort
# check at declare_capability time).
_MODE_TRANSPORT_OK = {
    "rpc": {"grpc", "mcp", "ros2"},
    "topic_in": {"ros2", "grpc"},
    "topic_out": {"ros2", "grpc"},
}


def _provider_bind_host(value: str | None = None) -> str:
    """Return the provider server bind host as a normalized IPv4 literal.

    The default preserves existing cross-host behavior. Deployments that must
    keep capability servers local can set ``ROBONIX_PROVIDER_BIND_HOST`` to
    ``127.0.0.1`` before constructing a provider.
    """

    raw = (
        os.environ.get("ROBONIX_PROVIDER_BIND_HOST", "0.0.0.0")
        if value is None
        else value
    )
    host = str(raw).strip()
    try:
        address = ipaddress.ip_address(host)
    except ValueError as exc:
        raise ValueError(
            "ROBONIX_PROVIDER_BIND_HOST must be an IPv4 address literal"
        ) from exc
    if not isinstance(address, ipaddress.IPv4Address):
        raise ValueError("ROBONIX_PROVIDER_BIND_HOST must be an IPv4 address literal")
    return str(address)


def _resolve_provider_id(default_id: str) -> str:
    """Resolve a package default id to its deploy-time instance identity."""
    instance_id = os.environ.get(_INSTANCE_NAME_ENV, "").strip()
    if not instance_id:
        if os.environ.get("RBNX_DEPLOY_MANAGED", "").strip():
            raise RuntimeError(
                "RBNX_INSTANCE_NAME must be non-empty for a deploy-managed package"
            )
        return default_id
    if instance_id != default_id:
        log.info(
            "deployment instance id overrides package default: %r -> %r",
            default_id,
            instance_id,
        )
    return instance_id


def _mcp_loopback_allowed_hosts(bind_host: str) -> list[str]:
    """Return exact/wildcard Host values accepted by a loopback MCP server."""

    if not ipaddress.ip_address(bind_host).is_loopback:
        return []
    hosts = [bind_host, f"{bind_host}:*"]
    if bind_host == "127.0.0.1":
        hosts.extend(["localhost", "localhost:*"])
    return hosts


# ── _ProviderBase ───────────────────────────────────────────────────────────


class _ProviderBase:
    """Internal base for Primitive / Service / Skill. NOT exported.

    Args:
        id:        stable provider id (e.g. "front_camera").
                   Convention: matches `name:` in package_manifest.yaml.
        namespace: primary contract grouping for this provider, e.g.
                   "robonix/primitive/camera". Domain contracts normally
                   use this prefix. Shared contracts may opt out; other
                   mismatches produce diagnostics but remain callable.
        pkg_root:  package root directory; auto-detected from the
                   caller's __file__ when omitted.
        md_path:   absolute path to CAPABILITY.md; defaults to
                   <pkg_root>/CAPABILITY.md if it exists.
    """

    # Concrete subclasses set this to their kind.
    _kind: Kind = Kind.UNSPECIFIED

    def __init__(
        self,
        id: str,
        namespace: str,
        *,
        pkg_root: Path | None = None,
        md_path: str | None = None,
    ) -> None:
        self.default_id = id
        self.id = _resolve_provider_id(id)
        self.namespace = namespace.strip("/")
        if not self.namespace:
            raise ValueError("namespace must be non-empty")
        self._bind_host = _provider_bind_host()

        # Locate pkg_root from the caller's frame if not given.
        if pkg_root is None:
            caller_file = _caller_file(skip=1)
            if caller_file is not None:
                pkg_root = find_pkg_root(caller_file)
        self.pkg_root: Path = (pkg_root or Path.cwd()).resolve()

        # Add the package's codegen output to sys.path so atlas_pb2 /
        # contracts are importable when run() actually needs them.
        ensure_proto_gen(self.pkg_root)

        # md_path: explicit overrides; else <pkg_root>/CAPABILITY.md if
        # it exists.
        if md_path is None:
            cand = self.pkg_root / "CAPABILITY.md"
            md_path = str(cand) if cand.is_file() else ""
        self._md_path = md_path

        # Ports are auto-allocated in run(): gRPC's `add_insecure_port`
        # returns the actually bound port; MCP uses a free-port preclaim.
        self._driver_port: int = 0
        self._mcp_port: int = 0

        self._spawn = SpawnRegistry()

        # User-registered handlers (filled by decorators).
        self._on_init: Callable | None = None
        self._on_activate: Callable | None = None
        self._on_deactivate: Callable | None = None
        self._on_shutdown: Callable | None = None

        # Lifecycle state. Source of truth on the provider side; pushed to
        # atlas via the privileged `_set_lifecycle_state` whenever it
        # transitions.
        self._state: LifecycleState = LifecycleState.REGISTERED

        # Channels we opened via connect_capability(); force-closed on
        # teardown so atlas doesn't accumulate dangling edges.
        self._channels: list[Channel] = []

        # Decorator-registered handlers.
        self._mcp_app = None
        self._mcp_handlers: list[Callable] = []
        # (contract_id, fn, description)
        self._grpc_handlers: list[tuple[str, Callable, str]] = []
        # (contract_id, servicer instance)
        self._grpc_servicers: list[tuple[str, Any]] = []
        self._publishers: dict[str, Any] = {}

        self._driver_server = None
        self._mcp_server_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._stopping = threading.Event()

    # -- lifecycle decorators ----------------------------------------------

    def on_init(self, fn: Callable[[dict], Any]) -> Callable[[dict], Any]:
        """REGISTERED -> INACTIVE. Parse config, validate dependencies,
        bind logical device. NO hot runtime resources yet."""
        if self._on_init is not None:
            raise RuntimeError("on_init handler already registered")
        self._on_init = fn
        return fn

    def on_activate(self, fn: Callable[[], Any]) -> Callable[[], Any]:
        """INACTIVE -> ACTIVE. Acquire hot runtime resources (threads,
        models, ROS subs, hardware fds). Optional for Primitives /
        Services (framework auto-promotes); REQUIRED for Skills."""
        self._on_activate = fn
        return fn

    def on_deactivate(self, fn: Callable[[], Any]) -> Callable[[], Any]:
        """ACTIVE -> INACTIVE. Release hot resources, keep config /
        atlas registration."""
        self._on_deactivate = fn
        return fn

    def on_shutdown(self, fn: Callable[[], Any]) -> Callable[[], Any]:
        """any -> TERMINATED. Last-chance cleanup before process exit."""
        self._on_shutdown = fn
        return fn

    # -- lifecycle state ---------------------------------------------------

    @property
    def state(self) -> LifecycleState:
        return self._state

    def _set_state(self, new_state: LifecycleState | str | None, detail: str = "") -> None:
        """Update local state + push to atlas (privileged). Idempotent
        on no-change. `new_state=None` updates only state_detail."""
        if new_state is None:
            try:
                _set_lifecycle_state(self.id, self._state, detail)
            except Exception:  # noqa: BLE001
                pass
            return
        if isinstance(new_state, str):
            new_state = LifecycleState[new_state.upper()]
        if new_state == self._state:
            return
        prev = self._state
        self._state = new_state
        log.info(
            "[%s] state %s -> %s%s",
            self.id,
            prev.name,
            new_state.name,
            f" ({detail})" if detail else "",
        )
        try:
            _set_lifecycle_state(self.id, new_state, detail)
        except Exception:  # noqa: BLE001
            pass

    # -- Layer 1: raw atlas declares ---------------------------------------

    def declare_capability(
        self,
        contract_id: str,
        endpoint: str,
        transport: Transport | str | int,
        params: GrpcParams | Ros2Params | McpParams | None = None,
        description: str = "",
    ) -> str:
        """Declare a Capability for `contract_id` on this CapabilityProvider.
        `description` is the instance-specific natural-language string
        Pilot/LLM sees; empty means "use the contract's generic
        description from the TOML at consume time" (the two are merged,
        not picked-one-of). Namespace alignment is advisory: a regular
        contract outside this provider's primary namespace emits a warning
        but is still declared; contracts marked `cross_namespace` do not."""
        contract = ATLAS.query_contract(contract_id)
        cross_namespace = bool(contract and contract.cross_namespace)
        namespace = self.namespace.strip("/")
        normalized_contract = contract_id.strip("/")
        namespace_matches = normalized_contract == namespace or normalized_contract.startswith(
            f"{namespace}/"
        )
        if not namespace_matches and not cross_namespace:
            log.warning(
                "[%s] namespace mismatch: declaring '%s' outside primary "
                "namespace '%s'; Atlas will accept it and expose a diagnostic",
                self.id,
                contract_id,
                self.namespace,
            )
        return ATLAS.declare_capability(
            provider_id=self.id,
            contract_id=contract_id,
            transport=transport,
            endpoint=endpoint,
            params=params,
            description=description,
        )

    # -- Layer 1 conveniences (per-transport declare helpers) --------------

    def declare_ros2_topic(
        self,
        contract_id: str,
        topic: str,
        *,
        qos: str = "best_effort",
        description: str = "",
    ) -> str:
        """Declare a ROS 2 topic endpoint for a topic_in / topic_out contract."""
        return self.declare_capability(
            contract_id=contract_id,
            endpoint=topic,
            transport=Transport.ROS2,
            params=Ros2Params(qos_profile=qos),
            description=description,
        )

    def declare_ros2_service(
        self,
        contract_id: str,
        service: str,
        *,
        description: str = "",
    ) -> str:
        """Declare a ROS 2 service endpoint for an rpc contract over ROS 2."""
        return self.declare_capability(
            contract_id=contract_id,
            endpoint=service,
            transport=Transport.ROS2,
            params=Ros2Params(qos_profile=""),
            description=description,
        )

    def declare_grpc(
        self,
        contract_id: str,
        endpoint: str,
        service_name: str,
        method: str,
        proto_file: str = "robonix_contracts.proto",
        description: str = "",
    ) -> str:
        return self.declare_capability(
            contract_id=contract_id,
            endpoint=endpoint,
            transport=Transport.GRPC,
            params=GrpcParams(
                proto_file=proto_file,
                service_name=service_name,
                method=method,
            ),
            description=description,
        )

    def declare_mcp(
        self,
        contract_id: str,
        endpoint: str,
        input_schema_json: str = "{}",
        description: str = "",
    ) -> str:
        return self.declare_capability(
            contract_id=contract_id,
            endpoint=endpoint,
            transport=Transport.MCP,
            params=McpParams(input_schema_json=input_schema_json),
            description=description,
        )

    # -- Layer 1: connect (consumer side) ----------------------------------

    def connect_capability(
        self,
        provider: CapabilityProvider | Capability,
        contract_id: str,
        transport: Transport | str | int,
    ) -> Channel:
        """Open a channel to another CapabilityProvider's Capability.
        `provider` may be a `CapabilityProvider` (from `ATLAS.query_*`) or a
        `Capability` (from `ATLAS.find_capability`); both carry the
        provider id."""
        provider_id = provider.id if isinstance(provider, CapabilityProvider) else provider.provider_id
        ch = ATLAS.connect_capability(
            consumer_id=self.id,
            provider_id=provider_id,
            contract_id=contract_id,
            transport=transport,
        )
        self._channels.append(ch)
        return ch

    # -- Layer 1: subprocess + ROS sentinel --------------------------------

    def spawn(
        self,
        argv,
        *,
        env: dict | None = None,
        log: str | Path | None = None,
        cwd: Path | None = None,
    ):
        log_path: Path | None
        if log is None:
            log_path = None
        elif isinstance(log, Path):
            log_path = log
        else:
            log_path = self.pkg_root / "rbnx-build" / "data" / str(log)
        return self._spawn.spawn(argv, env=env, log_path=log_path, cwd=cwd)

    def wait_for_topic(
        self, topic: str, msg_type: str | type, timeout_s: float = 30.0
    ) -> bool:
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        return RosBackend.get().wait_for_topic(topic, cls, timeout_s)

    def resolve_host_ip(self, target_ip: str) -> str | None:
        """Return the source address selected by `ip route get <target>`.

        Some vendor drivers require that address in their generated config.
        """
        import subprocess

        try:
            out = subprocess.run(
                ["ip", "-4", "route", "get", target_ip],
                capture_output=True,
                text=True,
                timeout=2,
                check=False,
            )
        except FileNotFoundError:
            return None
        toks = out.stdout.split()
        if "src" in toks:
            i = toks.index("src")
            if i + 1 < len(toks):
                return toks[i + 1]
        return None

    def _advertise_host(self) -> str:
        """The IP a provider publishes into atlas for its gRPC / MCP
        endpoints. Must be reachable by a cross-host consumer (e.g. an
        executor on another machine), so hardcoding 127.0.0.1 breaks any
        deployment where consumer and provider are not co-located: atlas
        would hand the consumer "127.0.0.1:<port>" and it would dial its
        own loopback instead of this host.

        Resolution order: ROBONIX_ADVERTISE_HOST (explicit override) -> an
        explicit non-wildcard provider bind host -> the local source IP that
        routes toward the atlas host -> 127.0.0.1 when route resolution fails.
        A loopback bind therefore advertises loopback automatically instead of
        publishing an endpoint that the server does not accept."""
        explicit = os.environ.get("ROBONIX_ADVERTISE_HOST", "").strip()
        if explicit:
            return explicit
        if not ipaddress.ip_address(self._bind_host).is_unspecified:
            return self._bind_host
        atlas = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
        atlas_host = atlas.rsplit(":", 1)[0] if ":" in atlas else atlas
        return self.resolve_host_ip(atlas_host) or "127.0.0.1"

    # -- Layer 2: ROS publisher / subscriber -------------------------------

    def create_publisher(
        self,
        contract_id: str,
        *,
        topic: str,
        msg_type: type | str,
        qos: str | int = "best_effort",
        declare: bool = True,
        description: str = "",
    ):
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        pub = RosBackend.get().create_publisher(cls, topic, qos)
        self._publishers[contract_id] = pub
        if declare:
            self.declare_capability(
                contract_id=contract_id,
                endpoint=topic,
                transport=Transport.ROS2,
                params=Ros2Params(qos_profile=qos if isinstance(qos, str) else "reliable"),
                description=description,
            )
        return pub

    def create_subscription(
        self,
        contract_id: str,
        *,
        topic: str,
        msg_type: type | str,
        callback: Callable[[Any], None],
        qos: str | int = "best_effort",
        declare: bool = True,
    ):
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        sub = RosBackend.get().create_subscription(cls, topic, callback, qos)
        if declare:
            try:
                self.declare_capability(
                    contract_id=contract_id,
                    endpoint=topic,
                    transport=Transport.ROS2,
                    params=Ros2Params(qos_profile=qos if isinstance(qos, str) else "reliable"),
                )
            except Exception:  # noqa: BLE001
                # Consumer-side declare is optional; don't fail if atlas refuses.
                pass
        return sub

    def create_subscription_from_channel(
        self,
        channel: Channel,
        *,
        msg_type: type | str,
        callback: Callable[[Any], None],
    ):
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        qos = 0
        if isinstance(channel.params, Ros2Params) and channel.params.qos_profile:
            qos_profile = channel.params.qos_profile
            qos = qos_profile if isinstance(qos_profile, int) else 0
        return RosBackend.get().create_subscription(cls, channel.endpoint, callback, qos)

    def emit(self, contract_id: str, msg: Any) -> None:
        pub = self._publishers.get(contract_id)
        if pub is None:
            raise RuntimeError(
                f"no publisher for contract {contract_id!r} -- "
                f"call create_publisher(...) first"
            )
        pub.publish(msg)

    # -- Layer 2: provides_mcp decorator -----------------------------------

    def provides_mcp(self, contract_id: str, *, description: str = ""):
        """Register an MCP tool bound to `contract_id`. The MCP-server-
        side tool name is the contract_id's leaf segment — same value
        executor's dispatch derives — so there is no overridable
        `name=`. The natural-language description is taken from the
        wrapped function's docstring unless `description=` is passed
        explicitly."""
        self._check_mode("mcp", contract_id)
        self._ensure_mcp_app()

        def decorator(fn):
            mcp_contract(
                self._mcp_app,  # pyright: ignore[reportArgumentType]
                contract_id=contract_id,
            )(fn)  # pyright: ignore[reportArgumentType]
            # Resolve description: explicit kwarg wins; else docstring;
            # else empty (consumer falls back to contract default).
            desc = description.strip() or (fn.__doc__ or "").strip()
            fn._robonix_description = desc  # type: ignore[attr-defined]
            self._mcp_handlers.append(fn)
            return fn

        return decorator

    # `mcp` is the canonical decorator name (used throughout the dev guide
    # and all packages); `provides_mcp` is an equivalent long-form alias.
    mcp = provides_mcp

    def _ensure_mcp_app(self) -> None:
        if self._mcp_app is not None:
            return
        from mcp.server.fastmcp import FastMCP
        from mcp.server.transport_security import TransportSecuritySettings

        # Preserve cross-host compatibility for wildcard/LAN binds. A
        # loopback-only deployment is also reachable from a local browser, so
        # keep the MCP SDK's Host-header/DNS-rebinding guard enabled there.
        # The SDK defaults allowed_hosts to an empty list when callers provide
        # explicit transport settings, which otherwise rejects every Atlas MCP
        # request with HTTP 421. Allow only this loopback endpoint (with its
        # dynamically allocated port) and the equivalent localhost spelling.
        protect_loopback = ipaddress.ip_address(self._bind_host).is_loopback
        self._mcp_app = FastMCP(
            self.id,
            host=self._bind_host,
            transport_security=TransportSecuritySettings(
                enable_dns_rebinding_protection=protect_loopback,
                allowed_hosts=_mcp_loopback_allowed_hosts(self._bind_host),
            ),
        )

    def use_mcp_app(self, app) -> None:
        if self._mcp_app is not None and self._mcp_app is not app:
            raise RuntimeError(
                "MCP app already set; use_mcp_app conflicts with @provides_mcp"
            )
        self._mcp_app = app

    @property
    def mcp_endpoint(self) -> str:
        return f"http://{self._advertise_host()}:{self._mcp_port}/mcp/"

    # -- Layer 2: provides_grpc decorator + attach_grpc_servicer -----------

    def attach_grpc_servicer(self, contract_id: str, servicer,
                             *, description: str = "") -> None:
        """Attach an already-built Servicer instance for `contract_id`.
        Use this for multi-method services; for single-method handlers
        prefer `@provider.provides_grpc(...)`."""
        self._check_mode("grpc", contract_id)
        self._grpc_servicers.append((contract_id, servicer))
        # Description currently dropped for full servicers — they handle
        # multiple methods and don't have a single docstring. Future:
        # walk each method's docstring.
        if description:
            log.debug("attach_grpc_servicer(%s): description ignored "
                      "(use provides_grpc for per-method docs)", contract_id)

    def provides_grpc(self, contract_id: str, *, description: str = ""):
        """Bind a handler to `contract_id`'s generated gRPC Servicer.
        Description is taken from the function's docstring unless
        `description=` is passed explicitly."""
        self._check_mode("grpc", contract_id)

        def decorator(fn):
            desc = description.strip() or (fn.__doc__ or "").strip()
            self._grpc_handlers.append((contract_id, fn, desc))
            return fn

        return decorator

    # `grpc` is the canonical decorator name (used throughout the dev guide
    # and all packages); `provides_grpc` is an equivalent long-form alias.
    grpc = provides_grpc

    # -- mode/transport compat check (best-effort) -------------------------

    def _check_mode(self, transport: str, contract_id: str) -> None:
        # Best-effort: would scan capabilities/<full_path>.v1.toml for
        # [mode] type. For now we trust the user.
        return

    # -- run / bootstrap ---------------------------------------------------

    def bootstrap(self) -> None:
        """Non-blocking setup. Idempotent on re-entry."""
        if self._driver_server is not None:
            return
        self._do_bootstrap()

    def run(self) -> None:
        """Blocking. Calls bootstrap() then signal.pause()-equivalents
        until SIGTERM / SIGINT."""
        self._do_bootstrap()

        signal.signal(signal.SIGTERM, lambda *_: self._teardown_and_exit())
        signal.signal(signal.SIGINT, lambda *_: self._teardown_and_exit())

        log.info("ready -- awaiting Driver(CMD_INIT)")
        try:
            while not self._stopping.is_set():
                self._stopping.wait(60.0)
        finally:
            self._teardown()

    # Subclasses override to call the right Register* RPC.
    def _atlas_register(self) -> bool:
        raise NotImplementedError

    def _do_bootstrap(self) -> None:
        # 0. Route ALL stdlib logging through Scribe under this provider's id
        # (the same tag rbnx uses for its log file). This replaces any handler
        # installed by imports — fastmcp/uvicorn's rich handler, a host app's
        # own config — so the whole provider lifecycle (register, ready, Driver
        # commands, on_init, ACTIVE) lands in `rbnx logs -t <id>` instead of a
        # raw stderr that the unified log never sees. Deferred from import time
        # so a bare `import robonix_api` does not touch the host's logging.
        scribe_logger.install_stdlib_bridge(self.id)

        # 1. atlas register
        registered_ok = False
        try:
            self._atlas_register()
            log.info("registered %s '%s'", self._kind.name.lower(), self.id)
            registered_ok = True
        except Exception as e:  # noqa: BLE001
            log.warning("Register%s failed: %s", self._kind.name.capitalize(), e)
        if registered_ok:
            self._set_state(LifecycleState.REGISTERED)

        # 2. gRPC server
        import grpc
        import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore
        import lifecycle_pb2  # type: ignore
        from concurrent import futures

        server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))

        # 2a. driver lifecycle servicer (only when codegen emitted a
        # `<ns>/driver` contract; system services without hardware init
        # don't need it).
        driver_decl: tuple[str, str] | None = None
        lifecycle_info = build_lifecycle_servicer(
            self.namespace,
            contracts_grpc,
            lifecycle_pb2.Driver_Response,
            on_init=self._on_init,
            on_activate=self._on_activate,
            on_deactivate=self._on_deactivate,
            on_shutdown=self._user_shutdown_then_teardown,
            on_state_change=self._set_state,
            log_tag=self.id,
        )
        if lifecycle_info is not None:
            lifecycle_inst, lifecycle_add_fn, driver_base, driver_method = lifecycle_info
            lifecycle_add_fn(lifecycle_inst, server)
            driver_decl = (driver_base, driver_method)

        # 2b. user @provider.provides_grpc handlers
        user_grpc_decls: list[tuple[str, str, str, str]] = []
        for contract_id, fn, desc in self._grpc_handlers:
            info = resolve_servicer(contract_id, contracts_grpc)
            if info is None:
                log.warning(
                    "@provides_grpc(%r): no generated Servicer found "
                    "(did codegen run for this contract?). Skipping.",
                    contract_id,
                )
                continue
            servicer_cls, method_name, add_fn, base = info
            DynServicer = bind_user_handler(servicer_cls, method_name, fn)
            add_fn(DynServicer(), server)
            user_grpc_decls.append((contract_id, base, method_name, desc))
            log.info("wired @provides_grpc %s -> %s.%s", contract_id, base, method_name)

        # 2b'. attach_grpc_servicer
        for contract_id, servicer in self._grpc_servicers:
            info = resolve_servicer(contract_id, contracts_grpc)
            if info is None:
                log.warning(
                    "attach_grpc_servicer(%r): no generated Servicer found. Skipping.",
                    contract_id,
                )
                continue
            servicer_cls, method_name, add_fn, base = info
            if not isinstance(servicer, servicer_cls):
                log.warning(
                    "attach_grpc_servicer(%r): servicer %r is not a %s",
                    contract_id, type(servicer).__name__, servicer_cls.__name__,
                )
            add_fn(servicer, server)
            user_grpc_decls.append((contract_id, base, method_name, ""))
            log.info("attached gRPC servicer for %s -> %s.%s",
                     contract_id, base, method_name)

        # 2c. bind on port 0 -- OS picks free port.
        self._driver_port = server.add_insecure_port(f"{self._bind_host}:0")
        if self._driver_port == 0:
            raise RuntimeError(
                f"cannot bind lifecycle gRPC server on {self._bind_host}"
            )
        server.start()
        self._driver_server = server
        log.info(
            "Lifecycle gRPC serving on %s:%d", self._bind_host, self._driver_port
        )

        # 3. atlas-declare every gRPC capability
        endpoint = f"{self._advertise_host()}:{self._driver_port}"
        if driver_decl is not None:
            driver_base, driver_method = driver_decl
            try:
                self.declare_capability(
                    contract_id=f"{self.namespace}/driver",
                    endpoint=endpoint,
                    transport=Transport.GRPC,
                    params=GrpcParams(
                        proto_file="robonix_contracts.proto",
                        service_name=driver_base,
                        method=driver_method,
                    ),
                )
            except Exception as e:  # noqa: BLE001
                log.warning("DeclareCapability(driver) failed: %s", e)
        for contract_id, service_name, method, desc in user_grpc_decls:
            try:
                self.declare_capability(
                    contract_id=contract_id,
                    endpoint=endpoint,
                    transport=Transport.GRPC,
                    params=GrpcParams(
                        proto_file="robonix_contracts.proto",
                        service_name=service_name,
                        method=method,
                    ),
                    description=desc,
                )
            except Exception as e:  # noqa: BLE001
                log.warning("DeclareCapability(%s) failed: %s", contract_id, e)

        # 4. FastMCP server
        if self._mcp_app is not None:
            self._start_mcp_server()
            if self._mcp_handlers:
                self._declare_mcp_handlers()

        # 5. heartbeat — pass the provider's stop Event so the thread
        # exits on _teardown instead of pinging atlas after TERMINATED.
        self._heartbeat_thread = ATLAS.start_heartbeat(self.id, stop=self._stopping)

        # 6. state promotion: caps WITHOUT a Driver contract (system
        # services with only MCP tools) are fully ready as soon as gRPC
        # + MCP listen, so promote to ACTIVE here. Caps WITH a Driver
        # contract wait for rbnx-boot to fire CMD_INIT / CMD_ACTIVATE.
        if driver_decl is None and registered_ok:
            self._set_state(LifecycleState.ACTIVE)

    def _start_mcp_server(self) -> None:
        import socket
        import uvicorn

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self._bind_host, 0))
        self._mcp_port = s.getsockname()[1]
        s.close()
        cfg = uvicorn.Config(
            self._mcp_app.streamable_http_app(),  # pyright: ignore[reportOptionalMemberAccess]
            host=self._bind_host,
            port=self._mcp_port,
            log_level="warning",
        )
        server = uvicorn.Server(cfg)
        thread = threading.Thread(target=server.run, name="robonix-mcp", daemon=True)
        thread.start()
        self._mcp_server_thread = thread
        log.info("MCP HTTP serving on %s:%d", self._bind_host, self._mcp_port)

    def _declare_mcp_handlers(self) -> None:
        endpoint = f"http://{self._advertise_host()}:{self._mcp_port}/mcp/"
        for fn in self._mcp_handlers:
            cid = getattr(fn, "_robonix_contract_id", None)
            if cid is None:
                continue
            description = getattr(fn, "_robonix_description", "") or (fn.__doc__ or "").strip()
            input_cls = getattr(fn, "_robonix_input_cls", None)
            schema_json = json.dumps(
                input_cls.json_schema()
                if input_cls is not None
                else {"type": "object", "properties": {}, "required": []}
            )
            try:
                self.declare_capability(
                    contract_id=cid,
                    endpoint=endpoint,
                    transport=Transport.MCP,
                    params=McpParams(input_schema_json=schema_json),
                    description=description,
                )
            except Exception as e:  # noqa: BLE001
                log.warning("declare mcp %s failed: %s", cid, e)

    def _user_shutdown_then_teardown(self):
        result = None
        if self._on_shutdown is not None:
            try:
                result = self._on_shutdown()
            except Exception as exc:  # noqa: BLE001
                log.exception("[%s] on_shutdown raised", self.id)
                result = Err(f"{type(exc).__name__}: {exc}")
        self._teardown()
        return result if result is not None else Ok()

    def _teardown(self) -> None:
        for ch in self._channels:
            ch.close()
        self._channels.clear()
        self._spawn.shutdown_all()
        if self._driver_server is not None:
            try:
                self._driver_server.stop(grace=2.0)
            except Exception:  # noqa: BLE001
                pass

    def _teardown_and_exit(self) -> None:
        self._stopping.set()
        if self._on_shutdown is not None:
            try:
                self._on_shutdown()
            except Exception:  # noqa: BLE001
                log.exception("[%s] on_shutdown raised", self.id)
        try:
            self._set_state(LifecycleState.TERMINATED, "process signal teardown")
        except Exception:  # noqa: BLE001
            pass
        self._teardown()


# ── concrete CapabilityProvider classes ─────────────────────────────────────


class Primitive(_ProviderBase):
    """A hardware / data-source driver CapabilityProvider.

        primitive_cam = Primitive(
            id="front_camera",
            namespace="robonix/primitive/camera",
        )
    """

    _kind = Kind.PRIMITIVE

    def _atlas_register(self) -> bool:
        ATLAS.register_primitive(self.id, self.namespace, self._md_path or "")
        return True


class Service(_ProviderBase):
    """A composed CapabilityProvider built on top of Primitives /
    Services.  e.g. mapping, navigation, scene; also the platform-
    internal pilot / executor / scene / memory / liaison services.

        service_mapping = Service(
            id="mapping",
            namespace="robonix/service/mapping",
        )
    """

    _kind = Kind.SERVICE

    def _atlas_register(self) -> bool:
        ATLAS.register_service(self.id, self.namespace, self._md_path or "")
        return True


class Skill(_ProviderBase):
    """A model-backed, executor-activated CapabilityProvider.  Sits at
    INACTIVE between calls; the executor flips it to ACTIVE on demand
    (and MAY flip back when idle, configurable).

        skill_explore = Skill(
            id="explore",
            namespace="robonix/skill/explore",
        )
    """

    _kind = Kind.SKILL

    def _atlas_register(self) -> bool:
        ATLAS.register_skill(self.id, self.namespace, self._md_path or "")
        return True


# ── helpers ─────────────────────────────────────────────────────────────────


def _caller_file(skip: int = 0) -> Path | None:
    """Walk up the stack looking for the first frame outside this package."""
    here = Path(__file__).parent.resolve()
    frame = inspect.currentframe()
    if frame is None:
        return None
    frame = frame.f_back  # caller of the helper
    for _ in range(skip):
        if frame is None:
            return None
        frame = frame.f_back
    while frame is not None:
        f = frame.f_code.co_filename
        if not f.startswith(str(here)):
            return Path(f).resolve()
        frame = frame.f_back
    return None


__all__ = ["Primitive", "Service", "Skill"]
